"""One script convention and scheduler: optional PARAMETERS/TELEMETRY, and run(flight)."""

import ast
import json
import math
import runpy
from dataclasses import dataclass
from pathlib import Path

from libs.flightlog import TelemetryRecorder, TelemetrySpec, flight_snapshot, telemetry_path
from libs.simrun import START_OPTIONS, STARTS

CONTROL_PERIOD_S = 0.02
DEFAULT_LIMIT_S = 600.0
CONTROLLERS = tuple(sorted(path.stem for path in (Path(__file__).resolve().parent.parent / "controllers").glob("*.py")
                           if path.stem != "__init__"))
SCENARIOS_DIR = Path(__file__).resolve().parent.parent / "data/scenarios"


@dataclass(frozen=True)
class ScenarioSpec:
    description: str
    parameters: dict
    telemetry: TelemetrySpec | None
    phases: dict[str, str]
    log_fields: tuple[str, ...]
    start: dict  # START: default start ("start") and start options, as the CLI names them; the CLI and toolbar override

    @classmethod
    def from_source(cls, source, controller):
        tree = ast.parse(source)
        functions = {node.name: node for node in tree.body if isinstance(node, ast.FunctionDef)}
        if "run" not in functions:
            raise ValueError("Every scenario needs def run(flight)")
        values = {}
        for node in tree.body:
            if isinstance(node, ast.ImportFrom) and node.module == "controllers":
                for item in node.names:
                    if item.name != controller:
                        raise ValueError(f"The {controller} script imports controller {item.name}")
            if isinstance(node, ast.Assign):
                for target in node.targets:
                    if isinstance(target, ast.Name) and target.id in ("PARAMETERS", "TELEMETRY", "LOG_FIELDS", "START"):
                        if target.id in values:
                            raise ValueError(f"Define {target.id} once")
                        values[target.id] = ast.literal_eval(node.value)
                    if isinstance(target, ast.Name) and target.id in ("EXPERIMENT", "ENGINES"):
                        raise ValueError("Use PARAMETERS/TELEMETRY and the controller's script directory")
        parameters = values.get("PARAMETERS", {})
        if not isinstance(parameters, dict) or any(not isinstance(key, str) for key in parameters):
            raise ValueError("PARAMETERS must be a dictionary with named values")
        json.dumps(parameters, allow_nan=False)
        phases = {}
        for node in functions["run"].body:
            if (isinstance(node, ast.Expr) and isinstance(node.value, ast.Call)
                    and isinstance(node.value.func, ast.Attribute) and node.value.func.attr == "phase"):
                call = node.value
                if len(call.args) != 2 or call.keywords:
                    continue
                if not all(isinstance(arg, ast.Constant) and isinstance(arg.value, str) for arg in call.args):
                    continue
                name, description = (arg.value for arg in call.args)
                if not isinstance(name, str) or not name or not isinstance(description, str):
                    raise ValueError("phase needs a name and description string")
                phases[name] = description
        telemetry = TelemetrySpec.read(values["TELEMETRY"]) if "TELEMETRY" in values else None
        log_fields = values.get("LOG_FIELDS", ())
        if not isinstance(log_fields, (tuple, list)) or any(not isinstance(name, str) or not name for name in log_fields):
            raise ValueError("LOG_FIELDS must list the recorded column names")
        if len(set(log_fields)) != len(log_fields):
            raise ValueError("LOG_FIELDS must be unique")
        start = read_start(values.get("START", {}))
        return cls(ast.get_docstring(tree) or ast.get_docstring(functions["run"]) or "", parameters, telemetry, phases, tuple(log_fields), start)

    def payload(self):
        return {"description": self.description, "parameters": self.parameters, "phases": self.phases, "start": self.start,
                "telemetry": None if self.telemetry is None else {"fields": self.telemetry.fields, "rate_hz": self.telemetry.rate_hz}}


def read_start(start):
    """A scenario's START: optional "start" (one of STARTS) and START_OPTIONS values (finite numbers), each one that
    applies to that start."""
    if not isinstance(start, dict):
        raise ValueError("START must be a dictionary of start conditions")
    unknown = start.keys() - {"start", *START_OPTIONS}
    if unknown:
        raise ValueError(f"START: unknown {sorted(unknown)}; use start and {list(START_OPTIONS)}")
    if "start" in start and start["start"] not in STARTS:
        raise ValueError(f"START: start must be one of {STARTS}")
    for option, value in start.items():
        if option == "start":
            continue
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f"START: {option} must be a finite number")
        if "start" in start and not START_OPTIONS[option].applies_to(start["start"]):
            raise ValueError(f"START: {option} does not apply to a {start['start']} start")
    return dict(start)


def scenario_names(directory, controller):
    if controller not in CONTROLLERS:
        raise ValueError(f"Controller must be one of {CONTROLLERS}")
    return sorted({path.stem for folder in (SCENARIOS_DIR / controller, directory / controller)
                   for path in folder.glob("*.py")})


def scenario_path(directory, controller, name):
    if controller not in CONTROLLERS or not name or Path(name).name != name:
        raise ValueError("Choose a controller and a plain scenario name")
    path = directory / controller / (name + ".py")
    return path if path.is_file() else SCENARIOS_DIR / controller / (name + ".py")


def load_scenario(path):
    return runpy.run_path(str(path))["run"]


class ScenarioStopped(Exception):
    """A scheduled stop condition or time limit fired."""


class Flight:
    """Simulation clock, conditions, truth, raw sensors, recording and scheduling; controller commands live elsewhere."""

    def __init__(self, engine, start, initial, measurements_path, *, airframe, export, control_count, duration_s):
        self._engine = engine
        self.start, self.initial = start, initial
        self.airframe, self.export, self.control_count = airframe, export, control_count
        self.motor_count = len(export.motor_properties)
        self.cruise_m_s = airframe.operating_point.airspeed_m_s
        self.sensors = engine.hardware
        self.measurements_path = measurements_path
        self.measurements = []
        self.parameters = {}
        self.telemetry = None
        self.current_phase = ""
        self.duration_s = duration_s
        self._origin = engine.elapsed
        self._stop_condition = None
        self._stop_reason = "stop condition"
        self._highest_agl_m = 0.0
        engine.flight = self

    @property
    def elapsed(self):
        return self._engine.elapsed - self._origin

    @property
    def truth(self):
        return self._engine.plant.truth()

    @property
    def state(self):
        return flight_snapshot(self._engine, self.export) | self.sensors._sensor_columns() | self.sensors._logged

    @property
    def raw(self):
        return self._engine.plant.fdm

    def micros(self):
        return round(self.elapsed * 1e6)

    def millis(self):
        return self.micros() // 1000

    def log(self, name, value):
        if name not in self.sensors._logged:
            raise ValueError(f"Declare {name!r} in LOG_FIELDS before recording it")
        self.sensors._logged[name] = value

    def stop_when(self, condition, reason="stop condition"):
        self._stop_condition, self._stop_reason = condition, reason

    def stop(self, reason="script requested stop"):
        raise ScenarioStopped(reason)

    def phase(self, name, description):
        if not isinstance(name, str) or not name or not isinstance(description, str):
            raise ValueError("phase needs a name and description string")
        self.current_phase = name
        print(f"Phase: {name} at {self.elapsed:.3f} s", flush=True)

    def _tick(self):
        from libs.simrun import GroundHit, ground_hit_text, hit_ground
        self._highest_agl_m = max(self._highest_agl_m, self.truth["agl_m"])
        if hit_ground(self.raw, self._highest_agl_m):
            raise GroundHit(ground_hit_text(self.elapsed, self.raw))
        if self.elapsed >= self.duration_s:
            self.stop(f"{self.duration_s:g} s simulated time limit")
        if self._stop_condition is not None and self._stop_condition(self):
            self.stop(self._stop_reason)
        if self.telemetry is not None:
            self.telemetry.sample()

    def wait(self, seconds, *, until=None, control=None):
        if not math.isfinite(seconds) or seconds <= 0:
            raise ValueError("wait seconds must be finite and positive")
        end = self.elapsed + seconds
        while self.elapsed < end - 1e-9:
            self._tick()
            if until is not None and until(self):
                return
            if control is not None:
                control(self)
            self._engine.run(min(CONTROL_PERIOD_S, end - self.elapsed))
        self._tick()
        if until is not None and not until(self):
            raise TimeoutError(f"scenario condition not reached after {seconds:g} flight seconds")

    def measure(self, label, **values):
        point = {"label": label, "t": self.elapsed, **(values or self.truth)}
        self.measurements.append(point)
        self.measurements_path.write_text(json.dumps(self.measurements, indent=2) + "\n")
        # The run output shows the status line only: the label, the time and any explicitly passed values,
        # rounded; the full point (with truth data) stays in the measurements file.
        brief = {"label": label, "t": round(self.elapsed, 3),
                 **{name: round(value, 4) if isinstance(value, float) else value for name, value in values.items()}}
        print("Measurement: " + json.dumps(brief), flush=True)
        return point

    def run(self, script, spec):
        self.parameters = spec.parameters.copy()
        if spec.telemetry is not None:
            path = telemetry_path(self.measurements_path.with_suffix(""))
            self.telemetry = TelemetryRecorder(self, spec.telemetry, path)
            print(f"Selected telemetry: {path.name}, {spec.telemetry.rate_hz:g} Hz maximum", flush=True)
        completed = False
        try:
            self._tick()
            script(self)
            completed = True
        finally:
            if self.telemetry is not None:
                self.telemetry.close()
                self.telemetry = None
            self._engine.flight = None
            print("Scenario outcome: " + ("completed" if completed else f"interrupted during {self.current_phase or 'run'}; partial logs retained"), flush=True)

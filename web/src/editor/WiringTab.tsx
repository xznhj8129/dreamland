import { useEffect, useState } from 'react'
import type { ReactElement } from 'react'
import type { Path, Snapshot } from '../types'
import { Card, ChoiceField, EntryField } from './fields'
import { batteryConnector, syncedMotorRuns } from './propulsion'
import { isObject } from './patch'
import { useFormApi } from './session'

type Vec3 = [number, number, number]

function vec3(value: unknown): Vec3 | null {
  if (!Array.isArray(value) || value.length !== 3) return null
  const [x, y, z] = value
  return typeof x === 'number' && typeof y === 'number' && typeof z === 'number' ? [x, y, z] : null
}

function route(value: unknown): Vec3[] | null {
  if (!Array.isArray(value)) return null
  const points = value.map(vec3)
  return points.every((point): point is Vec3 => point !== null) ? points : null
}

/** libs.geometry.route_length_m (1013-1014): the sum of the segment lengths. */
function routeLengthM(points: Vec3[]): number {
  let total = 0.0
  for (let index = 1; index < points.length; index += 1) {
    const [ax, ay, az] = points[index - 1]
    const [bx, by, bz] = points[index]
    total += Math.hypot(bx - ax, by - ay, bz - az)
  }
  return total
}

/** editor_view.build_wiring_tab (627-682), sync_motor_runs (611-625), plug_box (666-682). */
export function WiringTab({ snapshot }: { snapshot: Snapshot }): ReactElement {
  const api = useFormApi()
  const context = api.context
  const definition = api.definition
  const propulsion = isObject(definition.propulsion) ? definition.propulsion : null
  const electric = propulsion !== null && isObject(propulsion.electric) ? propulsion.electric : null
  const wiring = electric !== null && isObject(electric.wiring) ? electric.wiring : null
  const batteryRun = wiring !== null && isObject(wiring.battery) ? wiring.battery : null
  const motorRuns = wiring !== null && Array.isArray(wiring.motors) ? wiring.motors.filter(isObject) : []

  // 611-625: propulsion.electric.wiring.motors follows the motor count on every snapshot change (an added motor copies the
  // last run), left alone while a mount names a missing surface. The server now ports this into apply_set/apply_op
  // before the recompute, so this is a no-op safety net; it sends at most one set when a snapshot arrives with the
  // runs out of step, and the server's own sync makes the result stable (no loop).
  useEffect(() => {
    if (propulsion === null || wiring === null) return
    const next = syncedMotorRuns(propulsion, definition, context, motorRuns)
    if (next !== null) void api.set(['propulsion', 'electric', 'wiring', 'motors'], next)
    // api is rebuilt per render; the definition and context identities drive this effect.
  }, [propulsion, wiring, definition, context])

  if (propulsion === null) {
    // 629-631: a hint until the Airframe tab's type flags add the propulsion block.
    return (
      <Card title="Wiring">
        <p className="form-hint">Not powered (type on the Airframe tab).</p>
      </Card>
    )
  }
  if (electric === null) {
    return (
      <Card title="Wiring">
        <p className="form-hint">No electric system (Propulsion tab, Electric).</p>
      </Card>
    )
  }
  if (wiring === null || batteryRun === null) {
    return (
      <Card title="Wiring">
        <p className="form-hint">propulsion.electric.wiring is missing (see Log).</p>
      </Card>
    )
  }

  const resolved = snapshot.resolved
  const wires = resolved !== null && isObject(resolved.wires) ? resolved.wires : null
  const current = snapshot.definition_valid && wires !== null && Array.isArray(wires.motors) && wires.motors.length === motorRuns.length
  const resolvedMotors = resolved !== null && Array.isArray(resolved.motors) ? resolved.motors : []
  const names: string[] = current
    ? resolvedMotors.map((motor) => (isObject(motor) && typeof motor.name === 'string' ? motor.name : 'motor'))
    : motorRuns.map((_, index) => `motor ${index + 1}`)
  // 637: routes come from the resolved wires only when their motor count still matches the definition's runs.
  const motorRoutes = wires !== null && Array.isArray(wires.motors) ? wires.motors : []
  const rows = [
    { label: 'battery -> ESC', path: ['propulsion', 'electric', 'wiring', 'battery'] as Path, run: batteryRun, route: current && wires !== null ? route(wires.battery) : null },
    ...motorRuns.map((run, index) => ({
      label: `ESC -> ${names[index]}`,
      path: ['propulsion', 'electric', 'wiring', 'motors', index] as Path,
      run,
      route: current ? route(motorRoutes[index]) : null,
    })),
  ]
  const battery = isObject(electric.battery) ? electric.battery : {}
  const gauges = context.wire_gauges.map(String)

  return (
    <Card title="Wire runs">
      <p className="form-hint">Routes follow the axes and run inside wing panels. Lengths update from the component positions.</p>
      {rows.map(row => <WireRow key={row.label} row={row} gauges={gauges} battery={row.label === 'battery -> ESC' ? battery : null} />)}
    </Card>
  )
}

interface WireRowProps {
  row: { label: string; path: Path; run: Record<string, unknown>; route: Vec3[] | null }
  gauges: string[]
  /** The battery run's own pack (its connector is the first plug, 653-655), else null. */
  battery: Record<string, unknown> | null
}

/** One run of the table: length, AWG, wires and the plug boxes (build_wiring_tab 642-657, plug_box 666-682). */
function WireRow({ row, gauges, battery }: WireRowProps): ReactElement {
  const api = useFormApi()
  const context = api.context
  const plugs = Array.isArray(row.run.plugs) ? row.run.plugs.filter((plug): plug is string => typeof plug === 'string') : []

  return (
    <fieldset className="form-group wire-run">
      <legend>{row.label}</legend>
      <p className="wire-length">Route length: {row.route === null ? 'Unavailable' : `${(routeLengthM(row.route) * 1000.0).toFixed(0)} mm`}</p>
        <ChoiceField
          label="Wire gauge (AWG)"
          path={[...row.path, 'gauge_awg']}
          value={row.run.gauge_awg}
          options={gauges}
          convert={Number}
        />
        <EntryField label="Conductors" path={[...row.path, 'conductors']} unitKey="conductors" />
      <div className="form-label">Connectors, in route order</div>
      <div className="wiring-plugs">
        {/* 653-655: the battery's own connector comes first, set in its part library entry (or the custom pack). */}
        {battery !== null && <span className="plug-fixed">Pack: {batteryConnector(battery, context)}</span>}
        {plugs.map((plug, index) => (
          <select
            key={index}
            className="field-input plug-select"
            aria-label={`${row.label} connector ${index + 1}`}
            value={plug}
            onChange={(event) => {
              const choice = event.target.value
              const next = choice === context.remove_plug ? plugs.filter((_, position) => position !== index) : plugs.map((item, position) => (position === index ? choice : item))
              void api.set([...row.path, 'plugs'], next)
            }}
          >
            {[...context.connectors, context.remove_plug].map((option) => (
              <option key={option} value={option}>
                {option}
              </option>
            ))}
          </select>
        ))}
        <AddPlug path={row.path} plugs={plugs} />
      </div>
    </fieldset>
  )
}

/** 666-682: the trailing ADD_PLUG box appends the picked connector (Tk rebuilds the box; here it resets to "+"). */
function AddPlug({ path, plugs }: { path: Path; plugs: string[] }): ReactElement {
  const api = useFormApi()
  const context = api.context
  const [choice, setChoice] = useState(context.add_plug)

  return (
    <select
      className="field-input plug-select"
      aria-label="Add connector"
      value={choice}
      onChange={(event) => {
        const picked = event.target.value
        setChoice(context.add_plug)
        if (picked !== context.add_plug) void api.set([...path, 'plugs'], [...plugs, picked])
      }}
    >
      <option value={context.add_plug}>{context.add_plug}</option>
      {context.connectors.map((name) => (
        <option key={name} value={name}>
          {name}
        </option>
      ))}
    </select>
  )
}

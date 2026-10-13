import { useEffect, useRef, useState } from 'react'
import type { ReactElement } from 'react'
import uPlot from 'uplot'
import 'uplot/dist/uPlot.min.css'
import { saveProject, useSnapshot } from '../store'
import { abortSim, simOptions, simState, startSim } from '../simApi'
import type { ScenarioOutline as Outline, SimOptions, SimStart } from '../simApi'
import { SimFileEditor } from './SimFileEditor'
import { FlightScene } from './FlightScene'
import { PerformanceResults } from './PerformanceResults'
import { ScenarioOutline } from './ScenarioOutline'
import { SplitPane } from '../components/SplitPane'

const INAV = 'inav'
const GENERATED = 'generated'
const DEFAULT_X = 't'
const DEFAULT_Y = ['agl_m', 'ias_kts', 'pitch']
const POLL_MS = 500

type LogRows = (number | null)[][]
type XZoom = readonly [number, number] | null

// FlightLogPlot: the retired Tk view's Plots tab (libs/views/sim_view.py: draw_series 389-430, on_scroll 433-441,
// on_press/drag/release 443-470, SERIES_COLORS 35, ZOOM_STEP/DRAG_MIN_FRACTION 36-37).
const PLOT_FALLBACK_HEIGHT = 400
const PLOT_TITLE_HEIGHT = 24 // matches .sim-plot-title's min-height in theme.css
const PLOT_FALLBACK_WIDTH = 600
const AXIS_COLOR = '#67ff67'
const GRID_COLOR = 'rgba(10, 95, 10, 0.55)'
const SERIES_COLORS = ['#19ff19', '#33ccff', '#ffcc33', '#ff6666', '#cc66ff', '#ffffff']
const ZOOM_STEP = 0.8
const DRAG_MIN_FRACTION = 0.01

/** True when a start condition does not apply to this start: such a field is disabled and left out of the run body,
 *  as the runner refuses one passed by hand. */
function conditionOff(option: { air_only: boolean; launcher_only: boolean; catapult_only: boolean }, start: string): boolean {
  return (
    (option.air_only && start !== 'air') ||
    (option.launcher_only && start !== 'catapult' && start !== 'rail') ||
    (option.catapult_only && start !== 'catapult')
  )
}

function conditionDefault(options: SimOptions, start: string, key: string): string {
  const value = options.launcher_defaults[start]?.[key] ?? options.condition_defaults[key]
  return value === undefined ? '' : String(value)
}

function isFiniteNumber(value: number | null | undefined): value is number {
  return typeof value === 'number' && Number.isFinite(value)
}

/** x/y pairs for one subplot; a row without a finite x cannot be placed and is dropped, a non-finite y is a gap. */
function subplotData(rows: LogRows, xIndex: number, yIndex: number): [number[], (number | null)[]] {
  const x: number[] = []
  const y: (number | null)[] = []
  for (const row of rows) {
    const xValue = row[xIndex]
    if (!isFiniteNumber(xValue)) continue
    const yValue = row[yIndex]
    x.push(xValue)
    y.push(isFiniteNumber(yValue) ? yValue : null)
  }
  return [x, y]
}

/**
 * The Tk y fit (sim_view.py:399-402): 5% of the shown range (or 5% of |high|, or 1e-3) around the data inside the
 * current X window; an empty window falls back to the full-data extent the caller offers, as Tk left matplotlib's
 * auto ylim in place when nothing was shown.
 */
function windowedYRange(
  u: uPlot,
  zoom: XZoom,
  dataMin: number | null,
  dataMax: number | null,
): [number, number] {
  const xs = u.data[0]
  const ys = u.data[1]
  let low = Infinity
  let high = -Infinity
  if (xs !== undefined && ys !== undefined) {
    for (let index = 0; index < xs.length; index++) {
      const x = xs[index]
      if (zoom !== null && (x < zoom[0] || x > zoom[1])) continue
      const y = ys[index]
      if (!isFiniteNumber(y)) continue
      if (y < low) low = y
      if (y > high) high = y
    }
  }
  if (!Number.isFinite(low) || !Number.isFinite(high)) {
    const fallbackLow = isFiniteNumber(dataMin) ? dataMin : 0
    const fallbackHigh = isFiniteNumber(dataMax) ? dataMax : 1
    if (fallbackHigh === fallbackLow) return [fallbackLow - 1, fallbackHigh + 1]
    const margin = (fallbackHigh - fallbackLow) * 0.05
    return [fallbackLow - margin, fallbackHigh + margin]
  }
  const margin = (high - low) * 0.05 || Math.max(Math.abs(high) * 0.05, 1e-3)
  return [low - margin, high + margin]
}

/** One stacked subplot; only the bottom chart shows the X axis, each y axis is labelled with its column. */
function subplotOptions(
  width: number,
  height: number,
  label: string,
  colorIndex: number,
  showX: boolean,
  xLabel: string,
  zoomRef: { current: XZoom },
): uPlot.Options {
  return {
    width,
    height,
    padding: [6, 8, 0, 0],
    cursor: { show: true, points: { show: false }, drag: { setScale: false, x: true, y: false } },
    legend: { show: false },
    scales: {
      x: { time: false },
      y: { range: (u, min, max) => windowedYRange(u, zoomRef.current, min, max) },
    },
    axes: [
      showX
        ? { label: xLabel, stroke: AXIS_COLOR, grid: { stroke: GRID_COLOR }, ticks: { stroke: GRID_COLOR } }
        : { show: false },
      { show: height >= 60, stroke: AXIS_COLOR, grid: { stroke: GRID_COLOR }, ticks: { stroke: GRID_COLOR } },
    ],
    series: [
      {},
      { label, stroke: SERIES_COLORS[colorIndex % SERIES_COLORS.length], width: 1.2, points: { show: false } },
    ],
  }
}

/** A chart reset to the whole run: its first and last x value, as Tk's x_range = None did. */
function resetPlotX(plot: uPlot): void {
  const xs = plot.data[0]
  if (xs !== undefined && xs.length > 0) plot.setScale('x', { min: xs[0], max: xs[xs.length - 1] })
  else plot.setScale('x', { min: 0, max: 1 })
}

/** Set a subplot's data and re-apply the shared zoom; a zoomed chart must keep the window while rows arrive. */
function setPlotData(plot: uPlot, data: uPlot.AlignedData, zoom: XZoom): void {
  if (zoom === null) {
    plot.setData(data, true)
  } else {
    plot.setData(data, false)
    plot.setScale('x', { min: zoom[0], max: zoom[1] })
  }
}

/** Wheel zoom, drag-to-zoom and double-click reset on one subplot, lifted to the shared zoom via onXZoom. */
function wireInteractions(plot: uPlot, onXZoom: (range: XZoom) => void): () => void {
  const over = plot.over
  const onWheel = (event: WheelEvent): void => {
    event.preventDefault()
    const rect = over.getBoundingClientRect()
    const value = plot.posToVal(event.clientX - rect.left, 'x')
    const low = plot.scales.x.min
    const high = plot.scales.x.max
    if (!Number.isFinite(value) || !isFiniteNumber(low) || !isFiniteNumber(high) || high <= low) return
    const factor = event.deltaY < 0 ? ZOOM_STEP : 1 / ZOOM_STEP
    onXZoom([value - (value - low) * factor, value + (high - value) * factor])
  }
  const onDouble = (): void => onXZoom(null)
  let pendingUp: ((event: MouseEvent) => void) | null = null
  const onDown = (event: MouseEvent): void => {
    if (event.button !== 0) return
    const startLeft = event.clientX - over.getBoundingClientRect().left
    const onUp = (up: MouseEvent): void => {
      window.removeEventListener('mouseup', onUp)
      pendingUp = null
      const endLeft = up.clientX - over.getBoundingClientRect().left
      plot.setSelect({ left: 0, top: 0, width: 0, height: 0 }, false) // uPlot shaded the stretch; clear it
      const x0 = plot.posToVal(startLeft, 'x')
      const x1 = plot.posToVal(endLeft, 'x')
      const low = plot.scales.x.min
      const high = plot.scales.x.max
      if (!Number.isFinite(x0) || !Number.isFinite(x1) || !isFiniteNumber(low) || !isFiniteNumber(high)) return
      if (Math.abs(x1 - x0) > DRAG_MIN_FRACTION * (high - low)) onXZoom([Math.min(x0, x1), Math.max(x0, x1)])
    }
    pendingUp = onUp
    window.addEventListener('mouseup', onUp)
  }
  over.addEventListener('wheel', onWheel, { passive: false })
  over.addEventListener('dblclick', onDouble)
  over.addEventListener('mousedown', onDown)
  return () => {
    over.removeEventListener('wheel', onWheel)
    over.removeEventListener('dblclick', onDouble)
    over.removeEventListener('mousedown', onDown)
    if (pendingUp !== null) window.removeEventListener('mouseup', pendingUp)
  }
}

interface FlightLogPlotProps {
  header: string[] | null
  rows: LogRows
  xColumn: string
  yColumns: readonly string[]
  xZoom: XZoom
  onXZoom: (range: XZoom) => void
}

/**
 * The Plots tab: one uPlot per checked Y against the picked X, stacked and sharing one X window. Instances are
 * created/destroyed only when the checked set changes; data updates go in place, live rows keep the zoom, and the
 * zoom itself lives in SimView so every subplot follows it.
 */
function FlightLogPlot({ header, rows, xColumn, yColumns, xZoom, onXZoom }: FlightLogPlotProps): ReactElement {
  const hostRef = useRef<HTMLDivElement | null>(null)
  const chartsRef = useRef<Map<string, uPlot>>(new Map())
  const zoomRef = useRef<XZoom>(xZoom)
  zoomRef.current = xZoom
  const yKey = yColumns.join('\u0000')
  const hasHeader = header !== null
  const [maximized, setMaximized] = useState<string | null>(null)

  // Create one chart per checked Y; React re-renders leave the instances alone.
  useEffect(() => {
    const host = hostRef.current
    if (host === null || header === null) return undefined
    const columns = yColumns.filter((name) => header.includes(name) && (maximized === null || !yColumns.includes(maximized) || name === maximized))
    if (columns.length === 0) return undefined
    const width = host.clientWidth > 0 ? host.clientWidth : PLOT_FALLBACK_WIDTH
    const height = Math.max(20, Math.floor((host.clientHeight || PLOT_FALLBACK_HEIGHT) / columns.length) - PLOT_TITLE_HEIGHT)
    const xIndex = header.indexOf(xColumn)
    if (xIndex < 0) return undefined
    const charts = new Map<string, uPlot>()
    const boxes: HTMLDivElement[] = []
    const cleanups: (() => void)[] = []
    columns.forEach((name, index) => {
      const box = document.createElement('div')
      box.className = 'sim-subplot'
      host.appendChild(box)
      boxes.push(box)
      const title = document.createElement('button')
      title.type = 'button'
      title.className = 'sim-plot-title'
      title.textContent = name + (maximized === name ? ' · Show all' : ' · Maximize')
      title.onclick = () => setMaximized(current => current === name ? null : name)
      box.appendChild(title)
      const plot = new uPlot(
        subplotOptions(width, height, name, index, index === columns.length - 1, xColumn, zoomRef),
        subplotData(rows, xIndex, header.indexOf(name)),
        box,
      )
      cleanups.push(wireInteractions(plot, onXZoom))
      setPlotData(plot, subplotData(rows, xIndex, header.indexOf(name)), zoomRef.current)
      charts.set(name, plot)
    })
    chartsRef.current = charts
    return () => {
      for (const cleanup of cleanups) cleanup()
      for (const plot of charts.values()) plot.destroy()
      for (const box of boxes) box.remove()
      if (chartsRef.current === charts) chartsRef.current = new Map()
    }
    // Data updates are applied by the effect below; recreating only on a checked-set change keeps instances alive.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [hasHeader, yKey, maximized, xColumn])

  // Redraw in place when the rows, header, X column or checked set change.
  useEffect(() => {
    if (header === null) return
    const xIndex = header.indexOf(xColumn)
    if (xIndex < 0) return
    for (const [name, plot] of chartsRef.current) {
      const yIndex = header.indexOf(name)
      if (yIndex < 0) continue
      setPlotData(plot, subplotData(rows, xIndex, yIndex), zoomRef.current)
    }
    const charts = Array.from(chartsRef.current.values())
    const bottom = charts[charts.length - 1]
    if (bottom !== undefined && bottom.axes[0] !== undefined) bottom.axes[0].label = xColumn
  }, [header, rows, xColumn, yKey, maximized])

  // The shared zoom: a wheel/drag/double-click in any subplot moves the window of all of them.
  useEffect(() => {
    for (const plot of chartsRef.current.values()) {
      if (xZoom === null) resetPlotX(plot)
      else plot.setScale('x', { min: xZoom[0], max: xZoom[1] })
    }
  }, [xZoom])

  // A hidden tab has no width: when it becomes visible, give every chart the container's real size.
  useEffect(() => {
    const host = hostRef.current
    if (host === null) return undefined
    const observer = new ResizeObserver(() => {
      const width = host.clientWidth
      if (width < 1) return
      const count = chartsRef.current.size
      if (!count) return
      const height = Math.max(20, Math.floor(host.clientHeight / count) - PLOT_TITLE_HEIGHT)
      for (const plot of chartsRef.current.values()) {
        if (plot.width !== width || plot.height !== height) plot.setSize({ width, height })
      }
    })
    observer.observe(host)
    return () => observer.disconnect()
  }, [])

  return (
    <div className="sim-plots" ref={hostRef}>
      {header === null && <p className="placeholder-note">No flight log yet. Run a simulation to plot its columns.</p>}
      {header !== null && yColumns.length === 0 && (
        <p className="placeholder-note">No Y values picked. Check one or more columns on the right.</p>
      )}
    </div>
  )
}

/**
 * The web SIM view (webapp/sim.py routes; user-visible behavior in docs/ui/sim_view.md, ported from the retired Tk
 * SimMainView). The server owns the one run; this view polls /api/sim/state for the status, the summary lines and
 * the flight-log tail, and POSTs the run/abort requests directly (simApi bypasses the store's mutation queue).
 * The Plots tab draws the log tail with uPlot, the 3D View tab flies the airframe along it with three.js.
 */
export function SimView(): ReactElement {
  const snapshot = useSnapshot()
  const project = snapshot?.project ?? null

  const [options, setOptions] = useState<SimOptions | null>(null)
  const [controller, setController] = useState(INAV)
  const [scenario, setScenario] = useState('')
  const [start, setStart] = useState('air')
  const [conditions, setConditions] = useState<Record<string, string>>({})
  const [inavConfig, setInavConfig] = useState(GENERATED)
  const [duration, setDuration] = useState('')
  const [status, setStatus] = useState('Idle')
  const [summary, setSummary] = useState<string[]>([])
  const [running, setRunning] = useState(false)
  const [starting, setStarting] = useState(false)
  const [stateSeen, setStateSeen] = useState(false)
  const [logFile, setLogFile] = useState<string | null>(null)
  const [telemetryFile, setTelemetryFile] = useState<string | null>(null)
  const [logHeader, setLogHeader] = useState<string[] | null>(null)
  const [logRows, setLogRows] = useState<LogRows>([])
  const [runId, setRunId] = useState<number | null>(null)
  const [xColumn, setXColumn] = useState(DEFAULT_X)
  const [yColumns, setYColumns] = useState<readonly string[]>(DEFAULT_Y)
  const [xZoom, setXZoom] = useState<XZoom>(null)
  const [tab, setTab] = useState<'plots' | '3d' | 'scenario' | 'diff' | 'performance'>('plots')
  const [pathSeen, setPathSeen] = useState(false)
  const [clock, setClock] = useState<NonNullable<SimStart['clock']>>('realtime')
  const [rate, setRate] = useState('200')
  const [msp, setMsp] = useState(true)
  const [mavlink, setMavlink] = useState(true)
  const [diffRevision, setDiffRevision] = useState(0)
  const [optionsRevision, setOptionsRevision] = useState(0)
  const [scenarioReady, setScenarioReady] = useState(false)
  const [scenarioDirty, setScenarioDirty] = useState(false)
  const [diffDirty, setDiffDirty] = useState(false)
  const [outline, setOutline] = useState<Outline | null>(null)
  const appliedStartRef = useRef('')
  const scenarioFile = scenario === '__new__' ? '__new__' : scenario ? scenario + '.py' : null


  const summaryRef = useRef<HTMLPreElement | null>(null)
  const stdoutOffsetRef = useRef(0)
  const logOffsetRef = useRef(0)
  const runIdRef = useRef<number | null>(null)
  const serverStatusRef = useRef('')
  const runningRef = useRef(false)

  // One poller for the life of the view: the server's run outlives tab switches, so the first answer (run_id
  // changed) replays the whole summary and a run started elsewhere shows up here.
  useEffect(() => {
    let alive = true
    const tick = async (): Promise<void> => {
      const payload = await simState(stdoutOffsetRef.current, logOffsetRef.current, runIdRef.current)
      if (!alive || payload === null) return
      const reset = payload.run_id !== runIdRef.current || payload.log.reset
      stdoutOffsetRef.current = payload.stdout.offset
      logOffsetRef.current = payload.log.offset
      runIdRef.current = payload.run_id
      setRunId(payload.run_id)
      setStateSeen(true)
      setLogFile(payload.log.file)
      setTelemetryFile(payload.telemetry_file ?? null)
      setRunning(payload.running)
      if (runningRef.current && !payload.running) setDiffRevision(value => value + 1)
      runningRef.current = payload.running
      setSummary((lines) => {
        if (reset) return payload.stdout.lines
        return payload.stdout.lines.length === 0 ? lines : [...lines, ...payload.stdout.lines]
      })
      setLogRows((rows) => {
        if (reset) return payload.log.rows
        return payload.log.rows.length === 0 ? rows : [...rows, ...payload.log.rows]
      })
      if (reset) setXZoom(null) // a new run starts at the whole (empty) window
      const header = payload.log.header
      if (reset || header !== null) setLogHeader(header)
      if (header !== null) {
        setXColumn((current) => (header.includes(current) ? current : DEFAULT_X))
        // Keep the picks across runs, intersected with the new header; none left means the defaults again.
        setYColumns((current) => {
          const kept = current.filter((name) => header.includes(name))
          return kept.length > 0 ? kept : DEFAULT_Y.filter((name) => header.includes(name))
        })
      }
      // A refusal message stays until the server's status actually moves (start, finish, abort).
      if (payload.status !== serverStatusRef.current) {
        serverStatusRef.current = payload.status
        setStatus(payload.status)
      }
    }
    const timer = window.setInterval(() => void tick(), POLL_MS)
    void tick()
    return () => {
      alive = false
      window.clearInterval(timer)
    }
  }, [])

  // The option lists refresh on mount and after Load, as the Tk view refreshed them whenever it was shown.
  useEffect(() => {
    if (project === null) return undefined
    let alive = true
    void simOptions().then((next) => {
      if (!alive || next === null) return
      setOptions(next)
      setController(current => next.controllers.includes(current) ? current : INAV)
      setScenario(current => Object.values(next.scenarios).some(names => names.includes(current)) ? current : next.scenarios[INAV]?.[0] ?? '')
      setInavConfig((current) =>
        current === GENERATED || next.inav_configs.some((config) => config.file === current) ? current : GENERATED,
      )
      setDuration((current) => (current === '' ? String(next.default_duration_s) : current))
      if (!runningRef.current && next.starts.length === 0) setStatus('No valid airframe')
    })
    return () => {
      alive = false
    }
  }, [project, optionsRevision])

  // The available starts come from the aircraft; scenario changes preserve the toolbar selection.
  useEffect(() => {
    if (options === null) return
    const starts = options.starts
    setStart((current) => (starts.includes(current) ? current : (starts[0] ?? '')))
  }, [options, controller, scenario])

  // The 3D scene is initialized lazily on its first show and then stays mounted (hidden) so the orbit angle
  // survives tab switches; the plots live only while their tab is visible.
  useEffect(() => {
    if (tab === '3d' && logHeader !== null) setPathSeen(true)
  }, [tab, logHeader])

  // While running, keep the summary pinned to its end; an idle summary stays where the user left it.
  useEffect(() => {
    if (!running) return
    const node = summaryRef.current
    if (node !== null) node.scrollTop = node.scrollHeight
  }, [summary, running])

  const scenarioChoices = options?.scenarios[controller] ?? []
  const noAirframe = options !== null && options.starts.length === 0
  const aborting = status === 'Aborting...'
  const emptyCondition = options?.start_options.some(option => !conditionOff(option, start) && conditions[option.key]?.trim() === '') ?? false
  const displayStatus = !running && status === 'Idle' && noAirframe ? 'No valid airframe' : status

  const setCondition = (key: string, value: string): void => {
    setConditions((current) => ({ ...current, [key]: value }))
  }

  /** Pre-select a scenario's START in the toolbar, once per scenario file and START; the user may change it after. */
  const applyScenarioStart = (file: string | null, plan: Outline | null | undefined): void => {
    if (!plan) return
    const key = `${controller}/${file ?? ''}:${JSON.stringify(plan.start)}`
    if (key === appliedStartRef.current) return
    appliedStartRef.current = key
    const { start: chosen, ...values } = plan.start
    if (typeof chosen === 'string') {
      if ((options?.starts ?? []).includes(chosen)) setStart(chosen)
      else setStatus(`Scenario START: a ${chosen} start is not available for this aircraft`)
    }
    if (Object.keys(values).length > 0) {
      setConditions((current) => ({ ...current, ...Object.fromEntries(Object.entries(values).map(([option, value]) => [option, String(value)])) }))
    }
  }

  const toggleY = (name: string): void => {
    setYColumns((current) => (current.includes(name) ? current.filter((item) => item !== name) : [...current, name]))
  }

  const run = async (): Promise<void> => {
    if (options === null || running || starting) return
    setStarting(true)
    try {
      await saveProject() // flush the queued edits; what flies is the file on disk
      const body: SimStart = { controller, scenario, start, clock, rate, duration }
      if (controller === INAV) { body.inav_config = inavConfig; body.msp = msp; body.mavlink = mavlink }
      const picked: Record<string, string> = {}
      for (const option of options.start_options) {
        const value = (conditions[option.key] ?? String(conditionDefault(options, start, option.key))).trim()
        if (value === '' || conditionOff(option, start)) continue
        picked[option.key] = value
      }
      body.conditions = picked
      const answer = await startSim(body)
      if (!answer.ok) setStatus(answer.message)
    } finally {
      setStarting(false)
    }
  }

  const abort = async (): Promise<void> => {
    if (!running || aborting) return
    if (await abortSim()) setStatus('Aborting...')
  }

  const starts = options?.starts ?? []

  return (
    <SplitPane minSizes={[300, 280, 180]} initialRatios={[2, 4, 1]}>
      <section className="panel left-pane sim-left">
        <div className="form-card sim-run-card">
          <div className="form-card-header">
            <h3>Simulation run</h3>
          </div>
          <div className="sim-field">
            <label htmlFor="sim-controller">Flight controller</label>
            <select
              id="sim-controller"
              className="field-input"
              value={controller}
              disabled={options === null || running || scenarioDirty || diffDirty}
              onChange={(event) => {
                const next = event.target.value
                setController(next)
                setDuration(String(options?.default_duration_s ?? 600))
                setScenarioReady(false)
                setScenario(options?.scenarios[next]?.[0] ?? '')
              }}
            >
              {(options?.controllers ?? []).map(name => <option key={name} value={name}>{name === INAV ? 'INAV SITL' : name === 'mcu' ? 'MCU · sensors / PWM' : name.toUpperCase()}</option>)}
            </select>
          </div>
          <div className="sim-field">
            <label htmlFor="sim-scenario">Scenario</label>
            <select
              id="sim-scenario"
              className="field-input"
              value={scenario}
              disabled={options === null || running || scenarioDirty}
              onChange={(event) => { setScenario(event.target.value); setScenarioReady(false) }}
            >
              {scenario === '__new__' && <option value="__new__">New scenario (save first)</option>}
              {scenarioChoices.map(name => <option key={name} value={name}>{name} · Python</option>)}
            </select>
            <button type="button" disabled={options === null || running || scenarioDirty} onClick={() => { setScenario('__new__'); setTab('scenario') }}>New scenario</button>
          </div>
          <div className="sim-field">
            <label htmlFor="sim-start">Start</label>
            <select
              id="sim-start"
              className="field-input"
              value={start}
              disabled={options === null || running}
              onChange={(event) => setStart(event.target.value)}
            >
              {starts.map((name) => (
                <option key={name} value={name}>
                  {name}
                </option>
              ))}
            </select>
          </div>
          <div className="sim-field">
            <label htmlFor="sim-config">FC settings</label>
            <select
              id="sim-config"
              className="field-input"
              value={inavConfig}
              disabled={options === null || controller !== INAV || running || diffDirty}
              onChange={(event) => setInavConfig(event.target.value)}
            >
              <option value={GENERATED}>default</option>
              {(options?.inav_configs ?? []).map((config) => (
                <option key={config.file} value={config.file}>
                  {config.label}
                </option>
              ))}
            </select>
          </div>
          {<>
            <div className="sim-field"><label htmlFor="sim-clock">Clock</label><select id="sim-clock" className="field-input" value={clock} disabled={running} onChange={event => setClock(event.target.value as NonNullable<SimStart['clock']>)}>
              <option value="realtime">Realtime</option><option value="stepped">Stepped · as fast as possible</option><option value="stepped-faster">Stepped + --faster · dedicated CPU</option>
            </select></div>
            <div className="sim-field"><label htmlFor="sim-rate">Tick rate (Hz)</label><input id="sim-rate" className="field-input" value={rate} disabled={running} onChange={event => setRate(event.target.value)} /></div>
            {controller === INAV && <><div className="sim-links">
              <label className="check-row"><input type="checkbox" checked={msp} disabled={running} onChange={event => setMsp(event.target.checked)} />MSP · Configurator</label>
              <label className="check-row"><input type="checkbox" checked={mavlink} disabled={running} onChange={event => setMavlink(event.target.checked)} />MAVLink · telemetry</label>
            </div><p className="form-hint">Shared firmware: {options?.firmware}</p></>}
          </>}
          {<div className="sim-field">
            <label htmlFor="sim-duration">Time limit (simulated s)</label>
            <input
              id="sim-duration"
              className="field-input"
              value={duration}
              disabled={running}
              spellCheck={false}
              onChange={(event) => setDuration(event.target.value)}
            />
          </div>}
          <p className="form-hint">The Python script owns commands and stop conditions. The selected controller determines its script directory.</p>
          {(scenarioDirty || diffDirty) && <p className="form-hint">Save the edited SIM files before running.</p>}
          <details className="advanced-fields sim-start-details">
          <summary>Start conditions</summary>
          <fieldset className="sim-conditions">
            <legend>Initial state & environment</legend>
            {(options?.start_options ?? []).filter(option => !conditionOff(option, start)).map((option) => (
              <div key={option.key} className="sim-condition">
                <label htmlFor={`sim-condition-${option.key}`}>{option.label}</label>
                <input
                  id={`sim-condition-${option.key}`}
                  className="field-input"
                  value={conditions[option.key] ?? (options === null ? '' : String(conditionDefault(options, start, option.key)))}
                  placeholder="Enter a value"
                  aria-label={option.label}
                  spellCheck={false}
                  onChange={(event) => setCondition(option.key, event.target.value)}
                />
                {conditions[option.key] === '' && <span className="field-error">Enter a value.</span>}
              </div>
            ))}
          </fieldset>
          </details>
          <div className="sim-buttons">
            <button
              type="button"
              className="action"
              disabled={options === null || !stateSeen || running || starting || noAirframe || emptyCondition || scenarioDirty || diffDirty || scenario === '__new__' || !scenarioReady}
              onClick={() => void run()}
            >
              Run
            </button>
            <button type="button" disabled={!running || aborting} onClick={() => void abort()}>
              Abort
            </button>
            <span className="status sim-status" role="status">
              {displayStatus}
            </span>
          </div>
        </div>

        <div className="form-card sim-summary-card">
          <div className="form-card-header">
            <h3>Run output</h3>
            {logFile !== null && logHeader !== null && <a href={`/files/${logFile}`} download>CSV</a>}
            {logFile !== null && summary.some(line => line.startsWith('Measurement:')) && <a href={`/files/${logFile.replace(/\.csv$/, '.measurements.json')}`} download>Measurements</a>}
            {telemetryFile !== null && <a href={`/files/${telemetryFile}`} download>Selected telemetry</a>}
          </div>
          <pre className="sim-summary" ref={summaryRef}>
            {summary.length === 0 ? 'No simulation run yet.' : summary.join('\n')}
          </pre>
        </div>
      </section>

      <section className="panel centre-pane sim-centre">
        <div className="tab-row">
          <button type="button" className={tab === 'plots' ? 'tab active' : 'tab'} onClick={() => setTab('plots')}>
            Plots
          </button>
          <button type="button" className={tab === '3d' ? 'tab active' : 'tab'} onClick={() => setTab('3d')}>
            3D View
          </button>
          <button type="button" className={tab === 'scenario' ? 'tab active' : 'tab'} onClick={() => setTab('scenario')}>Scenario</button>
          <button type="button" className={tab === 'diff' ? 'tab active' : 'tab'} onClick={() => setTab('diff')}>INAV diff</button>
          <button type="button" className={tab === 'performance' ? 'tab active' : 'tab'} onClick={() => setTab('performance')}>Performance</button>
        </div>
        <SimFileEditor controller={controller} kind="scenario" name={scenarioFile} project={project} template={options?.scenario_templates[controller]?.source ?? ''} templateOutline={options?.scenario_templates[controller]?.scenario} running={running} visible={tab === 'scenario'} onDirty={setScenarioDirty} onDiscard={() => { if (scenario === '__new__') setScenario(options?.scenarios[controller]?.[0] ?? ''); setScenarioDirty(false) }}
          onLoad={resource => { setOutline(resource.scenario ?? null); setScenarioReady(!resource.error); applyScenarioStart(scenarioFile, resource.scenario) }}
          onSaved={(name, resource) => {
            setScenario(name.slice(0, -3)); setScenarioDirty(false); setScenarioReady(true)
            setOutline(resource.scenario ?? null)
            applyScenarioStart(name, resource.scenario)
            void simOptions().then(setOptions)
          }} />
        <SimFileEditor controller={controller} kind="diff" name={inavConfig} project={project} template="" revision={diffRevision} running={running} visible={tab === 'diff'} onDirty={setDiffDirty} onDiscard={() => setDiffDirty(false)} onLoad={() => {}}
          onSaved={name => { setInavConfig(name); setDiffDirty(false); void simOptions().then(setOptions) }}
          onDeleted={() => { setInavConfig(GENERATED); setDiffDirty(false); setDiffRevision(value => value + 1); setOptionsRevision(value => value + 1) }} />
        {tab === 'plots' ? (
          <FlightLogPlot
            header={logHeader}
            rows={logRows}
            xColumn={xColumn}
            yColumns={yColumns}
            xZoom={xZoom}
            onXZoom={(range) => setXZoom(range)}
          />
        ) : null}
        {pathSeen && <FlightScene snapshot={snapshot} header={logHeader} rows={logRows} runId={runId} visible={tab === '3d' && logHeader !== null} />}
        {tab === '3d' && logHeader === null && <p className="placeholder">Run a simulation to see its flight path.</p>}
        {tab === 'performance' && <PerformanceResults lines={summary} model={options?.aero_model ?? ''} />}
      </section>

      <section className="panel right-pane sim-right">
        {tab === 'scenario' || tab === 'diff' ? (
          <div className="form-card">{tab === 'scenario' && <ScenarioOutline plan={outline} dirty={scenarioDirty} />}<div className="form-card-header"><h3>{tab === 'scenario' ? 'Scenario API' : 'FC settings'}</h3></div>
            {tab === 'scenario' ? <>
              {controller === INAV && <p className="form-hint">RC roll, pitch and yaw: −1…1. Throttle: 0…1. Positive pitch stick pushes the nose down.</p>}
              {controller === 'mcu' && <p className="form-hint">Write controller.esc[index] and controller.servo[index] in PWM microseconds (1000…2000). Read flight.sensors for the simulated I2C/SPI hardware. Control loops belong in the script.</p>}
              <p className="form-hint">Controller commands come from the imported module. flight.truth and flight.state report simulator state; flight.sensors exposes the raw hardware readings.</p>
              <p className="form-hint">The toolbar sets initial conditions; an optional START pre-selects the scenario's own. PARAMETERS and TELEMETRY are optional for every scenario. flight.wait runs on simulated time; flight.measure records a measurement.</p>
              <p className="form-hint">An unmet wait condition fails visibly at its timeout. Returning from run ends the flight. phase(name, description) labels a step.</p>
            </> : <p className="form-hint">Project SITL diffs are flown as saved. Generated diffs are refreshed on each run; save a copy to customize them. Real FC dumps retain the existing SITL conversion of mixer and AUX settings.</p>}
          </div>
        ) : logHeader === null ? (
          <div className="form-card">
            <div className="form-card-header">
              <h3>Flight log</h3>
            </div>
            <p className="form-hint">Fly a run to see the log's columns.</p>
          </div>
        ) : (
          <>
            <div className="form-card">
              <div className="form-card-header">
                <h3>X axis</h3>
              </div>
              <select
                className="field-input"
                value={xColumn}
                onChange={(event) => {
                  setXColumn(event.target.value)
                  setXZoom(null) // a new X column shows the whole run again
                }}
              >
                {logHeader.map((name) => (
                  <option key={name} value={name}>
                    {name}
                  </option>
                ))}
              </select>
            </div>
            <div className="form-card sim-y-card">
              <div className="form-card-header">
                <h3>Y values</h3>
              </div>
              <div className="sim-y-list">
                {logHeader.map((name) => (
                  <label key={name} className="check-row">
                    <input type="checkbox" checked={yColumns.includes(name)} onChange={() => toggleY(name)} />
                    {name}
                  </label>
                ))}
              </div>
            </div>
          </>
        )}
      </section>
    </SplitPane>
  )
}

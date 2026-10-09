import { useEffect, useRef, useState } from 'react'
import type { ReactElement } from 'react'
import * as THREE from 'three'
import { OrbitControls } from 'three/addons/controls/OrbitControls.js'
import uPlot from 'uplot'
import 'uplot/dist/uPlot.min.css'
import { saveProject, useSnapshot } from '../store'
import { abortSim, simOptions, simState, startSim } from '../simApi'
import type { SimOptions, SimStart } from '../simApi'
import { SplitPane } from '../components/SplitPane'

const INAV = 'inav'
const SCRIPT_PREFIX = 'script:'
const GENERATED = 'generated'
const DEFAULT_X = 't'
const DEFAULT_Y = ['agl_m', 'ias_kts', 'pitch']
const POLL_MS = 500

type LogRows = (number | null)[][]
type XZoom = readonly [number, number] | null

// FlightLogPlot: the retired Tk view's Plots tab (libs/views/sim_view.py: draw_series 389-430, on_scroll 433-441,
// on_press/drag/release 443-470, SERIES_COLORS 35, ZOOM_STEP/DRAG_MIN_FRACTION 36-37).
const PLOT_HEIGHT = 180
const PLOT_FALLBACK_WIDTH = 600
const AXIS_COLOR = '#67ff67'
const GRID_COLOR = 'rgba(10, 95, 10, 0.55)'
const SERIES_COLORS = ['#19ff19', '#33ccff', '#ffcc33', '#ff6666', '#cc66ff', '#ffffff']
const ZOOM_STEP = 0.8
const DRAG_MIN_FRACTION = 0.01

// FlightPath3D: PATH_COLUMNS/PATH_VIEW/colors/limits in the Tk view (sim_view.py:43-49, draw_path 471-517).
const PATH_COLUMNS = ['east_m', 'north_m', 'agl_m'] as const
const PATH_MIN_HALF_M = 50
const PATH_MIN_HEIGHT_M = 50
const PATH_BACKGROUND = '#001000'
const PATH_LINE_COLOR = '#19ff19'
const PATH_BOX_COLOR = '#14601a'
const PATH_START_COLOR = '#19ff19'
const PATH_END_COLOR = '#ff4d4d'
const PATH_FIT_MARGIN = 1.15
const PATH_DIRECTION = (() => {
  const elevation = THREE.MathUtils.degToRad(25) // PATH_VIEW elevation
  const azimuth = THREE.MathUtils.degToRad(-60) // PATH_VIEW azimuth
  return new THREE.Vector3(
    Math.cos(elevation) * Math.cos(azimuth),
    Math.cos(elevation) * Math.sin(azimuth),
    Math.sin(elevation),
  ).normalize()
})()

/** The script name of a "script: <name>" controller choice, or null for INAV SITL. */
function scriptName(controller: string): string | null {
  return controller.startsWith(SCRIPT_PREFIX) ? controller.slice(SCRIPT_PREFIX.length) : null
}

/** The starts the chosen controller/scenario offers: hover always stands on the ground, scripts fly any start. */
function startsFor(options: SimOptions, controller: string, scenario: string): string[] {
  if (scriptName(controller) === null && scenario === 'hover') return ['ground']
  return options.starts
}

/** True when a start condition does not apply to this start: such a field is disabled and left out of the run body,
 *  as the runner refuses one passed by hand. */
function conditionOff(option: { air_only: boolean; launcher_only: boolean; catapult_only: boolean }, start: string): boolean {
  return (
    (option.air_only && start !== 'air') ||
    (option.launcher_only && start !== 'catapult' && start !== 'rail') ||
    (option.catapult_only && start !== 'catapult')
  )
}

/** The selected launch start's default for one condition (the payload's launcher_defaults), as a placeholder. */
function launcherPlaceholder(options: SimOptions, start: string, key: string): string {
  const value = options.launcher_defaults[start]?.[key]
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
  label: string,
  colorIndex: number,
  showX: boolean,
  xLabel: string,
  zoomRef: { current: XZoom },
): uPlot.Options {
  return {
    width,
    height: PLOT_HEIGHT,
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
      { label, stroke: AXIS_COLOR, grid: { stroke: GRID_COLOR }, ticks: { stroke: GRID_COLOR } },
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

  // Create one chart per checked Y; React re-renders leave the instances alone.
  useEffect(() => {
    const host = hostRef.current
    if (host === null || header === null) return undefined
    const columns = yColumns.filter((name) => header.includes(name))
    if (columns.length === 0) return undefined
    const width = host.clientWidth > 0 ? host.clientWidth : PLOT_FALLBACK_WIDTH
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
      const plot = new uPlot(
        subplotOptions(width, name, index, index === columns.length - 1, xColumn, zoomRef),
        subplotData(rows, xIndex, header.indexOf(name)),
        box,
      )
      cleanups.push(wireInteractions(plot, onXZoom))
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
  }, [hasHeader, yKey])

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
  }, [header, rows, xColumn, yKey])

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
      for (const plot of chartsRef.current.values()) {
        if (plot.width !== width) plot.setSize({ width, height: PLOT_HEIGHT })
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

interface PathPoint {
  east: number
  north: number
  height: number
}

interface PathFrame {
  cx: number
  cy: number
  half: number
  top: number
}

interface FlightPathHandle {
  update: (samples: (PathPoint | null)[] | null) => void
}

/** The log's east/north/agl samples; a row missing any of the three is a gap, null when the columns are absent. */
function pathSamples(header: string[], rows: LogRows): (PathPoint | null)[] | null {
  const eastIndex = header.indexOf(PATH_COLUMNS[0])
  const northIndex = header.indexOf(PATH_COLUMNS[1])
  const heightIndex = header.indexOf(PATH_COLUMNS[2])
  if (eastIndex < 0 || northIndex < 0 || heightIndex < 0) return null
  return rows.map((row) => {
    const east = row[eastIndex]
    const north = row[northIndex]
    const height = row[heightIndex]
    if (!isFiniteNumber(east) || !isFiniteNumber(north) || !isFiniteNumber(height)) return null
    return { east, north, height }
  })
}

/** The 12 edges of the ground box as LineSegments vertices. */
function boxEdges(frame: PathFrame): Float32Array {
  const x0 = frame.cx - frame.half
  const x1 = frame.cx + frame.half
  const y0 = frame.cy - frame.half
  const y1 = frame.cy + frame.half
  const corners = [
    [x0, y0, 0], [x1, y0, 0], [x1, y1, 0], [x0, y1, 0],
    [x0, y0, frame.top], [x1, y0, frame.top], [x1, y1, frame.top], [x0, y1, frame.top],
  ]
  const edges = [0, 1, 1, 2, 2, 3, 3, 0, 4, 5, 5, 6, 6, 7, 7, 4, 0, 4, 1, 5, 2, 6, 3, 7]
  const positions = new Float32Array(edges.length * 3)
  edges.forEach((corner, index) => positions.set(corners[corner], index * 3))
  return positions
}

/**
 * The 3D View tab: the accumulated log's flight path (east, north, height above ground), start green, end red,
 * inside a ground box that starts at 50 m either side of the start and 50 m high and grows to fit the path. The
 * scene is built once; row updates only move geometry, the orbit angle survives them, and the renderer is disposed
 * with the view. It is mounted lazily on first show and stays mounted (hidden) so the angle survives tab switches.
 */
function FlightPath3D({ header, rows, visible }: { header: string[] | null; rows: LogRows; visible: boolean }): ReactElement {
  const hostRef = useRef<HTMLDivElement | null>(null)
  const handleRef = useRef<FlightPathHandle | null>(null)
  const [notice, setNotice] = useState('')

  useEffect(() => {
    const host = hostRef.current
    if (host === null) return undefined
    let renderer: THREE.WebGLRenderer
    try {
      renderer = new THREE.WebGLRenderer({ antialias: true })
    } catch {
      setNotice('WebGL is unavailable in this browser.')
      return undefined
    }
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2))
    // The ground clips whatever dives below it, as the Tk view's zlim bottom of 0 did (sim_view.py:500).
    renderer.clippingPlanes = [new THREE.Plane(new THREE.Vector3(0, 0, 1), 0)]
    host.appendChild(renderer.domElement)

    const scene = new THREE.Scene()
    scene.background = new THREE.Color(PATH_BACKGROUND)
    const camera = new THREE.PerspectiveCamera(45, 1, 0.1, 1e6)
    camera.up.set(0, 0, 1)
    const controls = new OrbitControls(camera, renderer.domElement)
    controls.enableDamping = true
    controls.dampingFactor = 0.12

    const boxGeometry = new THREE.BufferGeometry()
    const boxMaterial = new THREE.LineBasicMaterial({ color: PATH_BOX_COLOR })
    const box = new THREE.LineSegments(boxGeometry, boxMaterial)
    box.frustumCulled = false
    const pathGeometry = new THREE.BufferGeometry()
    const pathMaterial = new THREE.LineBasicMaterial({ color: PATH_LINE_COLOR })
    const path = new THREE.LineSegments(pathGeometry, pathMaterial)
    path.frustumCulled = false
    const markerGeometry = new THREE.SphereGeometry(1, 16, 12)
    const startMaterial = new THREE.MeshBasicMaterial({ color: PATH_START_COLOR })
    const endMaterial = new THREE.MeshBasicMaterial({ color: PATH_END_COLOR })
    const startMarker = new THREE.Mesh(markerGeometry, startMaterial)
    const endMarker = new THREE.Mesh(markerGeometry, endMaterial)
    startMarker.visible = false
    endMarker.visible = false
    scene.add(box, path, startMarker, endMarker)

    let dirty = true
    let frame: PathFrame | null = null
    let interacting = false
    let pendingFit: PathFrame | null = null

    /** Keep the current orbit direction, re-centre on the box and back off to fit it. */
    const fit = (target: PathFrame): void => {
      const centre = new THREE.Vector3(target.cx, target.cy, target.top / 2)
      const radius = Math.sqrt(2 * target.half * target.half + (target.top / 2) * (target.top / 2))
      const rect = host.getBoundingClientRect()
      const aspect = rect.width > 0 && rect.height > 0 ? rect.width / rect.height : camera.aspect
      const verticalFov = THREE.MathUtils.degToRad(camera.fov)
      const horizontalFov = 2 * Math.atan(Math.tan(verticalFov / 2) * Math.max(aspect, 1e-4))
      const distance = (radius / Math.sin(Math.min(verticalFov, horizontalFov) / 2)) * PATH_FIT_MARGIN
      const direction = camera.position.clone().sub(controls.target)
      if (direction.lengthSq() < 1e-12) direction.copy(PATH_DIRECTION)
      direction.normalize()
      camera.position.copy(centre).addScaledVector(direction, distance)
      controls.target.copy(centre)
      camera.near = Math.max(distance / 1000, 0.01)
      camera.far = Math.max(distance + radius * 6, 1000)
      camera.updateProjectionMatrix()
      controls.update()
      dirty = true
    }

    const update = (samples: (PathPoint | null)[] | null): void => {
      const positions: number[] = []
      let previous: PathPoint | null = null
      let first: PathPoint | null = null
      let last: PathPoint | null = null
      let half = PATH_MIN_HALF_M
      let top = PATH_MIN_HEIGHT_M
      let cx = 0
      let cy = 0
      for (const sample of samples ?? []) {
        if (sample === null) {
          previous = null
          continue
        }
        if (first === null) {
          first = sample
          cx = sample.east
          cy = sample.north
        }
        last = sample
        if (previous !== null) {
          positions.push(previous.east, previous.north, previous.height, sample.east, sample.north, sample.height)
        }
        half = Math.max(half, Math.abs(sample.east - cx), Math.abs(sample.north - cy))
        top = Math.max(top, sample.height)
        previous = sample
      }
      pathGeometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(positions), 3))
      pathGeometry.setDrawRange(0, positions.length / 3)
      if (first !== null && last !== null) {
        startMarker.visible = true
        endMarker.visible = true
        startMarker.position.set(first.east, first.north, first.height)
        endMarker.position.set(last.east, last.north, last.height)
      } else {
        startMarker.visible = false
        endMarker.visible = false
      }
      const next: PathFrame = { cx, cy, half, top }
      if (frame === null || frame.cx !== next.cx || frame.cy !== next.cy || frame.half !== next.half || frame.top !== next.top) {
        frame = next
        boxGeometry.setAttribute('position', new THREE.BufferAttribute(boxEdges(next), 3))
        const markerScale = Math.max(half * 0.02, 1)
        startMarker.scale.setScalar(markerScale)
        endMarker.scale.setScalar(markerScale)
        if (interacting) pendingFit = next // refit when the drag ends, so a live update does not fight the user
        else fit(next)
      }
      dirty = true
    }

    const resize = (): void => {
      const rect = host.getBoundingClientRect()
      if (rect.width < 1 || rect.height < 1) return
      renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2))
      renderer.setSize(Math.floor(rect.width), Math.floor(rect.height))
      camera.aspect = rect.width / rect.height
      camera.updateProjectionMatrix()
      dirty = true
    }
    const observer = new ResizeObserver(() => resize())
    observer.observe(host)
    resize()

    const onStart = (): void => {
      interacting = true
    }
    const onEnd = (): void => {
      interacting = false
      if (pendingFit !== null) {
        const target = pendingFit
        pendingFit = null
        fit(target)
      }
    }
    controls.addEventListener('start', onStart)
    controls.addEventListener('end', onEnd)

    update(null) // the minimum box, also before any data

    renderer.setAnimationLoop(() => {
      const moved = controls.update()
      if (moved || dirty) {
        dirty = false
        renderer.render(scene, camera)
      }
    })

    handleRef.current = { update }

    return () => {
      handleRef.current = null
      observer.disconnect()
      renderer.setAnimationLoop(null)
      controls.removeEventListener('start', onStart)
      controls.removeEventListener('end', onEnd)
      controls.dispose()
      boxGeometry.dispose()
      boxMaterial.dispose()
      pathGeometry.dispose()
      pathMaterial.dispose()
      markerGeometry.dispose()
      startMaterial.dispose()
      endMaterial.dispose()
      renderer.dispose()
      renderer.domElement.remove()
    }
  }, [])

  // Live rows move the geometry in place; a run reset restarts the path in the same scene.
  useEffect(() => {
    handleRef.current?.update(header === null ? null : pathSamples(header, rows))
  }, [header, rows])

  return (
    <div className="sim-path" style={{ display: visible ? 'flex' : 'none' }} ref={hostRef}>
      {notice !== '' && <div className="model-notice">{notice}</div>}
    </div>
  )
}

/**
 * The web SIM view (webapp/sim.py routes; user-visible behavior in docs/ui/sim_view.md, ported from the retired Tk
 * SimMainView). The server owns the one run; this view polls /api/sim/state for the status, the summary lines and
 * the flight-log tail, and POSTs the run/abort requests directly (simApi bypasses the store's mutation queue).
 * The Plots tab draws the log tail with uPlot, the 3D View tab draws the flight path with three.js.
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
  const [logHeader, setLogHeader] = useState<string[] | null>(null)
  const [logRows, setLogRows] = useState<LogRows>([])
  const [xColumn, setXColumn] = useState(DEFAULT_X)
  const [yColumns, setYColumns] = useState<readonly string[]>(DEFAULT_Y)
  const [xZoom, setXZoom] = useState<XZoom>(null)
  const [tab, setTab] = useState<'plots' | '3d'>('plots')
  const [pathSeen, setPathSeen] = useState(false)

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
      setStateSeen(true)
      setRunning(payload.running)
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
      setController((current) => {
        const name = scriptName(current)
        return name === null || next.controllers.includes(name) ? current : INAV
      })
      setScenario((current) => (next.scenarios.includes(current) ? current : (next.scenarios[0] ?? '')))
      setInavConfig((current) =>
        current === GENERATED || next.inav_configs.some((config) => config.file === current) ? current : GENERATED,
      )
      setDuration((current) => (current === '' ? String(next.default_duration_s) : current))
      if (!runningRef.current && next.scenarios.length === 0) setStatus('No valid airframe')
    })
    return () => {
      alive = false
    }
  }, [project])

  // The starts follow the scenario (hover is ground only) and a script flies any start; the Tk refresh_start.
  useEffect(() => {
    if (options === null) return
    const starts = startsFor(options, controller, scenario)
    setStart((current) => (starts.includes(current) ? current : (starts[0] ?? '')))
  }, [options, controller, scenario])

  // The 3D scene is initialized lazily on its first show and then stays mounted (hidden) so the orbit angle
  // survives tab switches; the plots live only while their tab is visible.
  useEffect(() => {
    if (tab === '3d') setPathSeen(true)
  }, [tab])

  // While running, keep the summary pinned to its end; an idle summary stays where the user left it.
  useEffect(() => {
    if (!running) return
    const node = summaryRef.current
    if (node !== null) node.scrollTop = node.scrollHeight
  }, [summary, running])

  const script = scriptName(controller)
  const noAirframe = options !== null && options.scenarios.length === 0
  const aborting = status === 'Aborting...'
  const displayStatus = !running && status === 'Idle' && noAirframe ? 'No valid airframe' : status

  const setCondition = (key: string, value: string): void => {
    setConditions((current) => ({ ...current, [key]: value }))
  }

  const toggleY = (name: string): void => {
    setYColumns((current) => (current.includes(name) ? current.filter((item) => item !== name) : [...current, name]))
  }

  const run = async (): Promise<void> => {
    if (options === null || running || starting) return
    setStarting(true)
    try {
      await saveProject() // flush the queued edits; what flies is the file on disk
      const body: SimStart = { controller: script === null ? 'inav' : 'script', start }
      if (script !== null) {
        body.script = script
        body.duration = duration
      } else {
        body.scenario = scenario
        body.inav_config = inavConfig
      }
      const picked: Record<string, string> = {}
      for (const option of options.start_options) {
        const value = (conditions[option.key] ?? '').trim()
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

  const starts = options === null ? [] : startsFor(options, controller, scenario)

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
              disabled={options === null}
              onChange={(event) => setController(event.target.value)}
            >
              <option value={INAV}>INAV SITL</option>
              {(options?.controllers ?? []).map((name) => (
                <option key={name} value={`${SCRIPT_PREFIX}${name}`}>
                  {`${SCRIPT_PREFIX} ${name}`}
                </option>
              ))}
            </select>
          </div>
          <div className="sim-field">
            <label htmlFor="sim-scenario">Scenario</label>
            <select
              id="sim-scenario"
              className="field-input"
              value={scenario}
              disabled={options === null || script !== null}
              onChange={(event) => setScenario(event.target.value)}
            >
              {(options?.scenarios ?? []).map((name) => (
                <option key={name} value={name}>
                  {name}
                </option>
              ))}
            </select>
          </div>
          <div className="sim-field">
            <label htmlFor="sim-start">Start</label>
            <select
              id="sim-start"
              className="field-input"
              value={start}
              disabled={options === null}
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
              disabled={options === null || script !== null}
              onChange={(event) => setInavConfig(event.target.value)}
            >
              <option value={GENERATED}>generated from the design</option>
              {(options?.inav_configs ?? []).map((config) => (
                <option key={config.file} value={config.file}>
                  {config.label}
                </option>
              ))}
            </select>
          </div>
          <div className="sim-field">
            <label htmlFor="sim-duration">Duration (s)</label>
            <input
              id="sim-duration"
              className="field-input"
              value={duration}
              disabled={script === null}
              spellCheck={false}
              onChange={(event) => setDuration(event.target.value)}
            />
          </div>
          <fieldset className="sim-conditions">
            <legend>Start conditions (blank = default)</legend>
            {(options?.start_options ?? []).map((option) => (
              <div key={option.key} className="sim-condition">
                <label htmlFor={`sim-condition-${option.key}`}>{option.label}</label>
                <input
                  id={`sim-condition-${option.key}`}
                  className="field-input"
                  value={conditions[option.key] ?? ''}
                  placeholder={options === null ? '' : launcherPlaceholder(options, start, option.key)}
                  disabled={conditionOff(option, start)}
                  spellCheck={false}
                  onChange={(event) => setCondition(option.key, event.target.value)}
                />
              </div>
            ))}
          </fieldset>
          <div className="sim-buttons">
            <button
              type="button"
              className="action"
              disabled={options === null || !stateSeen || running || starting || noAirframe}
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
            <h3>Summary</h3>
          </div>
          <pre className="sim-summary" ref={summaryRef}>
            {summary.join('\n')}
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
        </div>
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
        {pathSeen ? <FlightPath3D header={logHeader} rows={logRows} visible={tab === '3d'} /> : null}
      </section>

      <section className="panel right-pane sim-right">
        {logHeader === null ? (
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

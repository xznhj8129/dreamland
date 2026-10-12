import { useCallback, useEffect, useMemo, useRef, useState } from 'react'
import type { ChangeEvent, MouseEvent as ReactMouseEvent, ReactElement } from 'react'
import uPlot from 'uplot'
import 'uplot/dist/uPlot.min.css'
import { useSnapshot } from '../store'
import { aeroLibrary, analyzeAero, importAeroDat } from '../aeroApi'
import type { AeroAirfoil, AeroAirfoilResult, AeroAnalysis, AeroDefaults, AeroLibrary, AeroValue } from '../aeroApi'
import { SplitPane } from '../components/SplitPane'
import type { Snapshot } from '../types'

/**
 * The web Aero Analysis view (webapp/airfoils.py routes; docs/ui/aero_analysis_view.md), ported from the retired
 * Tk AeroAnalysisMainView (libs/views/aero_analysis_view.py, git 4f52bc5^). Left: the Tk parameters form, the
 * Available/Selected airfoil lists and the summary report; right: a 2x2 uPlot grid of the NeuralFoil sweep.
 * The view is created when its tab is shown (lazy) and unmounts when it is left; on mount and on project switch
 * it reseeds the selection from the airframe's surface airfoils and auto-recomputes, as the Tk constructor did.
 */

// The Tk view's fixed constants (libs/airfoil_analysis.py: DEFAULT_AIRFOIL_OPERATING_POINT,
// DEFAULT_NEURALFOIL_SETTINGS, SUMMARY_CONFIDENCE_MIN) and the Tk plot colors.
const DEFAULT_ALPHA_MIN = -20
const DEFAULT_ALPHA_MAX = 20
const SLIDER_STEP = 0.5
const RECOMPUTE_DEBOUNCE_MS = 150
const CONFIDENCE_MIN = 0.5
const LOW_ALPHA = 0.3
const AXIS_COLOR = '#67ff67'
const GRID_COLOR = 'rgba(10, 95, 10, 0.55)'
// The house series palette (SimView.tsx SERIES_COLORS): one color per airfoil.
const SERIES_COLORS = ['#19ff19', '#33ccff', '#ffcc33', '#ff6666', '#cc66ff', '#ffffff']
// Helper series (low-confidence dashes, operating-point markers) are drawn but must not add legend entries.
const HELPER_CLASS = 'aero-helper'
const PLOT_FALLBACK_WIDTH = 480
const PLOT_FALLBACK_HEIGHT = 300

interface AeroParams {
  alpha_deg: number
  airspeed_m_s: number
  altitude_m: number
  temperature_c: number
  chord_mm: number
  n_crit: number
  xtr_upper: number
  xtr_lower: number
}

type AeroField = keyof AeroParams
type AeroTexts = Record<AeroField, string>

const PARAM_FIELDS: readonly { key: Exclude<AeroField, 'alpha_deg'>; label: string }[] = [
  { key: 'airspeed_m_s', label: 'Airspeed (m/s)' },
  { key: 'altitude_m', label: 'Altitude (m)' },
  { key: 'temperature_c', label: 'Temperature (C)' },
  { key: 'chord_mm', label: 'Reference chord (mm)' },
  { key: 'n_crit', label: 'N-Crit' },
  { key: 'xtr_upper', label: 'Xtr Upper' },
  { key: 'xtr_lower', label: 'Xtr Lower' },
]

interface FlowReadout {
  reynolds: number | null
  mach: number | null
  minDeg: number | null
  maxDeg: number | null
}

interface AeroPlotSpec {
  id: 'drag_polar' | 'lift_curve' | 'efficiency_curve' | 'moment_curve'
  title: string
  xLabel: string
  yLabel: string
  /** The sweep series the plot's X values come from; "cl" is the drag polar's inverted axis. */
  xKey: 'alpha_deg' | 'cl'
  yKey: 'cd' | 'cl' | 'cl_cd' | 'cm'
}

// The Tk 2x2 grid (aero_analysis_view.py 417-447): titles, axis labels and series exact.
const AERO_PLOTS: readonly AeroPlotSpec[] = [
  { id: 'drag_polar', title: 'Cd vs Cl', xLabel: 'Cl', yLabel: 'Cd', xKey: 'cl', yKey: 'cd' },
  { id: 'lift_curve', title: 'Cl vs Alpha', xLabel: 'Alpha (deg)', yLabel: 'Cl', xKey: 'alpha_deg', yKey: 'cl' },
  { id: 'efficiency_curve', title: 'Cl/Cd vs Alpha', xLabel: 'Alpha (deg)', yLabel: 'Cl/Cd', xKey: 'alpha_deg', yKey: 'cl_cd' },
  { id: 'moment_curve', title: 'Cm vs Alpha', xLabel: 'Alpha (deg)', yLabel: 'Cm', xKey: 'alpha_deg', yKey: 'cm' },
]

const EMPTY_RESULTS: readonly AeroAirfoilResult[] = []

/** A value to plot, or null when the server sent null (a non-finite NeuralFoil output). */
function finite(value: AeroValue | undefined): number | null {
  return typeof value === 'number' && Number.isFinite(value) ? value : null
}

/** The Tk `f"{value:g}"` display of a committed number. */
function formatNumber(value: number): string {
  if (Number.isInteger(value)) return String(value)
  return String(Number.parseFloat(value.toPrecision(12)))
}

function readoutValue(value: number | null, digits: number): string {
  return value === null ? 'n/a' : value.toFixed(digits)
}

/** The airfoil names the current airframe's surfaces use, in surface order, without duplicates. */
function surfaceAirfoilNames(snapshot: Snapshot | null): string[] {
  const surfaces = snapshot?.definition.surfaces
  if (!Array.isArray(surfaces)) return []
  const names: string[] = []
  for (const entry of surfaces) {
    if (typeof entry !== 'object' || entry === null) continue
    const name = (entry as { airfoil?: unknown }).airfoil
    if (typeof name === 'string' && name !== '' && !names.includes(name)) names.push(name)
  }
  return names
}

/** The sweep extent of the last response, for the flow readout. */
function sweepRange(result: AeroAnalysis): { min: number; max: number } | null {
  const alphas = result.airfoils[0]?.sweep.alpha_deg ?? []
  let min = Number.POSITIVE_INFINITY
  let max = Number.NEGATIVE_INFINITY
  for (const value of alphas) {
    const number = finite(value)
    if (number === null) continue
    if (number < min) min = number
    if (number > max) max = number
  }
  return min <= max ? { min, max } : null
}

function paramsFromDefaults(defaults: AeroDefaults): AeroParams {
  return {
    alpha_deg: defaults.alpha_deg,
    airspeed_m_s: defaults.airspeed_m_s,
    altitude_m: defaults.altitude_m,
    temperature_c: defaults.temperature_c,
    chord_mm: defaults.chord_mm,
    n_crit: defaults.n_crit,
    xtr_upper: defaults.xtr_upper,
    xtr_lower: defaults.xtr_lower,
  }
}

function textsFromParams(params: AeroParams): AeroTexts {
  return {
    alpha_deg: formatNumber(params.alpha_deg),
    airspeed_m_s: formatNumber(params.airspeed_m_s),
    altitude_m: formatNumber(params.altitude_m),
    temperature_c: formatNumber(params.temperature_c),
    chord_mm: formatNumber(params.chord_mm),
    n_crit: formatNumber(params.n_crit),
    xtr_upper: formatNumber(params.xtr_upper),
    xtr_lower: formatNumber(params.xtr_lower),
  }
}

function isConfident(value: AeroValue): boolean {
  return typeof value === 'number' && Number.isFinite(value) && value >= CONFIDENCE_MIN
}

/** The Tk low mask widening (aero_analysis_view.py 406-415): low stretches grow one point each side; the
 *  endpoints look inward, not across the array, so a low first point cannot wrap to the last. */
function widenLowMask(low: readonly boolean[]): boolean[] {
  const count = low.length
  if (count === 0) return []
  return low.map((_, index) => {
    if (index === 0) return count > 1 ? low[0] || low[1] : low[0]
    if (index === count - 1) return low[count - 1] || low[count - 2]
    return low[index - 1] || low[index] || low[index + 1]
  })
}

interface AeroSeries {
  label: string
  color: string
}

interface AeroPlotData {
  /** One unaligned [x, high, low, marker] table per airfoil; aligned() outer-joins them. The drag polar's tables
   *  carry only the operating-point markers: its sweep-order curves are drawn by polarPlugin instead. */
  tables: uPlot.AlignedData[]
  series: AeroSeries[]
  /** Present only for the drag polar (its x is each airfoil's own Cl, which uPlot.join would sort and collapse). */
  polar: { curves: PolarCurve[]; bounds: PolarBounds | null } | null
}

/** One drag-polar curve: the sweep series in sweep order plus the masks the solid and dashed strokes select. */
interface PolarCurve {
  color: string
  cl: AeroValue[]
  cd: AeroValue[]
  /** Confidence >= 0.5 at this sweep point (the solid stroke's mask). */
  high: boolean[]
  /** The widened low-confidence mask (the dashed stroke's mask). */
  low: boolean[]
}

/** One straight canvas segment between consecutive sweep points; `low` strokes it dashed. */
interface PolarSegment {
  from: number
  to: number
  low: boolean
}

/** The drag polar's scale bounds from every finite sweep point and operating-point mark. */
interface PolarBounds {
  xMin: number
  xMax: number
  yMin: number
  yMax: number
}

/**
 * One plot's per-airfoil data. The three alpha plots get the high-confidence line, the widened low-confidence
 * dashes and the operating-point marker as joined tables (their alpha x is monotonic, so uPlot.join keeps the
 * sweep order). The drag polar's x is each airfoil's own Cl, which uPlot.join would sort and collapse at equal
 * Cl: its tables carry only the operating-point markers, the sweep-order curve goes to `polar` and is stroked
 * by the polar plugin (drawPolarCurves), and its scale bounds come from that same raw data.
 */
export function buildPlotData(spec: AeroPlotSpec, results: readonly AeroAirfoilResult[], alphaDeg: number | null): AeroPlotData {
  const polar = spec.id === 'drag_polar'
  const tables: uPlot.AlignedData[] = []
  const series: AeroSeries[] = []
  const curves: PolarCurve[] = []
  results.forEach((result, index) => {
    const color = SERIES_COLORS[index % SERIES_COLORS.length]
    if (polar) {
      const markerX = finite(result.operating_point.cl)
      const markerY = finite(result.operating_point.cd)
      tables.push(markerX !== null && markerY !== null ? [[markerX], [null], [null], [markerY]] : [[], [], [], []])
      curves.push({
        color,
        cl: result.sweep.cl,
        cd: result.sweep.cd,
        high: result.sweep.confidence.map((value) => isConfident(value)),
        low: widenLowMask(result.sweep.confidence.map((value) => !isConfident(value))),
      })
    } else {
      const lowMask = result.sweep.confidence.map((value) => !isConfident(value))
      const drawnLow = widenLowMask(lowMask)
      const xSeries = result.sweep[spec.xKey]
      const ySeries = result.sweep[spec.yKey]
      const x: number[] = []
      const high: (number | null | undefined)[] = []
      const low: (number | null | undefined)[] = []
      const marker: (number | null | undefined)[] = []
      for (let point = 0; point < xSeries.length; point++) {
        const xValue = finite(xSeries[point])
        if (xValue === null) continue
        x.push(xValue)
        const yValue = finite(ySeries[point])
        high.push(lowMask[point] ? null : yValue)
        low.push(drawnLow[point] ? yValue : null)
        marker.push(null)
      }
      const markerX = spec.xKey === 'cl' ? finite(result.operating_point.cl) : alphaDeg
      const markerY = finite(result.operating_point[spec.yKey])
      if (markerX !== null && markerY !== null) {
        const existing = x.indexOf(markerX)
        if (existing >= 0) {
          marker[existing] = markerY
        } else {
          // The line columns have no value (undefined, not null) at the marker's own row: an undefined
          // alignment artifact keeps their lines joined, a null would clip the line's tail.
          x.push(markerX)
          high.push(undefined)
          low.push(undefined)
          marker.push(markerY)
        }
      }
      tables.push([x, high, low, marker])
    }
    series.push({ label: result.name, color })
  })
  return { tables, series, polar: polar ? { curves, bounds: polarBounds(results) } : null }
}

/**
 * The plot's uPlot.AlignedData. The airfoils' X values differ (the drag polar's x is each airfoil's own Cl), so
 * the tables are outer-joined: another airfoil's X positions become undefined alignment artifacts, which the
 * default Retain mode leaves undefined and the line path connects across; the explicit nulls that mark
 * confidence gaps are retained, so those stretches stay separate.
 */
function aligned(data: AeroPlotData): uPlot.AlignedData {
  if (data.tables.length === 0) return [[]]
  return uPlot.join(data.tables)
}

/** The drag polar's sweep-order segments: solid between consecutive confident points, dashed between consecutive
 *  points of the widened low mask. Both endpoints must be drawable; a non-finite Cl or Cd breaks the path. */
function polarSegments(curve: PolarCurve): PolarSegment[] {
  const segments: PolarSegment[] = []
  let previousHigh = false
  let previousLow = false
  for (let point = 0; point < curve.cl.length; point++) {
    const drawable = finite(curve.cl[point]) !== null && finite(curve.cd[point]) !== null
    const high = drawable && curve.high[point]
    const low = drawable && curve.low[point]
    if (high && previousHigh) segments.push({ from: point - 1, to: point, low: false })
    if (low && previousLow) segments.push({ from: point - 1, to: point, low: true })
    previousHigh = high
    previousLow = low
  }
  return segments
}

/** The polar's scale bounds: every finite sweep point and operating-point mark across the selected airfoils. */
function polarBounds(results: readonly AeroAirfoilResult[]): PolarBounds | null {
  const cl: number[] = []
  const cd: number[] = []
  for (const result of results) {
    for (let point = 0; point < result.sweep.cl.length; point++) {
      const x = finite(result.sweep.cl[point])
      const y = finite(result.sweep.cd[point])
      if (x !== null && y !== null) {
        cl.push(x)
        cd.push(y)
      }
    }
    const markerX = finite(result.operating_point.cl)
    const markerY = finite(result.operating_point.cd)
    if (markerX !== null && markerY !== null) {
      cl.push(markerX)
      cd.push(markerY)
    }
  }
  if (cl.length === 0) return null
  return { xMin: Math.min(...cl), xMax: Math.max(...cl), yMin: Math.min(...cd), yMax: Math.max(...cd) }
}

/**
 * Stroke one drag polar onto uPlot's canvas in sweep order (the Tk contract: aero_analysis_view.py 437
 * `axis.plot(sweep["cl"], sweep["cd"])`). uPlot.join sorts and collapses the polar's x, folding the post-stall
 * branch and dropping equal-Cl points, so the curve bypasses the joined data and is stroked here via
 * u.valToPos: solid where confidence >= 0.5, the widened low mask dashed at LOW_ALPHA in the same color.
 */
export function drawPolarCurves(u: uPlot, curves: readonly PolarCurve[]): void {
  const ctx = u.ctx
  ctx.save()
  ctx.beginPath()
  ctx.rect(u.bbox.left, u.bbox.top, u.bbox.width, u.bbox.height)
  ctx.clip()
  for (const curve of curves) {
    const segments = polarSegments(curve)
    for (const low of [false, true]) {
      // The Tk draw order: solid first, the faded dashes over it. One path per airfoil per style.
      const selected = segments.filter((segment) => segment.low === low)
      if (selected.length === 0) continue
      ctx.beginPath()
      ctx.strokeStyle = curve.color
      ctx.lineWidth = 2 * uPlot.pxRatio
      ctx.globalAlpha = low ? LOW_ALPHA : 1
      ctx.setLineDash(low ? [4, 4] : [])
      let previous = -1
      for (const segment of selected) {
        const fromX = finite(curve.cl[segment.from])
        const fromY = finite(curve.cd[segment.from])
        const toX = finite(curve.cl[segment.to])
        const toY = finite(curve.cd[segment.to])
        if (fromX === null || fromY === null || toX === null || toY === null) continue
        if (segment.from !== previous) ctx.moveTo(u.valToPos(fromX, 'x', true), u.valToPos(fromY, 'y', true))
        ctx.lineTo(u.valToPos(toX, 'x', true), u.valToPos(toY, 'y', true))
        previous = segment.to
      }
      ctx.stroke()
    }
  }
  ctx.restore()
}

/** The drag polar's draw hook: strokes the sweep-order curves the joined data cannot carry. */
function polarPlugin(dataRef: { current: AeroPlotData }): uPlot.Plugin {
  return {
    hooks: {
      draw: (u: uPlot) => {
        const polar = dataRef.current.polar
        if (polar !== null) drawPolarCurves(u, polar.curves)
      },
    },
  }
}

/** The drag polar's autoscale: x exact (as uPlot's linear x), y padded like uPlot's snapNumY. The bounds are
 *  re-read from the current data on every setData, so the plot still fits each new analysis. */
function polarRange(axis: 'x' | 'y', dataRef: { current: AeroPlotData }): uPlot.Range.Function {
  return (_u, dataMin, dataMax) => {
    const bounds = dataRef.current.polar?.bounds ?? null
    if (bounds === null) return [dataMin ?? 0, dataMax ?? 1]
    if (axis === 'x') return [bounds.xMin, bounds.xMax]
    const span = bounds.yMax - bounds.yMin
    const pad = span > 0 ? span * 0.1 : Math.max(Math.abs(bounds.yMax), 1) * 0.1
    return [bounds.yMin - pad, bounds.yMax + pad]
  }
}

function plotOptions(width: number, height: number, spec: AeroPlotSpec, dataRef: { current: AeroPlotData }): uPlot.Options {
  const polar = spec.id === 'drag_polar'
  const series: uPlot.Series[] = [{}]
  for (const entry of dataRef.current.series) {
    series.push({ label: entry.label, stroke: entry.color, width: 2, points: { show: false } })
    series.push({
      class: HELPER_CLASS,
      stroke: entry.color,
      width: 2,
      dash: [4, 4],
      alpha: LOW_ALPHA,
      points: { show: false },
    })
    series.push({ class: HELPER_CLASS, stroke: entry.color, width: 0, points: { show: true, size: 7, fill: entry.color } })
  }
  return {
    width,
    height,
    padding: [8, 10, 0, 0],
    cursor: { show: true, points: { show: false } },
    legend: { show: true },
    scales: polar
      ? { x: { time: false, range: polarRange('x', dataRef) }, y: { range: polarRange('y', dataRef) } }
      : { x: { time: false }, y: {} },
    plugins: polar ? [polarPlugin(dataRef)] : [],
    axes: [
      { label: spec.xLabel, stroke: AXIS_COLOR, grid: { stroke: GRID_COLOR }, ticks: { stroke: GRID_COLOR } },
      { label: spec.yLabel, stroke: AXIS_COLOR, grid: { stroke: GRID_COLOR }, ticks: { stroke: GRID_COLOR } },
    ],
    series,
  }
}

interface NumberFieldProps {
  id: string
  label: string
  value: string
  disabled: boolean
  onEdit: (value: string) => void
  onCommit: () => void
}

/** One committed-on-Enter/blur numeric entry, with the Tk entry's revert-to-last-valid on a parse failure. */
function NumberField({ id, label, value, disabled, onEdit, onCommit }: NumberFieldProps): ReactElement {
  return (
    <div className="form-row">
      <label className="form-label" htmlFor={id}>
        {label}
      </label>
      <input
        id={id}
        className="field-input"
        value={value}
        disabled={disabled}
        spellCheck={false}
        onChange={(event) => onEdit(event.target.value)}
        onKeyDown={(event) => {
          if (event.key === 'Enter') {
            event.preventDefault()
            onCommit()
          }
        }}
        onBlur={onCommit}
      />
    </div>
  )
}

/** One of the four charts; series are rebuilt only when the airfoil set changes, data updates go in place. */
function AeroPlot({ spec, data, seriesKey }: { spec: AeroPlotSpec; data: AeroPlotData; seriesKey: string }): ReactElement {
  const hostRef = useRef<HTMLDivElement | null>(null)
  const plotRef = useRef<uPlot | null>(null)
  const dataRef = useRef(data)
  dataRef.current = data

  useEffect(() => {
    const host = hostRef.current
    if (host === null) return undefined
    const width = host.clientWidth > 0 ? host.clientWidth : PLOT_FALLBACK_WIDTH
    const height = host.clientHeight > 0 ? host.clientHeight : PLOT_FALLBACK_HEIGHT
    const plot = new uPlot(plotOptions(width, height, spec, dataRef), aligned(dataRef.current), host)
    plotRef.current = plot
    const observer = new ResizeObserver(() => {
      const nextWidth = host.clientWidth
      const nextHeight = host.clientHeight
      if (nextWidth > 0 && nextHeight > 0 && (nextWidth !== plot.width || nextHeight !== plot.height)) {
        plot.setSize({ width: nextWidth, height: nextHeight })
      }
    })
    observer.observe(host)
    return () => {
      observer.disconnect()
      plot.destroy()
      plotRef.current = null
    }
  }, [spec, seriesKey])

  useEffect(() => {
    plotRef.current?.setData(aligned(data))
  }, [data])

  return (
    <div className="aero-plot">
      <div className="aero-plot-title">{spec.title}</div>
      <div className="aero-plot-host" ref={hostRef} />
    </div>
  )
}

function AeroPlots({ analysis, alphaDeg }: { analysis: AeroAnalysis | null; alphaDeg: number | null }): ReactElement {
  const results = analysis?.airfoils ?? EMPTY_RESULTS
  const seriesKey = results.map((result) => result.name).join('\u0000')
  const data = useMemo(() => AERO_PLOTS.map((spec) => buildPlotData(spec, results, alphaDeg)), [results, alphaDeg])
  return (
    <div className="aero-plot-grid">
      {AERO_PLOTS.map((spec, index) => (
        <AeroPlot key={spec.id} spec={spec} data={data[index]} seriesKey={seriesKey} />
      ))}
    </div>
  )
}

function pickedValues(select: HTMLSelectElement): string[] {
  return Array.from(select.selectedOptions, (option) => option.value)
}

export function AeroAnalysisView(): ReactElement {
  const snapshot = useSnapshot()
  const project = snapshot?.project ?? null

  const [library, setLibrary] = useState<AeroLibrary | null>(null)
  const [available, setAvailable] = useState<AeroAirfoil[]>([])
  const [selected, setSelectedState] = useState<string[]>([])
  const [params, setParamsState] = useState<AeroParams | null>(null)
  const [texts, setTextsState] = useState<AeroTexts | null>(null)
  const [modelSize, setModelSizeState] = useState('')
  const [include360, setInclude360State] = useState(false)
  const [analysis, setAnalysis] = useState<AeroAnalysis | null>(null)
  const [analyzedAlpha, setAnalyzedAlpha] = useState<number | null>(null)
  const [summary, setSummary] = useState('')
  const [readout, setReadout] = useState<FlowReadout | null>(null)
  const [availablePicked, setAvailablePicked] = useState<string[]>([])
  const [selectedPicked, setSelectedPicked] = useState<string[]>([])

  const snapshotRef = useRef(snapshot)
  snapshotRef.current = snapshot
  // The refs are the source of truth for recomputes so an event handler that just changed a field or list can
  // recompute with the new value before React re-renders.
  const selectedRef = useRef<string[]>([])
  const paramsRef = useRef<AeroParams | null>(null)
  const textsRef = useRef<AeroTexts | null>(null)
  const modelSizeRef = useRef('')
  const include360Ref = useRef(false)
  const requestSeq = useRef(0)
  const debounceRef = useRef<number | null>(null)
  const fileInputRef = useRef<HTMLInputElement | null>(null)

  const setSelected = (next: string[]): void => {
    selectedRef.current = next
    setSelectedState(next)
  }
  const setParams = (next: AeroParams): void => {
    paramsRef.current = next
    setParamsState(next)
  }
  const setTexts = (next: AeroTexts): void => {
    textsRef.current = next
    setTextsState(next)
  }
  const setModelSize = (value: string): void => {
    modelSizeRef.current = value
    setModelSizeState(value)
  }
  const setInclude360 = (value: boolean): void => {
    include360Ref.current = value
    setInclude360State(value)
  }

  const runAnalysis = useCallback(
    async (selection: string[], nextParams: AeroParams, nextModelSize: string, next360: boolean): Promise<void> => {
      const seq = ++requestSeq.current
      if (selection.length === 0) {
        setAnalysis(null)
        setAnalyzedAlpha(null)
        setSummary('No airfoils selected.')
        return
      }
      const answer = await analyzeAero({
        airfoils: selection,
        alpha_deg: nextParams.alpha_deg,
        airspeed_m_s: nextParams.airspeed_m_s,
        altitude_m: nextParams.altitude_m,
        temperature_c: nextParams.temperature_c,
        chord_mm: nextParams.chord_mm,
        n_crit: nextParams.n_crit,
        xtr_upper: nextParams.xtr_upper,
        xtr_lower: nextParams.xtr_lower,
        model_size: nextModelSize,
        include_360_deg_effects: next360,
      })
      if (seq !== requestSeq.current) return
      if (!answer.ok || answer.result === null) {
        setSummary(answer.message)
        return
      }
      const result = answer.result
      setAnalysis(result)
      setAnalyzedAlpha(nextParams.alpha_deg)
      setSummary(result.summary_text)
      const range = sweepRange(result)
      setReadout((current) => ({
        reynolds: finite(result.flow.reynolds),
        mach: finite(result.flow.mach),
        minDeg: range?.min ?? current?.minDeg ?? null,
        maxDeg: range?.max ?? current?.maxDeg ?? null,
      }))
    },
    [],
  )

  const recompute = useCallback((): void => {
    if (debounceRef.current !== null) {
      window.clearTimeout(debounceRef.current)
      debounceRef.current = null
    }
    const nextParams = paramsRef.current
    if (nextParams === null) return
    void runAnalysis(selectedRef.current, nextParams, modelSizeRef.current, include360Ref.current)
  }, [runAnalysis])

  const scheduleRecompute = useCallback((): void => {
    if (debounceRef.current !== null) window.clearTimeout(debounceRef.current)
    debounceRef.current = window.setTimeout(() => {
      debounceRef.current = null
      recompute()
    }, RECOMPUTE_DEBOUNCE_MS)
  }, [recompute])

  // Lazy creation and project switches: fetch the library with the airframe's surface airfoils as `surfaces`,
  // reseed the selection/defaults from the answer and auto-recompute, as the Tk view did on reload_airfoils.
  useEffect(() => {
    if (project === null) return undefined
    let alive = true
    requestSeq.current += 1
    setLibrary(null)
    setAvailable([])
    setSelected([])
    setAvailablePicked([])
    setSelectedPicked([])
    setAnalysis(null)
    setAnalyzedAlpha(null)
    setSummary('')
    void aeroLibrary(surfaceAirfoilNames(snapshotRef.current)).then((answer) => {
      if (!alive) return
      if (!answer.ok || answer.result === null) {
        setSummary(answer.message)
        return
      }
      const next = answer.result
      const nextParams = paramsFromDefaults(next.defaults)
      setLibrary(next)
      setAvailable([...next.airfoils].sort((left, right) => (left.name < right.name ? -1 : left.name > right.name ? 1 : 0)))
      setSelected(next.selected)
      setParams(nextParams)
      setTexts(textsFromParams(nextParams))
      setModelSize(next.defaults.model_size)
      setInclude360(next.defaults.include_360_deg_effects)
      setReadout({ reynolds: null, mach: null, minDeg: next.alpha_sweep.min_deg, maxDeg: next.alpha_sweep.max_deg })
      void runAnalysis(next.selected, nextParams, next.defaults.model_size, next.defaults.include_360_deg_effects)
    })
    return () => {
      alive = false
    }
  }, [project, runAnalysis])

  // F5 recomputes whatever the active view shows (guibuilder.on_manual_recompute): this listener exists only
  // while the view is mounted, and App's Editor F5 listener only while the Editor is shown, so they never fight.
  useEffect(() => {
    const onKeyDown = (event: KeyboardEvent): void => {
      if (event.key !== 'F5') return
      event.preventDefault()
      recompute()
    }
    window.addEventListener('keydown', onKeyDown)
    return () => window.removeEventListener('keydown', onKeyDown)
  }, [recompute])

  useEffect(
    () => () => {
      if (debounceRef.current !== null) window.clearTimeout(debounceRef.current)
    },
    [],
  )

  const editField = (key: AeroField, value: string): void => {
    const current = textsRef.current
    if (current === null) return
    setTexts({ ...current, [key]: value })
  }

  const commitField = (key: AeroField): void => {
    const currentParams = paramsRef.current
    const currentTexts = textsRef.current
    if (currentParams === null || currentTexts === null) return
    const raw = currentTexts[key].trim()
    const parsed = raw === '' ? Number.NaN : Number(raw)
    if (!Number.isFinite(parsed)) {
      setTexts({ ...currentTexts, [key]: formatNumber(currentParams[key]) })
      return
    }
    setParams({ ...currentParams, [key]: parsed })
    setTexts({ ...currentTexts, [key]: formatNumber(parsed) })
    recompute()
  }

  const onAlphaScale = (value: string): void => {
    const currentParams = paramsRef.current
    if (currentParams === null) return
    const parsed = Number(value)
    if (!Number.isFinite(parsed)) return
    setParams({ ...currentParams, alpha_deg: parsed })
    const currentTexts = textsRef.current
    if (currentTexts !== null) setTexts({ ...currentTexts, alpha_deg: formatNumber(parsed) })
    scheduleRecompute()
  }

  const addNames = (names: readonly string[]): void => {
    const next = [...selectedRef.current]
    for (const name of names) if (!next.includes(name)) next.push(name)
    setSelected(next)
    recompute()
  }

  const removeNames = (names: readonly string[]): void => {
    if (names.length > 0) {
      const drop = new Set(names)
      setSelected(selectedRef.current.filter((name) => !drop.has(name)))
    }
    recompute()
  }

  const clearSelection = (): void => {
    setSelected([])
    recompute()
  }

  const onImportFile = async (event: ChangeEvent<HTMLInputElement>): Promise<void> => {
    const file = event.currentTarget.files?.[0] ?? null
    event.currentTarget.value = ''
    if (file === null) return
    let datText: string
    try {
      datText = await file.text()
    } catch {
      setSummary(`Could not read ${file.name}.`)
      return
    }
    const stem = file.name.replace(/\.[^.]+$/, '') || file.name
    const answer = await importAeroDat(stem, datText)
      if (!answer.ok || answer.result === null) {
        // A failed request blanks the plots: showing the previous parameters' curves next to an error would
        // present stale data as the current run. The error stays in the Summary.
        setAnalysis(null)
        setAnalyzedAlpha(null)
        setSummary(answer.message)
        return
      }
    const imported = answer.result
    setAvailable((current) => {
      const next = current.filter((entry) => entry.name !== imported.name)
      next.push(imported)
      next.sort((left, right) => (left.name < right.name ? -1 : left.name > right.name ? 1 : 0))
      return next
    })
    addNames([imported.name])
  }

  const sliderMin = library?.alpha_sweep.min_deg ?? DEFAULT_ALPHA_MIN
  const sliderMax = library?.alpha_sweep.max_deg ?? DEFAULT_ALPHA_MAX
  const flowText =
    readout === null
      ? ''
      : `Re = ${readoutValue(readout.reynolds, 0)} | M = ${readoutValue(readout.mach, 3)} | alpha sweep = ${readoutValue(readout.minDeg, 0)}..${readoutValue(readout.maxDeg, 0)} deg`
  const loaded = params !== null && texts !== null

  return (
    <SplitPane minSizes={[300, 360]} initialRatios={[1, 3]}>
      <section className="panel left-pane aero-left">
        <div className="form-card aero-parameters">
          <div className="form-card-header">
            <h3>Parameters</h3>
            <button type="button" className="action" disabled={!loaded} onClick={() => recompute()}>
              Recompute (F5)
            </button>
          </div>
          <p className="form-hint aero-flow" role="status">
            {flowText}
          </p>

          <div className="form-row">
            <label className="form-label" htmlFor="aero-alpha">
              AoA operating point (deg)
            </label>
            <div className="aero-alpha-row">
              <input
                type="range"
                min={sliderMin}
                max={sliderMax}
                step={SLIDER_STEP}
                value={params?.alpha_deg ?? DEFAULT_ALPHA_MIN}
                disabled={!loaded}
                aria-label="AoA operating point slider"
                onChange={(event) => onAlphaScale(event.target.value)}
              />
              <input
                id="aero-alpha"
                className="field-input"
                value={texts?.alpha_deg ?? ''}
                disabled={!loaded}
                spellCheck={false}
                onChange={(event) => editField('alpha_deg', event.target.value)}
                onKeyDown={(event) => {
                  if (event.key === 'Enter') {
                    event.preventDefault()
                    commitField('alpha_deg')
                  }
                }}
                onBlur={() => commitField('alpha_deg')}
              />
            </div>
          </div>

          {PARAM_FIELDS.map((field) => (
            <NumberField
              key={field.key}
              id={`aero-param-${field.key}`}
              label={field.label}
              value={texts?.[field.key] ?? ''}
              disabled={!loaded}
              onEdit={(value) => editField(field.key, value)}
              onCommit={() => commitField(field.key)}
            />
          ))}

          <div className="form-row">
            <label className="form-label" htmlFor="aero-model-size">
              Model size
            </label>
            <select
              id="aero-model-size"
              className="field-input"
              value={modelSize}
              disabled={library === null}
              onChange={(event) => {
                setModelSize(event.target.value)
                recompute()
              }}
            >
              {(library?.model_sizes ?? []).map((name) => (
                <option key={name} value={name}>
                  {name}
                </option>
              ))}
            </select>
          </div>

          <label className="check-row">
            <input
              type="checkbox"
              checked={include360}
              disabled={library === null}
              onChange={(event) => {
                setInclude360(event.target.checked)
                recompute()
              }}
            />
            Include 360 deg effects
          </label>
        </div>

        <div className="form-card aero-selection">
          <div className="form-card-header">
            <h3>Airfoil Selection</h3>
          </div>
          <div className="aero-selection-grid">
            <div className="aero-list-column">
              <span className="aero-list-label">Available</span>
              <select
                multiple
                size={6}
                className="field-input aero-list"
                value={availablePicked}
                disabled={library === null}
                aria-label="Available airfoils"
                onChange={(event) => setAvailablePicked(pickedValues(event.currentTarget))}
                onDoubleClick={(event: ReactMouseEvent<HTMLSelectElement>) => addNames(pickedValues(event.currentTarget))}
              >
                {available.map((entry) => (
                  <option key={entry.name} value={entry.name}>
                    {entry.name}
                  </option>
                ))}
              </select>
            </div>
            <div className="aero-selection-buttons">
              <button type="button" disabled={library === null} onClick={() => addNames(availablePicked)}>
                Add -&gt;
              </button>
              <button type="button" disabled={library === null} onClick={() => removeNames(selectedPicked)}>
                &lt;- Remove
              </button>
              <button type="button" disabled={library === null} onClick={() => fileInputRef.current?.click()}>
                Import DAT
              </button>
              <button type="button" disabled={library === null} onClick={clearSelection}>
                Clear
              </button>
            </div>
            <div className="aero-list-column">
              <span className="aero-list-label">Selected</span>
              <select
                multiple
                size={6}
                className="field-input aero-list"
                value={selectedPicked}
                disabled={library === null}
                aria-label="Selected airfoils"
                onChange={(event) => setSelectedPicked(pickedValues(event.currentTarget))}
                onDoubleClick={(event: ReactMouseEvent<HTMLSelectElement>) => removeNames(pickedValues(event.currentTarget))}
              >
                {selected.map((name) => (
                  <option key={name} value={name}>
                    {name}
                  </option>
                ))}
              </select>
            </div>
          </div>
          <input ref={fileInputRef} type="file" accept=".dat" hidden onChange={(event) => void onImportFile(event)} />
        </div>

        <div className="form-card aero-summary">
          <div className="form-card-header">
            <h3>Summary</h3>
          </div>
          <pre className="aero-summary-text">{summary}</pre>
        </div>
      </section>

      <section className="panel right-pane aero-right">
        <AeroPlots analysis={analysis} alphaDeg={analyzedAlpha} />
      </section>
    </SplitPane>
  )
}

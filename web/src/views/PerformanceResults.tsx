import type { ReactElement } from 'react'

interface Measurement {
  label: string
  t: number
  [key: string]: number | string
}
interface MeasurementResults { points: Measurement[]; error: string | null }

const METRICS = [
  { title: 'Top level speed', key: 'airspeed_m_s', unit: 'm/s', measurement: 'top speed' },
  { title: 'Slowest level flight', key: 'slowest_level_m_s', unit: 'm/s', measurement: 'performance' },
  { title: 'Best range speed', key: 'best_range_speed_m_s', unit: 'm/s', measurement: 'performance' },
  { title: 'Efficiency at best range', key: 'best_wh_km', unit: 'Wh/km', measurement: 'performance' },
  { title: 'Estimated range', key: 'range_km', unit: 'km', measurement: 'performance' },
  { title: 'Estimated endurance', key: 'endurance_min', unit: 'min', measurement: 'performance' },
  { title: 'Stall speed', key: 'stall_speed_m_s', unit: 'm/s', measurement: 'stall' },
  { title: 'Sustained turn rate', key: 'turn_rate_deg_s', unit: 'deg/s', measurement: 'sustained turn' },
  { title: 'Service ceiling', key: 'ceiling_m', unit: 'm', measurement: 'ceiling' },
] as const

function measurements(lines: string[]): MeasurementResults {
  const points: Measurement[] = []
  for (const line of lines) {
    if (!line.startsWith('Measurement: ')) continue
    let value: unknown
    try {
      value = JSON.parse(line.slice(13))
    } catch (error) {
      if (!(error instanceof SyntaxError)) throw error
      return { points, error: `Invalid measurement JSON: ${error.message}` }
    }
    if (typeof value === 'object' && value !== null && 'label' in value && typeof value.label === 'string' && 't' in value && typeof value.t === 'number') {
      points.push(value as Measurement)
    } else return { points, error: 'Measurement must contain a label and simulated time.' }
  }
  return { points, error: null }
}

export function PerformanceResults({ lines, model }: { lines: string[]; model: string }): ReactElement {
  const { points, error } = measurements(lines)
  const levels = points.filter(point => point.label === 'level step')
  const estimate = lines.find(line => line.startsWith('Editor estimate'))
  const outside = points.find(point => point.outside_speed_grid === 1)
  const envelope = points.some(point => point.envelope_fallback === 1)
  const performance = points.filter(point => point.label === 'performance').at(-1)
  return <div className="performance-results">
    {error && <p className="draw-error">{error}</p>}
    <div className="form-card">
      <div className="form-card-header"><h3>Flight performance predictions</h3></div>
      <p className="form-hint">Results from this run, at its altitude, voltage, wind and controller settings. Choose efficiency for a speed sweep or top-speed for maximum level speed.</p>
      {outside && <p className="draw-error">Some measurements exceeded the aero speed grid (upper limit {outside.aero_speed_limit_m_s} m/s). Coefficients are clamped outside it; those points are extrapolations requiring model validation.</p>}
      {envelope && <p className="draw-error">Some speed-sweep points use the high-angle envelope fallback. Treat the low-speed result as an exploratory prediction; it is not a validated stall limit.</p>}
      <div className="performance-grid">{METRICS.map(metric => {
        const point = points.filter(value => value.label === metric.measurement).at(-1)
        const value = point?.[metric.key]
        return <div className="performance-metric" key={metric.key}><span>{metric.title}</span><strong>{typeof value === 'number' && Number.isFinite(value) ? `${value.toFixed(1)} ${metric.unit}` : 'Not measured'}</strong></div>
      })}</div>
      <p className="form-hint">Range/endurance extrapolate measured electrical power{typeof performance?.usable_battery_fraction === 'number' ? ` using ${(performance.usable_battery_fraction * 100).toFixed(0)}% nominal battery energy` : ''}; battery depletion/sag is not yet simulated. Slowest level flight is not a stall measurement.</p>
    </div>
    <div className="form-card">
      <div className="form-card-header"><h3>Estimate versus flight response</h3></div>
      <p>Editor: static aerodynamic estimate at the chosen speed and AoA.</p>
      {estimate && <p className="form-hint">{estimate}</p>}
      <p>JSBSim: motion produced by the exported aero, mass, inertia, propulsion and controls. The stability scenario freezes actuator outputs after a pitch pulse; its plotted response is separate from the static estimate.</p>
      <p className="form-hint">{model}. These share a model; agreement is consistency, not independent validation. VLM is an inviscid wing-load cross-check, not the efficiency or stall evaluator.</p>
    </div>
    {levels.length > 0 && <div className="form-card"><h3>Speed sweep samples</h3><table><thead><tr><th>Target m/s</th><th>Actual m/s</th><th>AoA deg</th><th>Power W</th><th>Climb m/s</th></tr></thead><tbody>{levels.map((point, index) => <tr key={index}>{['target_m_s', 'airspeed_m_s', 'aoa_deg', 'power_w', 'climb_m_s'].map(key => <td key={key}>{typeof point[key] === 'number' ? point[key].toFixed(2) : '—'}</td>)}</tr>)}</tbody></table></div>}
    {points.length === 0 && <p className="form-hint">No measurements yet. Run a scenario; custom scripts can add points with measure().</p>}
  </div>
}

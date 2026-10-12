/**
 * Direct fetch helpers for the Aero Analysis view (webapp/airfoils.py routes).
 *
 * These bypass the store's mutation queue on purpose, like cadApi/simApi: a NeuralFoil analysis runs for seconds
 * and must not delay the editor's queued commands. GET /api/airfoils answers the library, the seeded selection
 * and the form defaults; the POST endpoints answer 400 with {message} on a refusal, which the view shows in the
 * Summary panel. Non-finite numbers arrive as null (JSON cannot carry NaN/Infinity).
 */

export interface AeroAirfoil {
  name: string
  /** "asb" for an AeroSandbox name, "dat_file" for an imported DAT, the library file's own source otherwise. */
  source: string
}

export interface AeroDefaults {
  alpha_deg: number
  airspeed_m_s: number
  altitude_m: number
  temperature_c: number
  chord_mm: number
  n_crit: number
  xtr_upper: number
  xtr_lower: number
  model_size: string
  include_360_deg_effects: boolean
}

/** The fixed alpha sweep the curves are computed over. */
export interface AeroAlphaSweep {
  min_deg: number
  max_deg: number
  step_deg: number
}

/** GET /api/airfoils: the library plus session imports, the seeded selection and the form defaults. */
export interface AeroLibrary {
  airfoils: AeroAirfoil[]
  selected: string[]
  defaults: AeroDefaults
  model_sizes: string[]
  alpha_sweep: AeroAlphaSweep
}

/** POST /api/airfoils/analyze body. */
export interface AeroAnalyzeRequest {
  airfoils: string[]
  alpha_deg: number
  airspeed_m_s: number
  altitude_m: number
  temperature_c: number
  chord_mm: number
  n_crit: number
  xtr_upper: number
  xtr_lower: number
  model_size: string
  include_360_deg_effects: boolean
}

/** A value that can be non-finite; such a value arrives as null. */
export type AeroValue = number | null

/** The analysis flow block (libs.airfoil_analysis.compute_flow_conditions); the view shows Re and Mach. */
export interface AeroFlow {
  reynolds: AeroValue
  mach: AeroValue
  [key: string]: unknown
}

export interface AeroOperatingPoint {
  cl: AeroValue
  cd: AeroValue
  cm: AeroValue
  cl_cd: AeroValue
  top_xtr: AeroValue
  bot_xtr: AeroValue
  confidence: AeroValue
}

/** One airfoil's alpha sweep: all series share the same length and alpha order. */
export interface AeroSweep {
  alpha_deg: AeroValue[]
  cl: AeroValue[]
  cd: AeroValue[]
  cl_cd: AeroValue[]
  cm: AeroValue[]
  confidence: AeroValue[]
}

export interface AeroAirfoilResult {
  name: string
  source: string
  operating_point: AeroOperatingPoint
  sweep: AeroSweep
  /** libs.airfoil_analysis.summarize_sweep; the text report is summary_text. */
  summary: Record<string, AeroValue>
}

/** POST /api/airfoils/analyze answer. */
export interface AeroAnalysis {
  flow: AeroFlow
  summary_text: string
  airfoils: AeroAirfoilResult[]
}

/** POST /api/airfoils/import answer: the session name after the server's "-2"/"-3" deduplication. */
export interface AeroAirfoilRef {
  name: string
  source: string
}

export interface AeroAnswer<T> {
  ok: boolean
  result: T | null
  message: string
}

function isAeroValue(value: unknown): value is AeroValue {
  return value === null || typeof value === 'number'
}

function asAeroAirfoil(value: unknown): AeroAirfoil | null {
  if (typeof value !== 'object' || value === null) return null
  const candidate = value as { name?: unknown; source?: unknown }
  if (typeof candidate.name !== 'string' || typeof candidate.source !== 'string') return null
  return { name: candidate.name, source: candidate.source }
}

function asAeroDefaults(value: unknown): AeroDefaults | null {
  if (typeof value !== 'object' || value === null) return null
  const candidate = value as Record<string, unknown>
  const numeric = ['alpha_deg', 'airspeed_m_s', 'altitude_m', 'temperature_c', 'chord_mm', 'n_crit', 'xtr_upper', 'xtr_lower'] as const
  for (const key of numeric) {
    if (typeof candidate[key] !== 'number' || !Number.isFinite(candidate[key])) return null
  }
  if (typeof candidate.model_size !== 'string' || typeof candidate.include_360_deg_effects !== 'boolean') return null
  return candidate as unknown as AeroDefaults
}

function asAeroAlphaSweep(value: unknown): AeroAlphaSweep | null {
  if (typeof value !== 'object' || value === null) return null
  const candidate = value as Record<string, unknown>
  for (const key of ['min_deg', 'max_deg', 'step_deg']) {
    if (typeof candidate[key] !== 'number' || !Number.isFinite(candidate[key])) return null
  }
  return candidate as unknown as AeroAlphaSweep
}

/** Validate enough of the library payload that every field the view reads is present and well-typed. */
function asAeroLibrary(payload: unknown): AeroLibrary | null {
  if (typeof payload !== 'object' || payload === null) return null
  const candidate = payload as Partial<AeroLibrary>
  if (!Array.isArray(candidate.airfoils)) return null
  const airfoils: AeroAirfoil[] = []
  for (const entry of candidate.airfoils) {
    const parsed = asAeroAirfoil(entry)
    if (parsed === null) return null
    airfoils.push(parsed)
  }
  if (!Array.isArray(candidate.selected) || !candidate.selected.every((name) => typeof name === 'string')) return null
  if (!Array.isArray(candidate.model_sizes) || !candidate.model_sizes.every((name) => typeof name === 'string')) return null
  const defaults = asAeroDefaults(candidate.defaults)
  const alphaSweep = asAeroAlphaSweep(candidate.alpha_sweep)
  if (defaults === null || alphaSweep === null) return null
  return { airfoils, selected: candidate.selected, defaults, model_sizes: candidate.model_sizes, alpha_sweep: alphaSweep }
}

function asAeroOperatingPoint(value: unknown): AeroOperatingPoint | null {
  if (typeof value !== 'object' || value === null) return null
  const candidate = value as Record<string, unknown>
  for (const key of ['cl', 'cd', 'cm', 'cl_cd', 'top_xtr', 'bot_xtr', 'confidence']) {
    if (!isAeroValue(candidate[key])) return null
  }
  return candidate as unknown as AeroOperatingPoint
}

function asAeroSweep(value: unknown): AeroSweep | null {
  if (typeof value !== 'object' || value === null) return null
  const candidate = value as Record<string, unknown>
  const sweep: Partial<AeroSweep> = {}
  for (const key of ['alpha_deg', 'cl', 'cd', 'cl_cd', 'cm', 'confidence'] as const) {
    const series = candidate[key]
    if (!Array.isArray(series) || !series.every(isAeroValue)) return null
    sweep[key] = series
  }
  return sweep as AeroSweep
}

function asAeroAnalysis(payload: unknown): AeroAnalysis | null {
  if (typeof payload !== 'object' || payload === null) return null
  const candidate = payload as Partial<AeroAnalysis>
  if (typeof candidate.flow !== 'object' || candidate.flow === null) return null
  const flow = candidate.flow as Record<string, unknown>
  if (!isAeroValue(flow.reynolds) || !isAeroValue(flow.mach)) return null
  if (typeof candidate.summary_text !== 'string' || !Array.isArray(candidate.airfoils)) return null
  const airfoils: AeroAirfoilResult[] = []
  for (const entry of candidate.airfoils) {
    if (typeof entry !== 'object' || entry === null) return null
    const item = entry as Partial<AeroAirfoilResult>
    const operatingPoint = asAeroOperatingPoint(item.operating_point)
    const sweep = asAeroSweep(item.sweep)
    if (typeof item.name !== 'string' || typeof item.source !== 'string') return null
    if (operatingPoint === null || sweep === null) return null
    if (typeof item.summary !== 'object' || item.summary === null) return null
    airfoils.push({ name: item.name, source: item.source, operating_point: operatingPoint, sweep, summary: item.summary as Record<string, AeroValue> })
  }
  return { flow: candidate.flow as AeroFlow, summary_text: candidate.summary_text, airfoils }
}

function asAeroAirfoilRef(payload: unknown): AeroAirfoilRef | null {
  return asAeroAirfoil(payload)
}

function refusalMessage(payload: unknown, fallback: string, status: number): string {
  if (typeof payload === 'object' && payload !== null) {
    const message = (payload as { message?: unknown }).message
    if (typeof message === 'string') return message
  }
  return `${fallback} (${status})`
}

async function post<T>(url: string, body: unknown, parse: (payload: unknown) => T | null, fallback: string): Promise<AeroAnswer<T>> {
  let response: Response
  try {
    response = await fetch(url, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body),
    })
  } catch {
    return { ok: false, result: null, message: 'Server unreachable.' }
  }
  const text = await response.text()
  let payload: unknown = null
  try {
    payload = text === '' ? null : JSON.parse(text)
  } catch {
    payload = null
  }
  if (!response.ok) return { ok: false, result: null, message: refusalMessage(payload, fallback, response.status) }
  const result = parse(payload)
  if (result === null) return { ok: false, result: null, message: `${fallback}: the server answered with no result.` }
  return { ok: true, result, message: '' }
}

/** GET /api/airfoils?surfaces=a,b,c: the library, the seeded selection and the defaults. The surfaces are the
 *  current airframe's airfoil names; the server falls back to its default selection when there are none. */
export async function aeroLibrary(surfaces: string[]): Promise<AeroAnswer<AeroLibrary>> {
  const params = new URLSearchParams({ surfaces: surfaces.join(',') })
  let response: Response
  try {
    response = await fetch(`/api/airfoils?${params.toString()}`)
  } catch {
    return { ok: false, result: null, message: 'Server unreachable.' }
  }
  const text = await response.text()
  let payload: unknown = null
  try {
    payload = text === '' ? null : JSON.parse(text)
  } catch {
    payload = null
  }
  if (!response.ok) return { ok: false, result: null, message: refusalMessage(payload, 'Could not load the airfoil library', response.status) }
  const result = asAeroLibrary(payload)
  if (result === null) return { ok: false, result: null, message: 'Could not load the airfoil library: the server answered with no result.' }
  return { ok: true, result, message: '' }
}

/** POST /api/airfoils/import {name, dat_text}: validate one Selig/Lednicer DAT and keep it for this session.
 *  A name already taken gets the Tk "-2"/"-3" suffix, so the answer's name is authoritative. */
export function importAeroDat(name: string, datText: string): Promise<AeroAnswer<AeroAirfoilRef>> {
  return post('/api/airfoils/import', { name, dat_text: datText }, asAeroAirfoilRef, 'Import failed')
}

/** POST /api/airfoils/analyze: the NeuralFoil sweep, operating point and summary for the selected airfoils. */
export function analyzeAero(request: AeroAnalyzeRequest): Promise<AeroAnswer<AeroAnalysis>> {
  return post('/api/airfoils/analyze', request, asAeroAnalysis, 'Analysis failed')
}

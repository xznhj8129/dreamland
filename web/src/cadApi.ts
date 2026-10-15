/**
 * Direct fetch helpers for the CAD view and Compute Geometry (webapp/cad.py routes).
 *
 * These bypass the store's mutation queue on purpose: an export holds the server's export lock for seconds and
 * must not delay the editor's queued commands. A refusal answers with the state snapshot whose last Log entry
 * is shown here; snapshots are not published (the websocket push carries the Log to the Log tab).
 */

export type StepMode = 'assembly' | 'fused'

export interface CadResult {
  files: string[]
  notes: string[]
  detail: string
  /** The mode the STEP writer actually produced ("assembly" | "fused" | "compound"), when it is a STEP export. */
  mode?: string
}

export interface CadAnswer {
  ok: boolean
  result: CadResult | null
  message: string
}

/** One progress message the CAD worker sent, and when it arrived (seconds since the run started). */
export interface CadStep {
  label: string
  done: number
  total: number
  at_s: number
}

/** The time left in the current phase at the pace of its steps so far (webapp.cad._estimate). */
export interface CadEstimate {
  phase: string
  left_s: number
  per_step_s: number
}

/** The current or last CAD run, owned by the server: it goes on whether or not a view is open. */
export interface CadRun {
  task: string
  options: Record<string, unknown>
  started_at: number
  steps: CadStep[]
  state: 'running' | 'done' | 'failed'
  message: string
  elapsed_s: number
  estimate: CadEstimate | null
}

/** The last result of one export, kept by the server for that export's row (message: the result line, or the step a
 *  failure happened at). */
export interface CadOutcome {
  project: string
  ok: boolean
  message: string
  files: string[]
  notes: string[]
  finished_at: number
}

/** The CAD worker's CPUs (settable up to max_cpus) and its fixed priority. */
export interface CadWorkerSettings {
  cpus: number
  max_cpus: number
  cpu_ids: number[]
  nice: number
  alive: boolean
}

/** GET /api/cad/state. */
export interface CadState {
  run: CadRun | null
  outcomes: Record<string, CadOutcome>
  worker: CadWorkerSettings
}

function lastLogMessage(payload: unknown): string | null {
  if (typeof payload !== 'object' || payload === null) return null
  const log = (payload as { log?: unknown }).log
  if (!Array.isArray(log) || log.length === 0) return null
  const last = log[log.length - 1] as { text?: unknown; summary?: unknown }
  if (typeof last.summary === 'string' && last.summary !== '') return last.summary
  return typeof last.text === 'string' ? last.text : null
}

function asCadResult(payload: unknown): CadResult | null {
  if (typeof payload !== 'object' || payload === null) return null
  const candidate = payload as Partial<CadResult>
  if (!Array.isArray(candidate.files) || !candidate.files.every((file) => typeof file === 'string')) return null
  if (!Array.isArray(candidate.notes) || !candidate.notes.every((note) => typeof note === 'string')) return null
  if (typeof candidate.detail !== 'string') return null
  return {
    files: candidate.files,
    notes: candidate.notes,
    detail: candidate.detail,
    ...(typeof candidate.mode === 'string' ? { mode: candidate.mode } : {}),
  }
}

function asCadState(payload: unknown): CadState | null {
  if (typeof payload !== 'object' || payload === null) return null
  const candidate = payload as Partial<CadState>
  if (candidate.run === undefined || typeof candidate.outcomes !== 'object' || candidate.outcomes === null) return null
  if (typeof candidate.worker !== 'object' || candidate.worker === null) return null
  if (candidate.run !== null && (typeof candidate.run.task !== 'string' || !Array.isArray(candidate.run.steps))) return null
  return candidate as CadState
}

async function post(url: string, body?: unknown): Promise<CadAnswer> {
  let response: Response
  try {
    response = await fetch(url, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body ?? {}),
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
  if (!response.ok) {
    return { ok: false, result: null, message: lastLogMessage(payload) ?? `Export failed (${response.status})` }
  }
  const result = asCadResult(payload)
  if (result === null) return { ok: false, result: null, message: 'Export answered with no result.' }
  return { ok: true, result, message: result.detail }
}

export function exportStep(mode: StepMode, includePropDiscs: boolean): Promise<CadAnswer> {
  return post('/api/cad/step', { mode, include_prop_discs: includePropDiscs })
}

export function exportStl(includePropDiscs: boolean): Promise<CadAnswer> {
  return post('/api/cad/stl', { include_prop_discs: includePropDiscs })
}

export function exportSections(format: 'step' | 'stl'): Promise<CadAnswer> {
  return post('/api/cad/sections', { format })
}

export function exportDrawings(): Promise<CadAnswer> {
  return post('/api/cad/drawings')
}

export function exportVarsets(): Promise<CadAnswer> {
  return post('/api/cad/varsets')
}

/**
 * POST /api/geometry/compute: Compute Geometry for the current design (minutes, so it bypasses the editor queue
 * like the exports). The result arrives with the state push (Snapshot.geometry and the exact mass in the report);
 * a failure is in the Log. False when the server was unreachable or refused.
 */
export async function computeGeometry(): Promise<boolean> {
  try {
    const response = await fetch('/api/geometry/compute', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: '{}' })
    return response.ok
  } catch {
    return false
  }
}

/** The current or last CAD run, each export's last outcome and the worker settings; null when unreachable or malformed. */
export async function cadState(): Promise<CadState | null> {
  try {
    const response = await fetch('/api/cad/state')
    if (!response.ok) return null
    return asCadState(await response.json())
  } catch {
    return null
  }
}

/** POST /api/cad/worker: how many CPUs the CAD worker may use, applied at once; the new state, or null on failure. */
export async function setWorkerCpus(cpus: number): Promise<CadState | null> {
  try {
    const response = await fetch('/api/cad/worker', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ cpus }),
    })
    if (!response.ok) return null
    return asCadState(await response.json())
  } catch {
    return null
  }
}

/** POST /api/cad/open: open an export's output folder in the file manager; false when the server refused. */
export async function openFolder(task: string): Promise<boolean> {
  try {
    const response = await fetch('/api/cad/open', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ task }),
    })
    return response.ok
  } catch {
    return false
  }
}

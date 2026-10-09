/**
 * Direct fetch helpers for the CAD view (webapp/cad.py routes).
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

function lastLogText(payload: unknown): string | null {
  if (typeof payload !== 'object' || payload === null) return null
  const log = (payload as { log?: unknown }).log
  if (!Array.isArray(log) || log.length === 0) return null
  const last = log[log.length - 1] as { text?: unknown }
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
    return { ok: false, result: null, message: lastLogText(payload) ?? `Export failed (${response.status})` }
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

export function exportSections(): Promise<CadAnswer> {
  return post('/api/cad/sections')
}

export function exportDrawings(): Promise<CadAnswer> {
  return post('/api/cad/drawings')
}

export function exportVarsets(): Promise<CadAnswer> {
  return post('/api/cad/varsets')
}

/** The read-only URL for a project-relative output path (GET /files/<tail>). */
export function filesUrl(relative: string): string {
  return '/files/' + relative.split('/').map(encodeURIComponent).join('/')
}

/**
 * Direct fetch helpers for the SIM view (webapp/sim.py routes).
 *
 * These bypass the store's mutation queue on purpose, like cadApi: polling /api/sim/state every 500 ms must not
 * queue behind editor mutations, and the run itself is the server's one subprocess, not a state mutation. A
 * refused start answers 400 with {ok:false, message}; the caller shows that message as the status line.
 */

/** GET /api/sim/options: the panel's option lists for the current project (webapp.sim.options). */
export interface SimOptions {
  aero_model: string
  scenario_templates: Record<string, SimResource>
  firmware: string
  /** Available command modules in controllers/ (libs.scenario.CONTROLLERS). */
  controllers: string[]
  /** This plane's SIM defaults (the project's sim.json, set with "Set as default"): the controller and scenario the
   *  panel opens a project with when the browser has no remembered setup for it. */
  defaults: { controller: string | null; scenario: string | null }
  /** Processes solving the aero tables on a model rebuild (libs.jsbsim.DEFAULT_WORKERS) and this machine's count. */
  default_workers: number
  max_workers: number
  /** The project's inav/*.txt diffs, each with the label the Tk view showed ("SITL diff: ...", "real FC diff: ..."). */
  inav_configs: { file: string; label: string }[]
  /** The scenarios the current airframe can fly; empty means no valid airframe. */
  scenarios: Record<string, string[]>
  /** The starts those scenarios allow (air, ground, catapult, rail). */
  starts: string[]
  /** One row per libs.simrun START_OPTIONS entry, in order; displayed values are sent to the runner. A condition's
   *  flags say which start it applies to: air_only, launcher_only and catapult_only. */
  start_options: { key: string; label: string; air_only: boolean; launcher_only: boolean; catapult_only: boolean }[]
  /** The defaults each launch start shows as its fields' placeholders, keyed by START_OPTIONS key (a rail has no
   *  launch-speed). */
  launcher_defaults: Record<string, Record<string, number>>
  condition_defaults: Record<string, number>
  default_duration_s: number
}

/** POST /api/sim/run body (webapp.sim._plan): what one run request carries. */
export interface SimStart {
  clock?: 'realtime' | 'stepped' | 'stepped-faster'
  rate?: string
  msp?: boolean
  mavlink?: boolean
  controller: string
  /** The scenario name in scenarios/<controller>/. */
  scenario?: string
  start: string
  /** A value per START_OPTIONS key ("wind-from" for wind_from); all applicable displayed values are sent. */
  conditions?: Record<string, string>
  /** An inav/*.txt file name, or "generated" (absent = the default, generated from the design). */
  inav_config?: string
  /** Maximum simulated seconds for any scenario. */
  duration?: string
  /** Processes solving the aero tables on a model rebuild; 1 is sequential. */
  workers?: number
}

/** One model rebuild's stage and counts (libs.jsbsim.BuildProgress): the SIM panel's model-build section. */
export interface SimBuild {
  stage: string
  detail: string
  done: number
  total: number
  workers: number
  elapsed_s: number
  /** Tasks the last solve did not have to redo (cache hits), in the order they were met. */
  cached: string[]
  finished: boolean
  failed: boolean
}

export interface SimResource {
  source: string
  readonly: boolean
  note: string
  error?: string | null
  scenario?: ScenarioOutline | null
}

export interface ScenarioOutline {
  description: string
  parameters: Record<string, unknown>
  phases: Record<string, string>
  telemetry: { fields: string[]; rate_hz: number } | null
  /** START: the scenario's default start ("start") and start conditions, keyed like SimOptions.start_options. */
  start: Record<string, string | number>
}

export async function simResource(controller: string, kind: string, name: string, source?: string, create = false): Promise<SimResource> {
  const response = source === undefined
    ? await fetch(`/api/sim/resource?${new URLSearchParams({ controller, kind, name })}`)
    : await fetch('/api/sim/resource', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({controller, kind, name, source, create}) })
  const payload = await response.json()
  if (!response.ok) throw new Error(payload.message ?? 'Could not open SIM file')
  return payload as SimResource
}

/** POST /api/sim/resource/delete: remove one of the project's SIM files. The generated diff has no deletable name. */
export async function deleteSimResource(controller: string, kind: string, name: string): Promise<void> {
  const response = await fetch('/api/sim/resource/delete', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ controller, kind, name }),
  })
  const payload = await response.json()
  if (!response.ok) throw new Error(payload.message ?? 'Could not delete SIM file')
}

/** One GET /api/sim/state flight-log tail (webapp.sim._read_log). */
export interface SimLogState {
  /** The project-relative log path, for /files/ links. */
  file: string | null
  /** True when run_id changed (or the client's first poll): the streams below start from the beginning. */
  reset: boolean
  /** Byte offset into the CSV to pass to the next poll. */
  offset: number
  /** The CSV header, only when the poll read from byte 0. */
  header: string[] | null
  /** Parsed rows; a non-finite cell (a late log() column) is null, never NaN. */
  rows: (number | null)[][]
}

/** GET /api/sim/state: the run's status, stdout lines and flight-log rows since the client's offsets. */
export interface SimState {
  run_id: number
  running: boolean
  /** Exactly the Tk view's status strings: Idle | Running <scenario>... | Aborting... | Aborted | Finished (exit code N). */
  status: string
  exit_code: number | null
  /** The repo-relative equivalent command line, also stdout's first line. */
  command: string
  stdout: { offset: number; lines: string[] }
  log: SimLogState
  telemetry_file: string | null
  /** The current project's model rebuild, or null when there is no record; every poll refreshes it. */
  build: SimBuild | null
}

export interface SimStartResult {
  ok: boolean
  message: string
}

function asSimOptions(payload: unknown): SimOptions | null {
  if (typeof payload !== 'object' || payload === null) return null
  const candidate = payload as Partial<SimOptions>
  if (!Array.isArray(candidate.controllers) || !candidate.controllers.every((name) => typeof name === 'string')) return null
  if (typeof candidate.default_workers !== 'number' || typeof candidate.max_workers !== 'number') return null
  if (!Array.isArray(candidate.inav_configs)) return null
  if (typeof candidate.defaults !== 'object' || candidate.defaults === null) return null
  if ((typeof candidate.scenarios !== "object" || candidate.scenarios === null) || !Array.isArray(candidate.starts) || !Array.isArray(candidate.start_options)) return null
  if (typeof candidate.launcher_defaults !== 'object' || candidate.launcher_defaults === null) return null
  if (typeof candidate.default_duration_s !== 'number') return null
  return candidate as SimOptions
}

function asSimBuild(value: unknown): SimBuild | null {
  if (typeof value !== 'object' || value === null) return null
  const candidate = value as Partial<SimBuild>
  if (typeof candidate.stage !== 'string' || typeof candidate.detail !== 'string') return null
  if (typeof candidate.done !== 'number' || typeof candidate.total !== 'number' || typeof candidate.workers !== 'number') return null
  if (typeof candidate.elapsed_s !== 'number' || typeof candidate.finished !== 'boolean' || !Array.isArray(candidate.cached)) return null
  return { ...candidate, failed: candidate.failed === true } as SimBuild
}

function asSimState(payload: unknown): SimState | null {
  if (typeof payload !== 'object' || payload === null) return null
  const candidate = payload as Partial<SimState>
  if (typeof candidate.run_id !== 'number' || typeof candidate.running !== 'boolean') return null
  if (typeof candidate.status !== 'string' || typeof candidate.command !== 'string') return null
  if (typeof candidate.stdout !== 'object' || candidate.stdout === null) return null
  if (typeof candidate.log !== 'object' || candidate.log === null) return null
  return { ...(candidate as SimState), build: asSimBuild(candidate.build) }
}

export async function simOptions(): Promise<SimOptions | null> {
  try {
    const response = await fetch('/api/sim/options')
    if (!response.ok) return null
    return asSimOptions(await response.json())
  } catch {
    return null
  }
}

/** Start one run; a refusal (400) carries the server's message, e.g. "Altitude (m): 'abc' is not a number". */
export async function startSim(payload: SimStart): Promise<SimStartResult> {
  let response: Response
  try {
    response = await fetch('/api/sim/run', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(payload),
    })
  } catch {
    return { ok: false, message: 'Server unreachable.' }
  }
  const text = await response.text()
  let body: unknown = null
  try {
    body = text === '' ? null : JSON.parse(text)
  } catch {
    body = null
  }
  if (!response.ok) {
    const message =
      typeof body === 'object' && body !== null && typeof (body as { message?: unknown }).message === 'string'
        ? (body as { message: string }).message
        : `Run refused (${response.status})`
    return { ok: false, message }
  }
  return { ok: true, message: '' }
}

/** One SIM command that answers {ok, message} (webapp.sim routes). */
async function postSimCommand(url: string, body?: unknown): Promise<SimStartResult> {
  let response: Response
  try {
    response = await fetch(url, {
      method: 'POST',
      ...(body === undefined ? {} : { headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) }),
    })
  } catch {
    return { ok: false, message: 'Server unreachable.' }
  }
  const text = await response.text()
  let payload: unknown = null
  try {
    payload = text === '' ? null : JSON.parse(text)
  } catch {
    payload = null
  }
  const message =
    typeof payload === 'object' && payload !== null && typeof (payload as { message?: unknown }).message === 'string'
      ? (payload as { message: string }).message
      : ''
  if (!response.ok) return { ok: false, message: message || `Request failed (${response.status})` }
  return { ok: true, message }
}

/** Remember this plane's controller and scenario (POST /api/sim/default, webapp.sim.save_default): writes the
 *  project's sim.json. A refusal (400) carries the server's message. */
export function setSimDefault(controller: string, scenario: string): Promise<SimStartResult> {
  return postSimCommand('/api/sim/default', { controller, scenario })
}

/** SIGINT the live run; false when there was none (or one was already aborting). */
export async function abortSim(): Promise<boolean> {
  try {
    const response = await fetch('/api/sim/abort', { method: 'POST' })
    if (!response.ok) return false
    const payload: unknown = await response.json()
    return typeof payload === 'object' && payload !== null && (payload as { ok?: unknown }).ok === true
  } catch {
    return false
  }
}

/** One poll of the run state; the offsets and run_id come from the previous answer. */
export async function simState(stdoutOffset: number, logOffset: number, runId: number | null): Promise<SimState | null> {
  const params = new URLSearchParams({ stdout_offset: String(stdoutOffset), log_offset: String(logOffset) })
  if (runId !== null) params.set('run_id', String(runId))
  try {
    const response = await fetch(`/api/sim/state?${params.toString()}`)
    if (!response.ok) return null
    return asSimState(await response.json())
  } catch {
    return null
  }
}

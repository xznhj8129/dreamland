import { useSyncExternalStore } from 'react'
import type {
  DrawPayload,
  EditorContext,
  LibraryDeleteBody,
  LibraryPayload,
  LibrarySaveBody,
  MoveBody,
  Path,
  PathEdit,
  Snapshot,
  StateMessage,
} from './types'

const RECONNECT_MS = 1000

let snapshot: Snapshot | null = null
let started = false
let socket: WebSocket | null = null
const listeners = new Set<() => void>()

/** The published session/revision; -1 until the first snapshot. */
let publishedSession = -1
let publishedRevision = -1
/** The published snapshot's library revision; -1 until the first. The editor-context cache follows it. */
let publishedLibraryRevision = -1
/** Set when the websocket closes: its next message is the server's current state even after a server restart. */
let afterWebsocketClose = false

/**
 * Publish a snapshot only when it is not stale. Session ids only move forward while one server runs, and every
 * accepted definition change bumps definition_revision, so a response from a project the browser has left, or
 * one computed before a later edit, is dropped instead of overwriting the forms. The first websocket message
 * after a close is exempt: a restarted server begins its session ids over, and that message is its live state.
 */
function publish(next: Snapshot, fromWebsocket = false): void {
  const stale =
    next.session_id < publishedSession ||
    (next.session_id === publishedSession && next.definition_revision < publishedRevision)
  if (stale && !(fromWebsocket && afterWebsocketClose)) return
  publishedSession = next.session_id
  publishedRevision = next.definition_revision
  if (typeof next.library_revision === 'number') publishedLibraryRevision = next.library_revision
  afterWebsocketClose = false
  snapshot = next
  for (const listener of listeners) listener()
}

function subscribe(listener: () => void): () => void {
  listeners.add(listener)
  return () => {
    listeners.delete(listener)
  }
}

export function useSnapshot(): Snapshot | null {
  return useSyncExternalStore(subscribe, () => snapshot)
}

/** The published snapshot's session id; -1 until the first. The draw path fences its responses with it. */
export function currentSessionId(): number {
  return publishedSession
}

/** The published snapshot's library_revision; -1 until the first. A part-library change moves it, and the cached
 * editor context (parts_data for the aircraft combos) is refetched when it does. */
export function currentLibraryRevision(): number {
  return publishedLibraryRevision
}

export interface CommandResult {
  ok: boolean
  message: string
  /** The server's HTTP status when it answered; undefined when the request never reached it. */
  status?: number
}

function asSnapshot(value: unknown): Snapshot | null {
  if (typeof value !== 'object' || value === null) return null
  const candidate = value as Partial<Snapshot>
  if (typeof candidate.project !== 'string' || !Array.isArray(candidate.log)) return null
  return candidate as Snapshot
}

async function refresh(): Promise<void> {
  try {
    const response = await fetch('/api/state')
    if (response.ok) publish((await response.json()) as Snapshot)
  } catch {
    // server not reachable yet; the websocket reconnect keeps trying
  }
}

/**
 * Send a command that answers with a snapshot. A refused command (4xx) and a logged server failure (5xx) both
 * carry the state, so the log entry reaches the Log tab; the returned message is the last log entry's one-line
 * summary (the full text when it has none) for the modal or status to show.
 */
async function request(method: string, url: string, body?: unknown): Promise<CommandResult> {
  let response: Response
  try {
    response = await fetch(url, {
      method,
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
  const next = asSnapshot(payload)
  if (next !== null) publish(next)
  if (response.ok) return { ok: true, message: '', status: response.status }
  const last = next?.log[next.log.length - 1]
  return { ok: false, message: last === undefined ? text || `Request failed (${response.status})` : last.summary || last.text, status: response.status }
}

/**
 * The one mutation queue: every set, op, preset op, save and project command waits for the previous one to
 * finish, so the server sees them in the order the user issued them. The queue never holds a rejection.
 */
let queue: Promise<unknown> = Promise.resolve()

function enqueue<T>(task: () => Promise<T>): Promise<T> {
  const result = queue.then(task)
  queue = result.then(
    () => undefined,
    () => undefined,
  )
  return result
}

function post(url: string, body?: unknown): Promise<CommandResult> {
  return request('POST', url, body)
}

/** F5: validate, resolve and re-run aero on the current definition. Also serialized with the mutations. */
export function recompute(): Promise<CommandResult> {
  return enqueue(() => post('/api/recompute'))
}

export function saveProject(): Promise<CommandResult> {
  return enqueue(() => post('/api/save'))
}

export function openProject(name: string): Promise<CommandResult> {
  return enqueue(() => post('/api/projects/open', { name }))
}

export function newProject(name: string): Promise<CommandResult> {
  return enqueue(() => post('/api/projects/new', { name }))
}

export function saveProjectAs(name: string): Promise<CommandResult> {
  return enqueue(() => post('/api/save-as', { name }))
}

function setRequest(sessionId: number, edit: PathEdit): Promise<CommandResult> {
  const body = 'remove' in edit
    ? { session_id: sessionId, path: edit.path, remove: true }
    : { session_id: sessionId, path: edit.path, value: edit.value }
  return post('/api/editor/set', body)
}

/** One field edit: set the value at path. The session it was rendered from fences it against a project switch. */
export function editorSet(sessionId: number, path: Path, value: unknown): Promise<CommandResult> {
  return enqueue(() => setRequest(sessionId, { path, value }))
}

/** One field edit: delete a dict key, or set a list entry to null (Tk's optional-entry removal). */
export function editorRemove(sessionId: number, path: Path): Promise<CommandResult> {
  return enqueue(() => setRequest(sessionId, { path, remove: true }))
}

/** A compound action as several atomic edits, in order and contiguous in the queue (one slot), stopping on the
 * first refusal so a half-applied action cannot be followed by edits built on it. */
export function editorEdit(sessionId: number, edits: readonly PathEdit[]): Promise<CommandResult> {
  return enqueue(async () => {
    let result: CommandResult = { ok: true, message: '' }
    for (const edit of edits) {
      result = await setRequest(sessionId, edit)
      if (!result.ok) break
    }
    return result
  })
}

/** A structural editor operation (section cuts, preset load/save/new); the server validates and recomputes. */
export function editorOp(sessionId: number, body: Record<string, unknown>): Promise<CommandResult> {
  return enqueue(() => post('/api/editor/op', { session_id: sessionId, ...body }))
}

/** The drag fast path's answer: the /api/draw payload the move was drawn into, or the refusal. */
export interface MoveDrawResult {
  ok: boolean
  status: number
  payload?: DrawPayload
  /** The last Log entry of a refusal, or a transport error, for the status line. */
  message: string
}

function asDrawPayload(value: unknown): DrawPayload | null {
  if (typeof value !== 'object' || value === null) return null
  const candidate = value as Partial<DrawPayload>
  if (typeof candidate.session_id !== 'number' || !Array.isArray(candidate.scene)) return null
  return candidate as DrawPayload
}

/**
 * One drag motion on the fast path (POST /api/editor/move with the pane size): a direct fetch, never queued and
 * never publishing, so a drag step cannot starve the /api/draw queue and its draw payload re-renders no forms.
 * The server answers with the drawing instead of the snapshot and skips the websocket push. A response that
 * arrives after the project moved on is treated as the server's stale-session 409, which ends the drag.
 */
export async function moveDraw(
  sessionId: number,
  body: MoveBody,
  width: number,
  height: number,
): Promise<MoveDrawResult> {
  let response: Response
  try {
    response = await fetch('/api/editor/move', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ session_id: sessionId, ...body, width, height }),
    })
  } catch {
    return { ok: false, status: 0, message: 'Server unreachable.' }
  }
  const text = await response.text()
  let payload: unknown = null
  try {
    payload = text === '' ? null : JSON.parse(text)
  } catch {
    payload = null
  }
  if (!response.ok) {
    const next = asSnapshot(payload)
    const last = next?.log[next.log.length - 1]
    return { ok: false, status: response.status, message: last === undefined ? `Move failed (${response.status})` : last.summary || last.text }
  }
  if (sessionId !== currentSessionId()) {
    return { ok: false, status: 409, message: 'Project switched.' }
  }
  const draw = asDrawPayload(payload)
  if (draw === null) return { ok: false, status: response.status, message: 'Move answered with no drawing.' }
  return { ok: true, status: response.status, payload: draw, message: '' }
}

export async function getEditorContext(): Promise<EditorContext | null> {
  try {
    const response = await fetch('/api/editor/context')
    if (!response.ok) return null
    return (await response.json()) as EditorContext
  } catch {
    return null
  }
}

/** GET /api/library: the whole part library and its form metadata (webapp.library.library_payload). */
export async function getLibrary(): Promise<LibraryPayload | null> {
  try {
    const response = await fetch('/api/library')
    if (!response.ok) return null
    const payload: unknown = await response.json()
    if (typeof payload !== 'object' || payload === null) return null
    const parts = (payload as { parts?: unknown }).parts
    if (typeof parts !== 'object' || parts === null) return null
    return payload as LibraryPayload
  } catch {
    return null
  }
}

/** One part-library save: a New part (original null) or an edit, rename or category move. The answer is the
 * snapshot (library_revision bumped on success; a refusal is 400 with the snapshot and a Log entry). */
export function saveLibraryPart(body: LibrarySaveBody): Promise<CommandResult> {
  return enqueue(() => post('/api/library/save', body))
}

/** One part-library delete; the server reloads the library, recomputes the aircraft and pushes state. */
export function deleteLibraryPart(body: LibraryDeleteBody): Promise<CommandResult> {
  return enqueue(() => post('/api/library/delete', body))
}

export async function listProjects(): Promise<string[] | null> {
  try {
    const response = await fetch('/api/projects')
    if (!response.ok) return null
    const data = (await response.json()) as { projects?: unknown }
    if (!Array.isArray(data.projects)) return null
    return data.projects.filter((name): name is string => typeof name === 'string')
  } catch {
    return null
  }
}

function connect(): void {
  const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:'
  const active = new WebSocket(`${protocol}//${window.location.host}/ws/events`)
  socket = active
  active.onmessage = (event) => {
    const message = JSON.parse(String(event.data)) as StateMessage
    if (message.type === 'state') publish(message, true)
  }
  active.onclose = () => {
    if (socket === active) socket = null
    afterWebsocketClose = true
    window.setTimeout(connect, RECONNECT_MS)
  }
  active.onerror = () => {
    active.close()
  }
}

export function startClient(): void {
  if (started) return
  started = true
  void refresh()
  connect()
}

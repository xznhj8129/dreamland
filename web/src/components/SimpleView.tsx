import { handleFromSelection, selectionFromHandle, useSelection } from '../editor/selection'
import { useCallback, useEffect, useRef, useState } from 'react'
import type { ChangeEvent, PointerEvent as ReactPointerEvent, ReactElement } from 'react'
import { CanvasRenderer } from '../renderer'
import type { DrawHighlight } from '../renderer'
import { currentSessionId, moveDraw, recompute } from '../store'
import type { DrawHandle, DrawPayload, MoveBody, RollDetent, ScrollThumb, Snapshot } from '../types'

type ViewMode = DrawPayload['view']['mode']

type Action =
  | { kind: 'redraw' }
  | { kind: 'reset' }
  | { kind: 'pan_delta'; dx: number; dy: number }
  | { kind: 'zoom_at'; x: number; y: number; factor: number }
  | { kind: 'scroll'; axis: 'x' | 'y'; mode: 'moveto' | 'units' | 'pages'; value: number }
  | { kind: 'set_mode'; mode: ViewMode }
  | { kind: 'set_roll'; roll_deg: number }

/** One movable element as the hit test and the highlight name it (handles refresh as it moves). */
interface HandleKey {
  target: DrawHandle['target']
  index: number
}

interface PanDrag {
  pointerId: number
  x: number
  y: number
}

interface ElementDrag extends HandleKey {
  pointerId: number
  horizontal: DrawHandle['horizontal']
  vertical: DrawHandle['vertical']
  /** The last pointer sample, screen px. */
  x: number
  y: number
  /** Screen px accumulated since the last rAF, folded into pending metres at the live scale. */
  dxPx: number
  dyPx: number
  frame: number
  /** Folded metres not yet sent; while a request is in flight the pointer keeps accumulating here. */
  pending: MoveBody['delta']
  /** At most one move request in flight: the next step is sent when this one lands. */
  busy: boolean
  /** Set on pointerup/cancel: only the final coalesced move may still go out, then the release recompute. */
  finishing: boolean
}

const ZOOM_FACTOR = 1.1 // editor_view.py on_view_mousewheel
const FULL_THUMB: ScrollThumb = { first: 0, last: 1 }
const HIT_TOLERANCE_PX = 6
const ROLL_MIN = -90
const ROLL_MAX = 90
const ROLL_SNAP_WINDOW_DEG = 4
/** webapp/editor.py MOVE_QUANTUM_M: the server snaps dragged dimensions to this 0.01 mm grid. */
const MOVE_QUANTUM_M = 1e-5

function hasDelta(delta: MoveBody['delta']): boolean {
  return (delta.x ?? 0) !== 0 || (delta.y ?? 0) !== 0 || (delta.z ?? 0) !== 0
}

/** A pending delta at least one server quantum in some axis is worth a request; smaller ones keep accumulating. */
function crossesQuantum(delta: MoveBody['delta']): boolean {
  return Math.abs(delta.x ?? 0) >= MOVE_QUANTUM_M || Math.abs(delta.y ?? 0) >= MOVE_QUANTUM_M || Math.abs(delta.z ?? 0) >= MOVE_QUANTUM_M
}

function sameKey(a: HandleKey | null, b: HandleKey | null): boolean {
  return a === b || (a !== null && b !== null && a.target === b.target && a.index === b.index)
}

function pointInPolygon(coords: readonly number[], x: number, y: number): boolean {
  const count = coords.length / 2
  if (count < 3) return false
  let inside = false
  for (let i = 0, j = count - 1; i < count; j = i++) {
    const xi = coords[i * 2]
    const yi = coords[i * 2 + 1]
    const xj = coords[j * 2]
    const yj = coords[j * 2 + 1]
    if ((yi > y) !== (yj > y) && x < ((xj - xi) * (y - yi)) / (yj - yi) + xi) inside = !inside
  }
  return inside
}

function distanceToSegment(px: number, py: number, x0: number, y0: number, x1: number, y1: number): number {
  const dx = x1 - x0
  const dy = y1 - y0
  const lengthSq = dx * dx + dy * dy
  const t = lengthSq === 0 ? 0 : Math.min(Math.max(((px - x0) * dx + (py - y0) * dy) / lengthSq, 0), 1)
  return Math.hypot(px - (x0 + t * dx), py - (y0 + t * dy))
}

function pointNearLine(coords: readonly number[], x: number, y: number): boolean {
  for (let index = 0; index + 3 < coords.length; index += 2) {
    if (distanceToSegment(x, y, coords[index], coords[index + 1], coords[index + 2], coords[index + 3]) <= HIT_TOLERANCE_PX) return true
  }
  return false
}

/** The topmost handle under a point: by descending order, since a later handle is drawn on top. */
function hitHandle(handles: readonly DrawHandle[], x: number, y: number): DrawHandle | null {
  for (const handle of [...handles].sort((a, b) => b.order - a.order)) {
    if (handle.kind === 'polygon' ? pointInPolygon(handle.coords, x, y) : pointNearLine(handle.coords, x, y)) return handle
  }
  return null
}

function clamp(value: number, low: number, high: number): number {
  return Math.min(Math.max(value, low), high)
}

function clamp01(value: number): number {
  return clamp(value, 0, 1)
}

/** The nearest detent when the raw slider value is within the snap window, otherwise the raw value. */
function snapRoll(value: number, detents: readonly RollDetent[]): number {
  let snapped = value
  let bestDistance = ROLL_SNAP_WINDOW_DEG
  for (const detent of detents) {
    const distance = Math.abs(detent.roll_deg - value)
    if (distance <= bestDistance) {
      snapped = detent.roll_deg
      bestDistance = distance
    }
  }
  return snapped
}

export function SimpleView({ snapshot }: { snapshot: Snapshot | null }): ReactElement {
  const { active, select } = useSelection()
  const selectedRef = useRef<HandleKey | null>(null)
  selectedRef.current = handleFromSelection(active)
  const paneRef = useRef<HTMLDivElement | null>(null)
  const canvasRef = useRef<HTMLCanvasElement | null>(null)
  const rendererRef = useRef<CanvasRenderer | null>(null)
  const sizeRef = useRef({ width: 0, height: 0 })
  const aliveRef = useRef(true)
  const pendingRef = useRef<Action[]>([])
  const inFlightRef = useRef(false)
  /** Monotonic ids for every payload request (draw queue and drag fast path); an older answer never paints. */
  const payloadSeqRef = useRef(0)
  const appliedSeqRef = useRef(0)
  const payloadRef = useRef<DrawPayload | null>(null)
  const dragRef = useRef<PanDrag | null>(null)
  const elementRef = useRef<ElementDrag | null>(null)
  const hoverRef = useRef<HandleKey | null>(null)
  const panMoveRef = useRef({ dx: 0, dy: 0, frame: 0 })
  const [size, setSize] = useState({ width: 0, height: 0 })
  const [payload, setPayload] = useState<DrawPayload | null>(null)
  const [mode, setMode] = useState<ViewMode>('Longitudinal')
  const [roll, setRoll] = useState(0)
  const [error, setError] = useState('')

  const pump = useCallback((): void => {
    if (inFlightRef.current) return
    const action = pendingRef.current[0]
    if (action === undefined) return
    const { width, height } = sizeRef.current
    if (width < 1 || height < 1) return
    pendingRef.current.shift()
    inFlightRef.current = true
    const seq = ++payloadSeqRef.current
    void (async () => {
      try {
        const session = currentSessionId()
        const response = await fetch('/api/draw', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ width, height, action, ...(session >= 0 ? { session_id: session } : {}) }),
        })
        if (!aliveRef.current) return
        if (response.ok) {
          const next = (await response.json()) as DrawPayload
          // A resize may have landed while this was in flight; that size-change redraw is already queued,
          // so drop a scene drawn for the old pane instead of painting a glitch. A project switch makes the
          // session unknown here to the old draw; the new snapshot's redraw paints instead. A drag move with a
          // newer id has painted a newer scene; an older draw answer must not overwrite it.
          const current = currentSessionId()
          if (
            aliveRef.current &&
            seq > appliedSeqRef.current &&
            (current < 0 || next.session_id === current) &&
            next.width === sizeRef.current.width &&
            next.height === sizeRef.current.height
          ) {
            appliedSeqRef.current = seq
            payloadRef.current = next
            setPayload(next)
            setMode(next.view.mode)
            setError('')
          }
        } else if (response.status !== 409) {
          setError(`draw failed (${response.status})`)
        }
      } catch {
        if (aliveRef.current) setError('draw request failed')
      } finally {
        inFlightRef.current = false
        if (aliveRef.current) pump()
      }
    })()
  }, [])

  // At most one in-flight request: coalesce consecutive motions, let a direct command replace them.
  // zoom_at is not coalesced: two zooms at different cursors cannot merge into one zoom with the product factor.
  const request = useCallback(
    (action: Action): void => {
      const pending = pendingRef.current
      const last = pending[pending.length - 1]
      if (action.kind === 'pan_delta' && last?.kind === 'pan_delta') {
        last.dx += action.dx
        last.dy += action.dy
        return
      }
      if (action.kind === 'redraw' && last?.kind === 'redraw') {
        // The queued redraw may still be waiting for a pane size; pump it now that one may exist.
        pump()
        return
      }
      if (action.kind !== 'pan_delta' && action.kind !== 'zoom_at' && action.kind !== 'redraw') pending.length = 0
      pending.push(action)
      pump()
    },
    [pump],
  )

  /** Restroke the hovered/dragged handle from the newest payload (its coords move as the server recomputes). */
  const applyHighlight = useCallback((): void => {
    const renderer = rendererRef.current
    if (renderer === null) return
    const payload = payloadRef.current
    const key = elementRef.current ?? hoverRef.current ?? selectedRef.current
    const handle =
      payload === null || key === null
        ? null
        : payload.handles.find((candidate) => candidate.target === key.target && candidate.index === key.index) ?? null
    const highlight: DrawHighlight | null = handle === null ? null : { kind: handle.kind, coords: handle.coords }
    renderer.setHighlight(highlight)
  }, [])

  useEffect(() => { applyHighlight() }, [active, applyHighlight])

  const endElementDrag = useCallback((): void => {
    const drag = elementRef.current
    if (drag === null) return
    if (drag.frame !== 0) window.cancelAnimationFrame(drag.frame)
    elementRef.current = null
    const canvas = canvasRef.current
    if (canvas !== null && canvas.hasPointerCapture(drag.pointerId)) canvas.releasePointerCapture(drag.pointerId)
  }, [])

  const setHover = useCallback(
    (key: HandleKey | null): void => {
      if (sameKey(hoverRef.current, key)) return
      hoverRef.current = key
      const canvas = canvasRef.current
      if (canvas !== null) canvas.style.cursor = key === null ? '' : 'grab'
      applyHighlight()
    },
    [applyHighlight],
  )

  /** Fold the screen pixels accumulated since the last rAF into the drag's pending metre delta. */
  const foldElementPixels = useCallback((drag: ElementDrag): void => {
    const dxPx = drag.dxPx
    const dyPx = drag.dyPx
    drag.dxPx = 0
    drag.dyPx = 0
    if (dxPx === 0 && dyPx === 0) return
    const scale = payloadRef.current?.metrics.scale
    if (typeof scale !== 'number' || !(scale > 0)) return
    if (drag.horizontal !== null) drag.pending[drag.horizontal] = (drag.pending[drag.horizontal] ?? 0) + dxPx / scale
    // Screen y is down, the definition z is up.
    if (drag.vertical !== null) drag.pending[drag.vertical] = (drag.pending[drag.vertical] ?? 0) - dyPx / scale
  }, [])

  /**
   * Send the pending metre delta as one request, if none is in flight; returns whether one was dispatched.
   * A fast drag coalesces into one step per round trip instead of one per pointer sample, and the answer paints
   * the returned drawing directly (no snapshot publish, so no forms re-render). A 409, or an answer for a
   * session the browser has left, ends the drag without another request.
   */
  function sendElementMove(): boolean {
    const drag = elementRef.current
    if (drag === null || drag.busy || !hasDelta(drag.pending)) return false
    const { width, height } = sizeRef.current
    if (width < 1 || height < 1) return false
    const session = currentSessionId()
    if (session < 0) return false
    const delta: MoveBody['delta'] = { ...drag.pending }
    drag.pending = {}
    drag.busy = true
    const seq = ++payloadSeqRef.current
    void moveDraw(session, { target: drag.target, index: drag.index, delta }, width, height).then((result) => {
      if (!aliveRef.current) return
      if (result.ok && result.payload !== undefined && seq > appliedSeqRef.current) {
        const next = result.payload
        // The same fences as the draw queue: an answer for another session or pane size must not be painted.
        if (
          next.session_id === currentSessionId() &&
          next.width === sizeRef.current.width &&
          next.height === sizeRef.current.height
        ) {
          appliedSeqRef.current = seq
          payloadRef.current = next
          setPayload(next)
          setError('')
        }
      } else if (!result.ok && result.status !== 409) {
        setError(result.message)
      }
      if (elementRef.current !== drag) return // a newer drag owns the pointer now
      drag.busy = false
      if (!result.ok && result.status === 409) {
        // The project switched under the drag: end cleanly, no further requests and no release recompute. The
        // highlight stays keyed by target+index and follows the new session's payload if that handle survives.
        endElementDrag()
        applyHighlight()
        return
      }
      if (drag.finishing) {
        if (hasDelta(drag.pending) && sendElementMove()) return
        endElementDrag()
        applyHighlight()
        void recompute() // the Tk release (on_view_drag_end) runs the full recompute, aero included
      } else if (crossesQuantum(drag.pending)) {
        sendElementMove()
      }
    })
    return true
  }

  /** One rAF of an element drag: fold pixels into pending metres, then send a coalesced step when due. */
  function flushElementMove(): void {
    const drag = elementRef.current
    if (drag === null) return
    drag.frame = 0
    foldElementPixels(drag)
    if (drag.busy || drag.finishing) return
    if (crossesQuantum(drag.pending)) sendElementMove()
  }

  useEffect(() => {
    aliveRef.current = true
    return () => {
      aliveRef.current = false
      if (panMoveRef.current.frame !== 0) window.cancelAnimationFrame(panMoveRef.current.frame)
      if (elementRef.current !== null && elementRef.current.frame !== 0) window.cancelAnimationFrame(elementRef.current.frame)
      rendererRef.current?.dispose()
      rendererRef.current = null
    }
  }, [])

  // The pane size is the Studio view's canvas size; a resize redraws (as the Tk <Configure> binding does).
  useEffect(() => {
    const pane = paneRef.current
    if (pane === null) return undefined
    const observer = new ResizeObserver((entries) => {
      const rect = entries[0]?.contentRect
      if (rect === undefined) return
      const width = Math.max(1, Math.round(rect.width))
      const height = Math.max(1, Math.round(rect.height))
      if (sizeRef.current.width === width && sizeRef.current.height === height) return
      sizeRef.current = { width, height }
      setSize({ width, height })
      request({ kind: 'redraw' })
    })
    observer.observe(pane)
    return () => observer.disconnect()
  }, [request])

  // resolved/aero change with recompute; redraw with the persistent server-side view.
  useEffect(() => {
    request({ kind: 'redraw' })
  }, [snapshot, request])

  useEffect(() => {
    payloadRef.current = payload
    const canvas = canvasRef.current
    if (canvas === null || size.width < 1 || size.height < 1) return
    let renderer = rendererRef.current
    if (renderer === null) {
      renderer = new CanvasRenderer(canvas)
      rendererRef.current = renderer
    }
    renderer.resize(size.width, size.height)
    if (payload !== null) renderer.render(payload.scene)
    applyHighlight() // the retained handle may have new coords in this payload (drag motion)
  }, [payload, size, applyHighlight])

  // Wheel zoom at the cursor; native listener so preventDefault is allowed.
  useEffect(() => {
    const canvas = canvasRef.current
    if (canvas === null) return undefined
    const onWheel = (event: WheelEvent): void => {
      event.preventDefault()
      const rect = canvas.getBoundingClientRect()
      request({ kind: 'zoom_at', x: event.clientX - rect.left, y: event.clientY - rect.top, factor: event.deltaY < 0 ? ZOOM_FACTOR : 1 / ZOOM_FACTOR })
    }
    canvas.addEventListener('wheel', onWheel, { passive: false })
    return () => canvas.removeEventListener('wheel', onWheel)
  }, [request])

  const flushPan = useCallback((): void => {
    const move = panMoveRef.current
    move.frame = 0
    if (move.dx !== 0 || move.dy !== 0) request({ kind: 'pan_delta', dx: move.dx, dy: move.dy })
    move.dx = 0
    move.dy = 0
  }, [request])

  const onPointerDown = (event: ReactPointerEvent<HTMLCanvasElement>): void => {
    if (event.button !== 0) return
    const rect = event.currentTarget.getBoundingClientRect()
    const payload = payloadRef.current
    const hit = payload === null ? null : hitHandle(payload.handles, event.clientX - rect.left, event.clientY - rect.top)
    event.currentTarget.setPointerCapture(event.pointerId)
    if (hit !== null) {
      const selection = selectionFromHandle(hit)
      select(selection.kind, selection.index)
      if (elementRef.current !== null) return // one element drag at a time
      elementRef.current = {
        pointerId: event.pointerId,
        target: hit.target,
        index: hit.index,
        horizontal: hit.horizontal,
        vertical: hit.vertical,
        x: event.clientX,
        y: event.clientY,
        dxPx: 0,
        dyPx: 0,
        frame: 0,
        pending: {},
        busy: false,
        finishing: false,
      }
      setHover({ target: hit.target, index: hit.index })
      return
    }
    dragRef.current = { pointerId: event.pointerId, x: event.clientX, y: event.clientY }
  }

  const onPointerMove = (event: ReactPointerEvent<HTMLCanvasElement>): void => {
    const element = elementRef.current
    if (element !== null) {
      if (element.pointerId !== event.pointerId || element.finishing) return
      element.dxPx += event.clientX - element.x
      element.dyPx += event.clientY - element.y
      element.x = event.clientX
      element.y = event.clientY
      if (element.frame === 0) element.frame = window.requestAnimationFrame(flushElementMove)
      return
    }
    const drag = dragRef.current
    if (drag !== null) {
      if (drag.pointerId !== event.pointerId) return
      const move = panMoveRef.current
      move.dx += event.clientX - drag.x
      move.dy += event.clientY - drag.y
      drag.x = event.clientX
      drag.y = event.clientY
      if (move.frame === 0) move.frame = window.requestAnimationFrame(flushPan)
      return
    }
    if (event.buttons !== 0) return // hit-test only the bare hover, never mid-button
    const payload = payloadRef.current
    const rect = event.currentTarget.getBoundingClientRect()
    const hit = payload === null ? null : hitHandle(payload.handles, event.clientX - rect.left, event.clientY - rect.top)
    setHover(hit === null ? null : { target: hit.target, index: hit.index })
  }

  const onPointerLeave = (): void => {
    if (elementRef.current !== null || dragRef.current !== null) return // captured drags keep their highlight
    setHover(null)
  }

  const onPointerEnd = (event: ReactPointerEvent<HTMLCanvasElement>): void => {
    const element = elementRef.current
    if (element !== null && element.pointerId === event.pointerId) {
      if (element.frame !== 0) {
        window.cancelAnimationFrame(element.frame)
        element.frame = 0
      }
      foldElementPixels(element) // the last pixels may not have reached an rAF yet
      element.finishing = true
      if (event.currentTarget.hasPointerCapture(event.pointerId)) event.currentTarget.releasePointerCapture(event.pointerId)
      // With a request in flight its answer flushes the rest; otherwise the final coalesced move goes now.
      if (!element.busy && !(hasDelta(element.pending) && sendElementMove())) {
        endElementDrag()
        applyHighlight()
        void recompute() // the Tk release (on_view_drag_end) runs the full recompute, aero included
      }
      return
    }
    if (dragRef.current === null) return
    dragRef.current = null
    if (event.currentTarget.hasPointerCapture(event.pointerId)) event.currentTarget.releasePointerCapture(event.pointerId)
  }

  const chooseMode = (next: ViewMode): void => {
    setMode(next)
    request({ kind: 'set_mode', mode: next })
  }

  const detents = payload?.roll_detents ?? []

  const onRollChange = (event: ChangeEvent<HTMLInputElement>): void => {
    const value = snapRoll(Number(event.target.value), detents)
    setRoll(value)
    request({ kind: 'set_roll', roll_deg: value })
  }

  const onScrollbar = useCallback(
    (axis: 'x' | 'y', scrollMode: 'moveto' | 'units' | 'pages', value: number): void => {
      request({ kind: 'scroll', axis, mode: scrollMode, value })
    },
    [request],
  )

  const scrollbars = payload?.scrollbars ?? { x: FULL_THUMB, y: FULL_THUMB }

  return (
    <div className="simple-view">
      <div className="graphics-controls">
        <span className="axes-label">X = Front   Y = Right   Z = Up</span>
        <span className="axes-label">NP = static estimate</span>
        <label className="mode-radio">
          <input type="radio" name="view-mode" checked={mode === 'Longitudinal'} onChange={() => chooseMode('Longitudinal')} />
          Longitudinal
        </label>
        <label className="mode-radio">
          <input type="radio" name="view-mode" checked={mode === 'Front'} onChange={() => chooseMode('Front')} />
          Front
        </label>
        <button type="button" onClick={() => request({ kind: 'reset' })}>
          Reset View
        </button>
        {error !== '' && <span className="draw-error">{error}</span>}
      </div>
      <div className="drawing-row">
        <div className="canvas-area">
          <div className="canvas-pane" ref={paneRef}>
            <canvas
              ref={canvasRef}
              className="drawing-canvas"
              onPointerDown={onPointerDown}
              onPointerMove={onPointerMove}
              onPointerUp={onPointerEnd}
              onPointerCancel={onPointerEnd}
              onPointerLeave={onPointerLeave}
            />
          </div>
          <Scrollbar axis="x" thumb={scrollbars.x} onScroll={onScrollbar} />
          <Scrollbar axis="y" thumb={scrollbars.y} onScroll={onScrollbar} />
        </div>
        {mode === 'Longitudinal' && (
          <div className="roll-column">
            <span className="roll-label">90 top</span>
            <div className="roll-slider-row">
              <span className="roll-label roll-mid">0 side</span>
              <div className="roll-slider-wrap">
                <input className="roll-slider" type="range" min={ROLL_MIN} max={ROLL_MAX} step={1} value={roll} onChange={onRollChange} />
                {detents.map((detent) => (
                  <span
                    key={`${detent.surface}:${detent.kind}:${detent.roll_deg}`}
                    className={`roll-detent roll-detent-${detent.kind}`}
                    style={{ top: `${((ROLL_MAX - detent.roll_deg) / (ROLL_MAX - ROLL_MIN)) * 100}%` }}
                    title={`${detent.surface} ${detent.kind}`}
                  />
                ))}
              </div>
            </div>
            <span className="roll-label">−90 bottom</span>
            <span className="roll-value">{payload?.roll_display ?? '0.0 deg'}</span>
          </div>
        )}
      </div>
    </div>
  )
}

interface ScrollbarProps {
  axis: 'x' | 'y'
  thumb: ScrollThumb
  onScroll: (axis: 'x' | 'y', mode: 'moveto' | 'units' | 'pages', value: number) => void
}

function Scrollbar({ axis, thumb, onScroll }: ScrollbarProps): ReactElement {
  const trackRef = useRef<HTMLDivElement | null>(null)
  const dragRef = useRef<{ start: number; first: number } | null>(null)
  const moveRef = useRef({ target: 0, frame: 0 })
  const first = clamp01(thumb.first)
  const last = Math.max(first, clamp01(thumb.last))
  const thumbSize = last - first

  const flush = useCallback((): void => {
    moveRef.current.frame = 0
    onScroll(axis, 'moveto', moveRef.current.target)
  }, [axis, onScroll])

  useEffect(() => () => {
    if (moveRef.current.frame !== 0) window.cancelAnimationFrame(moveRef.current.frame)
  }, [])

  const onTrackPointerDown = (event: ReactPointerEvent<HTMLDivElement>): void => {
    const track = trackRef.current
    if (track === null) return
    const rect = track.getBoundingClientRect()
    const fraction = axis === 'x' ? (event.clientX - rect.left) / Math.max(rect.width, 1) : (event.clientY - rect.top) / Math.max(rect.height, 1)
    if (fraction < first) onScroll(axis, 'pages', -1)
    else if (fraction > last) onScroll(axis, 'pages', 1)
  }

  const onThumbPointerDown = (event: ReactPointerEvent<HTMLDivElement>): void => {
    event.stopPropagation()
    event.currentTarget.setPointerCapture(event.pointerId)
    dragRef.current = { start: axis === 'x' ? event.clientX : event.clientY, first }
  }

  const onThumbPointerMove = (event: ReactPointerEvent<HTMLDivElement>): void => {
    const drag = dragRef.current
    const track = trackRef.current
    if (drag === null || track === null) return
    const trackLength = axis === 'x' ? track.clientWidth : track.clientHeight
    if (trackLength < 1) return
    const delta = ((axis === 'x' ? event.clientX : event.clientY) - drag.start) / trackLength
    moveRef.current.target = clamp(drag.first + delta, 0, 1 - thumbSize)
    if (moveRef.current.frame === 0) moveRef.current.frame = window.requestAnimationFrame(flush)
  }

  const onThumbPointerEnd = (event: ReactPointerEvent<HTMLDivElement>): void => {
    dragRef.current = null
    if (event.currentTarget.hasPointerCapture(event.pointerId)) event.currentTarget.releasePointerCapture(event.pointerId)
  }

  const style = axis === 'x'
    ? { left: `${first * 100}%`, width: `${thumbSize * 100}%` }
    : { top: `${first * 100}%`, height: `${thumbSize * 100}%` }

  return (
    <div ref={trackRef} className={`scrollbar scrollbar-${axis}`} onPointerDown={onTrackPointerDown}>
      <div
        className="scrollbar-thumb"
        style={style}
        onPointerDown={onThumbPointerDown}
        onPointerMove={onThumbPointerMove}
        onPointerUp={onThumbPointerEnd}
        onPointerCancel={onThumbPointerEnd}
      />
    </div>
  )
}

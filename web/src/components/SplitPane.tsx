import { Children, Fragment, useRef, useState } from 'react'
import type { KeyboardEvent as ReactKeyboardEvent, PointerEvent as ReactPointerEvent, ReactElement, ReactNode } from 'react'

export interface SplitPaneProps {
  /** Two or three panes, left to right. The count must stay stable while mounted. */
  children: ReactNode
  /** Minimum width in px per child; missing entries take DEFAULT_MIN_WIDTH. */
  minSizes?: number[]
  /** Initial width proportions per child; missing entries take 1. */
  initialRatios?: number[]
}

// The Tk editor's paned window: 1:2:1 with practical minimums (editor_view.py 166-174).
const DEFAULT_MIN_SIZES = [220, 280, 200]
const DEFAULT_RATIOS = [1, 2, 1]
const DEFAULT_MIN_WIDTH = 120
const KEYBOARD_STEP_PX = 24

interface DragState {
  divider: number
  startX: number
  widths: number[]
  total: number
}

function normalize(weights: number[]): number[] {
  const total = weights.reduce((sum, weight) => sum + weight, 0)
  if (total <= 0) return weights.map(() => 1 / Math.max(weights.length, 1))
  return weights.map((weight) => weight / total)
}

/**
 * A horizontal paned area: draggable dividers, pointer capture while dragging, arrow keys when a divider
 * has focus. Sizes live in component state only (session-only, like the Tk paned window).
 */
export function SplitPane({ children, minSizes = DEFAULT_MIN_SIZES, initialRatios = DEFAULT_RATIOS }: SplitPaneProps): ReactElement {
  const panes = Children.toArray(children)
  const count = panes.length
  const mins = Array.from({ length: count }, (_, index) => minSizes[index] ?? DEFAULT_MIN_WIDTH)
  const [fractions, setFractions] = useState<number[]>(() =>
    normalize(Array.from({ length: count }, (_, index) => initialRatios[index] ?? 1)),
  )
  const itemRefs = useRef<(HTMLDivElement | null)[]>([])
  const dragRef = useRef<DragState | null>(null)

  const measure = (): number[] => itemRefs.current.map((item) => (item === null ? 0 : item.getBoundingClientRect().width))

  const applyWidth = (divider: number, leftWidth: number, widths: number[], total: number): void => {
    const right = divider + 1
    const pair = widths[divider] + widths[right]
    const low = Math.min(mins[divider], pair)
    const high = Math.max(pair - mins[right], low)
    const clamped = Math.min(Math.max(leftWidth, low), high)
    const next = widths.slice()
    next[divider] = clamped
    next[right] = pair - clamped
    setFractions(next.map((width) => (total <= 0 ? 0 : width / total)))
  }

  const onDividerPointerDown = (event: ReactPointerEvent<HTMLDivElement>, divider: number): void => {
    if (event.button !== 0) return
    event.preventDefault()
    const widths = measure()
    const total = widths.reduce((sum, width) => sum + width, 0)
    if (total <= 0) return
    dragRef.current = { divider, startX: event.clientX, widths, total }
    event.currentTarget.setPointerCapture(event.pointerId)
  }

  const onDividerPointerMove = (event: ReactPointerEvent<HTMLDivElement>): void => {
    const drag = dragRef.current
    if (drag === null) return
    applyWidth(drag.divider, drag.widths[drag.divider] + event.clientX - drag.startX, drag.widths, drag.total)
  }

  const onDividerPointerEnd = (event: ReactPointerEvent<HTMLDivElement>): void => {
    dragRef.current = null
    if (event.currentTarget.hasPointerCapture(event.pointerId)) event.currentTarget.releasePointerCapture(event.pointerId)
  }

  const onDividerKeyDown = (event: ReactKeyboardEvent<HTMLDivElement>, divider: number): void => {
    if (event.key !== 'ArrowLeft' && event.key !== 'ArrowRight') return
    event.preventDefault()
    const widths = measure()
    const total = widths.reduce((sum, width) => sum + width, 0)
    if (total <= 0) return
    const delta = event.key === 'ArrowLeft' ? -KEYBOARD_STEP_PX : KEYBOARD_STEP_PX
    applyWidth(divider, widths[divider] + delta, widths, total)
  }

  return (
    <div className="split-pane">
      {panes.map((pane, index) => (
        <Fragment key={index}>
          {index > 0 && (
            <div
              className="split-divider"
              role="separator"
              aria-orientation="vertical"
              aria-label="Resize panes"
              tabIndex={0}
              onPointerDown={(event) => onDividerPointerDown(event, index - 1)}
              onPointerMove={onDividerPointerMove}
              onPointerUp={onDividerPointerEnd}
              onPointerCancel={onDividerPointerEnd}
              onKeyDown={(event) => onDividerKeyDown(event, index - 1)}
            />
          )}
          <div
            className="split-pane-item"
            ref={(element) => {
              itemRefs.current[index] = element
            }}
            style={{ flexGrow: fractions[index] ?? 0, minWidth: `${mins[index]}px` }}
          >
            {pane}
          </div>
        </Fragment>
      ))}
    </div>
  )
}

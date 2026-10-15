import { useEffect, useRef, useState } from 'react'
import type { ReactElement, ReactNode } from 'react'

/**
 * One menu in the top bar's idiom (menu-root / menu-button / menu-dropdown), for a panel that needs a menu rather
 * than a row of buttons: click to open, click outside or Escape to close, and the items get a close callback so a
 * chosen item can dismiss the menu itself.
 */
export function DropdownMenu({ label, disabled = false, children }: {
  label: string
  disabled?: boolean
  children: ReactNode | ((close: () => void) => ReactNode)
}): ReactElement {
  const [open, setOpen] = useState(false)
  const root = useRef<HTMLDivElement | null>(null)
  useEffect(() => {
    if (!open) return undefined
    const onPointerDown = (event: PointerEvent): void => {
      if (root.current !== null && event.target instanceof Node && !root.current.contains(event.target)) setOpen(false)
    }
    const onKeyDown = (event: KeyboardEvent): void => {
      if (event.key === 'Escape') setOpen(false)
    }
    document.addEventListener('pointerdown', onPointerDown)
    document.addEventListener('keydown', onKeyDown)
    return () => {
      document.removeEventListener('pointerdown', onPointerDown)
      document.removeEventListener('keydown', onKeyDown)
    }
  }, [open])
  return (
    <div className="menu-root" ref={root}>
      <button
        type="button"
        className={open ? 'menu-button active' : 'menu-button'}
        aria-haspopup="menu"
        aria-expanded={open}
        disabled={disabled}
        onClick={() => setOpen((value) => !value)}
      >
        {label}
      </button>
      {open && (
        <div className="menu-dropdown" role="menu">
          {typeof children === 'function' ? children(() => setOpen(false)) : children}
        </div>
      )}
    </div>
  )
}

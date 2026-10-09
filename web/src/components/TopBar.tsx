import { useCallback, useEffect, useRef, useState } from 'react'
import type { FormEvent, ReactElement } from 'react'
import { listProjects, newProject, openProject, saveProject, saveProjectAs } from '../store'
import type { CommandResult } from '../store'
import { Modal } from './Modal'

type DialogKind = 'new' | 'load' | 'saveAs'

/** The main views, in order (App renders the matching one). */
export const VIEWS = ['Editor', 'Aero Analysis', 'VLM', 'SIM', 'CAD', 'Part Editor'] as const
export type View = (typeof VIEWS)[number]

interface TopBarProps {
  view: View
  onView: (view: View) => void
}

/**
 * The Tk menu bar: File > New, Load, Save, Save As (guibuilder.BuilderApp.build_menu), with the main-view
 * buttons beside it instead of a separate bottom bar.
 */
export function TopBar({ view, onView }: TopBarProps): ReactElement {
  const [menuOpen, setMenuOpen] = useState(false)
  const [dialog, setDialog] = useState<DialogKind | null>(null)
  const [notice, setNotice] = useState('')
  const barRef = useRef<HTMLElement | null>(null)

  useEffect(() => {
    if (!menuOpen) return undefined
    const onPointerDown = (event: PointerEvent): void => {
      const bar = barRef.current
      if (bar !== null && event.target instanceof Node && !bar.contains(event.target)) setMenuOpen(false)
    }
    const onKeyDown = (event: KeyboardEvent): void => {
      if (event.key === 'Escape') setMenuOpen(false)
    }
    document.addEventListener('pointerdown', onPointerDown)
    window.addEventListener('keydown', onKeyDown)
    return () => {
      document.removeEventListener('pointerdown', onPointerDown)
      window.removeEventListener('keydown', onKeyDown)
    }
  }, [menuOpen])

  const choose = (kind: DialogKind): void => {
    setMenuOpen(false)
    setNotice('')
    setDialog(kind)
  }

  const save = (): void => {
    setMenuOpen(false)
    setNotice('')
    // A failed save is a logged snapshot (4xx/5xx); show its last log text here so it is never silent.
    void saveProject().then((result) => {
      if (!result.ok) setNotice(result.message)
    })
  }

  return (
    <header className="top-bar" ref={barRef}>
      <div className="menu-root">
        <button
          type="button"
          className={menuOpen ? 'menu-button active' : 'menu-button'}
          aria-haspopup="menu"
          aria-expanded={menuOpen}
          onClick={() => setMenuOpen((open) => !open)}
        >
          File
        </button>
        {menuOpen && (
          <div className="menu-dropdown" role="menu">
            <button type="button" role="menuitem" onClick={() => choose('new')}>
              New…
            </button>
            <button type="button" role="menuitem" onClick={() => choose('load')}>
              Load…
            </button>
            <button type="button" role="menuitem" onClick={save}>
              Save
            </button>
            <button type="button" role="menuitem" onClick={() => choose('saveAs')}>
              Save As…
            </button>
          </div>
        )}
      </div>
      <nav className="view-tabs">
        {VIEWS.map((name) => (
          <button
            key={name}
            type="button"
            className={name === view ? 'view-button active' : 'view-button'}
            onClick={() => onView(name)}
          >
            {name}
          </button>
        ))}
      </nav>
      {notice !== '' && <span className="top-notice">{notice}</span>}
      {dialog === 'new' && <NameModal title="New Project" submitLabel="Create" action={newProject} onClose={() => setDialog(null)} />}
      {dialog === 'saveAs' && <NameModal title="Save Project As" submitLabel="Save As" action={saveProjectAs} onClose={() => setDialog(null)} />}
      {dialog === 'load' && <LoadModal onClose={() => setDialog(null)} />}
    </header>
  )
}

interface NameModalProps {
  title: string
  submitLabel: string
  action: (name: string) => Promise<CommandResult>
  onClose: () => void
}

function NameModal({ title, submitLabel, action, onClose }: NameModalProps): ReactElement {
  const [name, setName] = useState('')
  const [error, setError] = useState('')
  const [busy, setBusy] = useState(false)

  const submit = (event: FormEvent<HTMLFormElement>): void => {
    event.preventDefault()
    if (name === '' || busy) return
    setBusy(true)
    void action(name).then((result) => {
      setBusy(false)
      if (result.ok) onClose()
      else setError(result.message)
    })
  }

  return (
    <Modal title={title} onClose={onClose}>
      <form className="modal-form" onSubmit={submit}>
        <label className="modal-label" htmlFor="project-name">
          Project name
        </label>
        <input
          id="project-name"
          value={name}
          autoFocus
          spellCheck={false}
          onChange={(event) => {
            setName(event.target.value)
            setError('')
          }}
        />
        {error !== '' && <div className="modal-error">{error}</div>}
        <div className="modal-actions">
          <button type="button" onClick={onClose}>
            Cancel
          </button>
          <button type="submit" className="action" disabled={busy || name === ''}>
            {busy ? 'Working…' : submitLabel}
          </button>
        </div>
      </form>
    </Modal>
  )
}

function LoadModal({ onClose }: { onClose: () => void }): ReactElement {
  const [projects, setProjects] = useState<string[] | null>(null)
  const [error, setError] = useState('')
  const [busy, setBusy] = useState('')

  const load = useCallback((): void => {
    setProjects(null)
    setError('')
    void listProjects().then((names) => {
      setProjects(names ?? [])
      if (names === null) setError('Could not list projects.')
    })
  }, [])

  useEffect(() => {
    load()
  }, [load])

  const open = (name: string): void => {
    if (busy !== '') return
    setBusy(name)
    void openProject(name).then((result) => {
      setBusy('')
      if (result.ok) onClose()
      else setError(result.message)
    })
  }

  return (
    <Modal title="Load Project" onClose={onClose}>
      <div className="modal-projects">
        {projects === null ? (
          <div className="modal-note">Loading…</div>
        ) : projects.length === 0 ? (
          <div className="modal-note">No projects found.</div>
        ) : (
          <div className="project-list">
            {projects.map((name) => (
              <button key={name} type="button" disabled={busy !== ''} onClick={() => open(name)}>
                {name}
              </button>
            ))}
          </div>
        )}
      </div>
      {error !== '' && <div className="modal-error">{error}</div>}
      <div className="modal-actions">
        <button type="button" onClick={load} disabled={busy !== ''}>
          Refresh
        </button>
        <button type="button" onClick={onClose}>
          Cancel
        </button>
      </div>
    </Modal>
  )
}

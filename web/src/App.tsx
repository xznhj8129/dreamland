import { useEffect, useState } from 'react'
import type { ReactElement } from 'react'
import { recompute, startClient, useSnapshot } from './store'
import { TopBar } from './components/TopBar'
import { EditorView } from './views/EditorView'
import { PartEditorView } from './parteditor/PartEditorView'
import { PlaceholderView } from './views/PlaceholderView'

const VIEWS = ['Editor', 'Aero Analysis', 'VLM', 'SIM', 'CAD', 'Part Editor'] as const
type View = (typeof VIEWS)[number]

export function App(): ReactElement {
  const snapshot = useSnapshot()
  const [view, setView] = useState<View>('Editor')

  useEffect(() => {
    startClient()
  }, [])

  useEffect(() => {
    document.title = snapshot?.title ?? 'quadrocketbuilder'
  }, [snapshot])

  useEffect(() => {
    if (view !== 'Editor') return undefined
    const onKeyDown = (event: KeyboardEvent) => {
      if (event.key === 'F5') {
        event.preventDefault()
        void recompute()
      }
    }
    window.addEventListener('keydown', onKeyDown)
    return () => window.removeEventListener('keydown', onKeyDown)
  }, [view])

  return (
    <div className="app">
      <TopBar />
      <main className="view-host">
        {view === 'Editor' ? (
          <EditorView snapshot={snapshot} />
        ) : view === 'Part Editor' ? (
          <PartEditorView />
        ) : (
          <PlaceholderView name={view} />
        )}
      </main>
      <nav className="view-bar">
        {VIEWS.map((name) => (
          <button
            key={name}
            type="button"
            className={name === view ? 'view-button active' : 'view-button'}
            onClick={() => setView(name)}
          >
            {name}
          </button>
        ))}
      </nav>
    </div>
  )
}

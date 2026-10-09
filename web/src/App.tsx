import { useEffect, useState } from 'react'
import type { ReactElement } from 'react'
import { recompute, startClient, useSnapshot } from './store'
import { TopBar } from './components/TopBar'
import type { View } from './components/TopBar'
import { EditorView } from './views/EditorView'
import { PartEditorView } from './parteditor/PartEditorView'
import { CadView } from './views/CadView'
import { SimView } from './views/SimView'
import { PlaceholderView } from './views/PlaceholderView'

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
      <TopBar view={view} onView={setView} />
      <main className="view-host">
        {view === 'Editor' ? (
          <EditorView snapshot={snapshot} />
        ) : view === 'Part Editor' ? (
          <PartEditorView />
        ) : view === 'CAD' ? (
          <CadView />
        ) : view === 'SIM' ? (
          <SimView />
        ) : (
          <PlaceholderView name={view} />
        )}
      </main>
    </div>
  )
}

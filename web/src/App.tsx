import { useEffect, useState } from 'react'
import type { ReactElement } from 'react'
import { recompute, startClient, useSnapshot } from './store'
import { TopBar } from './components/TopBar'
import type { View } from './components/TopBar'
import { EditorView } from './views/EditorView'
import { PartEditorView } from './parteditor/PartEditorView'
import { CadView } from './views/CadView'
import { SimView } from './views/SimView'
import { AeroAnalysisView } from './views/AeroAnalysisView'
import { PlaceholderView } from './views/PlaceholderView'

export function App(): ReactElement {
  const snapshot = useSnapshot()
  const [view, setView] = useState<View>('Editor')

  useEffect(() => {
    startClient()
  }, [])

  useEffect(() => {
    document.title = snapshot?.title ?? 'Dreamland'
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
      <TopBar
        view={view}
        onView={setView}
        project={snapshot?.project ?? null}
        dirty={snapshot?.dirty ?? false}
        aircraftName={typeof snapshot?.definition.name === 'string' ? snapshot.definition.name : null}
        status={snapshot?.status ?? null}
        geometry={snapshot?.geometry ?? null}
      />
      <main className="view-host">
        {view === 'Editor' ? (
          <EditorView snapshot={snapshot} />
        ) : view === 'Part Editor' ? (
          <PartEditorView />
        ) : view === 'CAD' ? (
          <CadView />
        ) : view === 'SIM' ? (
          <SimView />
        ) : view === 'Aero Analysis' ? (
          <AeroAnalysisView />
        ) : (
          <PlaceholderView name={view} />
        )}
      </main>
    </div>
  )
}

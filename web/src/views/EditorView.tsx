import type { ReactElement } from 'react'
import type { Snapshot } from '../types'
import { recompute } from '../store'
import { computeGeometry } from '../cadApi'
import { useRememberedState } from '../viewMemory'
import { ModelView } from '../components/ModelView'
import { ReportPanel } from '../components/ReportPanel'
import { SimpleView } from '../components/SimpleView'
import { SplitPane } from '../components/SplitPane'
import { EditorPanel } from '../editor/EditorPanel'
import { SelectionProvider } from '../editor/selection'

export function EditorView({ snapshot }: { snapshot: Snapshot | null }): ReactElement {
  return (
    <SelectionProvider key={snapshot?.session_id ?? -1}><SplitPane>
      <section className="panel left-pane">
        <div className="pane-header">
          <button type="button" className="action" onClick={() => void recompute()}>
            Recompute (F5)
          </button>
          {/* Builds every CAD solid and measures the exact mass in the CAD worker; its status is in the top bar. */}
          <button
            type="button"
            className="action"
            disabled={snapshot === null || snapshot.geometry === 'computing' || snapshot.geometry === 'current'}
            onClick={() => void computeGeometry()}
          >
            Compute Geometry
          </button>
        </div>
        {snapshot !== null && !snapshot.definition_valid && <p className="last-valid-note">Showing the last valid analysis. See Log for definition errors.</p>}
        <ReportPanel pages={snapshot?.report_pages ?? null} log={snapshot?.log ?? []} inspectEnabled={snapshot?.definition_valid ?? false} />
      </section>
      <section className="panel centre-pane">
        <DrawingTabs snapshot={snapshot} />
      </section>
      <section className="panel right-pane">
        <EditorPanel snapshot={snapshot} />
      </section>
    </SplitPane></SelectionProvider>
  )
}

function DrawingTabs({ snapshot }: { snapshot: Snapshot | null }): ReactElement {
  const [tab, setTab] = useRememberedState<'simple' | '3d'>('editor.drawing-tab', 'simple', value => value === 'simple' || value === '3d')
  return (
    <>
      <div className="tab-row">
        <button type="button" className={tab === 'simple' ? 'tab active' : 'tab'} onClick={() => setTab('simple')}>
          Simple
        </button>
        <button type="button" className={tab === '3d' ? 'tab active' : 'tab'} onClick={() => setTab('3d')}>
          3D
        </button>
      </div>
      {/* ModelView stays mounted (hidden) so the canvas and camera survive tab switches and form commits. */}
      {tab === 'simple' && <SimpleView snapshot={snapshot} />}
      <ModelView snapshot={snapshot} visible={tab === '3d'} />
    </>
  )
}

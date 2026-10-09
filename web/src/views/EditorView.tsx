import { useState } from 'react'
import type { ReactElement } from 'react'
import type { Snapshot } from '../types'
import { recompute } from '../store'
import { ModelView } from '../components/ModelView'
import { ReportPanel } from '../components/ReportPanel'
import { SimpleView } from '../components/SimpleView'
import { SplitPane } from '../components/SplitPane'
import { EditorPanel } from '../editor/EditorPanel'

export function EditorView({ snapshot }: { snapshot: Snapshot | null }): ReactElement {
  return (
    <SplitPane>
      <section className="panel left-pane">
        <div className="pane-header">
          <button type="button" className="action" onClick={() => void recompute()}>
            Recompute (F5)
          </button>
          <span className="status">{snapshot?.status ?? 'Connecting...'}</span>
        </div>
        <ReportPanel pages={snapshot?.report_pages ?? null} log={snapshot?.log ?? []} />
      </section>
      <section className="panel centre-pane">
        <DrawingTabs snapshot={snapshot} />
      </section>
      <section className="panel right-pane">
        <EditorPanel snapshot={snapshot} />
      </section>
    </SplitPane>
  )
}

function DrawingTabs({ snapshot }: { snapshot: Snapshot | null }): ReactElement {
  const [tab, setTab] = useState<'simple' | '3d'>('simple')
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

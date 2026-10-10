import { useCallback, useEffect, useRef, useState } from 'react'
import type { ReactElement } from 'react'
import type { EditorContext, Path, PathEdit, Snapshot } from '../types'
import { editorEdit, editorRemove, editorSet } from '../store'
import { AirframeTab } from './AirframeTab'
import { loadEditorContext } from './context-cache'
import { FuselageTab } from './FuselageTab'
import { MountsTab } from './MountsTab'
import { PartsTab } from './PartsTab'
import { PropulsionTab } from './PropulsionTab'
import { SectionsTab } from './SectionsTab'
import { FormApiContext } from './session'
import type { EditorSession, FormApi } from './session'
import { SurfacesTab } from './SurfacesTab'
import { VolumesTab } from './VolumesTab'
import { WiringTab } from './WiringTab'
import { useSelection } from './selection'

const DESIGN_TABS = ['Airframe', 'Fuselage', 'Sections', 'Surfaces', 'Volumes', 'Motor mounts', 'Optimization'] as const
const BUILD_TABS = ['Propulsion', 'Parts', 'Wiring'] as const

const SELECTION_TABS = { surface: 'Surfaces', volume: 'Volumes', mount: 'Motor mounts', section: 'Sections', fuselage: 'Fuselage' } as const
type Group = 'Design' | 'Build'

/** Aircraft forms grouped into Design and Build; Optimization is a planned evaluator view. */
export function EditorPanel({ snapshot }: { snapshot: Snapshot | null }): ReactElement {
  const { active } = useSelection()
  const [group, setGroup] = useState<Group>('Design')
  const [designTab, setDesignTab] = useState<string>(DESIGN_TABS[0])
  const [buildTab, setBuildTab] = useState<string>(BUILD_TABS[0])
  const [context, setContext] = useState<EditorContext | null>(null)

  useEffect(() => {
    if (active === null) return
    if (active.kind === 'part' || active.kind === 'battery') {
      setGroup('Build')
      setBuildTab(active.kind === 'part' ? 'Parts' : 'Propulsion')
    } else {
      setGroup('Design')
      setDesignTab(SELECTION_TABS[active.kind])
    }
  }, [active])

  // Loads once the snapshot is known, and reloads when a part-library save/delete moves library_revision:
  // loadEditorContext refetches only when the revision it is asked under differs from the cached one.
  const libraryRevision = snapshot?.library_revision
  useEffect(() => {
    if (libraryRevision === undefined) return undefined
    let alive = true
    void loadEditorContext().then((loaded) => {
      if (alive && loaded !== null) setContext(loaded)
    })
    return () => {
      alive = false
    }
  }, [libraryRevision])

  const refreshContext = useCallback(async (): Promise<void> => {
    const loaded = await loadEditorContext(true)
    if (loaded !== null) setContext(loaded)
  }, [])

  const tabs: readonly string[] = group === 'Design' ? DESIGN_TABS : BUILD_TABS
  const tab = group === 'Design' ? designTab : buildTab
  const chooseTab = (name: string): void => {
    if (group === 'Design') setDesignTab(name)
    else setBuildTab(name)
  }

  return (
    <div className="editor-panel">
      <div className="tab-row">
        <button type="button" className={group === 'Design' ? 'tab active' : 'tab'} onClick={() => setGroup('Design')}>
          Design
        </button>
        <button type="button" className={group === 'Build' ? 'tab active' : 'tab'} onClick={() => setGroup('Build')}>
          Build
        </button>
      </div>
      <div className="subtab-row">
        {tabs.map((name) => (
          <button key={name} type="button" className={name === tab ? 'subtab active' : 'subtab'} onClick={() => chooseTab(name)}>
            {name}
          </button>
        ))}
      </div>
      {snapshot === null || context === null ? (
        <div className="placeholder">{snapshot === null ? 'Connecting…' : 'Loading editor data…'}</div>
      ) : (
        <EditorForms key={snapshot.session_id} snapshot={snapshot} context={context} group={group} tab={tab} onPresetsChanged={refreshContext} />
      )}
    </div>
  )
}

interface EditorFormsProps {
  snapshot: Snapshot
  context: EditorContext
  group: Group
  tab: string
  onPresetsChanged: () => Promise<void>
}

/**
 * The forms subtree, keyed on session_id by the caller: a project switch remounts it (fresh field text and
 * stashes, as Tk's reset_definition_state), while ordinary definition edits only update the values. Every
 * migrated tab stays mounted, hidden, so their field text, scroll and local picks survive tab switches.
 *
 * Field state derives from snapshot.definition (never a local working copy): each commit is one queued atomic
 * set carrying this session, and the store drops stale responses, so a slow edit cannot overwrite a later one
 * or leak into the next project. Structural picks (layout, mount on, shape, kind, rename, volume change) go
 * through editorOp instead, which owns the change and its side effects.
 */
function EditorForms({ snapshot, context, group, tab, onPresetsChanged }: EditorFormsProps): ReactElement {
  const sessionId = snapshot.session_id
  const stashRef = useRef<EditorSession>({ stashedPropulsion: undefined, stashedReinforcement: undefined, stashedBatteries: {} })
  const set = useCallback((path: Path, value: unknown) => editorSet(sessionId, path, value), [sessionId])
  const remove = useCallback((path: Path) => editorRemove(sessionId, path), [sessionId])
  const edit = useCallback((edits: readonly PathEdit[]) => editorEdit(sessionId, edits), [sessionId])
  const api: FormApi = { definition: snapshot.definition, sessionId, context, stash: stashRef.current, set, remove, edit }
  const design = group === 'Design'
  return (
    <FormApiContext.Provider value={api}>
      <div className="editor-forms" hidden={!design || tab !== 'Airframe'}>
        <AirframeTab />
      </div>
      <div className="editor-forms" hidden={!design || tab !== 'Fuselage'}>
        <FuselageTab snapshot={snapshot} onPresetsChanged={onPresetsChanged} />
      </div>
      <div className="editor-forms" hidden={!design || tab !== 'Sections'}>
        <SectionsTab snapshot={snapshot} />
      </div>
      <div className="editor-forms" hidden={!design || tab !== 'Surfaces'}>
        <SurfacesTab snapshot={snapshot} />
      </div>
      <div className="editor-forms" hidden={!design || tab !== 'Volumes'}>
        <VolumesTab />
      </div>
      <div className="editor-forms" hidden={!design || tab !== 'Motor mounts'}>
        <MountsTab />
      </div>
      <div className="editor-forms" hidden={!design || tab !== 'Optimization'}>
        <h3>Optimization</h3>
        <p className="form-hint">Planned: choose an objective and ranges on part values, then evaluate candidates in the simulator.</p>
      </div>
      <div className="editor-forms" hidden={design || tab !== 'Propulsion'}>
        <PropulsionTab snapshot={snapshot} />
      </div>
      <div className="editor-forms" hidden={design || tab !== 'Parts'}>
        <PartsTab />
      </div>
      <div className="editor-forms" hidden={design || tab !== 'Wiring'}>
        <WiringTab snapshot={snapshot} />
      </div>
    </FormApiContext.Provider>
  )
}

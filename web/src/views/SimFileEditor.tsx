import { useEffect, useState } from 'react'
import type { ReactElement } from 'react'
import { deleteSimResource, simResource } from '../simApi'
import type { ScenarioOutline, SimResource } from '../simApi'

interface Props {
  controller: string
  kind: 'scenario' | 'diff'
  name: string | null
  project: string | null
  template: string
  templateOutline?: ScenarioOutline | null
  running: boolean
  visible: boolean
  revision?: number
  onDirty: (dirty: boolean) => void
  onLoad: (resource: SimResource) => void
  onDiscard: () => void
  onSaved: (name: string, resource: SimResource) => void
  /** Called after the file is gone, so the caller can drop the selection and refresh the option lists. */
  onDeleted?: (name: string) => void
}

export function SimFileEditor({controller, kind, name, project, template, templateOutline, running, visible, revision, onDirty, onLoad, onSaved, onDiscard, onDeleted}: Props): ReactElement {
  const [source, setSource] = useState('')
  const [saved, setSaved] = useState('')
  const [filename, setFilename] = useState('')
  const [readonly, setReadonly] = useState(false)
  const [note, setNote] = useState('')
  const [error, setError] = useState('')
  const [busy, setBusy] = useState(false)
  const dirty = source !== saved || name === '__new__'
  useEffect(() => { onDirty(dirty) }, [dirty, onDirty])
  useEffect(() => {
    let alive = true
    setError('')
    setSource('')
    setSaved('')
    setReadonly(false)
    setBusy(false)
    if (name === '__new__') {
      setSource(template)
      setFilename(kind === 'scenario' ? 'scenario.py' : 'custom.txt')
      setNote('New project file')
      onLoad({source: template, readonly: false, scenario: templateOutline, note: 'New project file'})
      return
    }
    if (name === null) { setNote('Choose or create a scenario.'); return }
    setBusy(true)
    void simResource(controller, kind, name).then(resource => {
      if (!alive) return
      setSource(resource.source); setSaved(resource.source); setReadonly(resource.readonly); setNote(resource.note)
      setFilename(kind === 'diff' && resource.readonly ? 'custom.txt' : name)
      setError(resource.error ?? '')
      onLoad(resource)
    }).catch((reason: Error) => { if (alive) setError(reason.message) }).finally(() => { if (alive) setBusy(false) })
    return () => { alive = false }
  }, [controller, kind, name, project, revision]) // callbacks do not reload an edited file

  const save = async (copy: boolean): Promise<void> => {
    setBusy(true); setError('')
    try {
      const target = copy || name === '__new__' ? filename : name
      if (target === null) return
      const resource = await simResource(controller, kind, target, source, copy || name === '__new__')
      setSaved(source); setReadonly(false); setNote(resource.note)
      onSaved(target, resource)
    } catch (reason) { setError(reason instanceof Error ? reason.message : String(reason)) }
    finally { setBusy(false) }
  }

  const remove = async (): Promise<void> => {
    if (name === null || name === '__new__') return
    if (!window.confirm(`Delete ${name}?`)) return
    setBusy(true); setError('')
    try {
      await deleteSimResource(controller, kind, name)
      onDeleted?.(name)
    } catch (reason) { setError(reason instanceof Error ? reason.message : String(reason)) }
    finally { setBusy(false) }
  }

  return <div className="sim-source-panel" style={{display: visible ? 'flex' : 'none'}}>
    <div className="sim-source-toolbar">
      <span>{dirty ? 'Unsaved' : 'Saved'} · {note}</span>
      <label>Filename <input aria-label={`${kind} filename`} className="field-input" value={filename} onChange={event => setFilename(event.target.value)} /></label>
      <button type="button" disabled={running || busy || readonly || name === null || !dirty} onClick={() => void save(false)}>Save {kind}</button>
      <button type="button" disabled={running || busy || !dirty} onClick={() => { setSource(saved); onDiscard() }}>Discard edits</button>
      <button type="button" disabled={running || busy || name === null || name === '__new__'} onClick={() => void save(true)}>Save copy</button>
      {kind === 'diff' && name !== null && name !== '__new__' && !readonly &&
        <button type="button" disabled={running || busy} onClick={() => void remove()}>Delete diff</button>}
    </div>
    {kind === 'scenario' && <p className="form-hint">Every script defines run(flight) and imports its controller. Set initial conditions in the toolbar. PARAMETERS and TELEMETRY are optional. All times are simulated seconds. Use print() for freeform status lines; measure() records a point in the Measurements file and prints a brief line.</p>}
    {error !== '' && <p className="field-error" role="alert">{error}</p>}
    <textarea aria-label={kind === 'scenario' ? 'Python scenario' : 'INAV diff'} className="sim-source" spellCheck={false} value={source} disabled={busy || name === null} readOnly={readonly || running}
      onChange={event => setSource(event.target.value)} onKeyDown={event => {
        if (event.key === 'Tab' && !readonly && !running) {
          event.preventDefault()
          const node = event.currentTarget
          const start = node.selectionStart
          setSource(source.slice(0, start) + '    ' + source.slice(node.selectionEnd))
          requestAnimationFrame(() => { node.selectionStart = node.selectionEnd = start + 4 })
        }
      }} />
  </div>
}

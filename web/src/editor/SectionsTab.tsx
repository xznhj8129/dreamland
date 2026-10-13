import { useState } from 'react'
import type { ReactElement } from 'react'
import { editorOp } from '../store'
import type { ResolvedSection, Snapshot } from '../types'
import { Card, CheckboxField, ChoiceField, EntryField, Fields, HelpNote } from './fields'
import { formatValue } from './format'
import { isObject } from './patch'
import { ComponentPicker, useComponentIndex, useSelection } from './selection'
import { useFormApi } from './session'

function itemisedLabel(section: ResolvedSection): string {
  const [low, high] = section.complete_itemised_kg
  return low === high ? `${(low * 1000).toFixed(1)} g` : `${(low * 1000).toFixed(1)}–${(high * 1000).toFixed(1)} g`
}

export function SectionsTab({ snapshot }: { snapshot: Snapshot }): ReactElement {
  const api = useFormApi()
  const sections = isObject(api.definition.sections) ? api.definition.sections : {}
  const cuts = Array.isArray(sections.cuts_m) ? sections.cuts_m : []
  const fuselage = isObject(api.definition.fuselage) ? api.definition.fuselage : {}
  const joints = isObject(sections.joints) ? sections.joints : {}
  const boundaries = [0, ...cuts, fuselage.length_m].map(Number)
  const cutJoints = Array.isArray(sections.cut_joints) ? sections.cut_joints : []
  const rows = Array.isArray(sections.per_section) ? sections.per_section.filter(isObject) : []
  const resolved = snapshot.definition_valid && snapshot.resolved?.sections.length === rows.length ? snapshot.resolved.sections : []
  const selected = useComponentIndex('section', rows.length)
  const { select } = useSelection()
  const [notice, setNotice] = useState('')
  async function run(body: Record<string, unknown>): Promise<void> {
    const result = await editorOp(api.sessionId, body)
    setNotice(result.ok ? '' : result.message)
    if (result.ok && body.op === 'section.add_cut') select('section', selected + (body.side === 'after' ? 1 : 0))
  }
  return <>
    <Card title="Fuselage sections">
      <HelpNote>Select a section to edit its boundaries, joint and mass. Cuts do not change the fuselage shape.</HelpNote>
      {notice && <div className="field-error">{notice}</div>}
    </Card>
    <ComponentPicker kind="section" items={rows.map((row, index) => ({ name: `S${index + 1}`, detail: `${formatValue('length_m', boundaries[index + 1]! - boundaries[index]!, api.context)} mm · ${row.solid === true ? 'Solid' : 'Hollow'}` }))} />
    <div className="button-row" role="group" aria-label="Add section">
      <button type="button" disabled={!snapshot.definition_valid || rows.length === 0}
        onClick={() => void run({ op: 'section.add_cut', index: selected, side: 'before' })}>Add before</button>
      <button type="button" disabled={!snapshot.definition_valid || rows.length === 0}
        onClick={() => void run({ op: 'section.add_cut', index: selected, side: 'after' })}>Add after</button>
    </div>
    <HelpNote>Splits the selected span in half; airframe length and loft stay unchanged.</HelpNote>
    {rows.map((row, index) => <div key={index} hidden={index !== selected}>
      <Card title={`S${index + 1}`}>
        {index > 0 ? <EntryField path={['sections','cuts_m',index - 1]} unitKey="cuts_m" label="Start (mm)" />
          : <div className="form-row"><label className="form-label" htmlFor="section-nose-start">Start (mm)</label><div className="form-control"><input id="section-nose-start" className="field-input" value="0" readOnly /></div><span className="form-hint">Nose origin</span></div>}
        <EntryField path={index < cuts.length ? ['sections','cuts_m',index] : ['fuselage','length_m']} unitKey="length_m" label="End (mm)" />
        <HelpNote>A shared boundary also changes the adjoining section.</HelpNote>
        <EntryField path={['sections','cuts_m',index]} unitKey="length_m" label="Length (mm)"
          derivedValue={boundaries[index + 1]! - boundaries[index]!}
          onValue={value => void run({ op: 'section.length', index, value })} />
        <CheckboxField path={['sections','per_section',index,'fixed_length']} checked={row.fixed_length === true} label="Fix length" />
        <HelpNote>Fix preserves length during automatic resizing. Editing length keeps the airframe length and resizes the other unfixed sections proportionally.</HelpNote>
        {index < cuts.length ? <>
          <ChoiceField path={['sections','cut_joints',index]} value={cutJoints[index]} options={api.context.cut_joints} label="Joint at end cut" />
        </> : null}
        <CheckboxField path={['sections','per_section',index,'solid']} checked={row.solid === true} label="Solid section" />
        <EntryField path={['sections','per_section',index,'empty_mass_kg']} unitKey="empty_mass_kg" label="Empty mass (g)" optional
          defaultValue={resolved[index]?.empty_estimate_kg ?? undefined} defaultLabel="Geometry estimate" />
        <EntryField path={['sections','per_section',index,'complete_mass_kg']} unitKey="complete_mass_kg" label="Complete mass (g)" optional
          defaultValue={resolved[index]?.complete_itemised_kg} defaultLabel="Itemised components" />
        {resolved[index] && <p className="form-hint">Itemised total: {itemisedLabel(resolved[index])}. A complete weighed mass replaces all components attached in this section.</p>}
        {index < cuts.length && <button type="button" className="danger" onClick={() => void run({ op: 'section.delete_cut', index })}>Remove end cut & merge sections</button>}
      </Card>
    </div>)}
    <Card title="Joints">
      <Fields data={joints} path={['sections', 'joints']} context={api.context} />
    </Card>
    {!snapshot.definition_valid && <p className="field-error">Fix the definition errors in Log to refresh the section estimates.</p>}
  </>
}

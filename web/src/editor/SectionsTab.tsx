import { useState } from 'react'
import type { ReactElement } from 'react'
import { editorOp } from '../store'
import type { ResolvedSection, Snapshot } from '../types'
import { Card, CheckboxField, ChoiceField, EntryField, Group } from './fields'
import { isObject } from './patch'
import { ComponentPicker, useComponentIndex } from './selection'
import { useFormApi } from './session'

const SECTION_PROFILE_KEYS = ['width_m', 'height_m', 'shape']

function itemisedLabel(section: ResolvedSection): string {
  const [low, high] = section.complete_itemised_kg
  return low === high ? `${(low * 1000).toFixed(1)} g` : `${(low * 1000).toFixed(1)}–${(high * 1000).toFixed(1)} g`
}

export function SectionsTab({ snapshot }: { snapshot: Snapshot }): ReactElement {
  const api = useFormApi()
  const sections = isObject(api.definition.sections) ? api.definition.sections : {}
  const cuts = Array.isArray(sections.cuts_m) ? sections.cuts_m : []
  const fuselage = isObject(api.definition.fuselage) ? api.definition.fuselage : {}
  const defaultSection = isObject(fuselage.section) ? fuselage.section : {}
  const boundaries = [0, ...cuts, fuselage.length_m].map(Number)
  const cutJoints = Array.isArray(sections.cut_joints) ? sections.cut_joints : []
  const rows = Array.isArray(sections.per_section) ? sections.per_section.filter(isObject) : []
  const resolved = snapshot.definition_valid && snapshot.resolved?.sections.length === rows.length ? snapshot.resolved.sections : []
  const selected = useComponentIndex('section', rows.length)
  const [notice, setNotice] = useState('')
  async function run(body: Record<string, unknown>): Promise<void> {
    const result = await editorOp(api.sessionId, body)
    setNotice(result.ok ? '' : result.message)
  }
  return <>
    <Card title="Fuselage sections">
      <p className="form-hint">Select a section to edit its boundaries, cross-section, joint and mass.</p>
      {notice && <div className="field-error">{notice}</div>}
    </Card>
    <ComponentPicker kind="section" items={rows.map((row, index) => ({ name: `S${index + 1}`, detail: row.solid === true ? 'Solid' : 'Hollow' }))} />
    {rows.map((row, index) => <div key={index} hidden={index !== selected}>
      <Card title={`S${index + 1}`}>
        {index > 0 ? <EntryField path={['sections','cuts_m',index - 1]} unitKey="cuts_m" label="Start (mm)" />
          : <div className="form-row"><label className="form-label" htmlFor="section-nose-start">Start (mm)</label><div className="form-control"><input id="section-nose-start" className="field-input" value="0" readOnly /></div><span className="form-hint">Nose origin</span></div>}
        <EntryField path={index < cuts.length ? ['sections','cuts_m',index] : ['fuselage','length_m']} unitKey="length_m" label="End (mm)" />
        <p className="form-hint">A shared boundary also changes the adjoining section.</p>
        <EntryField path={['sections','cuts_m',index]} unitKey="length_m" label="Length (mm)"
          derivedValue={boundaries[index + 1]! - boundaries[index]!}
          onValue={value => void run({ op: 'section.length', index, value })} />
        <CheckboxField path={['sections','per_section',index,'fixed_length']} checked={row.fixed_length === true} label="Fix length" />
        <p className="form-hint">Fix preserves length during automatic resizing. Editing length keeps the airframe length and resizes the other unfixed sections proportionally.</p>
        <Group label="Loft cross-section">
          {SECTION_PROFILE_KEYS.map(key => <EntryField key={key}
            path={['sections','per_section',index,'section',key]} unitKey={key}
            label={key === 'shape' ? 'Shape exponent' : key === 'width_m' ? 'Width (mm)' : 'Height (mm)'} optional
            defaultValue={defaultSection[key]} defaultLabel="Fuselage default" />)}
          <p className="form-hint">Cross-section at this section’s midpoint ({((boundaries[index]! + boundaries[index + 1]!) * 500).toFixed(1)} mm), before nose/tail taper. Smooth transitions join adjoining midpoints. Shape: 1 = diamond, 2 = ellipse, 4 = rounded rectangle. Wall thickness stays global.</p>
        </Group>
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
    {!snapshot.definition_valid && <p className="field-error">Fix the definition errors in Log to refresh the section estimates.</p>}
  </>
}

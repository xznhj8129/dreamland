import { useState } from 'react'
import type { ReactElement } from 'react'
import { editorOp } from '../store'
import { Card, EntryField, HelpNote } from './fields'
import { formatValue } from './format'
import { isObject } from './patch'
import { ComponentPicker, useComponentIndex, useSelection } from './selection'
import { useFormApi } from './session'

const PROFILE_KEYS = ['width_m', 'height_m', 'shape']

export function LoftGuides(): ReactElement {
  const api = useFormApi()
  const fuselage = isObject(api.definition.fuselage) ? api.definition.fuselage : {}
  const defaults = isObject(fuselage.section) ? fuselage.section : {}
  const guides = Array.isArray(fuselage.loft_stations) ? fuselage.loft_stations.filter(isObject) : []
  const selected = useComponentIndex('loft', guides.length)
  const { select } = useSelection()
  const [position, setPosition] = useState('')
  const [notice, setNotice] = useState('')

  async function run(body: Record<string, unknown>, index?: number): Promise<void> {
    const result = await editorOp(api.sessionId, body)
    setNotice(result.ok ? '' : result.message)
    if (result.ok && index !== undefined) select('loft', index)
  }

  const add = (): void => {
    const x = Number(position) / 1000
    void run({ op: 'loft.add', x_m: x }, guides.filter(guide => Number(guide.x_m) < x).length)
  }

  return <Card title="Loft guides">
    <HelpNote>Shape profiles at independent positions, before nose/tail taper. Guides do not cut the fuselage. Smooth transitions join them; no guides uses the default cross-section throughout.</HelpNote>
    <div className="form-row">
      <label className="form-label" htmlFor="new-loft-position">New guide position (mm)</label>
      <div className="form-control"><input id="new-loft-position" className="field-input" type="number" step="any" value={position} onChange={event => setPosition(event.target.value)} /></div>
    </div>
    <button type="button" onClick={add} disabled={position.trim() === ''}>Add guide</button>
    {notice && <div className="field-error">{notice}</div>}
    <ComponentPicker kind="loft" items={guides.map((guide, index) => ({ name: `G${index + 1}`, detail: `${formatValue('x_m', guide.x_m, api.context)} mm` }))} />
    {guides.map((_, index) => <div key={index} hidden={index !== selected}>
      <h4>G{index + 1}</h4>
      <EntryField path={['fuselage', 'loft_stations', index, 'x_m']} unitKey="x_m" label="Position (mm)"
        onValue={value => {
          const x = Number(value)
          void run({ op: 'loft.position', index, x_m: x }, guides.filter((other, i) => i !== index && Number(other.x_m) < x).length)
        }} />
      {PROFILE_KEYS.map(key => <EntryField key={key} path={['fuselage', 'loft_stations', index, key]} unitKey={key}
        label={key === 'shape' ? 'Shape exponent' : key === 'width_m' ? 'Width (mm)' : 'Height (mm)'} optional
        defaultValue={defaults[key]} defaultLabel="Fuselage default" />)}
      <HelpNote>Shape: 1 = diamond, 2 = ellipse, 4 = rounded rectangle. Wall thickness stays global. New guides start with the current interpolated profile.</HelpNote>
      <button type="button" className="danger" onClick={() => void run({ op: 'loft.delete', index }, Math.max(0, index - 1))}>Remove guide</button>
    </div>)}
  </Card>
}

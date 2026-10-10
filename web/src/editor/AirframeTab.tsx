import type { ReactElement } from 'react'
import { editorOp } from '../store'
import { Card, CheckboxField, ChoiceField, EntryField, Fields, VectorField } from './fields'
import { fieldLabel } from './format'
import { isObject } from './patch'
import { newPropulsion } from './propulsion'
import { useFormApi } from './session'

/** editor_view.build_airframe_tab (552-575): name, notes, type flags, operating point, print settings. */
export function AirframeTab(): ReactElement {
  const api = useFormApi()
  const definition = api.definition
  const context = api.context
  const flags = context.type_flags
  const powered = flags.indexOf('powered')

  /**
   * editor_view.set_type_flag (577-593): each flag implies the ones after it; powered adds or removes its
   * propulsion block. The whole change is one server op, so a recompute never sees powered without a block (or
   * the reverse). The block removed by unticking powered is stashed here and restored on ticking it again.
   */
  const setTypeFlag = (key: string, checked: boolean): void => {
    const chosen = flags.map((flag) => (flag === key ? checked : definition[flag] === true))
    const index = flags.indexOf(key)
    if (chosen[index]) {
      for (let later = index + 1; later < flags.length; later += 1) chosen[later] = true
    } else {
      for (let earlier = 0; earlier < index; earlier += 1) chosen[earlier] = false
    }
    let install: unknown
    if (chosen[powered] && definition.propulsion === undefined) {
      // editor_view 588-589: restore the stash, or build new_propulsion (597-609) exactly as Tk does.
      install = api.stash.stashedPropulsion !== undefined ? api.stash.stashedPropulsion : newPropulsion(definition, context)
    } else if (!chosen[powered] && definition.propulsion !== undefined) {
      api.stash.stashedPropulsion = definition.propulsion
    }
    void editorOp(api.sessionId, { op: 'flags.set', key, checked, propulsion: install })
  }

  const operatingPoint = isObject(definition.operating_point) ? definition.operating_point : null
  const printSettings = isObject(definition.print) ? definition.print : null
  const materials = Object.keys(context.parts_data.material ?? {}).sort()

  return (
    <>
      <Card title="Airframe">
        <EntryField path={['name']} unitKey="name" />
        <EntryField path={['notes']} unitKey="notes" />
        <div className="form-row">
          <span className="form-label">type</span>
          <div className="check-row">
            {flags.map((flag) => (
              <CheckboxField key={flag} path={[flag]} checked={definition[flag] === true} label={flag} onToggle={(checked) => setTypeFlag(flag, checked)} />
            ))}
          </div>
        </div>
      </Card>
      {operatingPoint !== null && (
        <Card title="Operating point (aero and stability)">
          <Fields data={operatingPoint} path={['operating_point']} context={context} />
        </Card>
      )}
      {printSettings !== null && (
        <Card title="Print">
          <ChoiceField path={['print', 'material']} value={printSettings.material} options={materials} label={fieldLabel('material', context)} />
          <VectorField path={['print', 'printer_bed_m']} unitKey="printer_bed_m" axes={['x', 'y', 'z']} />
          <EntryField path={['print', 'bulk_density_kg_m3']} unitKey="bulk_density_kg_m3" optional />
        </Card>
      )}
    </>
  )
}

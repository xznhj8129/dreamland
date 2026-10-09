import { useState } from 'react'
import type { ReactElement } from 'react'
import { editorOp } from '../store'
import type { Snapshot } from '../types'
import { Card, CheckboxField, ChoiceField, EntryField, Fields, OptionalBlock } from './fields'
import { fieldLabel } from './format'
import { isObject } from './patch'
import { useFormApi } from './session'

/** editor_view.build_surfaces_tab (852-942): per-set card, generic fields, control surface, spars, root base. */
export function SurfacesTab({ snapshot }: { snapshot: Snapshot }): ReactElement {
  const api = useFormApi()
  const definition = api.definition
  const surfaces = Array.isArray(definition.surfaces) ? definition.surfaces.filter(isObject) : []
  const [notice, setNotice] = useState('')

  const run = async (body: Record<string, unknown>): Promise<void> => {
    const result = await editorOp(api.sessionId, body)
    setNotice(result.ok ? '' : result.message)
  }

  const numbers = new Map<string, number>()
  for (const placement of snapshot.resolved?.controls ?? []) {
    numbers.set(`${placement.surface}#${placement.panel_index}`, placement.number)
  }

  return (
    <>
      <Card title="Surface sets">
        <div className="button-row">
          <button type="button" onClick={() => void run({ op: 'surface.add' })}>
            Add surface set
          </button>
        </div>
        {notice !== '' && <div className="field-error">{notice}</div>}
      </Card>
      {surfaces.map((surface, index) => (
        <SurfaceCard key={index} index={index} surface={surface} numbers={numbers} onOp={run} />
      ))}
    </>
  )
}

interface SurfaceCardProps {
  index: number
  surface: Record<string, unknown>
  /** Resolved control numbers by `${surface name}#${panel index}` (build_control 891-896). */
  numbers: Map<string, number>
  onOp: (body: Record<string, unknown>) => Promise<void>
}

/** editor_view.build_surfaces_tab 855-865: one LabelFrame per set, titled with its name. */
function SurfaceCard({ index, surface, numbers, onOp }: SurfaceCardProps): ReactElement {
  const api = useFormApi()
  const context = api.context
  const path = ['surfaces', index]
  const layout = typeof surface.layout === 'string' ? surface.layout : ''
  // 857: fin_position is only meaningful for a single fin or three fins.
  const skip = ['mass_kg', 'control', 'spars', 'root_base', ...(layout === '1' || layout === '3' ? [] : ['fin_position'])]
  const control = isObject(surface.control) ? surface.control : null
  const spars = isObject(surface.spars) ? surface.spars : null
  const rootBase = isObject(surface.root_base) ? surface.root_base : null
  const materials = Object.keys(context.parts_data.material ?? {}).sort()

  return (
    <Card title={typeof surface.name === 'string' ? surface.name : `surface ${index + 1}`} onDelete={() => void onOp({ op: 'surface.delete', index })}>
      <Fields
        data={surface}
        path={path}
        context={context}
        skip={skip}
        // 858: a layout change fits the servos, keeping one per panel.
        onChoice={{ layout: (value) => void onOp({ op: 'surface.layout', index, layout: value }) }}
      />
      <EntryField path={[...path, 'mass_kg']} unitKey="mass_kg" optional />
      <OptionalBlock
        label="control surface"
        present={control !== null}
        onToggle={(checked) => void onOp({ op: 'surface.control.toggle', index, enabled: checked })}
      >
        {control !== null && (
          <ControlFields
            index={index}
            surfaceName={typeof surface.name === 'string' ? surface.name : ''}
            control={control}
            numbers={numbers}
            onOp={onOp}
          />
        )}
      </OptionalBlock>
      <OptionalBlock
        label="spars"
        present={spars !== null}
        onToggle={(checked) => void onOp({ op: 'surface.spars.toggle', index, enabled: checked })}
      >
        {spars !== null && (
          <>
            <ChoiceField
              path={[...path, 'spars', 'material']}
              value={spars.material}
              options={materials}
              label={fieldLabel('material', context)}
            />
            <Fields data={spars} path={[...path, 'spars']} context={context} skip={['material', 'carry_through']} />
            <OptionalBlock
              label="carry-through"
              present={isObject(spars.carry_through)}
              onToggle={(checked) => void onOp({ op: 'surface.carry_through.toggle', index, enabled: checked })}
            >
              <Fields data={isObject(spars.carry_through) ? spars.carry_through : {}} path={[...path, 'spars', 'carry_through']} context={context} />
            </OptionalBlock>
          </>
        )}
      </OptionalBlock>
      <OptionalBlock
        label="root base (not built yet)"
        present={rootBase !== null}
        onToggle={(checked) => void onOp({ op: 'surface.root_base.toggle', index, enabled: checked })}
      >
        {rootBase !== null && <Fields data={rootBase} path={[...path, 'root_base']} context={context} />}
      </OptionalBlock>
    </Card>
  )
}

interface ControlFieldsProps {
  index: number
  surfaceName: string
  control: Record<string, unknown>
  numbers: Map<string, number>
  onOp: (body: Record<string, unknown>) => Promise<void>
}

/** editor_view.build_control (867-897): kind and its fields, roles, deflection limits, one servo per panel. */
function ControlFields({ index, surfaceName, control, numbers, onOp }: ControlFieldsProps): ReactElement {
  const api = useFormApi()
  const context = api.context
  const path = ['surfaces', index, 'control']
  const kind = typeof control.kind === 'string' ? control.kind : ''
  const kindFields = context.control_kind_fields[kind] ?? {}
  const roles = Array.isArray(control.roles) ? control.roles : []
  const servos = Array.isArray(control.servos) ? control.servos : []
  return (
    <>
      <ChoiceField
        path={[...path, 'kind']}
        value={control.kind}
        options={context.control_kinds}
        onValue={(value) => void onOp({ op: 'surface.control.kind', index, kind: value })}
      />
      {Object.keys(kindFields).map((key) => (
        <EntryField key={key} path={[...path, key]} unitKey={key} />
      ))}
      <div className="form-row">
        <span className="form-label">roles</span>
        <div className="check-row">
          {context.control_roles.map((role) => (
            <CheckboxField
              key={role}
              checked={roles.includes(role)}
              label={role}
              onToggle={(on) => void onOp({ op: 'surface.control.role', index, role, on })}
            />
          ))}
        </div>
      </div>
      {['deflection_min_deg', 'deflection_max_deg', 'rate_deg_s'].map((key) => (
        <EntryField key={key} path={[...path, key]} unitKey={key} />
      ))}
      {servos.map((servo, panel) => {
        const number = numbers.get(`${surfaceName}#${panel}`)
        const label = `panel ${panel + 1} servo${number === undefined ? '' : ` (C${number})`}`
        return <ChoiceField key={panel} path={[...path, 'servos', panel]} value={servo} options={context.servo_outputs} label={label} />
      })}
    </>
  )
}

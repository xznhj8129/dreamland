import { useState } from 'react'
import type { ReactElement } from 'react'
import { editorOp } from '../store'
import type { ResolvedControl, Snapshot } from '../types'
import { Card, CheckboxField, ChoiceField, EntryField, Fields, OptionalBlock, HelpNote } from './fields'
import { fieldLabel, formatValue } from './format'
import { isObject } from './patch'
import { useFormApi } from './session'
import { ComponentList, ComponentPicker, useComponentIndex, useSelection } from './selection'

/** editor_view.build_surfaces_tab (852-942): per-set card, generic fields, control surface, spars, root base. */
export function SurfacesTab({ snapshot }: { snapshot: Snapshot }): ReactElement {
  const api = useFormApi()
  const definition = api.definition
  const surfaces = Array.isArray(definition.surfaces) ? definition.surfaces.filter(isObject) : []
  const [notice, setNotice] = useState('')
  const selected = useComponentIndex('surface', surfaces.length)
  const { select } = useSelection()

  const run = async (body: Record<string, unknown>): Promise<void> => {
    const result = await editorOp(api.sessionId, body)
    setNotice(result.ok ? '' : result.message)
    if (result.ok && body.op === 'surface.add') select('surface', surfaces.length)
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
      <ComponentPicker kind="surface" items={surfaces.map((item, index) => ({ name: String(item.name ?? `surface ${index + 1}`), detail: typeof item.span_m === 'number' ? `span ${formatValue('span_m', item.span_m, api.context)} mm` : undefined }))} />
      {surfaces.map((surface, index) => (
        <div key={index} hidden={index !== selected}><SurfaceCard index={index} surface={surface} snapshot={snapshot} numbers={numbers} onOp={run} /></div>
      ))}
    </>
  )
}

interface SurfaceCardProps {
  index: number
  snapshot: Snapshot
  surface: Record<string, unknown>
  /** Resolved control numbers by `${surface name}#${panel index}` (build_control 891-896). */
  numbers: Map<string, number>
  onOp: (body: Record<string, unknown>) => Promise<void>
}

/** editor_view.build_surfaces_tab 855-865: one LabelFrame per set, titled with its name. */
function SurfaceCard({ index, surface, snapshot, numbers, onOp }: SurfaceCardProps): ReactElement {
  const api = useFormApi()
  const context = api.context
  const path = ['surfaces', index]
  const layout = typeof surface.layout === 'string' ? surface.layout : ''
  // 857: fin_position is only meaningful for a single fin or three fins.
  const skip = ['mass_kg', 'control', 'spars', 'root_base', 'sections', ...(layout === '1' || layout === '3' ? [] : ['fin_position'])]
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
      <EntryField path={[...path, 'mass_kg']} unitKey="mass_kg" optional defaultLabel="Geometry estimate"
        defaultValue={snapshot.definition_valid ? snapshot.resolved?.mass_points?.find(point => point.name === `surface ${String(surface.name)}`)?.estimate_kg ?? undefined : undefined} />
      <SurfaceSections index={index} surface={surface} onOp={onOp} />
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
            placements={snapshot.definition_valid ? (snapshot.resolved?.controls ?? []).filter(item => item.surface === surface.name) : []}
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
      <details className="advanced-fields">
        <summary>Root base <span>Not implemented</span></summary>
        <p className="form-hint">These stored dimensions are not used to build geometry yet.</p>
        {rootBase !== null && <fieldset disabled className="form-group"><Fields data={rootBase} path={[...path, 'root_base']} context={context} /></fieldset>}
      </details>
    </Card>
  )
}

function SurfaceSections({ index, surface, onOp }: Pick<SurfaceCardProps, 'index' | 'surface' | 'onOp'>): ReactElement {
  const api = useFormApi()
  const [selection, setSelection] = useState(0)
  const sections = isObject(surface.sections) ? surface.sections : {}
  const cuts = Array.isArray(sections.cuts_m) ? sections.cuts_m.map(Number) : []
  const bounds = [0, ...cuts, Number(surface.span_m)]
  const selected = Math.min(selection, cuts.length)
  const path = ['surfaces', index, 'sections', 'cuts_m']
  async function add(side: 'before' | 'after'): Promise<void> {
    await onOp({ op: 'surface.section.add', index, section_index: selected, side })
    setSelection(selected + (side === 'after' ? 1 : 0))
  }
  return <details className="form-group" open>
    <summary>Sections</summary>
    <HelpNote>S1 stays attached; later fixed segments print separately. Controls stay in one piece.</HelpNote>
    <ComponentList label="Select surface section" index={selected} onSelect={setSelection}
      items={bounds.slice(1).map((end, position) => ({ name: position === 0 ? 'S1 · Root' : `S${position + 1}`,
        detail: `${formatValue('length_m', end - bounds[position]!, api.context)} mm` }))} />
    <div className="button-row" role="group" aria-label="Add surface section">
      <button type="button" onClick={() => void add('before')}>Add before</button>
      <button type="button" onClick={() => void add('after')}>Add after</button>
    </div>
    <HelpNote>First cut: 15 mm root (max half span). Later cuts halve the selected segment.</HelpNote>
    {selected > 0 ? <EntryField path={[...path, selected - 1]} unitKey="length_m" label="Start (mm)" />
      : <div className="form-row"><span className="form-label">Start (mm)</span><output>0</output></div>}
    {selected < cuts.length ? <EntryField path={[...path, selected]} unitKey="length_m" label="End (mm)" />
      : <div className="form-row"><span className="form-label">End (mm)</span><output>{formatValue('length_m', surface.span_m, api.context)}</output></div>}
    <HelpNote>Start and End are cut positions in mm from the root; the first Start is 0, the last End is the fixed wing span. A shared boundary also changes the adjoining segment.</HelpNote>
    {selected < cuts.length && <button type="button" className="danger" onClick={() => void onOp({ op: 'surface.section.delete', index, section_index: selected })}>Remove end cut & merge sections</button>}
  </details>
}

interface ControlFieldsProps {
  index: number
  surfaceName: string
  control: Record<string, unknown>
  numbers: Map<string, number>
  placements: ResolvedControl[]
  onOp: (body: Record<string, unknown>) => Promise<void>
}

/** editor_view.build_control (867-903): kind and its fields, roles, deflection limits, the gap and pivot rod, one servo per panel. */
function ControlFields({ index, surfaceName, control, numbers, placements, onOp }: ControlFieldsProps): ReactElement {
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
      <CheckboxField
        path={[...path, 'align_hinge_vertically']}
        checked={control.align_hinge_vertically === true}
        label="Align hinge vertically"
      />
      {placements.map(placement => <p className="form-hint" key={placement.number}>
        C{placement.number}: span {formatValue('length_m', placement.span_length_m, context)} mm
        {placement.rod_diameter_m !== null && ` · Pivot rod ${formatValue('length_m', placement.hinge_length_m, context)} mm`}
      </p>)}
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
      {[
        ['gap_m', 'blank = 0'],
        ['rod_diameter_m', 'blank = no rod'],
        ['rod_x_offset_m', 'blank = 0'],
        ['rod_y_offset_m', 'blank = 0'],
      ].map(([key, blankHint]) => (
        <EntryField key={key} path={[...path, key]} unitKey={key} optional blankHint={blankHint} />
      ))}
      {servos.map((servo, panel) => {
        const number = numbers.get(`${surfaceName}#${panel}`)
        const label = `panel ${panel + 1} servo${number === undefined ? '' : ` (C${number})`}`
        return <ChoiceField key={panel} path={[...path, 'servos', panel]} value={servo} options={context.servo_outputs} label={label} />
      })}
    </>
  )
}

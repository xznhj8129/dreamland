import { useState } from 'react'
import type { ReactElement } from 'react'
import { editorOp } from '../store'
import { Card, ChoiceField, EntryField, Fields, OptionalBlock, HelpNote } from './fields'
import { isObject } from './patch'
import { useFormApi } from './session'
import { ComponentPicker, useComponentIndex, useSelection } from './selection'

const MOUNT_HOLE_TYPE_LABELS = { round: 'Round', slot: 'Slot' }
const MOUNT_TYPE_LABELS = { electric: 'Electric motor', rocket: 'Rocket motor' }

/** editor_view.build_mounts_tab (1021-1042), normalize_mount (1100-1113), add_mount (1115-1118). */
export function MountsTab(): ReactElement {
  const api = useFormApi()
  const definition = api.definition
  const propulsion = isObject(definition.propulsion) ? definition.propulsion : null
  const surfaces = Array.isArray(definition.surfaces) ? definition.surfaces.filter(isObject) : []
  const surfaceNames = surfaces.map((surface) => (typeof surface.name === 'string' ? surface.name : ''))
  const [notice, setNotice] = useState('')
  const selected = useComponentIndex('mount', (propulsion !== null && Array.isArray(propulsion.mounts) ? propulsion.mounts.length : 0))
  const { select } = useSelection()

  const run = async (body: Record<string, unknown>): Promise<void> => {
    const result = await editorOp(api.sessionId, body)
    setNotice(result.ok ? '' : result.message)
    if (result.ok && body.op === 'mount.add') select('mount', (propulsion !== null && Array.isArray(propulsion.mounts) ? propulsion.mounts.length : 0))
  }

  if (propulsion === null) {
    // 1023-1025: a mount needs the propulsion block.
    return (
      <Card title="Motor mounts">
        <p className="form-hint">Not powered (type on the Airframe tab).</p>
      </Card>
    )
  }
  const mounts = Array.isArray(propulsion.mounts) ? propulsion.mounts.filter(isObject) : []

  return (
    <>
      <Card title="Motor mounts">
        <div className="button-row">
          <button type="button" onClick={() => void run({ op: 'mount.add' })}>
            Add motor mount
          </button>
        </div>
        {notice !== '' && <div className="field-error">{notice}</div>}
      </Card>
      <ComponentPicker kind="mount" items={mounts.map((item, index) => ({ name: `Mount ${index + 1}`, detail: `${String(item.type)}, ${String(item.on)}` }))} />
      {mounts.map((mount, index) => (
        <div key={index} hidden={index !== selected}><MountCard index={index} mount={mount} surfaceNames={surfaceNames} onOp={run} /></div>
      ))}
    </>
  )
}

interface MountCardProps {
  index: number
  mount: Record<string, unknown>
  surfaceNames: string[]
  onOp: (body: Record<string, unknown>) => Promise<void>
}

/** editor_view.build_mounts_tab 1029-1042: mount N card, motor type, on, surface placement, direction, spin, pod.
 * A rocket mount sits on the tail or a surface and has no direction, spin, pod or holes. */
function MountCard({ index, mount, surfaceNames, onOp }: MountCardProps): ReactElement {
  const api = useFormApi()
  const context = api.context
  const path = ['propulsion', 'mounts', index]
  const on = typeof mount.on === 'string' ? mount.on : ''
  const rocket = mount.type === 'rocket'
  const pod = isObject(mount.pod) ? mount.pod : null
  const holes = isObject(mount.holes) ? mount.holes : null
  // 1039: a surface mount's motors spin out|in, a nose or tail mount's cw|ccw.
  const spins = on === 'surface' ? context.surface_prop_spins : context.single_prop_spins

  return (
    <Card title={`mount ${index + 1}`} onDelete={() => void onOp({ op: 'mount.delete', index })}>
      <ChoiceField
        label="Motor type"
        path={[...path, 'type']}
        value={mount.type}
        options={context.mount_types}
        optionLabels={MOUNT_TYPE_LABELS}
        // Picking the type normalizes the mount: a rocket mount drops the electric-only fields.
        onValue={(value) => void onOp({ op: 'mount.type', index, type: value })}
      />
      <ChoiceField
        path={[...path, 'on']}
        value={mount.on}
        options={rocket ? context.rocket_mount_targets : context.choices.on ?? []}
        // 1031: picking on normalizes the mount's surface fields and direction/spin.
        onValue={(value) => void onOp({ op: 'mount.on', index, on: value })}
      />
      {on === 'surface' && (
        <>
          <ChoiceField path={[...path, 'surface']} value={mount.surface} options={surfaceNames} />
          <EntryField path={[...path, 'span_position_m']} unitKey="span_position_m" />
        </>
      )}
      {rocket ? <>
        <EntryField path={[...path, 'x_offset_m']} unitKey="x_offset_m" label="Motor aft end (mm, positive aft)" />
        <HelpNote>Where the loaded motor's nozzle end sits, from the {on === 'surface' ? 'surface root leading edge' : 'tail face'}; the motor runs forward from there. Load a motor on the Propulsion tab (Rocket).</HelpNote>
        <EntryField path={[...path, 'thrust_axis_tilt_deg']} unitKey="thrust_axis_tilt_deg" />
      </> : <>
      <EntryField path={[...path, 'x_offset_m']} unitKey="x_offset_m" label="X offset (mm, positive aft)" />
      {on !== 'surface' && <HelpNote>Moves the motor and pod together from the {on} mounting face.</HelpNote>}
      <ChoiceField path={[...path, 'direction']} value={mount.direction} options={context.choices.direction ?? []} />
      <ChoiceField path={[...path, 'spin']} value={mount.spin} options={spins} />
      <EntryField path={[...path, 'thrust_axis_tilt_deg']} unitKey="thrust_axis_tilt_deg" />
      <EntryField path={[...path, 'clearance_m']} unitKey="clearance_m" label="Mount clearance (mm, added to the bell diameter)" />
      <OptionalBlock label="Mounting holes" present={holes !== null} onToggle={(checked) => void onOp({ op: 'mount.holes.toggle', index, enabled: checked })}>
        {holes !== null && <>
          <ChoiceField
            path={[...path, 'holes', 'type']}
            value={holes.type}
            options={context.mount_hole_types}
            optionLabels={MOUNT_HOLE_TYPE_LABELS}
            label="Hole type"
            // Switching the type swaps its diameters: round has the pattern diameter, slot the inner and outer.
            onValue={(value) => void onOp({ op: 'mount.holes.type', index, type: value })}
          />
          <EntryField path={[...path, 'holes', 'count']} unitKey="count" label="Pattern count" />
          <EntryField path={[...path, 'holes', 'angle_deg']} unitKey="angle_deg" label="Pattern angle (deg)" />
          <ChoiceField path={[...path, 'holes', 'screw']} value={holes.screw} options={context.screw_sizes} label="Hole diameter (screw)" />
          {holes.type === 'round' ? (
            <EntryField path={[...path, 'holes', 'pattern_diameter_m']} unitKey="pattern_diameter_m" label="Pattern diameter (mm)" />
          ) : <>
            <EntryField path={[...path, 'holes', 'inner_diameter_m']} unitKey="inner_diameter_m" label="Pattern inner diameter (mm)" />
            <EntryField path={[...path, 'holes', 'outer_diameter_m']} unitKey="outer_diameter_m" label="Pattern outer diameter (mm)" />
          </>}
          <EntryField path={[...path, 'holes', 'center_diameter_m']} unitKey="center_diameter_m" label="Center relief diameter (mm)" />
          <EntryField path={[...path, 'holes', 'center_depth_m']} unitKey="center_depth_m" label="Center relief depth (mm)" />
          <OptionalBlock label="Head bore" present={typeof holes.head_depth_m === 'number'} onToggle={(checked) => void onOp({ op: 'mount.holes.head', index, enabled: checked })}>
            {typeof holes.head_depth_m === 'number' && <>
              <EntryField path={[...path, 'holes', 'head_diameter_m']} unitKey="head_diameter_m" label="Head bore diameter (mm)" />
              <EntryField path={[...path, 'holes', 'head_depth_m']} unitKey="head_depth_m" label="Head seat depth from mount face (mm)" />
            </>}
          </OptionalBlock>
        </>}
      </OptionalBlock>
      <OptionalBlock label="pod" present={pod !== null} onToggle={(checked) => void onOp({ op: 'mount.pod.toggle', index, enabled: checked })}>
        {pod !== null && <Fields data={pod} path={[...path, 'pod']} context={context} />}
      </OptionalBlock>
      </>}
    </Card>
  )
}

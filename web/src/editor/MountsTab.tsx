import { useState } from 'react'
import type { ReactElement } from 'react'
import { editorOp } from '../store'
import { Card, ChoiceField, EntryField, Fields, OptionalBlock } from './fields'
import { isObject } from './patch'
import { useFormApi } from './session'

/** editor_view.build_mounts_tab (1021-1042), normalize_mount (1100-1113), add_mount (1115-1118). */
export function MountsTab(): ReactElement {
  const api = useFormApi()
  const definition = api.definition
  const propulsion = isObject(definition.propulsion) ? definition.propulsion : null
  const surfaces = Array.isArray(definition.surfaces) ? definition.surfaces.filter(isObject) : []
  const surfaceNames = surfaces.map((surface) => (typeof surface.name === 'string' ? surface.name : ''))
  const [notice, setNotice] = useState('')

  const run = async (body: Record<string, unknown>): Promise<void> => {
    const result = await editorOp(api.sessionId, body)
    setNotice(result.ok ? '' : result.message)
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
      {mounts.map((mount, index) => (
        <MountCard key={index} index={index} mount={mount} surfaceNames={surfaceNames} onOp={run} />
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

/** editor_view.build_mounts_tab 1029-1042: mount N card, on, surface placement, direction, spin, pod. */
function MountCard({ index, mount, surfaceNames, onOp }: MountCardProps): ReactElement {
  const api = useFormApi()
  const context = api.context
  const path = ['propulsion', 'mounts', index]
  const on = typeof mount.on === 'string' ? mount.on : ''
  const pod = isObject(mount.pod) ? mount.pod : null
  // 1039: a surface mount's motors spin out|in, a nose or tail mount's cw|ccw.
  const spins = on === 'surface' ? context.surface_prop_spins : context.single_prop_spins

  return (
    <Card title={`mount ${index + 1}`} onDelete={() => void onOp({ op: 'mount.delete', index })}>
      <ChoiceField
        path={[...path, 'on']}
        value={mount.on}
        options={context.choices.on ?? []}
        // 1031: picking on normalizes the mount's surface fields and direction/spin.
        onValue={(value) => void onOp({ op: 'mount.on', index, on: value })}
      />
      {on === 'surface' && (
        <>
          <ChoiceField path={[...path, 'surface']} value={mount.surface} options={surfaceNames} />
          <EntryField path={[...path, 'span_position_m']} unitKey="span_position_m" />
          <EntryField path={[...path, 'x_offset_m']} unitKey="x_offset_m" />
        </>
      )}
      <ChoiceField path={[...path, 'direction']} value={mount.direction} options={context.choices.direction ?? []} />
      <ChoiceField path={[...path, 'spin']} value={mount.spin} options={spins} />
      <EntryField path={[...path, 'thrust_axis_tilt_deg']} unitKey="thrust_axis_tilt_deg" />
      <OptionalBlock label="pod" present={pod !== null} onToggle={(checked) => void onOp({ op: 'mount.pod.toggle', index, enabled: checked })}>
        {pod !== null && <Fields data={pod} path={[...path, 'pod']} context={context} />}
      </OptionalBlock>
    </Card>
  )
}

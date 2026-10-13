import { useState } from 'react'
import type { ReactElement } from 'react'
import { editorOp } from '../store'
import { Card, ChoiceField, EntryField, VectorField } from './fields'
import { formatValue } from './format'
import { isObject } from './patch'
import { useFormApi } from './session'
import { ComponentPicker, useComponentIndex, useSelection } from './selection'

/** editor_view.build_volumes_tab (1124-1172): per-volume card, shape fields, contents, delete freezes placements. */
export function VolumesTab(): ReactElement {
  const api = useFormApi()
  const definition = api.definition
  const volumes = Array.isArray(definition.volumes) ? definition.volumes.filter(isObject) : []
  const parts = Array.isArray(definition.parts) ? definition.parts.filter(isObject) : []
  const electric = isObject(definition.propulsion) && isObject(definition.propulsion.electric) ? definition.propulsion.electric : null
  const battery = electric !== null && isObject(electric.battery) ? electric.battery : null
  const [notice, setNotice] = useState('')
  const selected = useComponentIndex('volume', volumes.length)
  const { select } = useSelection()

  const run = async (body: Record<string, unknown>): Promise<void> => {
    const result = await editorOp(api.sessionId, body)
    setNotice(result.ok ? '' : result.message)
    if (result.ok && body.op === 'volume.add') select('volume', volumes.length)
  }

  return (
    <>
      <Card title="Volumes">
        <p className="form-hint">
          Internal space; parts sit in volumes (Parts tab). Mass: grams, or a range like 10-30. Drag volumes along the body in the side view (their parts
          move with them).
        </p>
        <div className="button-row">
          <button type="button" onClick={() => void run({ op: 'volume.add' })}>
            Add volume
          </button>
        </div>
        {notice !== '' && <div className="field-error">{notice}</div>}
      </Card>
      <ComponentPicker kind="volume" items={volumes.map((item, index) => ({ name: String(item.name ?? `volume ${index + 1}`), detail: typeof item.x_m === 'number' ? `x ${formatValue('x_m', item.x_m, api.context)} mm` : undefined }))} />
      {volumes.map((volume, index) => (
        <div key={index} hidden={index !== selected}><VolumeCard
          index={index}
          volume={volume}
          contents={[...parts.filter((part) => part.volume === volume.name).map((part) => (typeof part.name === 'string' ? part.name : '')), ...(battery?.volume === volume.name ? ['battery'] : [])]}
          onOp={run}
        /></div>
      ))}
    </>
  )
}

interface VolumeCardProps {
  index: number
  volume: Record<string, unknown>
  contents: string[]
  onOp: (body: Record<string, unknown>) => Promise<void>
}

/** editor_view.build_volumes_tab 1129-1149: name (rename carries to parts), shape and its fields, mass, contents. */
function VolumeCard({ index, volume, contents, onOp }: VolumeCardProps): ReactElement {
  const api = useFormApi()
  const context = api.context
  const path = ['volumes', index]
  const shape = typeof volume.shape === 'string' ? volume.shape : ''
  const shapeFields = context.volume_shape_fields[shape] ?? {}

  return (
    <Card title={typeof volume.name === 'string' ? volume.name : `volume ${index + 1}`} onDelete={() => void onOp({ op: 'volume.delete', index })}>
      <EntryField
        path={[...path, 'name']}
        unitKey="name"
        // 1131: a rename must carry to the parts placed in this volume.
        onValue={(value) => void onOp({ op: 'volume.rename', index, name: String(value) })}
      />
      <ChoiceField
        path={[...path, 'shape']}
        value={volume.shape}
        options={context.volume_shapes}
        // 1132: a shape change resets the shape's fields to their template values.
        onValue={(value) => void onOp({ op: 'volume.shape', index, shape: value })}
      />
      <EntryField path={[...path, 'x_m']} unitKey="x_m" />
      <EntryField path={[...path, 'y_m']} unitKey="y_m" optional blankHint="blank = 0" />
      <EntryField path={[...path, 'z_m']} unitKey="z_m" optional blankHint="blank = 0" />
      {Object.keys(shapeFields).map((key) => {
        if (key === 'size_m') {
          return <VectorField key={key} path={[...path, 'size_m']} unitKey="size_m" axes={['x', 'y', 'z']} />
        }
        if (key === 'axis') {
          return <ChoiceField key={key} path={[...path, 'axis']} value={volume.axis} options={context.choices.axis ?? []} />
        }
        return <EntryField key={key} path={[...path, key]} unitKey={key} />
      })}
      <EntryField path={[...path, 'mass_kg']} unitKey="mass_kg" optional blankHint="blank = space only" />
      <div className="form-row">
        <span className="form-static">{`parts: ${contents.length > 0 ? contents.join(', ') : 'none'}`}</span>
      </div>
    </Card>
  )
}

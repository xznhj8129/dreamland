import { useState } from 'react'
import type { ReactElement } from 'react'
import { editorOp } from '../store'
import { Card, ChoiceField, EntryField, PlacementFields } from './fields'
import { isObject } from './patch'
import { useFormApi } from './session'
import { ComponentPicker, useComponentIndex, useSelection } from './selection'

/** editor_view.build_parts_tab (1174-1234): library component, volume placement or own x, y, z, orientation, mass. */
export function PartsTab(): ReactElement {
  const api = useFormApi()
  const context = api.context
  const definition = api.definition
  const parts = Array.isArray(definition.parts) ? definition.parts.filter(isObject) : []
  const volumes = Array.isArray(definition.volumes) ? definition.volumes.filter(isObject) : []
  const volumeNames = volumes.map((volume) => (typeof volume.name === 'string' ? volume.name : ''))
  // 1179: every library entry as category:name, categories and names in their stored sorted order.
  const library = context.part_categories.flatMap((category) =>
    Object.keys(context.parts_data[category] ?? {})
      .sort()
      .map((name) => `${category}:${name}`),
  )
  const [notice, setNotice] = useState('')
  const selected = useComponentIndex('part', parts.length)
  const { select } = useSelection()

  const run = async (body: Record<string, unknown>): Promise<void> => {
    const result = await editorOp(api.sessionId, body)
    setNotice(result.ok ? '' : result.message)
    if (result.ok && body.op === 'part.add') select('part', parts.length)
  }

  return (
    <>
      <Card title="Parts">
        <p className="form-hint">
          Library components (data/parts/), in a volume (offset from its centre) or at their own x, y, z. Mass: grams, or a range like 10-30; 0 = counted
          elsewhere (e.g. a weighed section).
        </p>
        <div className="button-row">
          <button type="button" onClick={() => void run({ op: 'part.add' })}>
            Add part
          </button>
        </div>
        {notice !== '' && <div className="field-error">{notice}</div>}
      </Card>
      <ComponentPicker kind="part" items={parts.map((item, index) => ({ name: String(item.name ?? `part ${index + 1}`) }))} />
      {parts.map((part, index) => (
        <div key={index} hidden={index !== selected}><PartCard index={index} part={part} library={library} volumeNames={volumeNames} onOp={run} /></div>
      ))}
    </>
  )
}

interface PartCardProps {
  index: number
  part: Record<string, unknown>
  library: string[]
  volumeNames: string[]
  onOp: (body: Record<string, unknown>) => Promise<void>
}

/** editor_view.build_parts_tab 1180-1188 and add_placement (1190-1207). */
function PartCard({ index, part, library, volumeNames, onOp }: PartCardProps): ReactElement {
  const api = useFormApi()
  const context = api.context
  const path = ['parts', index]
  const placement = typeof part.part === 'string' ? part.part : ''
  const separator = placement.indexOf(':')
  const category = separator < 0 ? '' : placement.slice(0, separator)
  const name = separator < 0 ? '' : placement.slice(separator + 1)
  // 1186-1187: a library mass means a blank mass_kg falls back to it; none means the field is required.
  const libraryMass = context.parts_data[category]?.[name]?.mass
  const massHint = libraryMass ? 'blank = library' : 'required (none in library)'

  return (
    <Card title={typeof part.name === 'string' ? part.name : `part ${index + 1}`} onDelete={() => void onOp({ op: 'part.delete', index })}>
      <EntryField
        path={[...path, 'name']}
        unitKey="name"
        // 1182: a rename must carry to the propulsion ESC reference.
        onValue={(value) => void onOp({ op: 'part.rename', index, name: String(value) })}
      />
      <ChoiceField path={[...path, 'part']} value={part.part} options={library} searchable />
      {/* 1190-1207: the volume and offsets or own x, y, z, then the orientation, shared with the battery. */}
      <PlacementFields
        path={path}
        item={part}
        volumeNames={volumeNames}
        // 1197 and set_part_volume 1209-1220: into a volume at its centre; out of one frozen where it resolves.
        onVolume={(value) => void onOp({ op: 'part.volume', index, volume: value })}
      />
      <EntryField path={[...path, 'mass_kg']} unitKey="mass_kg" optional blankHint={massHint} defaultValue={libraryMass} defaultLabel="Library mass" />
    </Card>
  )
}

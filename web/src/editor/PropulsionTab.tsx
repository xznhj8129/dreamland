import { useId, useState } from 'react'
import type { ReactElement } from 'react'
import type { Snapshot } from '../types'
import { editorOp } from '../store'
import { Card, ChoiceField, EntryField, PlacementFields } from './fields'
import { deepCopy, firstBatteryPart, placedPartsOf } from './propulsion'
import { MotorChart } from './MotorChart'
import { isObject } from './patch'
import { useFormApi } from './session'

/** editor_view.build_propulsion_tab (944-1019): battery, motor/prop/ESC and the motorlib motor info. */
export function PropulsionTab({ snapshot }: { snapshot: Snapshot }): ReactElement {
  const api = useFormApi()
  const propulsion = isObject(api.definition.propulsion) ? api.definition.propulsion : null

  if (propulsion === null) {
    // 946-949: a hint until the Airframe tab's type flags add the propulsion block.
    return (
      <Card title="Propulsion">
        <p className="form-hint">Not powered (type on the Airframe tab).</p>
      </Card>
    )
  }
  return (
    <>
      <BatteryCard snapshot={snapshot} propulsion={propulsion} />
      <MotorCard propulsion={propulsion} />
      {snapshot.propulsion_chart !== null && (
        <Card title="Motor info (motorlib, static)">
          <MotorChart chart={snapshot.propulsion_chart} />
          {/* 1097: the summary lines, in the order libs.propulsion.summary_lines gives them. */}
          <pre className="motor-lines">{snapshot.propulsion.map((line) => line.text).join('\n')}</pre>
        </Card>
      )}
    </>
  )
}

interface BatteryCardProps {
  snapshot: Snapshot
  propulsion: Record<string, unknown>
}

/** editor_view.build_battery_group (969-1005) and set_battery_model (1007-1019). */
function BatteryCard({ snapshot, propulsion }: BatteryCardProps): ReactElement {
  const api = useFormApi()
  const context = api.context
  const definition = api.definition
  const battery = isObject(propulsion.battery) ? propulsion.battery : {}
  const modelId = useId()
  const labels = context.battery_model_labels
  const currentLabel = typeof battery.model === 'string' ? labels[battery.model] ?? battery.model : ''
  const parts = Object.keys(context.parts_data.battery ?? {}).sort()
  const volumeNames = (Array.isArray(definition.volumes) ? definition.volumes.filter(isObject) : [])
    .map((volume) => (typeof volume.name === 'string' ? volume.name : ''))
  const [notice, setNotice] = useState('')

  /** 1007-1019: another model keeps the placement; its other fields come back as they were when it was last left. */
  const setModel = (label: string): void => {
    const model = Object.keys(labels).find((key) => labels[key] === label)
    if (model === undefined || model === battery.model) return
    const kept: string[] = [...context.placement_keys, ...context.orientation_keys]
    const stashed: Record<string, unknown> = {}
    for (const [key, value] of Object.entries(battery)) {
      if (!kept.includes(key)) stashed[key] = value
    }
    api.stash.stashedBatteries[String(battery.model)] = stashed
    const placement: Record<string, unknown> = {}
    for (const key of kept) {
      if (key in battery) placement[key] = battery[key]
    }
    const remembered = api.stash.stashedBatteries[model]
    const fields = remembered ?? (model === 'custom_liion'
      ? deepCopy(context.new_custom_pack)
      : { model: 'commercial', part: firstBatteryPart(context) })
    void api.set(['propulsion', 'battery'], { ...fields, ...placement })
  }

  const run = async (body: Record<string, unknown>): Promise<void> => {
    const result = await editorOp(api.sessionId, body)
    setNotice(result.ok ? '' : result.message)
  }

  return (
    <Card title="Battery">
      <div className="form-row">
        <label className="form-label" htmlFor={modelId}>
          Battery model
        </label>
        <div className="form-control">
          <select id={modelId} className="field-input" value={currentLabel} onChange={(event) => setModel(event.target.value)}>
            {Object.entries(labels).map(([model, text]) => (
              <option key={model} value={text}>
                {text}
              </option>
            ))}
          </select>
        </div>
      </div>
      {battery.model === 'commercial' ? (
        <>
          {/* 976-978: the pack from data/parts/battery.json, and an optional mass. */}
          <ChoiceField label="pack" path={['propulsion', 'battery', 'part']} value={battery.part} options={parts} />
          <EntryField path={['propulsion', 'battery', 'mass_kg']} unitKey="mass_kg" optional blankHint="blank = library" />
        </>
      ) : (
        <>
          {/* 980-990: a custom stack of motorlib cells and its own leads. */}
          <EntryField path={['propulsion', 'battery', 's']} unitKey="s" label="S (cells in series)" />
          <EntryField path={['propulsion', 'battery', 'p']} unitKey="p" label="P (in parallel)" />
          <ChoiceField label="cell (motorlib)" path={['propulsion', 'battery', 'cell']} value={battery.cell} options={context.cells} />
          <ChoiceField path={['propulsion', 'battery', 'connector']} value={battery.connector} options={context.connectors} />
          <ChoiceField
            label="cell length along"
            path={['propulsion', 'battery', 'cell_axis']}
            value={battery.cell_axis}
            options={context.choices.axis ?? []}
          />
          {(context.choices.axis ?? []).map((axis, index) => (
            <EntryField key={axis} path={['propulsion', 'battery', 'stacking', index]} unitKey="stacking" label={`cells along ${axis}`} />
          ))}
          <ChoiceField
            label="lead AWG"
            path={['propulsion', 'battery', 'lead_awg']}
            value={battery.lead_awg}
            options={context.wire_gauges.map(String)}
            convert={Number}
          />
          <EntryField path={['propulsion', 'battery', 'lead_length_m']} unitKey="lead_length_m" />
        </>
      )}
      {/* 1003: add_placement 1190-1207, with the battery among the resolved parts (BATTERY_PART_NAME). */}
      <PlacementFields
        path={['propulsion', 'battery']}
        item={battery}
        volumeNames={volumeNames}
        onVolume={(value) => void run({ op: 'propulsion.battery.volume', volume: value })}
      />
      {snapshot.pack_summary !== null && (
        <div className="form-row">
          <span className="form-static">{packSummaryText(snapshot.pack_summary)}</span>
        </div>
      )}
      {notice !== '' && <div className="field-error">{notice}</div>}
    </Card>
  )
}

interface MotorCardProps {
  propulsion: Record<string, unknown>
}

/** editor_view.build_propulsion_tab (952-965): manufacturer, motor (the first of the manufacturer), prop and ESC. */
function MotorCard({ propulsion }: MotorCardProps): ReactElement {
  const api = useFormApi()
  const context = api.context
  const definition = api.definition
  const motor = isObject(propulsion.motor) ? propulsion.motor : {}
  const manufacturer = typeof motor.manufacturer === 'string' ? motor.manufacturer : ''
  const names = context.motors_by_manufacturer[manufacturer] ?? []

  return (
    <Card title="Motor, prop and ESC (motorlib)">
      <ChoiceField
        path={['propulsion', 'motor', 'manufacturer']}
        value={motor.manufacturer}
        options={context.motor_manufacturers}
        // 958-959: Tk updates manufacturer and name together, then recomputes once; one atomic set of the motor.
        onValue={(value) => {
          void api.set(['propulsion', 'motor'], { manufacturer: value, name: (context.motors_by_manufacturer[value] ?? [])[0] ?? '' })
        }}
      />
      <ChoiceField label="motor" path={['propulsion', 'motor', 'name']} value={motor.name} options={names} />
      <ChoiceField path={['propulsion', 'prop']} value={propulsion.prop} options={context.props} />
      <ChoiceField label="ESC" path={['propulsion', 'esc']} value={propulsion.esc} options={placedPartsOf(definition, 'esc')} />
    </Card>
  )
}

function packSummaryText(summary: NonNullable<Snapshot['pack_summary']>): string {
  const size = summary.size_mm === null ? 'no size' : `${summary.size_mm.map((value) => value.toFixed(0)).join(' x ')} mm`
  return `${summary.label}: ${size}, ${summary.mass_g.toFixed(0)} g, ${summary.capacity_mah.toFixed(0)} mAh`
}

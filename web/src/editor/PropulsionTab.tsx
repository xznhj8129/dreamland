import { ReportSections } from '../components/ReportPanel'
import { useId, useState } from 'react'
import type { ReactElement } from 'react'
import type { RocketMotorInfo, Snapshot } from '../types'
import { editorOp } from '../store'
import { Card, ChoiceField, EntryField, OptionalBlock, PlacementFields, ValueList } from './fields'
import { deepCopy, electricMotorCount, firstBatteryPart, newElectric, placedPartsOf } from './propulsion'
import { MotorChart } from './MotorChart'
import { RocketChart } from './RocketChart'
import { isObject } from './patch'
import { useFormApi } from './session'

const KINDS = ['Electric', 'Rocket'] as const

/** Propulsion: the electric system and the rocket motors, each on its own sub-tab. */
export function PropulsionTab({ snapshot }: { snapshot: Snapshot }): ReactElement {
  const api = useFormApi()
  const propulsion = isObject(api.definition.propulsion) ? api.definition.propulsion : null
  const mounts = propulsion !== null && Array.isArray(propulsion.mounts) ? propulsion.mounts.filter(isObject) : []
  const [kind, setKind] = useState<string>(mounts.length > 0 && mounts.every((mount) => mount.type === 'rocket') ? 'Rocket' : 'Electric')

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
      <div className="subtab-row">
        {KINDS.map((name) => (
          <button key={name} type="button" className={name === kind ? 'subtab active' : 'subtab'} onClick={() => setKind(name)}>
            {name}
          </button>
        ))}
      </div>
      <div hidden={kind !== 'Electric'}>
        <ElectricTab snapshot={snapshot} propulsion={propulsion} />
      </div>
      <div hidden={kind !== 'Rocket'}>
        <RocketTab propulsion={propulsion} />
      </div>
    </>
  )
}

/** editor_view.build_propulsion_tab (944-1019): battery, motor/prop/ESC and the motorlib motor info, under an
 * optional electric system (installed from new_propulsion's electric part, one run per electric motor). */
function ElectricTab({ snapshot, propulsion }: { snapshot: Snapshot; propulsion: Record<string, unknown> }): ReactElement {
  const api = useFormApi()
  const electric = isObject(propulsion.electric) ? propulsion.electric : null
  const [notice, setNotice] = useState('')

  const toggle = async (enabled: boolean): Promise<void> => {
    const block = enabled ? newElectric(api.definition, api.context, electricMotorCount(propulsion, api.definition, api.context) ?? 0) : undefined
    const result = await editorOp(api.sessionId, { op: 'electric.toggle', enabled, electric: block })
    setNotice(result.ok ? '' : result.message)
  }

  return (
    <>
      <Card title="Electric system">
        <OptionalBlock label="Electric system (motor, prop, battery, ESC, wiring)" present={electric !== null} onToggle={(checked) => void toggle(checked)}>
          <p className="form-hint">One motor and prop model drives every electric mount (Design, Motor mounts).</p>
        </OptionalBlock>
        {notice !== '' && <div className="field-error">{notice}</div>}
      </Card>
      {electric !== null && <>
      <MotorCard electric={electric} />
      <BatteryCard snapshot={snapshot} electric={electric} />
      {snapshot.propulsion_chart !== null && (
        <Card title="Motor info (motorlib, static)">
          <MotorChart chart={snapshot.propulsion_chart} />
          <ReportSections sections={(snapshot.report_pages?.propulsion ?? []).filter(section => ['Full throttle', 'Hover', 'Model limitations'].includes(section.title))} />
          <p className="form-hint">See the Propulsion report for the complete battery and circuit details.</p>
        </Card>
      )}
      </>}
    </>
  )
}

/** Solid rocket motors from data/rocket_motors/, each loaded into a rocket mount. */
function RocketTab({ propulsion }: { propulsion: Record<string, unknown> }): ReactElement {
  const api = useFormApi()
  const context = api.context
  const rocket = isObject(propulsion.rocket) ? propulsion.rocket : null
  const loads = rocket !== null && Array.isArray(rocket.motors) ? rocket.motors.filter(isObject) : []
  const mounts = Array.isArray(propulsion.mounts) ? propulsion.mounts.filter(isObject) : []
  const rocketMounts = mounts.flatMap((mount, index) => (mount.type === 'rocket' ? [String(index + 1)] : []))
  const mountLabels = Object.fromEntries(rocketMounts.map((number) => [number, `Mount ${number} (${String(mounts[Number(number) - 1].on)})`]))
  const motorNames = Object.keys(context.rocket_motors)
  const [notice, setNotice] = useState('')

  const run = async (body: Record<string, unknown>): Promise<void> => {
    const result = await editorOp(api.sessionId, body)
    setNotice(result.ok ? '' : result.message)
  }

  return (
    <>
      <Card title="Rocket motors">
        <div className="button-row">
          <button type="button" onClick={() => void run({ op: 'rocket.add' })}>
            Add rocket motor
          </button>
        </div>
        {rocketMounts.length === 0 && <p className="form-hint">No rocket mounts yet: set a mount's Motor type to rocket (Design, Motor mounts).</p>}
        {notice !== '' && <div className="field-error">{notice}</div>}
      </Card>
      {loads.map((load, index) => {
        const info = context.rocket_motors[String(load.motor)]
        return (
          <Card key={index} title={`rocket motor ${index + 1}`} onDelete={() => void run({ op: 'rocket.delete', index })}>
            <ChoiceField label="mount" path={['propulsion', 'rocket', 'motors', index, 'mount']} value={String(load.mount)} options={rocketMounts} optionLabels={mountLabels} convert={Number} />
            <ChoiceField label="motor" path={['propulsion', 'rocket', 'motors', index, 'motor']} value={load.motor} options={motorNames} searchable />
            {info !== undefined && (
              <>
                <ValueList rows={rocketMotorRows(info)} />
                <RocketChart curve={info.curve} />
              </>
            )}
          </Card>
        )
      })}
    </>
  )
}

function rocketMotorRows(info: RocketMotorInfo): [string, string][] {
  return [
    ['Motor', `${info.manufacturer} ${info.designation}`],
    ['Size', `${(info.diameter_m * 1000).toFixed(0)} x ${(info.length_m * 1000).toFixed(0)} mm`],
    ['Loaded mass', `${(info.total_kg * 1000).toFixed(1)} g`],
    ['Propellant', `${(info.propellant_kg * 1000).toFixed(1)} g`],
    ['Total impulse', `${info.total_impulse_ns.toFixed(1)} Ns`],
    ['Average thrust', `${info.average_thrust_n.toFixed(1)} N`],
    ['Peak thrust', `${info.peak_thrust_n.toFixed(1)} N`],
    ['Burn time', `${info.burn_time_s.toFixed(2)} s`],
    ['Delays', info.delays],
  ]
}

interface BatteryCardProps {
  snapshot: Snapshot
  electric: Record<string, unknown>
}

/** editor_view.build_battery_group (969-1005) and set_battery_model (1007-1019). */
function BatteryCard({ snapshot, electric }: BatteryCardProps): ReactElement {
  const api = useFormApi()
  const context = api.context
  const definition = api.definition
  const battery = isObject(electric.battery) ? electric.battery : {}
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
    void api.set(['propulsion', 'electric', 'battery'], { ...fields, ...placement })
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
          <ChoiceField label="pack" path={['propulsion', 'electric', 'battery', 'part']} value={battery.part} options={parts} />
          <EntryField path={['propulsion', 'electric', 'battery', 'mass_kg']} unitKey="mass_kg" optional defaultLabel="Library mass" defaultValue={context.parts_data.battery?.[String(battery.part)]?.mass} />
        </>
      ) : (
        <>
          {/* 980-990: a custom stack of motorlib cells and its own leads. */}
          <EntryField path={['propulsion', 'electric', 'battery', 's']} unitKey="s" label="S (cells in series)" />
          <EntryField path={['propulsion', 'electric', 'battery', 'p']} unitKey="p" label="P (in parallel)" />
          <ChoiceField label="cell (motorlib)" path={['propulsion', 'electric', 'battery', 'cell']} value={battery.cell} options={context.cells} />
          <ChoiceField path={['propulsion', 'electric', 'battery', 'connector']} value={battery.connector} options={context.connectors} />
          <ChoiceField
            label="cell length along"
            path={['propulsion', 'electric', 'battery', 'cell_axis']}
            value={battery.cell_axis}
            options={context.choices.axis ?? []}
          />
          {(context.choices.axis ?? []).map((axis, index) => (
            <EntryField key={axis} path={['propulsion', 'electric', 'battery', 'stacking', index]} unitKey="stacking" label={`cells along ${axis}`} />
          ))}
          <ChoiceField
            label="lead AWG"
            path={['propulsion', 'electric', 'battery', 'lead_awg']}
            value={battery.lead_awg}
            options={context.wire_gauges.map(String)}
            convert={Number}
          />
          <EntryField path={['propulsion', 'electric', 'battery', 'lead_length_m']} unitKey="lead_length_m" />
        </>
      )}
      {/* 1003: add_placement 1190-1207, with the battery among the resolved parts (BATTERY_PART_NAME). */}
      <details className="advanced-fields">
      <summary>Battery placement & orientation</summary>
      <PlacementFields
        path={['propulsion', 'electric', 'battery']}
        item={battery}
        volumeNames={volumeNames}
        onVolume={(value) => void run({ op: 'propulsion.battery.volume', volume: value })}
      />
      </details>
      {snapshot.pack_summary !== null && (
        <ValueList rows={packSummaryRows(snapshot.pack_summary)} />
      )}
      {notice !== '' && <div className="field-error">{notice}</div>}
    </Card>
  )
}

interface MotorCardProps {
  electric: Record<string, unknown>
}

/** editor_view.build_propulsion_tab (952-965): manufacturer, motor (the first of the manufacturer), prop and ESC. */
function MotorCard({ electric }: MotorCardProps): ReactElement {
  const api = useFormApi()
  const context = api.context
  const definition = api.definition
  const motor = isObject(electric.motor) ? electric.motor : {}
  const manufacturer = typeof motor.manufacturer === 'string' ? motor.manufacturer : ''
  const names = context.motors_by_manufacturer[manufacturer] ?? []

  return (
    <Card title="Motor, prop and ESC (motorlib)">
      <ChoiceField
        path={['propulsion', 'electric', 'motor', 'manufacturer']}
        value={motor.manufacturer}
        options={context.motor_manufacturers}
        // 958-959: Tk updates manufacturer and name together, then recomputes once; one atomic set of the motor.
        onValue={(value) => {
          void api.set(['propulsion', 'electric', 'motor'], { manufacturer: value, name: (context.motors_by_manufacturer[value] ?? [])[0] ?? '' })
        }}
      />
      <ChoiceField label="motor" path={['propulsion', 'electric', 'motor', 'name']} value={motor.name} options={names} searchable />
      <ChoiceField path={['propulsion', 'electric', 'prop']} value={electric.prop} options={context.props} searchable />
      <ChoiceField label="ESC" path={['propulsion', 'electric', 'esc']} value={electric.esc} options={placedPartsOf(definition, 'esc')} />
    </Card>
  )
}

function packSummaryRows(summary: NonNullable<Snapshot['pack_summary']>): [string, string][] {
  return [
    ['Pack', summary.label],
    ['Size', summary.size_mm === null ? 'no size' : `${summary.size_mm.map((value) => value.toFixed(0)).join(' x ')} mm`],
    ['Mass', `${summary.mass_g.toFixed(0)} g`],
    ['Capacity', `${summary.capacity_mah.toFixed(0)} mAh`],
  ]
}

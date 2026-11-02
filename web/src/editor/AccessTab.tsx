import { useState } from 'react'
import type { ReactElement } from 'react'
import { editorOp } from '../store'
import { Card, ChoiceField, EntryField, Group, HelpNote, OptionalBlock } from './fields'
import { isObject } from './patch'
import { PlacementTable } from './PlacementTable'
import { useFormApi } from './session'
import { ComponentPicker, useComponentIndex, useSelection } from './selection'

const KIND_LABELS: Record<string, string> = { hole: 'Hole', hatch: 'Hatch' }
const PROFILE_LABELS: Record<string, string> = { slot: 'Slot / circle', rounded_rectangle: 'Rounded rectangle' }
const EXTENT_LABELS: Record<string, string> = { depth: 'Depth', wall: 'Through wall', all: 'Through all' }
const EXTENT_HINTS: Record<string, string> = {
  depth: 'From the profile plane along its rotated local +Z.',
  wall: 'Through the selected skin into its cavity.',
  all: 'Through the entire fuselage, including the opposite wall.',
}
const DUCT_TYPE_LABELS: Record<string, string> = { naca: 'NACA' }

/** Saved holes, hatches and ducts (definition access[] and ducts[]): profile/placement/cuts and the duct inlets' curves and patterns. The live preview uses triangle geometry; CAD runs only on export; every edit goes through the queued editor set/op handlers. */
export function AccessTab(): ReactElement {
  const api = useFormApi()
  const definition = api.definition
  const entries = Array.isArray(definition.access) ? definition.access.filter(isObject) : []
  const ducts = Array.isArray(definition.ducts) ? definition.ducts.filter(isObject) : []
  const [notice, setNotice] = useState('')
  const [ductNotice, setDuctNotice] = useState('')
  const selected = useComponentIndex('access', entries.length)
  const ductSelected = useComponentIndex('duct', ducts.length)
  const { select } = useSelection()

  const run = async (body: Record<string, unknown>): Promise<void> => {
    const result = await editorOp(api.sessionId, body)
    setNotice(result.ok ? '' : result.message)
    if (result.ok && body.op === 'access.add') select('access', entries.length)
  }

  const runDuct = async (body: Record<string, unknown>): Promise<void> => {
    const result = await editorOp(api.sessionId, body)
    setDuctNotice(result.ok ? '' : result.message)
    if (result.ok && body.op === 'duct.add') select('duct', ducts.length)
  }

  return (
    <>
      <Card title="Access">
        <HelpNote>
          Named holes and hatches on the fuselage printed material, saved in the definition. Profile X is the length, Y the width, and +Z the cut
          direction; the placement turns roll about X, then pitch about Y, then yaw about Z. Holes cut fuselage material; hatches keep separate curved pieces. The Simple view shows cut direction; 3D and CAD show the physical cuts.
        </HelpNote>
        <div className="button-row">
          <button type="button" onClick={() => void run({ op: 'access.add' })}>
            Add Hole
          </button>
          <button type="button" onClick={() => void run({ op: 'access.add', kind: 'hatch' })}>
            Add Hatch
          </button>
        </div>
        {notice !== '' && <div className="field-error">{notice}</div>}
      </Card>
      <ComponentPicker
        kind="access"
        items={entries.map((entry, index) => ({
          name: String(entry.name ?? `access ${index + 1}`),
          detail: `${typeof entry.kind === 'string' ? KIND_LABELS[entry.kind] ?? entry.kind : 'access'}${isObject(entry.radial) ? ` ×${String(entry.radial.count)}` : ''}`,
        }))}
      />
      {entries.map((entry, index) => (
        <div key={index} hidden={index !== selected}>
          <AccessCard index={index} entry={entry} onOp={run} />
        </div>
      ))}
      <Card title="Ducts">
        <HelpNote>
          NACA cooling inlets on the fuselage skin. The lip sits at X, Clock turns it around the body (+y = 0°, +z = 90° on top), and the
          ratios derive the throat width, ramp length and ramp angle. The Simple view shows the surface cut rim; 3D shows the wrapped flow
          volume. A duct on or close to a section cut is a definition error; rods and interior volumes it reaches are flagged.
        </HelpNote>
        <div className="button-row">
          <button type="button" onClick={() => void runDuct({ op: 'duct.add' })}>
            Add Duct
          </button>
        </div>
        {ductNotice !== '' && <div className="field-error">{ductNotice}</div>}
      </Card>
      <ComponentPicker
        kind="duct"
        items={ducts.map((duct, index) => ({
          name: String(duct.name ?? `duct ${index + 1}`),
          detail: `${typeof duct.type === 'string' ? DUCT_TYPE_LABELS[duct.type] ?? duct.type : 'naca'}${typeof duct.radial === 'number' ? ` ×${duct.radial}` : ''}`,
        }))}
      />
      {ducts.map((duct, index) => (
        <div key={index} hidden={index !== ductSelected}>
          <DuctCard index={index} duct={duct} onOp={runDuct} />
        </div>
      ))}
    </>
  )
}

interface AccessCardProps {
  index: number
  entry: Record<string, unknown>
  onOp: (body: Record<string, unknown>) => Promise<void>
}

/** One saved entry: editor fields in mm and degrees, with the structural picks owned by their ops. */
function AccessCard({ index, entry, onOp }: AccessCardProps): ReactElement {
  const api = useFormApi()
  const path = ['access', index]
  const kind = typeof entry.kind === 'string' ? entry.kind : 'hole'
  const profile = typeof entry.profile === 'string' ? entry.profile : 'slot'
  const extent = typeof entry.extent === 'string' ? entry.extent : 'wall'
  const radial = isObject(entry.radial) ? entry.radial : null

  return (
    <Card title={typeof entry.name === 'string' ? entry.name : `access ${index + 1}`} onDelete={() => void onOp({ op: 'access.delete', index })}>
      <EntryField path={[...path, 'name']} unitKey="name" label="Name" />
      <ChoiceField
        label="Type"
        path={[...path, 'kind']}
        value={kind}
        options={api.context.access_kinds}
        optionLabels={KIND_LABELS}
        onValue={(value) => void onOp({ op: 'access.kind', index, kind: value })}
      />
      <Group label="Profile">
        <ChoiceField
          label="Shape"
          path={[...path, 'profile']}
          value={profile}
          options={api.context.access_profiles}
          optionLabels={PROFILE_LABELS}
          onValue={(value) => void onOp({ op: 'access.profile', index, profile: value })}
        />
        <EntryField path={[...path, 'width_m']} unitKey="width_m" label="Width (mm)" />
        <EntryField path={[...path, 'length_m']} unitKey="length_m" label="Length (mm)" />
        {profile === 'rounded_rectangle' ? (
          <EntryField path={[...path, 'corner_radius_m']} unitKey="corner_radius_m" label="Corner radius (mm)" />
        ) : (
          <HelpNote>Length includes the rounded ends. Equal width and length makes a circle.</HelpNote>
        )}
        {kind === 'hatch' && (
          <>
            <EntryField path={[...path, 'gap_m']} unitKey="gap_m" label="Gap (mm)" />
            <HelpNote>Clearance on each edge in the profile plane; fasteners and hinges are deferred.</HelpNote>
          </>
        )}
      </Group>
      <PlacementTable
        positionLabel="Centre"
        position={[
          <EntryField key="x" path={[...path, 'x_m']} unitKey="x_m" label="Centre X (mm)" showLabel={false} />,
          <EntryField key="y" path={[...path, 'y_m']} unitKey="y_m" label="Centre Y (mm)" showLabel={false} optional blankHint="blank = 0" />,
          <EntryField key="z" path={[...path, 'z_m']} unitKey="z_m" label="Centre Z (mm)" showLabel={false} optional blankHint="blank = 0" />,
        ]}
        angles={[
          <EntryField key="roll" path={[...path, 'roll_deg']} unitKey="roll_deg" label="Roll (deg)" showLabel={false} optional blankHint="blank = 0" />,
          <EntryField key="pitch" path={[...path, 'pitch_deg']} unitKey="pitch_deg" label="Pitch (deg)" showLabel={false} optional blankHint="blank = 0" />,
          <EntryField key="yaw" path={[...path, 'yaw_deg']} unitKey="yaw_deg" label="Yaw (deg)" showLabel={false} optional blankHint="blank = 0" />,
        ]}
      />
      <HelpNote>Profile centre; roll about X, then pitch about Y, then yaw about Z.</HelpNote>
      <Group label="Cut extent">
        <ChoiceField
          label="Mode"
          path={[...path, 'extent']}
          value={extent}
          options={api.context.access_extents}
          optionLabels={EXTENT_LABELS}
          onValue={(value) => void onOp({ op: 'access.extent', index, extent: value })}
        />
        {extent === 'depth' && <EntryField path={[...path, 'depth_m']} unitKey="depth_m" label="Depth (mm)" />}
        <HelpNote>{EXTENT_HINTS[extent] ?? ''}</HelpNote>
      </Group>
      <OptionalBlock label="Radial pattern" present={radial !== null} onToggle={(enabled) => void onOp({ op: 'access.radial', index, enabled })}>
        <EntryField path={[...path, 'radial', 'count']} unitKey="count" label="Count (including original)" />
        <ChoiceField
          label="Axis"
          path={[...path, 'radial', 'axis']}
          value={radial === null ? 'x' : String(radial.axis ?? 'x')}
          options={api.context.choices.axis ?? ['x', 'y', 'z']}
        />
        <PlacementTable
          label="Axis position"
          position={[
            <EntryField key="x" path={[...path, 'radial', 'x_m']} unitKey="x_m" label="Axis X (mm)" showLabel={false} />,
            <EntryField key="y" path={[...path, 'radial', 'y_m']} unitKey="y_m" label="Axis Y (mm)" showLabel={false} />,
            <EntryField key="z" path={[...path, 'radial', 'z_m']} unitKey="z_m" label="Axis Z (mm)" showLabel={false} />,
          ]}
        />
        <HelpNote>Axis position locates a point on the rotation axis. Copies are evenly spaced around a full circle; position and orientation rotate together.</HelpNote>
      </OptionalBlock>
    </Card>
  )
}

interface DuctCardProps {
  index: number
  duct: Record<string, unknown>
  onOp: (body: Record<string, unknown>) => Promise<void>
}

/** One NACA duct: placement, base sizes, ratios (derived values shown as text) and the radial pattern count. */
function DuctCard({ index, duct, onOp }: DuctCardProps): ReactElement {
  const api = useFormApi()
  const path = ['ducts', index]
  const type = typeof duct.type === 'string' ? duct.type : 'naca'
  const radial = typeof duct.radial === 'number' ? duct.radial : null
  const throatDepth = typeof duct.throat_depth_m === 'number' ? duct.throat_depth_m : 0.007
  const widthToDepth = typeof duct.width_to_depth === 'number' ? duct.width_to_depth : 4
  const lengthToWidth = typeof duct.length_to_width === 'number' ? duct.length_to_width : 2
  const throatWidth = widthToDepth * throatDepth
  const rampLength = lengthToWidth * throatWidth
  const rampAngle = rampLength > 0 ? (Math.atan2(throatDepth, rampLength) * 180) / Math.PI : 0

  return (
    <Card title={typeof duct.name === 'string' ? duct.name : `duct ${index + 1}`} onDelete={() => void onOp({ op: 'duct.delete', index })}>
      <EntryField path={[...path, 'name']} unitKey="name" label="Name" />
      <ChoiceField
        label="Type"
        path={[...path, 'type']}
        value={type}
        options={api.context.duct_types}
        optionLabels={DUCT_TYPE_LABELS}
      />
      <Group label="Placement">
        <EntryField path={[...path, 'x_m']} unitKey="x_m" label="Lip X (mm)" />
        <EntryField path={[...path, 'clock_deg']} unitKey="clock_deg" label="Clock (deg)" />
        <HelpNote>Clock runs around the body from +y (0°) toward +z (90°, top). In the Simple view, drag the rim along the body (Longitudinal) or around it (Front).</HelpNote>
      </Group>
      <Group label="Base sizes">
        <EntryField path={[...path, 'throat_depth_m']} unitKey="throat_depth_m" label="Throat depth (mm)" />
        <EntryField path={[...path, 'inlet_width_m']} unitKey="inlet_width_m" label="Inlet width (mm)" />
        <EntryField path={[...path, 'tunnel_length_m']} unitKey="tunnel_length_m" label="Tunnel length (mm)" />
        <EntryField path={[...path, 'wall_m']} unitKey="wall_m" label="Wall (mm)" />
      </Group>
      <Group label="Ratios">
        <EntryField path={[...path, 'width_to_depth']} unitKey="width_to_depth" label="Width : depth" />
        <EntryField path={[...path, 'length_to_width']} unitKey="length_to_width" label="Length : width" />
        <HelpNote>
          Derived: throat {`${(throatWidth * 1000).toFixed(1)} mm`}, ramp {`${(rampLength * 1000).toFixed(1)} mm`}, ramp angle {rampAngle.toFixed(2)}°.
          The inlet must be narrower than the throat.
        </HelpNote>
      </Group>
      <OptionalBlock label="Radial pattern" present={radial !== null} onToggle={(enabled) => void onOp({ op: 'duct.radial', index, enabled })}>
        <EntryField path={[...path, 'radial']} unitKey="count" label="Count (including original)" />
        <HelpNote>Copies are evenly spaced a full circle about the body axis from the clock angle; they share the lip station and dimensions.</HelpNote>
      </OptionalBlock>
    </Card>
  )
}

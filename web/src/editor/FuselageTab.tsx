import { useState } from 'react'
import type { FormEvent, ReactElement } from 'react'
import { editorOp } from '../store'
import type { Snapshot } from '../types'
import { Modal } from '../components/Modal'
import { Card, CheckboxField, ChoiceField, EntryField, Fields } from './fields'
import { fieldLabel } from './format'
import { isObject } from './patch'
import { reinforcementForTick } from './reinforcement'
import { useFormApi } from './session'

const ROD_ENTRY_KEYS = ['diameter_m', 'radial_position_m', 'polar_n', 'polar_offset_angle', 'polar_split', 'polar_split_angle', 'from_x_m', 'to_x_m']

/** editor_view.build_fuselage_tab (684-704): preset group, fuselage blocks, reinforcement. */
export function FuselageTab({ snapshot, onPresetsChanged }: { snapshot: Snapshot; onPresetsChanged: () => Promise<void> }): ReactElement {
  const api = useFormApi()
  const context = api.context
  const definition = api.definition
  const fuselage = isObject(definition.fuselage) ? definition.fuselage : {}
  const [choice, setChoice] = useState<string>(() => (typeof definition.preset === 'string' && definition.preset !== '' ? definition.preset : context.no_preset))
  const [notice, setNotice] = useState('')
  const [confirmOverwrite, setConfirmOverwrite] = useState<string | null>(null)
  const [newPresetOpen, setNewPresetOpen] = useState(false)

  /** editor_view.load_preset (721-727) and save_preset_as (729-743), through the server's structural ops. */
  const runPresetOp = async (body: Record<string, unknown>): Promise<boolean> => {
    const result = await editorOp(snapshot.session_id, body)
    if (!result.ok) {
      setNotice(result.message)
      return false
    }
    setNotice('')
    await onPresetsChanged()
    return true
  }

  const load = (): void => {
    void runPresetOp({ op: 'preset.load', name: choice })
  }

  const save = (): void => {
    if (choice === context.no_preset) {
      setNotice('Pick a preset to save over, or New')
      return
    }
    setConfirmOverwrite(choice)
  }

  const overwrite = (): void => {
    const name = confirmOverwrite
    setConfirmOverwrite(null)
    if (name === null) return
    void runPresetOp({ op: 'preset.save', name, overwrite: true })
  }

  const create = (name: string): void => {
    setNewPresetOpen(false)
    if (name === context.no_preset || context.preset_names.includes(name)) {
      setNotice(`A preset named ${name} already exists.`)
      return
    }
    void runPresetOp({ op: 'preset.new', name }).then((ok) => {
      if (ok) setChoice(name)
    })
  }

  /** editor_view.reset_profile_params (849-850), through the server's `fuselage.profile` op: profile and its params
   * are one definition mutation, so no recompute ever sees the old profile's params under the new profile. */
  const resetProfile = (end: 'nose' | 'tail', profile: string): void => {
    void editorOp(api.sessionId, { op: 'fuselage.profile', end, profile })
  }

  /** editor_view.toggle_reinforcement (772-789): unticking stashes the block; ticking stores what
   * reinforcementForTick picks (the stash as it was, or a default-extent preset/new pattern). */
  const toggleReinforcement = (enabled: boolean): void => {
    const currentFuselage = isObject(definition.fuselage) ? definition.fuselage : {}
    if (!enabled) {
      if (currentFuselage.reinforcement !== undefined) api.stash.stashedReinforcement = currentFuselage.reinforcement
      void api.remove(['fuselage', 'reinforcement'])
      return
    }
    const presetName = typeof definition.preset === 'string' ? definition.preset : null
    void api.set(['fuselage', 'reinforcement'], reinforcementForTick(currentFuselage, api.stash.stashedReinforcement, presetName, context))
  }

  const rods = isObject(fuselage.reinforcement) ? fuselage.reinforcement : null
  const section = isObject(fuselage.section) ? fuselage.section : {}
  const joints = isObject(fuselage.joints) ? fuselage.joints : {}
  const materials = Object.keys(context.parts_data.material ?? {}).sort()

  const renderEnd = (end: 'nose' | 'tail'): ReactElement => {
    const data = isObject(fuselage[end]) ? fuselage[end] : {}
    const params = isObject(data.params) ? data.params : {}
    return (
      <fieldset className="form-group" key={end}>
        <legend>{end}</legend>
        <ChoiceField path={['fuselage', end, 'profile']} value={data.profile} options={context.choices.profile ?? []} onValue={(profile) => resetProfile(end, profile)} />
        <EntryField path={['fuselage', end, 'length_m']} unitKey="length_m" />
        <Fields data={params} path={['fuselage', end, 'params']} context={context} />
      </fieldset>
    )
  }

  return (
    <>
      <Card title="Size preset (data/presets/)">
        <div className="form-row">
          <span className="form-label">aircraft</span>
          <span className="preset-status">{snapshot.preset_status}</span>
        </div>
        <div className="form-row">
          <label className="form-label" htmlFor="preset-choice">
            preset
          </label>
          <div className="form-control">
            <select id="preset-choice" className="field-input" value={choice} onChange={(event) => setChoice(event.target.value)}>
              <option value={context.no_preset}>{context.no_preset}</option>
              {context.preset_names.map((name) => (
                <option key={name} value={name}>
                  {name}
                </option>
              ))}
            </select>
          </div>
        </div>
        <div className="button-row">
          <button type="button" onClick={load}>
            Load
          </button>
          <button type="button" onClick={save}>
            Save
          </button>
          <button type="button" onClick={() => setNewPresetOpen(true)}>
            New
          </button>
        </div>
        {notice !== '' && <div className="field-error">{notice}</div>}
      </Card>

      <Card title="Fuselage">
        <EntryField path={['fuselage', 'length_m']} unitKey="length_m" />
        <fieldset className="form-group">
          <legend>section</legend>
          <Fields data={section} path={['fuselage', 'section']} context={context} />
        </fieldset>
        {renderEnd('nose')}
        {renderEnd('tail')}
        <fieldset className="form-group">
          <legend>joints</legend>
          <Fields data={joints} path={['fuselage', 'joints']} context={context} />
        </fieldset>
      </Card>

      <Card title="Reinforcement (longitudinal rods)">
        <CheckboxField path={['fuselage', 'reinforcement']} checked={rods !== null} label="rods" onToggle={toggleReinforcement} />
        {rods !== null && (
          <>
            <ChoiceField path={['fuselage', 'reinforcement', 'type']} value={rods.type} options={context.reinforcement_types} label={fieldLabel('type', context)} />
            <ChoiceField
              path={['fuselage', 'reinforcement', 'material']}
              value={rods.material}
              options={materials}
              label={fieldLabel('material', context)}
            />
            {ROD_ENTRY_KEYS.map((key) => (
              <EntryField key={key} path={['fuselage', 'reinforcement', key]} unitKey={key} />
            ))}
            <p className="form-hint">
              one rod position per quadrant, polar_offset_angle (deg) up from the horizontal, mirrored left/right and top/bottom; polar_split 2 = a pair at
              +/- polar_split_angle (deg), 0 = single; polar_n = all rods
            </p>
          </>
        )}
      </Card>

      {confirmOverwrite !== null && (
        <Modal title="Save preset" onClose={() => setConfirmOverwrite(null)}>
          <p className="modal-copy">
            Overwrite preset {confirmOverwrite} (data/presets/{confirmOverwrite}.json) with this aircraft&apos;s section, joints and rods?
            <br />
            Every aircraft labelled {confirmOverwrite} then shows as modified where it differs. Its spars, limits and CAD data stay.
          </p>
          <div className="modal-actions">
            <button type="button" onClick={() => setConfirmOverwrite(null)}>
              Cancel
            </button>
            <button type="button" className="action" onClick={overwrite}>
              Overwrite
            </button>
          </div>
        </Modal>
      )}
      {newPresetOpen && <NewPresetDialog onClose={() => setNewPresetOpen(false)} onCreate={create} />}
    </>
  )
}

/** editor_view.new_preset (745-752), with the simpledialog replaced by a modal. */
function NewPresetDialog({ onClose, onCreate }: { onClose: () => void; onCreate: (name: string) => void }): ReactElement {
  const [name, setName] = useState('')
  const submit = (event: FormEvent<HTMLFormElement>): void => {
    event.preventDefault()
    const trimmed = name.trim()
    if (trimmed === '') return
    onCreate(trimmed)
  }
  return (
    <Modal title="New preset" onClose={onClose}>
      <form className="modal-form" onSubmit={submit}>
        <label className="modal-label" htmlFor="new-preset-name">
          Preset name (e.g. tube62)
        </label>
        <input id="new-preset-name" value={name} autoFocus spellCheck={false} onChange={(event) => setName(event.target.value)} />
        <div className="modal-actions">
          <button type="button" onClick={onClose}>
            Cancel
          </button>
          <button type="submit" className="action" disabled={name.trim() === ''}>
            Save
          </button>
        </div>
      </form>
    </Modal>
  )
}

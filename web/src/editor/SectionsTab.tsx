import { useState } from 'react'
import type { ReactElement } from 'react'
import { editorOp } from '../store'
import type { ResolvedSection, Snapshot } from '../types'
import { Card, CheckboxField, ChoiceField, EntryField } from './fields'
import { isObject } from './patch'
import { useFormApi } from './session'

/** refresh_fuselage_status (399-409): the computed strings, from the resolved snapshot sections. */
function rangeLabel(index: number, section: ResolvedSection): string {
  return `S${index + 1} ${(section.x0_m * 1000).toFixed(0)}-${(section.x1_m * 1000).toFixed(0)}`
}

function emptyLabel(section: ResolvedSection | undefined): string {
  return section === undefined || section.empty_estimate_kg === null ? '-' : (section.empty_estimate_kg * 1000).toFixed(1)
}

function itemisedLabel(section: ResolvedSection | undefined): string {
  if (section === undefined) return ''
  const [low, high] = section.complete_itemised_kg
  return low === high ? (low * 1000).toFixed(1) : `${(low * 1000).toFixed(1)}-${(high * 1000).toFixed(1)}`
}

/** editor_view.build_sections_group (791-819): one row per section, cuts and joints for all but the last. */
export function SectionsTab({ snapshot }: { snapshot: Snapshot }): ReactElement {
  const api = useFormApi()
  const context = api.context
  const definition = api.definition
  const [notice, setNotice] = useState('')
  const sections = isObject(definition.sections) ? definition.sections : {}
  const cuts = Array.isArray(sections.cuts_m) ? sections.cuts_m : []
  const cutJoints = Array.isArray(sections.cut_joints) ? sections.cut_joints : []
  const perSection = Array.isArray(sections.per_section) ? sections.per_section.filter(isObject) : []
  const resolved = snapshot.resolved?.sections ?? []
  const resolvedMatches = resolved.length === perSection.length

  const run = async (body: Record<string, unknown>): Promise<void> => {
    const result = await editorOp(snapshot.session_id, body)
    setNotice(result.ok ? '' : result.message)
  }

  return (
    <Card title="Sections (cut lines; drag them in the side view)">
      <div className="button-row">
        <button type="button" onClick={() => void run({ op: 'section.add_cut' })}>
          Add cut
        </button>
      </div>
      {notice !== '' && <div className="field-error">{notice}</div>}
      <div className="section-grid">
        <span className="section-head">section mm</span>
        <span className="section-head">cut mm</span>
        <span className="section-head">joint</span>
        <span className="section-head">solid</span>
        <span className="section-head">empty g</span>
        <span className="section-head">calc</span>
        <span className="section-head">complete g</span>
        <span className="section-head">items</span>
        <span className="section-head" />
        {perSection.map((row, index) => {
          const section = resolvedMatches ? resolved[index] : undefined
          const hasCut = index < cuts.length
          return (
            <SectionRow
              key={index}
              index={index}
              hasCut={hasCut}
              label={section === undefined ? '' : rangeLabel(index, section)}
              joint={hasCut ? cutJoints[index] : undefined}
              jointOptions={context.cut_joints}
              computedEmpty={emptyLabel(section)}
              computedItems={itemisedLabel(section)}
              solid={row.solid === true}
              onDelete={() => void run({ op: 'section.delete_cut', index })}
            />
          )
        })}
      </div>
      <p className="form-hint">
        cut mm: the section&apos;s end. Blank empty = calc (hollow: shell wall; solid: less its volumes). Blank complete = items: everything attached in
        the section, which a weighed complete mass replaces.
      </p>
      {!resolvedMatches && resolved.length > 0 && <p className="form-hint">The drawing shows the last valid airframe; computed masses are for it.</p>}
    </Card>
  )
}

interface SectionRowProps {
  index: number
  hasCut: boolean
  label: string
  joint: unknown
  jointOptions: string[]
  computedEmpty: string
  computedItems: string
  solid: boolean
  onDelete: () => void
}

function SectionRow({ index, hasCut, label, joint, jointOptions, computedEmpty, computedItems, solid, onDelete }: SectionRowProps): ReactElement {
  return (
    <>
      <span className="section-label">{label}</span>
      {hasCut ? (
        <>
          <EntryField path={['sections', 'cuts_m', index]} unitKey="cuts_m" showLabel={false} width={5} />
          <ChoiceField path={['sections', 'cut_joints', index]} value={joint} options={jointOptions} showLabel={false} />
        </>
      ) : (
        <>
          <span />
          <span />
        </>
      )}
      <CheckboxField path={['sections', 'per_section', index, 'solid']} checked={solid} />
      <EntryField path={['sections', 'per_section', index, 'empty_mass_kg']} unitKey="empty_mass_kg" optional showLabel={false} width={5} />
      <span className="section-computed">{computedEmpty}</span>
      <EntryField path={['sections', 'per_section', index, 'complete_mass_kg']} unitKey="complete_mass_kg" optional showLabel={false} width={5} />
      <span className="section-computed">{computedItems}</span>
      {hasCut ? (
        <button type="button" className="danger section-del" onClick={onDelete}>
          Del
        </button>
      ) : (
        <span />
      )}
    </>
  )
}

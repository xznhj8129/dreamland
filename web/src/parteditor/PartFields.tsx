import { useId } from 'react'
import type { ChangeEvent, ReactElement } from 'react'
import type { LibraryPayload } from '../types'
import { formatLibraryValue, libraryFieldLabel, pathKeyOf, VECTOR_AXES } from './library'
import type { LibraryNode } from './library'

export interface FieldEdit {
  text?: string
  checked?: boolean
  components?: string[]
}

interface FieldNodeProps {
  node: LibraryNode
  payload: LibraryPayload
  edits: Record<string, FieldEdit>
  errors: Record<string, string>
  onEdit: (key: string, patch: FieldEdit) => void
}

/** One node of the entry's field tree: a labelled group, a check box or a text entry with units and inline error. */
export function PartFieldNode({ node, payload, edits, errors, onEdit }: FieldNodeProps): ReactElement {
  if (node.kind === 'group') {
    const keyOf = pathKeyOf(node.path)
    const label = libraryFieldLabel(node.key, payload)
    const children = node.children.map((child) => (
      <PartFieldNode key={pathKeyOf(child.path)} node={child} payload={payload} edits={edits} errors={errors} onEdit={onEdit} />
    ))
    // Top-level dicts are plain labelled groups; a dict inside a dict is collapsible (the "deep" case).
    if (node.path.length >= 2) {
      return (
        <details className="form-group part-group" open>
          <summary className="part-group-title">{label}</summary>
          {children}
        </details>
      )
    }
    return (
      <fieldset className="form-group part-group" data-path={keyOf}>
        <legend>{label}</legend>
        {children}
      </fieldset>
    )
  }
  return <FieldRow node={node} payload={payload} edits={edits} errors={errors} onEdit={onEdit} />
}

interface FieldRowProps {
  node: Extract<LibraryNode, { kind: 'leaf' }>
  payload: LibraryPayload
  edits: Record<string, FieldEdit>
  errors: Record<string, string>
  onEdit: (key: string, patch: FieldEdit) => void
}

function FieldRow({ node, payload, edits, errors, onEdit }: FieldRowProps): ReactElement {
  const id = useId()
  const key = pathKeyOf(node.path)
  const edit = edits[key]
  const error = errors[key]
  const label = libraryFieldLabel(node.key, payload)

  if (typeof node.value === 'boolean') {
    const checked = edit?.checked ?? node.value
    return (
      <div className="form-row">
        <span className="form-label">{label}</span>
        <div className="form-control">
          <label className="check-row">
            <input type="checkbox" checked={checked} onChange={(event) => onEdit(key, { checked: event.target.checked })} />
            <span className="part-bool-value">{checked ? 'Yes' : 'No'}</span>
          </label>
        </div>
      </div>
    )
  }

  const axes = VECTOR_AXES[node.key]
  if (axes && Array.isArray(node.value) && node.value.length === axes.length) {
    const components = edit?.components ?? node.value.map(String)
    return <fieldset className="form-group library-vector">
      <legend>{label}</legend>
      <div className="vector-fields">{axes.map((axis, index) => <label key={axis}>
        <span>{axis}</span><input className="field-input" value={components[index]} aria-label={`${label} ${axis}`}
          onChange={event => onEdit(key, { components: components.map((value, position) => position === index ? event.target.value : value) })} />
      </label>)}</div>
      {error !== undefined && <div className="field-error">{error}</div>}
    </fieldset>
  }
  const text = edit?.text ?? formatLibraryValue(node.key, node.value, payload)
  const onChange = (event: ChangeEvent<HTMLInputElement>): void => onEdit(key, { text: event.target.value })
  return (
    <div className="form-row">
      <label className="form-label" htmlFor={id} title={label}>
        {label}
      </label>
      <div className="form-control">
        <input
          id={id}
          className={error === undefined ? 'field-input' : 'field-input field-invalid'}
          value={text}
          spellCheck={false}
          aria-invalid={error !== undefined}
          title={node.optionalKind !== null ? 'blank = left out' : undefined}
          aria-description={node.optionalKind !== null ? 'blank = left out' : undefined}
          onChange={onChange}
        />
        {error !== undefined && <div className="field-error">{error}</div>}
      </div>
    </div>
  )
}

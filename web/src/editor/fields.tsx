import { useEffect, useId, useRef, useState } from 'react'
import type { ChangeEvent, KeyboardEvent as ReactKeyboardEvent, ReactElement, ReactNode } from 'react'
import type { EditorContext } from '../types'
import { FieldParseError, displayFactor, fieldLabel, formatValue, parseValue } from './format'
import { getIn, isObject } from './patch'
import type { Path } from './patch'
import { useFormApi } from './session'

interface EntryFieldProps {
  path: Path
  /** The definition key that names the units (a list index's entry uses the list key). */
  unitKey: string
  label?: string
  optional?: boolean
  blankHint?: string
  showLabel?: boolean
  width?: number
  /** A change's extra work (editor_view.make_entry on_commit: a rename carries to references). Called with the
   * previous and the new value once the queued set has landed; a removal passes null as the new value. */
  onCommit?: (previous: unknown, next: unknown) => void
  /** A commit's replacement for the queued set: a structural op that owns the change (a rename that must carry to
   * references, or a volume change). The text is parsed and validated as usual; no set is sent. */
  onValue?: (value: unknown) => void
}

/**
 * One text entry, editor_view.make_entry (422-472): unit-scaled display, %g formatting, strings and whole
 * numbers as such, "a-b" mass ranges, blank removes an optional key, invalid input reverts with an inline
 * error (not a Log entry), commits on Enter and blur, wheel steps numbers. While focused the typed text
 * wins; otherwise it syncs from the definition (so ordinary commits never remount or lose focus).
 */
export function EntryField({
  path,
  unitKey,
  label,
  optional = false,
  blankHint,
  showLabel = true,
  width,
  onCommit,
  onValue,
}: EntryFieldProps): ReactElement {
  const api = useFormApi()
  const id = useId()
  const value = getIn(api.definition, path)
  const [text, setText] = useState(() => formatValue(unitKey, value, api.context))
  const [focused, setFocused] = useState(false)
  const [error, setError] = useState('')
  const textRef = useRef(text)
  const committedRef = useRef(text)
  const latestRef = useRef(value)
  const inputRef = useRef<HTMLInputElement | null>(null)

  useEffect(() => {
    latestRef.current = value
    if (focused) return
    const formatted = formatValue(unitKey, value, api.context)
    committedRef.current = formatted
    textRef.current = formatted
    setText(formatted)
    setError('')
  }, [value, focused, unitKey, api.context])

  const commitText = (raw: string): void => {
    if (raw.trim() === committedRef.current.trim()) return
    const previous = latestRef.current
    if (optional && raw.trim() === '') {
      committedRef.current = raw
      latestRef.current = undefined
      setError('')
      void api.remove(path).then((result) => {
        if (result.ok) onCommit?.(previous, null)
      })
      return
    }
    const existing = latestRef.current ?? 0.0
    try {
      const parsed = parseValue(unitKey, raw, existing, api.context)
      committedRef.current = raw
      latestRef.current = parsed
      setError('')
      if (onValue !== undefined) {
        onValue(parsed)
        return
      }
      void api.set(path, parsed).then((result) => {
        if (result.ok) onCommit?.(previous, parsed)
      })
    } catch (parseError) {
      const message = parseError instanceof FieldParseError ? parseError.message : 'not a valid value'
      setError(`${label ?? fieldLabel(unitKey, api.context)}: ${message}`)
      textRef.current = committedRef.current
      setText(committedRef.current)
    }
  }

  const stepBy = (step: number): void => {
    const current = latestRef.current
    if (typeof current !== 'number') return
    let next: number
    if (api.context.integer_keys.includes(unitKey) && Number.isInteger(current)) {
      next = current + step
    } else {
      // Fixed wheel steps, independent of magnitude: 1 mm on mm-displayed fields, 0.1 otherwise.
      const stepDisplay = api.context.mm_keys.includes(unitKey) ? 1.0 : 0.1
      next = Math.round((current + (step * stepDisplay) / displayFactor(unitKey, api.context)) * 1e9) / 1e9
    }
    const formatted = formatValue(unitKey, next, api.context)
    textRef.current = formatted
    setText(formatted)
    commitText(formatted)
  }

  const stepRef = useRef(stepBy)
  stepRef.current = stepBy
  useEffect(() => {
    const input = inputRef.current
    if (input === null) return undefined
    const onWheel = (event: WheelEvent): void => {
      event.preventDefault()
      stepRef.current(event.deltaY < 0 ? 1 : -1)
    }
    input.addEventListener('wheel', onWheel, { passive: false })
    return () => input.removeEventListener('wheel', onWheel)
  }, [])

  const onChange = (event: ChangeEvent<HTMLInputElement>): void => {
    textRef.current = event.target.value
    setText(event.target.value)
  }

  return (
    <div className={showLabel ? 'form-row' : 'form-row form-row-bare'}>
      {showLabel && (
        <label className="form-label" htmlFor={id}>
          {label ?? fieldLabel(unitKey, api.context)}
        </label>
      )}
      <div className="form-control">
        <input
          ref={inputRef}
          id={id}
          type="text"
          className={error === '' ? 'field-input' : 'field-input field-invalid'}
          style={width === undefined ? undefined : { width: `${width + 2}ch` }}
          value={text}
          spellCheck={false}
          autoComplete="off"
          onChange={onChange}
          onFocus={() => setFocused(true)}
          onBlur={() => {
            setFocused(false)
            commitText(textRef.current)
          }}
          onKeyDown={(event: ReactKeyboardEvent<HTMLInputElement>) => {
            if (event.key === 'Enter') {
              event.preventDefault()
              commitText(textRef.current)
            }
          }}
        />
        {error !== '' && <div className="field-error">{error}</div>}
      </div>
      {showLabel && optional && (
        <span className="form-hint">{blankHint ?? (api.context.gram_keys.includes(unitKey) ? 'blank = computed' : 'blank = none')}</span>
      )}
    </div>
  )
}

interface ChoiceFieldProps {
  path: Path
  value: unknown
  options: string[]
  label?: string
  unitKey?: string
  showLabel?: boolean
  width?: number
  /** How a pick's text becomes the stored value, editor_view.make_choice convert (str by default, int for
   * gauge/awg fields). */
  convert?: (value: string) => unknown
  /** A pick's extra work (editor_view.make_choice on_change); by default the value is stored through convert. */
  onValue?: (value: string) => void
}

/** Readonly combobox, editor_view.make_choice (484-498): a pick stores convert(text) and recomputes. */
export function ChoiceField({ path, value, options, label, unitKey, showLabel = true, width, convert, onValue }: ChoiceFieldProps): ReactElement {
  const api = useFormApi()
  const id = useId()
  const current = value === null || value === undefined ? '' : String(value)
  const shown = current !== '' && !options.includes(current) ? [current, ...options] : options
  const select = (next: string): void => {
    if (onValue !== undefined) {
      onValue(next)
      return
    }
    void api.set(path, convert === undefined ? next : convert(next))
  }
  return (
    <div className={showLabel ? 'form-row' : 'form-row form-row-bare'}>
      {showLabel && (
        <label className="form-label" htmlFor={id}>
          {label ?? fieldLabel(unitKey ?? String(path[path.length - 1]), api.context)}
        </label>
      )}
      <div className="form-control" style={width === undefined ? undefined : { maxWidth: `${width + 4}ch` }}>
        <select id={id} className="field-input" value={current} onChange={(event) => select(event.target.value)}>
          {shown.map((option) => (
            <option key={option} value={option}>
              {option}
            </option>
          ))}
        </select>
      </div>
    </div>
  )
}

interface CheckboxFieldProps {
  /** Absent when onToggle owns the change (an optional block toggle). */
  path?: Path
  checked: boolean
  label?: string
  onToggle?: (checked: boolean) => void
}

/** Checkbutton; a toggle stores the boolean and recomputes (editor_view.set_solid 821-823 and friends). */
export function CheckboxField({ path, checked, label, onToggle }: CheckboxFieldProps): ReactElement {
  const api = useFormApi()
  const toggle = (next: boolean): void => {
    if (onToggle !== undefined) {
      onToggle(next)
      return
    }
    if (path !== undefined) void api.set(path, next)
  }
  return (
    <label className="check-row">
      <input type="checkbox" checked={checked} onChange={(event) => toggle(event.target.checked)} />
      {label !== undefined && <span>{label}</span>}
    </label>
  )
}

/** A labelled frame inside a form (ttk.LabelFrame). */
export function Group({ label, children }: { label: string; children: ReactNode }): ReactElement {
  return (
    <fieldset className="form-group">
      <legend>{label}</legend>
      {children}
    </fieldset>
  )
}

interface OptionalBlockProps {
  label: string
  present: boolean
  onToggle: (checked: boolean) => void
  children: ReactNode
}

/** editor_view.optional_block (1044-1060): a tick box that adds (from the template) or removes an optional block and
 * shows its fields when present (spars with a nested carry-through, root base, pod). */
export function OptionalBlock({ label, present, onToggle, children }: OptionalBlockProps): ReactElement {
  return (
    <>
      <CheckboxField checked={present} label={label} onToggle={onToggle} />
      {present && <Group label={label}>{children}</Group>}
    </>
  )
}

/** A top-level form card with a Delete button, editor_view.card (524-529). */
export function Card({ title, onDelete, children }: { title: string; onDelete?: () => void; children: ReactNode }): ReactElement {
  return (
    <section className="form-card">
      <header className="form-card-header">
        <h3>{title}</h3>
        {onDelete !== undefined && (
          <button type="button" className="danger" onClick={onDelete}>
            Delete
          </button>
        )}
      </header>
      {children}
    </section>
  )
}

/** editor_view.add_vector_entries (474-477): one entry per axis under the vector's own label. */
export function VectorField({ path, unitKey, axes, label }: { path: Path; unitKey: string; axes: string[]; label?: string }): ReactElement {
  const api = useFormApi()
  return (
    <>
      {axes.map((axis, index) => (
        <EntryField key={axis} path={[...path, index]} unitKey={unitKey} label={`${label ?? fieldLabel(unitKey, api.context)} ${axis}`} />
      ))}
    </>
  )
}

interface PlacementFieldsProps {
  path: Path
  item: Record<string, unknown>
  volumeNames: string[]
  /** The volume pick's owner: editor_view.set_part_volume (1209-1220), through the part or battery op. */
  onVolume: (volume: string) => void
}

/**
 * editor_view.add_placement (1190-1207): volume dropdown (NO_VOLUME first) and the offsets in it, or the item's
 * own x, y, z, then its yaw, pitch, roll. Shared by the Parts form and the Propulsion battery.
 */
export function PlacementFields({ path, item, volumeNames, onVolume }: PlacementFieldsProps): ReactElement {
  const api = useFormApi()
  const inVolume = typeof item.volume === 'string'
  return (
    <>
      <ChoiceField
        label="volume"
        path={[...path, 'volume']}
        value={inVolume ? item.volume : api.context.no_volume}
        options={[api.context.no_volume, ...volumeNames]}
        onValue={onVolume}
      />
      {inVolume ? (
        <>
          {['x_offset_m', 'y_offset_m', 'z_offset_m'].map((key) => (
            <EntryField key={key} path={[...path, key]} unitKey={key} optional blankHint="blank = 0" />
          ))}
        </>
      ) : (
        <>
          <EntryField path={[...path, 'x_m']} unitKey="x_m" />
          {['y_m', 'z_m'].map((key) => (
            <EntryField key={key} path={[...path, key]} unitKey={key} optional blankHint="blank = 0" />
          ))}
        </>
      )}
      {api.context.orientation_keys.map((key) => (
        <EntryField key={key} path={[...path, key]} unitKey={key} optional blankHint="blank = 0" />
      ))}
    </>
  )
}

interface FieldsProps {
  data: Record<string, unknown>
  path: Path
  context: EditorContext
  skip?: readonly string[]
  /** A structural choice's op (editor_view.add_fields on_change: layout refits the servos); the value is stored
   * through the hook instead of a plain set. */
  onChoice?: Record<string, (value: string) => void>
}

/**
 * Generic rows for every field of a definition dict in stored order, editor_view.add_fields (500-522):
 * nested dicts become groups, choice keys a readonly combobox, printer bed and volume size vectors, other
 * lists a static "N entries (edit in file)", everything else a text entry.
 */
export function Fields({ data, path, context, skip = [], onChoice }: FieldsProps): ReactElement {
  return (
    <>
      {Object.entries(data).map(([key, value]) => {
        if (skip.includes(key)) return null
        const childPath = [...path, key]
        if (isObject(value)) {
          return (
            <Group key={key} label={fieldLabel(key, context)}>
              <Fields data={value} path={childPath} context={context} />
            </Group>
          )
        }
        if (key in context.choices && typeof value === 'string') {
          return <ChoiceField key={key} path={childPath} unitKey={key} value={value} options={context.choices[key] ?? []} onValue={onChoice?.[key]} />
        }
        if (Array.isArray(value) && (key === 'printer_bed_m' || key === 'size_m')) {
          return <VectorField key={key} path={childPath} unitKey={key} axes={['x', 'y', 'z']} />
        }
        if (Array.isArray(value)) {
          return (
            <div key={key} className="form-row">
              <span className="form-static">{`${fieldLabel(key, context)}: ${value.length} entries (edit in file)`}</span>
            </div>
          )
        }
        return <EntryField key={key} path={childPath} unitKey={key} />
      })}
    </>
  )
}

import { formatG } from '../editor/format'
import type { LibraryOptionalKind, LibraryPayload } from '../types'

/**
 * The part-library field logic, ported from libs/library_editors.py (the Tk PartEditorFrame's module-level
 * helpers). The webapp library module is the server-side port of the same file; this is the browser-side one,
 * so the entries the editor posts carry the types and units the server expects.
 */

export type Values = Record<string, unknown>

/** library_editors.py ordered_keys (75-79): preferred keys present first, the rest sorted. */
export function orderedKeys(values: Values, preferred: readonly string[]): string[] {
  const ordered = preferred.filter((key) => key in values)
  const rest = Object.keys(values).filter((key) => !preferred.includes(key)).sort()
  return [...ordered, ...rest]
}

function isLengthKey(key: string, payload: LibraryPayload): boolean {
  return payload.length_keys.includes(key)
}

function isMassKey(key: string, payload: LibraryPayload): boolean {
  return payload.mass_keys.includes(key)
}

const FIELD_LABELS: Record<string, string> = {
  type: 'Chemistry / type', s: 'Cells in series (S)', p: 'Cells in parallel (P)', mah: 'Capacity (mAh)',
  c: 'Discharge rating (C)', curr: 'Current rating (A)', min_s: 'Minimum series cells (S)', max_s: 'Maximum series cells (S)',
  l: 'Length (mm)', w: 'Width (mm)', h: 'Height (mm)', dimensions: 'Dimensions',
  ir_mohm_per_cell: 'Cell resistance (mΩ)', lead_awg: 'Lead gauge (AWG)', lead_length_mm: 'Lead length (mm)',
  mosfet_rds_on_ohm: 'MOSFET on-resistance (Ω)', contact_resistance_ohm: 'Contact resistance (Ω)',
  wiring_origin_mm: 'Wiring origin (mm)', density_kg_m3: 'Density (kg/m³)', diameter_in: 'Diameter (in)',
  holes_mm: 'Mounting holes (mm, JSON)', outline_mm: 'Outline size (mm)', generic: 'Generic component',
  mount: 'Mounting pattern', notes: 'Notes', connector: 'Connector',
}
export function libraryFieldLabel(key: string, payload: LibraryPayload): string {
  if (key in FIELD_LABELS) return FIELD_LABELS[key]
  const label = key.replace(/_/g, ' ')
  if (isLengthKey(key, payload)) return `${label} (mm)`
  if (isMassKey(key, payload)) return `${label} (g)`
  return label.charAt(0).toUpperCase() + label.slice(1)
}

export const VECTOR_AXES: Record<string, readonly string[]> = { wiring_origin_mm: ['x','y','z'], outline_mm: ['Length','Width'] }
export function parseLibraryVector(components: string[]): number[] {
  return components.map(parseNumber)
}

/** library_editors.py format_field_value (82-91): blank for null, mm/g-scaled %g for numbers, JSON for lists. */
export function formatLibraryValue(key: string, value: unknown, payload: LibraryPayload): string {
  if (value === null || value === undefined) return ''
  if (typeof value === 'number') {
    const scale = isLengthKey(key, payload) || isMassKey(key, payload) ? 1000.0 : 1.0
    return formatG(value * scale)
  }
  if (Array.isArray(value)) return JSON.stringify(value)
  return String(value)
}

/** A library text field could not be parsed; the entry shows the message and Save is not posted. */
export class LibraryParseError extends Error {}

const INTEGER = /^[+-]?\d+$/
const NUMBER = /^[+-]?(\d+(\.\d*)?|\.\d+)([eE][+-]?\d+)?$/

function parseNumber(text: string): number {
  if (!NUMBER.test(text)) throw new LibraryParseError(`not a number: ${text}`)
  const value = Number(text)
  if (!Number.isFinite(value)) throw new LibraryParseError(`not a finite number: ${text}`)
  return value
}

function parseWhole(text: string): number {
  if (!INTEGER.test(text)) throw new LibraryParseError(`not a whole number: ${text}`)
  const value = Number(text)
  if (!Number.isSafeInteger(value)) throw new LibraryParseError(`not a whole number: ${text}`)
  return value
}

function scaled(key: string, value: number, payload: LibraryPayload): number {
  return isLengthKey(key, payload) || isMassKey(key, payload) ? value / 1000.0 : value
}

/**
 * library_editors.py parse_field_value (130-147): text back to the existing value's type. existing null stays
 * null for blank/none/null and text otherwise; numbers stay numbers (whole or decimal), a length/mass key is
 * divided back to m/kg; a list is JSON; a string is taken as typed. An absent optional field (optionalKind,
 * from the payload's optional_fields) parses by that declared type instead of an existing value.
 */
export function parseLibraryValue(
  key: string,
  text: string,
  existing: unknown,
  optionalKind: LibraryOptionalKind | null,
  payload: LibraryPayload,
): unknown {
  const trimmed = text.trim()
  if (optionalKind !== null) {
    if (optionalKind === 'str') return text
    if (optionalKind === 'int') return scaled(key, parseWhole(trimmed), payload)
    return scaled(key, parseNumber(trimmed), payload)
  }
  if (existing === null || existing === undefined) {
    const lower = trimmed.toLowerCase()
    return lower === '' || lower === 'none' || lower === 'null' ? null : trimmed
  }
  if (typeof existing === 'boolean') return trimmed.toLowerCase() === 'true'
  if (typeof existing === 'number') return scaled(key, parseNumber(trimmed), payload)
  if (Array.isArray(existing)) {
    let parsed: unknown
    try {
      parsed = JSON.parse(trimmed)
    } catch {
      throw new LibraryParseError('not valid JSON')
    }
    if (!Array.isArray(parsed)) throw new LibraryParseError('not a JSON list')
    return parsed
  }
  return text
}

/** The inline message for a parse failure; anything else is reported as an invalid value. */
export function libraryParseMessage(error: unknown): string {
  return error instanceof LibraryParseError ? error.message : 'invalid value'
}

export function isObject(value: unknown): value is Values {
  return typeof value === 'object' && value !== null && !Array.isArray(value)
}

/** The stable key of one field's path, used for edit and error maps. */
export function pathKeyOf(path: readonly string[]): string {
  return JSON.stringify(path)
}

export function setPath(root: Values, path: readonly string[], value: unknown): void {
  let cursor = root
  for (const key of path.slice(0, -1)) {
    const next = cursor[key]
    if (isObject(next)) cursor = next
    else {
      const created: Values = {}
      cursor[key] = created
      cursor = created
    }
  }
  cursor[path[path.length - 1]] = value
}

export function removePath(root: Values, path: readonly string[]): void {
  let cursor: Values = root
  for (const key of path.slice(0, -1)) {
    const next = cursor[key]
    if (!isObject(next)) return
    cursor = next
  }
  delete cursor[path[path.length - 1]]
}

/** A deep copy through JSON, matching the value shape the library payload carries. */
export function cloneValues(values: Values): Values {
  return JSON.parse(JSON.stringify(values)) as Values
}

export interface LibraryLeaf {
  kind: 'leaf'
  path: string[]
  key: string
  value: unknown
  /** Set for an optional field the entry does not carry yet; null otherwise. */
  optionalKind: LibraryOptionalKind | null
}

export interface LibraryGroup {
  kind: 'group'
  path: string[]
  key: string
  children: LibraryNode[]
}

export type LibraryNode = LibraryLeaf | LibraryGroup

function buildNodes(values: Values, preferred: readonly string[]): LibraryNode[] {
  return orderedKeys(values, preferred).map((key) => {
    const value = values[key]
    if (isObject(value)) return { kind: 'group', path: [key], key, children: buildNodes(value, []) }
    return { kind: 'leaf', path: [key], key, value, optionalKind: null }
  })
}

/**
 * The field tree for one entry: library_editors.py build_field_rows (289-316) plus the absent optional fields
 * (rebuild_editor 279-287). Field order is the payload's field_order first, the rest sorted; nested dicts become
 * groups; an absent optional field is a leaf parsed by its declared type and left out when blank.
 */
export function buildLibraryNodes(values: Values, category: string, payload: LibraryPayload): LibraryNode[] {
  const nodes = buildNodes(values, payload.field_order)
  const optional = payload.optional_fields[category] ?? {}
  for (const key of Object.keys(optional).sort()) {
    if (key in values) continue
    nodes.push({ kind: 'leaf', path: [key], key, value: undefined, optionalKind: optional[key] })
  }
  return nodes
}

/** Every leaf in render order, for the save pass. */
export function collectLeaves(nodes: readonly LibraryNode[]): LibraryLeaf[] {
  const leaves: LibraryLeaf[] = []
  for (const node of nodes) {
    if (node.kind === 'group') leaves.push(...collectLeaves(node.children))
    else leaves.push(node)
  }
  return leaves
}

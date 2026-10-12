import type { EditorContext } from '../types'

/** One field failed to parse; the entry keeps its last committed text and shows this inline. */
export class FieldParseError extends Error {}

const NUMBER = /^[+-]?(\d+(\.\d*)?|\.\d+)([eE][+-]?\d+)?$/
const INTEGER = /^[+-]?\d+$/

/** Python's f"{value:g}": six significant digits, trailing zeros stripped. C's %g chooses the exponent form
 * from the value AFTER rounding to six significant digits (exponent < -4 or >= 6), not from the input, so
 * e.g. 999999.5 prints as 1e+06 and 0.00009999999 as 0.0001. */
export function formatG(value: number): string {
  if (!Number.isFinite(value)) return String(value)
  if (value === 0) return '0'
  const rounded = Number(value.toPrecision(6))
  const [mantissa, exponent] = rounded.toExponential(5).split('e')
  const power = Number(exponent)
  if (power < -4 || power >= 6) {
    const trimmed = mantissa.replace(/(\.\d*?)0+$/, '$1').replace(/\.$/, '')
    const sign = power < 0 ? '-' : '+'
    return `${trimmed}e${sign}${String(Math.abs(power)).padStart(2, '0')}`
  }
  return String(rounded)
}

const FIELD_NAMES: Record<string, string> = {
  x_root_le_m: 'Root leading-edge x', chord_root_m: 'Root chord', chord_tip_m: 'Tip chord',
  sweep_le_deg: 'Leading-edge sweep', chord_start: 'Hinge position (chord fraction)',
  span_start: 'Start (span fraction)', span_end: 'End (span fraction)', k_prime: "Parabolic K′",
  fin_position: 'Fin position', wall_radial_m: 'Radial wall thickness',
  vertical_offset: 'Vertical offset (body radius fraction)', bulk_density_kg_m3: 'Printed bulk density',
  rod_diameter_m: 'Pivot rod diameter', rod_x_offset_m: 'Pivot rod x offset (aft)', rod_y_offset_m: 'Pivot rod y offset (up)',
}

/** editor_view.field_label (104-111). */
export function fieldLabel(key: string, context: EditorContext): string {
  let base = key
  for (const suffix of context.unit_suffixes) {
    if (base.endsWith(suffix)) {
      base = base.slice(0, -suffix.length)
      break
    }
  }
  const unit = context.mm_keys.includes(key)
    ? 'mm'
    : context.gram_keys.includes(key)
      ? 'g'
      : key.endsWith('_deg_s')
        ? 'deg/s'
        : key.endsWith('_deg')
          ? 'deg'
          : key.endsWith('_m_s')
            ? 'm/s'
            : key.endsWith('_kg_m3')
              ? 'kg/m^3'
              : key.endsWith('_m')
                ? 'm'
                : ''
  const label = FIELD_NAMES[key] ?? base.replace(/_/g, ' ')
  return unit === '' ? label : `${label} (${unit})`
}

/** editor_view.display_factor (114-115): lengths and masses enter and leave the fields in mm and g. */
export function displayFactor(key: string, context: EditorContext): number {
  return context.mm_keys.includes(key) || context.gram_keys.includes(key) ? 1000.0 : 1.0
}

/** editor_view.format_value (118-125): blank for None, "a-b" for a range, %g otherwise; int keys stay unscaled. */
export function formatValue(key: string, value: unknown, context: EditorContext): string {
  if (value === null || value === undefined) return ''
  if (Array.isArray(value)) {
    if (value.length === 2 && value[0] === value[1] && typeof value[0] === 'number') return formatG(value[0] * displayFactor(key, context))
    return value.map((item) => formatG((typeof item === 'number' ? item : Number.NaN) * displayFactor(key, context))).join('-')
  }
  if (typeof value === 'number') {
    if (context.integer_keys.includes(key) && Number.isInteger(value)) return String(value)
    return formatG(value * displayFactor(key, context))
  }
  return String(value)
}

function parseNumber(text: string): number {
  if (!NUMBER.test(text)) throw new FieldParseError(`not a number: ${text}`)
  const value = Number(text)
  if (!Number.isFinite(value)) throw new FieldParseError(`not a finite number: ${text}`)
  return value
}

function parseInteger(text: string): number {
  if (!INTEGER.test(text)) throw new FieldParseError(`not a whole number: ${text}`)
  const value = Number(text)
  if (!Number.isSafeInteger(value)) throw new FieldParseError(`not a whole number: ${text}`)
  return value
}

/**
 * editor_view.parse_value (128-138): text back to the stored value; a mass range gives [low, high]. existing is
 * the current value (or 0.0 for an absent field), as Tk uses it to tell text, whole numbers and numbers apart.
 */
export function parseValue(key: string, text: string, existing: unknown, context: EditorContext): unknown {
  const trimmed = text.trim()
  if (typeof existing === 'string' || key === 'notes' || key === 'name') return trimmed
  if (context.integer_keys.includes(key)) return parseInteger(trimmed)
  if (key === 'mass_kg') {
    const separator = trimmed.indexOf('-', 1)
    if (separator >= 0) {
      const low = parseNumber(trimmed.slice(0, separator))
      const high = parseNumber(trimmed.slice(separator + 1))
      return [low / displayFactor(key, context), high / displayFactor(key, context)]
    }
  }
  return parseNumber(trimmed) / displayFactor(key, context)
}

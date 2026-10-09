import type { Definition, EditorContext } from '../types'
import { isObject } from './patch'

/** A JSON-safe copy (definitions and context are plain JSON data). */
export function deepCopy<T>(value: T): T {
  return structuredClone(value)
}

function round4(value: number): number {
  return Math.round(value * 1e4) / 1e4
}

/** editor_view.build_battery_group 971: the first library pack, sorted by name. */
export function firstBatteryPart(context: EditorContext): string {
  return Object.keys(context.parts_data.battery ?? {}).sort()[0] ?? ''
}

/** editor_view.parts_of 595-596: names of the placed parts of one library category, in definition order. */
export function placedPartsOf(definition: Definition, category: string): string[] {
  const parts = Array.isArray(definition.parts) ? definition.parts.filter(isObject) : []
  return parts
    .filter((part) => typeof part.part === 'string' && part.part.split(':')[0] === category && typeof part.name === 'string')
    .map((part) => part.name as string)
}

/**
 * editor_view.new_propulsion (597-609): NEW_MOTOR, NEW_PROP, the first library pack (in the first volume, else at
 * mid-fuselage), the first ESC part (empty until placed and picked), one nose tractor.
 */
export function newPropulsion(definition: Definition, context: EditorContext): Record<string, unknown> {
  const volumes = Array.isArray(definition.volumes) ? definition.volumes.filter(isObject) : []
  const fuselage = isObject(definition.fuselage) ? definition.fuselage : {}
  const length = typeof fuselage.length_m === 'number' ? fuselage.length_m : 0.0
  const volume = typeof volumes[0]?.name === 'string' ? (volumes[0].name as string) : null
  const place = volume === null ? { x_m: round4(length / 2.0) } : { volume }
  const battery = { model: 'commercial', part: firstBatteryPart(context), ...place }
  return {
    motor: deepCopy(context.new_motor),
    prop: context.new_prop,
    battery,
    esc: placedPartsOf(definition, 'esc')[0] ?? '',
    mounts: [deepCopy(context.new_mount)],
    wiring: { battery: deepCopy(context.new_battery_run), motors: [deepCopy(context.new_motor_run)] },
  }
}

/**
 * editor_view.battery_connector (659-664): the propulsion battery's own connector: its library entry's, or the
 * custom pack's ("?" while it has none).
 */
export function batteryConnector(battery: Record<string, unknown>, context: EditorContext): string {
  if (battery.model === 'commercial') {
    const part = typeof battery.part === 'string' ? battery.part : ''
    const connector = context.parts_data.battery?.[part]?.connector
    return typeof connector === 'string' ? connector : '?'
  }
  return typeof battery.connector === 'string' ? battery.connector : '?'
}

/**
 * editor_view.sync_motor_runs (611-625): propulsion.wiring.motors follows the motor count (an added motor copies
 * the last run, a removed one loses its own). Returns the runs to store, or null to leave them alone: no change, or
 * a mount names a surface that does not exist (a definition error says so) or a layout the parser refuses.
 */
export function syncedMotorRuns(
  propulsion: Record<string, unknown>,
  definition: Definition,
  context: EditorContext,
  runs: readonly Record<string, unknown>[],
): Record<string, unknown>[] | null {
  const surfaces = Array.isArray(definition.surfaces) ? definition.surfaces.filter(isObject) : []
  const layouts: Record<string, string> = {}
  for (const surface of surfaces) {
    if (typeof surface.name === 'string' && typeof surface.layout === 'string') layouts[surface.name] = surface.layout
  }
  const mounts = Array.isArray(propulsion.mounts) ? propulsion.mounts.filter(isObject) : []
  if (mounts.some((mount) => mount.on === 'surface' && !(typeof mount.surface === 'string' && mount.surface in layouts))) return null
  let count = 0
  for (const mount of mounts) {
    if (mount.on === 'surface') {
      const panels = context.layout_panel_counts[layouts[String(mount.surface)]]
      if (typeof panels !== 'number') return null
      count += panels
    } else {
      count += 1
    }
  }
  if (count === runs.length) return null
  const next = runs.map((run) => deepCopy(run))
  while (next.length < count) next.push(deepCopy(next[next.length - 1] ?? context.new_motor_run))
  next.length = count
  return next
}

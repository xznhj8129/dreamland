import type { EditorContext } from '../types'
import { isObject } from './patch'

/** presets.default_rod_extent (65-68): rods from the end of the nose to the start of the tail. */
export function defaultRodExtent(fuselage: Record<string, unknown>): { from_x_m: number; to_x_m: number } {
  const nose = isObject(fuselage.nose) ? fuselage.nose : {}
  const tail = isObject(fuselage.tail) ? fuselage.tail : {}
  const from = typeof nose.length_m === 'number' ? nose.length_m : 0.0
  const to = typeof fuselage.length_m === 'number' && typeof tail.length_m === 'number' ? fuselage.length_m - tail.length_m : 0.0
  return { from_x_m: from, to_x_m: Math.round(to * 1e6) / 1e6 }
}

/** editor_view.toggle_reinforcement (785-788): NEW_ROD_PATTERN against the inner wall, computed from the current definition. */
export function defaultRodPattern(fuselage: Record<string, unknown>, context: EditorContext): Record<string, unknown> {
  const section = isObject(fuselage.section) ? fuselage.section : {}
  const width = typeof section.width_m === 'number' ? section.width_m : 0.0
  const height = typeof section.height_m === 'number' ? section.height_m : 0.0
  const wall = typeof section.wall_radial_m === 'number' ? section.wall_radial_m : 0.0
  const diameter = typeof context.new_rod_pattern.diameter_m === 'number' ? context.new_rod_pattern.diameter_m : 0.0
  const innerRadius = Math.min(width, height) / 2.0 - wall
  return { ...context.new_rod_pattern, radial_position_m: Math.round((innerRadius - diameter / 2.0) * 1e6) / 1e6 }
}

/**
 * editor_view.toggle_reinforcement (772-789): the reinforcement block ticking the rods checkbox stores.
 *
 * A stashed block is restored exactly as it was (779), with no default extent merged in. Only the preset-pattern
 * (781-783) and NEW_ROD_PATTERN (784-788) branches get the default extent (788). `stashed` is the session's
 * unticked block: undefined/null means there is none (the caller keeps it when unticking).
 */
export function reinforcementForTick(
  fuselage: Record<string, unknown>,
  stashed: unknown,
  presetName: string | null,
  context: EditorContext,
): unknown {
  if (stashed !== undefined && stashed !== null) return stashed
  const preset = presetName === null ? null : (context.preset_airframe_rods[presetName] ?? null)
  const pattern = preset ?? defaultRodPattern(fuselage, context)
  return { ...pattern, ...defaultRodExtent(fuselage) }
}

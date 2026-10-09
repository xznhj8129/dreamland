import type { Path } from '../types'

export type { Path }

export function isObject(value: unknown): value is Record<string, unknown> {
  return typeof value === 'object' && value !== null && !Array.isArray(value)
}

/** The value at a path, or undefined when any step is missing. */
export function getIn(root: unknown, path: Path): unknown {
  let value = root
  for (const step of path) {
    if (typeof step === 'number') {
      if (!Array.isArray(value)) return undefined
      value = value[step]
    } else {
      if (!isObject(value)) return undefined
      value = value[step]
    }
    if (value === undefined) return undefined
  }
  return value
}

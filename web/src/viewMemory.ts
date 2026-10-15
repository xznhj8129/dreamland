import { useCallback, useState } from 'react'

/**
 * View state that has to survive the view unmounting (switching to another tab) or a reload: one small JSON value
 * per key in localStorage, namespaced "dreamland.". Storage is best-effort: a private mode or a full quota refuses
 * the write and the memory is simply lost, never an error the user has to deal with.
 */

const PREFIX = 'dreamland.'

export function readMemory(key: string): unknown {
  try {
    const raw = window.localStorage.getItem(PREFIX + key)
    return raw === null ? null : (JSON.parse(raw) as unknown)
  } catch {
    return null
  }
}

export function writeMemory(key: string, value: unknown): void {
  try {
    window.localStorage.setItem(PREFIX + key, JSON.stringify(value))
  } catch {
    // refused or full: this memory is not kept
  }
}

/** useState whose value is remembered under key; a stored value that fails accepts is the fallback again. */
export function useRememberedState<T>(key: string, fallback: T, accepts: (value: unknown) => boolean): [T, (next: T) => void] {
  const [value, setValue] = useState<T>(() => {
    const stored = readMemory(key)
    return accepts(stored) ? (stored as T) : fallback
  })
  const remember = useCallback(
    (next: T) => {
      setValue(next)
      writeMemory(key, next)
    },
    [key],
  )
  return [value, remember]
}

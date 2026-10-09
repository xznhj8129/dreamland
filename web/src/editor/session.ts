import { createContext, useContext } from 'react'
import type { CommandResult } from '../store'
import type { Definition, EditorContext, Path, PathEdit } from '../types'

/** Session-only Tk editor stashes (editor_view.reset_definition_state): cleared when the forms remount. */
export interface EditorSession {
  stashedPropulsion: unknown
  stashedReinforcement: unknown
  /** Per battery model, its fields when another model was picked (editor_view.set_battery_model 997-1006). */
  stashedBatteries: Record<string, Record<string, unknown>>
}

export interface FormApi {
  definition: Definition
  sessionId: number
  context: EditorContext
  stash: EditorSession
  /** One atomic path edit (POST /api/editor/set), queued with every other mutation. */
  set: (path: Path, value: unknown) => Promise<CommandResult>
  /** One atomic removal: a dict key is deleted, a list entry becomes null. */
  remove: (path: Path) => Promise<CommandResult>
  /** A compound action as several atomic edits, in order and contiguous in the mutation queue. */
  edit: (edits: readonly PathEdit[]) => Promise<CommandResult>
}

export const FormApiContext = createContext<FormApi | null>(null)

export function useFormApi(): FormApi {
  const api = useContext(FormApiContext)
  if (api === null) throw new Error('useFormApi must be used inside the editor forms')
  return api
}

import { currentLibraryRevision, getEditorContext } from '../store'
import type { EditorContext } from '../types'

interface CachedContext {
  context: EditorContext
  /** The snapshot's library_revision the context was fetched under. A save/delete bumps it. */
  libraryRevision: number
}

let cached: CachedContext | null = null

/**
 * GET /api/editor/context, cached per library revision: the payload carries parts_data for the aircraft combos,
 * and a part-library save/delete moves the snapshot's library_revision, so the first read after that refetches
 * (webapp/editor.py context: "bumps the snapshot's library_revision, which invalidates the cache the same way").
 * force is the preset-op refresh, which is not tied to a library revision. A read before the first snapshot
 * (revision -1) is never cached.
 */
export async function loadEditorContext(force = false): Promise<EditorContext | null> {
  const revision = currentLibraryRevision()
  if (!force && cached !== null && revision >= 0 && cached.libraryRevision === revision) return cached.context
  const loaded = await getEditorContext()
  if (loaded !== null && revision >= 0) cached = { context: loaded, libraryRevision: revision }
  return loaded
}

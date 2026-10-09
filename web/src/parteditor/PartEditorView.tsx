import { useCallback, useEffect, useMemo, useState } from 'react'
import type { ReactElement } from 'react'
import type { LibraryPayload, LibrarySaveBody } from '../types'
import { deleteLibraryPart, getLibrary, saveLibraryPart } from '../store'
import { Modal } from '../components/Modal'
import { SplitPane } from '../components/SplitPane'
import {
  buildLibraryNodes,
  cloneValues,
  collectLeaves,
  formatLibraryValue,
  libraryParseMessage,
  parseLibraryValue,
  pathKeyOf,
  removePath,
  setPath,
} from './library'
import { PartFieldNode } from './PartFields'
import type { FieldEdit } from './PartFields'
import { buildRows, DRAFT_ROW_ID, PartTree, partRowId } from './PartTree'
import type { TreeRow } from './PartTree'

interface Work {
  /** The entry this save was opened on; null while it is still an unwritten New draft. */
  original: { category: string; name: string } | null
  category: string
  name: string
  values: Record<string, unknown>
  edits: Record<string, FieldEdit>
  dirty: boolean
}

/** webapp/library.py _plain_stem: a part name is a plain filename stem (no dot, separator or NUL). */
const INVALID_NAME = /[./\\\u0000]/

function buildPartWork(payload: LibraryPayload, category: string, name: string): Work {
  return {
    original: { category, name },
    category,
    name,
    values: cloneValues(payload.parts[category]?.[name] ?? {}),
    edits: {},
    dirty: false,
  }
}

function buildDraftWork(payload: LibraryPayload, category: string, name: string): Work {
  return {
    original: null,
    category,
    name,
    values: cloneValues(payload.defaults[category] ?? {}),
    edits: {},
    dirty: false,
  }
}

function nameProblem(name: string): string {
  if (name === '') return 'name is required'
  if (INVALID_NAME.test(name)) return 'name cannot contain . / \\ or NUL'
  return ''
}

/**
 * The Part Editor view (the Tk PartEditorFrame, libs/library_editors.py + libs/views/part_editor_view.py): a
 * part tree on the left, the entry editor on the right. The saved library is GET /api/library; Save/Delete are
 * POST /api/library/save|delete and the tree is refetched from the server after either lands. A New part is a
 * client-side draft shown in the tree until Save writes it. The flat category / name listbox is gone.
 */
export function PartEditorView(): ReactElement {
  const [payload, setPayload] = useState<LibraryPayload | null>(null)
  const [loadError, setLoadError] = useState('')
  const [search, setSearch] = useState('')
  const [collapsed, setCollapsed] = useState<ReadonlySet<string>>(() => new Set<string>())
  const [selection, setSelection] = useState<string | null>(null)
  const [work, setWork] = useState<Work | null>(null)
  const [parkedDraft, setParkedDraft] = useState<Work | null>(null)
  const [fieldErrors, setFieldErrors] = useState<Record<string, string>>({})
  const [nameError, setNameError] = useState('')
  const [notice, setNotice] = useState('')
  const [confirming, setConfirming] = useState(false)
  const [busy, setBusy] = useState(false)

  const reload = useCallback(async (): Promise<LibraryPayload | null> => {
    const loaded = await getLibrary()
    if (loaded === null) {
      setLoadError('Could not load the part library.')
      return null
    }
    setLoadError('')
    setPayload(loaded)
    return loaded
  }, [])

  useEffect(() => {
    void reload()
  }, [reload])

  const clearErrors = useCallback((): void => {
    setFieldErrors({})
    setNameError('')
  }, [])

  const activeDraft = useMemo(() => {
    if (work !== null && work.original === null) return { category: work.category, name: work.name }
    if (parkedDraft !== null) return { category: parkedDraft.category, name: parkedDraft.name }
    return null
  }, [work, parkedDraft])

  const rows = useMemo(
    () => (payload === null ? [] : buildRows(payload, activeDraft, collapsed, search)),
    [payload, activeDraft, collapsed, search],
  )

  const totalParts = useMemo(
    () => (payload === null ? 0 : Object.values(payload.parts).reduce((sum, entries) => sum + Object.keys(entries).length, 0)),
    [payload],
  )

  const toggleCategory = useCallback((category: string): void => {
    setCollapsed((previous) => {
      const next = new Set(previous)
      if (next.has(category)) next.delete(category)
      else next.add(category)
      return next
    })
  }, [])

  const activateRow = useCallback(
    (row: TreeRow): void => {
      if (row.kind === 'category') {
        setSelection(row.id)
        return
      }
      if (row.isDraft) {
        if (work === null || work.original !== null) {
          if (parkedDraft !== null) {
            setWork(parkedDraft)
            setParkedDraft(null)
            clearErrors()
          }
        }
        setSelection(DRAFT_ROW_ID)
        return
      }
      if (work !== null && work.original === null) {
        setParkedDraft(work)
        setWork(null)
      }
      const current = work !== null && work.original !== null && work.original.category === row.category && work.original.name === row.name
      if (!current && payload !== null) setWork(buildPartWork(payload, row.category, row.name))
      setSelection(partRowId(row.category, row.name))
      clearErrors()
    },
    [payload, work, parkedDraft, clearErrors],
  )

  const onNew = useCallback((): void => {
    if (payload === null || busy) return
    if (work !== null && work.original === null) {
      setSelection(DRAFT_ROW_ID)
      return
    }
    if (parkedDraft !== null) {
      setWork(parkedDraft)
      setParkedDraft(null)
      setSelection(DRAFT_ROW_ID)
      clearErrors()
      return
    }
    const selectedRow = rows.find((row) => row.id === selection)
    const category =
      selectedRow !== undefined ? selectedRow.category : work !== null ? work.category : payload.categories[0]?.name
    if (category === undefined) return
    const taken = new Set(Object.keys(payload.parts[category] ?? {}))
    let number = 1
    while (taken.has(`new-part-${number}`)) number += 1
    setWork(buildDraftWork(payload, category, `new-part-${number}`))
    setSelection(DRAFT_ROW_ID)
    clearErrors()
    setCollapsed((previous) => {
      if (!previous.has(category)) return previous
      const next = new Set(previous)
      next.delete(category)
      return next
    })
  }, [payload, busy, work, parkedDraft, rows, selection, clearErrors])

  const editField = useCallback((key: string, patch: FieldEdit): void => {
    setWork((previous) =>
      previous === null ? previous : { ...previous, edits: { ...previous.edits, [key]: { ...previous.edits[key], ...patch } }, dirty: true },
    )
  }, [])

  const updateName = (name: string): void => {
    setNameError('')
    setWork((previous) => (previous === null ? previous : { ...previous, name, dirty: true }))
  }

  const updateCategory = (category: string): void => {
    setNameError('')
    setWork((previous) => (previous === null ? previous : { ...previous, category, dirty: true }))
    setCollapsed((previous) => {
      if (!previous.has(category)) return previous
      const next = new Set(previous)
      next.delete(category)
      return next
    })
  }

  const onSave = useCallback(async (): Promise<void> => {
    if (work === null || payload === null || busy) return
    const name = work.name.trim()
    const problem = nameProblem(name)
    if (problem !== '') {
      setNameError(problem)
      return
    }
    const sameAsOriginal = work.original !== null && work.original.category === work.category && work.original.name === name
    if (!sameAsOriginal && name in (payload.parts[work.category] ?? {})) {
      setNameError(`a part named '${name}' already exists in ${work.category}`)
      return
    }
    // library_editors.py save_current (318-343): parse every shown field back to its existing type; an absent
    // optional field stays out when blank. Invalid text is rejected inline and nothing is posted.
    const values = cloneValues(work.values)
    const errors: Record<string, string> = {}
    for (const leaf of collectLeaves(buildLibraryNodes(work.values, work.category, payload))) {
      const key = pathKeyOf(leaf.path)
      const edit = work.edits[key]
      if (leaf.optionalKind !== null) {
        const text = edit?.text ?? ''
        if (text.trim() === '') {
          removePath(values, leaf.path)
          continue
        }
        try {
          setPath(values, leaf.path, parseLibraryValue(leaf.key, text, undefined, leaf.optionalKind, payload))
        } catch (error) {
          errors[key] = libraryParseMessage(error)
        }
        continue
      }
      if (typeof leaf.value === 'boolean') {
        setPath(values, leaf.path, edit?.checked ?? leaf.value)
        continue
      }
      const text = edit?.text ?? formatLibraryValue(leaf.key, leaf.value, payload)
      try {
        setPath(values, leaf.path, parseLibraryValue(leaf.key, text, leaf.value, null, payload))
      } catch (error) {
        errors[key] = libraryParseMessage(error)
      }
    }
    if (Object.keys(errors).length > 0) {
      setFieldErrors(errors)
      setNotice('Fix the highlighted fields.')
      return
    }
    const body: LibrarySaveBody = { category: work.category, name, original: work.original, values }
    setBusy(true)
    const result = await saveLibraryPart(body)
    setBusy(false)
    if (!result.ok) {
      setNotice(result.message === '' ? 'Save refused.' : result.message)
      return
    }
    const loaded = await reload()
    setNotice('')
    clearErrors()
    if (loaded === null) return
    setWork(buildPartWork(loaded, work.category, name))
    setSelection(partRowId(work.category, name))
    setCollapsed((previous) => {
      if (!previous.has(work.category)) return previous
      const next = new Set(previous)
      next.delete(work.category)
      return next
    })
  }, [work, payload, busy, reload, clearErrors])

  const onDelete = useCallback(async (): Promise<void> => {
    if (work === null || work.original === null || busy) return
    const { category, name } = work.original
    setConfirming(false)
    setBusy(true)
    const result = await deleteLibraryPart({ category, name })
    setBusy(false)
    if (!result.ok) {
      setNotice(result.message === '' ? 'Delete refused.' : result.message)
      return
    }
    await reload()
    setNotice('')
    clearErrors()
    setWork(null)
    setSelection(null)
  }, [work, busy, reload, clearErrors])

  const nodes = useMemo(
    () => (work === null || payload === null ? [] : buildLibraryNodes(work.values, work.category, payload)),
    [work?.values, work?.category, payload],
  )

  const canNew = payload !== null && payload.categories.length > 0

  return (
    <>
      <SplitPane minSizes={[240, 320]} initialRatios={[1, 3]}>
        <section className="panel part-tree-pane">
          <div className="part-tree-head">
            <span className="status">Part library</span>
            <span className="status">
              {totalParts} parts in {payload?.categories.length ?? 0} categories
            </span>
          </div>
          <input
            className="field-input part-search"
            type="search"
            placeholder="Search parts…"
            aria-label="Search parts"
            value={search}
            onChange={(event) => setSearch(event.target.value)}
          />
          {payload === null ? (
            <div className="placeholder">{loadError === '' ? 'Loading part library…' : loadError}</div>
          ) : (
            <PartTree rows={rows} selectedId={selection} collapsed={collapsed} onSelect={activateRow} onToggle={toggleCategory} />
          )}
        </section>
        <section className="panel part-editor-pane">
          {work === null || payload === null ? (
            <div className="placeholder">{loadError === '' ? 'Select a part in the tree, or press New.' : loadError}</div>
          ) : (
            <>
              <section className="form-card">
                <header className="form-card-header">
                  <h3>{work.original === null ? `New part (${work.category})` : `${work.original.category} / ${work.original.name}`}</h3>
                  <div className="button-row">
                    <button type="button" className="action" disabled={busy} onClick={() => void onSave()}>
                      Save
                    </button>
                    <button type="button" disabled={busy || !canNew} onClick={onNew}>
                      New
                    </button>
                    <button type="button" className="danger" disabled={busy || work.original === null} onClick={() => setConfirming(true)}>
                      Delete
                    </button>
                    {work.dirty && <span className="part-dirty">unsaved changes</span>}
                  </div>
                </header>
                <div className="form-row">
                  <label className="form-label" htmlFor="part-name">
                    name
                  </label>
                  <div className="form-control">
                    <input
                      id="part-name"
                      className={nameError === '' ? 'field-input' : 'field-input field-invalid'}
                      value={work.name}
                      spellCheck={false}
                      onChange={(event) => updateName(event.target.value)}
                    />
                    {nameError !== '' && <div className="field-error">{nameError}</div>}
                  </div>
                </div>
                <div className="form-row">
                  <label className="form-label" htmlFor="part-category">
                    category
                  </label>
                  <div className="form-control">
                    <select id="part-category" className="field-input" value={work.category} onChange={(event) => updateCategory(event.target.value)}>
                      {payload.categories.map((category) => (
                        <option key={category.name} value={category.name}>
                          {category.name}
                        </option>
                      ))}
                    </select>
                  </div>
                </div>
                {notice !== '' && <div className="field-error">{notice}</div>}
              </section>
              <section className="form-card">
                <header className="form-card-header">
                  <h3>Properties</h3>
                </header>
                {nodes.map((node) => (
                  <PartFieldNode
                    key={pathKeyOf(node.path)}
                    node={node}
                    payload={payload}
                    edits={work.edits}
                    errors={fieldErrors}
                    onEdit={editField}
                  />
                ))}
              </section>
            </>
          )}
        </section>
      </SplitPane>
      {confirming && work !== null && work.original !== null && (
        <Modal title="Delete Part" onClose={() => setConfirming(false)}>
          <p className="modal-copy">
            Delete part &apos;{work.name}&apos; from {work.original.category}?
          </p>
          <div className="modal-actions">
            <button type="button" onClick={() => setConfirming(false)}>
              Cancel
            </button>
            <button type="button" className="danger" onClick={() => void onDelete()}>
              Delete
            </button>
          </div>
        </Modal>
      )}
    </>
  )
}

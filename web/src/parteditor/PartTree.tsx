import { useRef } from 'react'
import type { KeyboardEvent as ReactKeyboardEvent, ReactElement } from 'react'
import type { LibraryPayload } from '../types'

/** The stable key of the one unsaved New draft, kept while its name changes. */
export const DRAFT_ROW_ID = 'draft\u0000new'

export function partRowId(category: string, name: string): string {
  return `part\u0000${category}\u0000${name}`
}

export function categoryRowId(category: string): string {
  return `category\u0000${category}`
}

export interface TreeRow {
  id: string
  kind: 'category' | 'part'
  category: string
  name: string
  /** Category rows only: the directory's full part count. */
  count: number
  /** The unsaved New draft, styled and tagged as such. */
  isDraft: boolean
}

/**
 * The visible rows: sorted categories (the payload's order) with their sorted part leaves. A non-empty query
 * filters leaves by name and hides categories with no match; the draft is always kept so it cannot vanish, and
 * collapsed categories are ignored while filtering so matches show.
 */
export function buildRows(
  payload: LibraryPayload,
  draft: { category: string; name: string } | null,
  collapsed: ReadonlySet<string>,
  query: string,
): TreeRow[] {
  const needle = query.trim().toLowerCase()
  const rows: TreeRow[] = []
  for (const category of payload.categories) {
    const parts = Object.keys(payload.parts[category.name] ?? {}).sort()
    const matched = needle === '' ? parts : parts.filter((name) => name.toLowerCase().includes(needle))
    const draftHere = draft !== null && draft.category === category.name ? draft.name : null
    if (needle !== '' && matched.length === 0 && draftHere === null) continue
    rows.push({
      id: categoryRowId(category.name),
      kind: 'category',
      category: category.name,
      name: category.name,
      count: parts.length,
      isDraft: false,
    })
    if (collapsed.has(category.name) && needle === '') continue
    const names = [...matched]
    if (draftHere !== null) names.push(draftHere)
    names.sort()
    for (const name of names) {
      const isDraft = draftHere !== null && name === draftHere
      rows.push({
        id: isDraft ? DRAFT_ROW_ID : partRowId(category.name, name),
        kind: 'part',
        category: category.name,
        name,
        count: 0,
        isDraft,
      })
    }
  }
  return rows
}

interface PartTreeProps {
  rows: TreeRow[]
  selectedId: string | null
  collapsed: ReadonlySet<string>
  onSelect: (row: TreeRow) => void
  onToggle: (category: string) => void
}

/**
 * The library tree (role="tree"): roving tabindex, arrow-key navigation over the visible rows, left/right to
 * collapse/expand, Enter/Space to open a category or select a part. Category rows carry their kind and part
 * count so role directories and table files read differently at a glance.
 */
export function PartTree({ rows, selectedId, collapsed, onSelect, onToggle }: PartTreeProps): ReactElement {
  const refs = useRef(new Map<string, HTMLDivElement>())
  const focusableId = selectedId ?? rows[0]?.id ?? ''

  const move = (index: number, delta: number): TreeRow | undefined => {
    if (rows.length === 0) return undefined
    if (index < 0) return delta > 0 ? rows[0] : rows[rows.length - 1]
    return rows[Math.min(Math.max(index + delta, 0), rows.length - 1)]
  }

  const onKeyDown = (event: ReactKeyboardEvent<HTMLDivElement>): void => {
    const index = rows.findIndex((row) => row.id === selectedId)
    const current = index < 0 ? undefined : rows[index]
    let next: TreeRow | undefined
    switch (event.key) {
      case 'ArrowDown':
        next = move(index, 1)
        break
      case 'ArrowUp':
        next = move(index, -1)
        break
      case 'Home':
        next = rows[0]
        break
      case 'End':
        next = rows[rows.length - 1]
        break
      case 'ArrowRight':
        if (current?.kind === 'category' && collapsed.has(current.category)) {
          event.preventDefault()
          onToggle(current.category)
          return
        }
        if (current?.kind === 'part') {
          const parent = rows.find((row) => row.kind === 'category' && row.category === current.category)
          if (parent !== undefined) {
            event.preventDefault()
            onSelect(parent)
            refs.current.get(parent.id)?.focus()
          }
        }
        return
      case 'ArrowLeft':
        if (current?.kind === 'category' && !collapsed.has(current.category)) {
          event.preventDefault()
          onToggle(current.category)
          return
        }
        if (current?.kind === 'part') {
          const parent = rows.find((row) => row.kind === 'category' && row.category === current.category)
          if (parent !== undefined) {
            event.preventDefault()
            onSelect(parent)
            refs.current.get(parent.id)?.focus()
          }
        }
        return
      case 'Enter':
      case ' ':
        if (current !== undefined) {
          event.preventDefault()
          if (current.kind === 'category') onToggle(current.category)
          else onSelect(current)
        }
        return
      default:
        return
    }
    if (next !== undefined && next.id !== selectedId) {
      event.preventDefault()
      onSelect(next)
      refs.current.get(next.id)?.focus()
    }
  }

  return (
    <div className="part-tree" role="tree" aria-label="Part library" onKeyDown={onKeyDown}>
      {rows.map((row) => {
        const selected = row.id === selectedId
        const isCategory = row.kind === 'category'
        const isCollapsed = isCategory && collapsed.has(row.category)
        return (
          <div
            key={row.id}
            ref={(element) => {
              if (element === null) refs.current.delete(row.id)
              else refs.current.set(row.id, element)
            }}
            role="treeitem"
            aria-selected={selected}
            aria-expanded={isCategory ? !isCollapsed : undefined}
            aria-level={isCategory ? 1 : 2}
            tabIndex={row.id === focusableId ? 0 : -1}
            className={`part-row ${isCategory ? 'category' : 'leaf'}${selected ? ' selected' : ''}${row.isDraft ? ' draft' : ''}`}
            onClick={() => {
              onSelect(row)
              refs.current.get(row.id)?.focus()
            }}
            onDoubleClick={isCategory ? () => onToggle(row.category) : undefined}
          >
            {isCategory ? (
              <button
                type="button"
                className="part-chevron"
                tabIndex={-1}
                aria-label={isCollapsed ? `Expand ${row.name}` : `Collapse ${row.name}`}
                onClick={(event) => {
                  event.stopPropagation()
                  onToggle(row.category)
                }}
              >
                {isCollapsed ? '\u25b8' : '\u25be'}
              </button>
            ) : (
              <span className="part-chevron" />
            )}
            <span className="part-name">{row.name}</span>
            {isCategory && <span className="part-count">{row.count}</span>}
            {row.isDraft && <span className="part-draft-tag">unsaved</span>}
          </div>
        )
      })}
    </div>
  )
}

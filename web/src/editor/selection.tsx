import { createContext, useContext, useEffect, useState } from 'react'
import type { ReactElement, ReactNode } from 'react'
import type { ComponentKind, ComponentSelection, DrawHandle } from '../types'

export type { ComponentKind, ComponentSelection } from '../types'
interface SelectionApi {
  active: ComponentSelection | null
  indices: Partial<Record<ComponentKind, number>>
  select: (kind: ComponentKind, index: number) => void
}
const SelectionContext = createContext<SelectionApi | null>(null)

export function SelectionProvider({ children }: { children: ReactNode }): ReactElement {
  const [active, setActive] = useState<ComponentSelection | null>(null)
  const [indices, setIndices] = useState<Partial<Record<ComponentKind, number>>>({})
  function select(kind: ComponentKind, index: number): void {
    setActive({ kind, index })
    setIndices(previous => ({ ...previous, [kind]: index }))
  }
  return <SelectionContext.Provider value={{ active, indices, select }}>{children}</SelectionContext.Provider>
}

export function useSelection(): SelectionApi {
  const api = useContext(SelectionContext)
  if (api === null) throw new Error('Component selection requires SelectionProvider')
  return api
}

export function useComponentIndex(kind: ComponentKind, count: number): number {
  const { active, indices, select } = useSelection()
  const index = Math.max(0, Math.min(indices[kind] ?? 0, count - 1))
  useEffect(() => {
    if (active?.kind === kind && active.index !== index) select(kind, index)
  }, [active, kind, index, select])
  return index
}

export function selectionFromHandle(handle: Pick<DrawHandle, 'target' | 'index'>): ComponentSelection {
  const kind = handle.target === 'cut' ? 'section' : handle.target === 'spars' ? 'surface' : handle.target === 'rod' ? 'fuselage' : handle.target
  return { kind, index: handle.index }
}

export function handleFromSelection(selection: ComponentSelection | null): Pick<DrawHandle, 'target' | 'index'> | null {
  // Mounts have no draw handle; access entries are not drawn yet (a later change gives them outlines).
  if (selection === null || selection.kind === 'mount') return null
  const target = selection.kind === 'section' ? 'cut' : selection.kind === 'fuselage' ? 'rod' : selection.kind
  return { target, index: selection.index }
}

export interface ComponentItem { name: string; detail?: string }
export function ComponentPicker({ kind, items }: { kind: ComponentKind; items: ComponentItem[] }): ReactElement {
  const { select } = useSelection()
  const index = useComponentIndex(kind, items.length)
  return <ComponentList items={items} index={index} label={`Select ${kind}`} onSelect={position => select(kind, position)} />
}

export function ComponentList({ items, index, label, onSelect }: {
  items: ComponentItem[]; index: number; label: string; onSelect: (index: number) => void
}): ReactElement {
  return <div className="component-picker" role="group" aria-label={label}>
    {items.length === 0 ? <p className="form-hint">No components yet.</p> : items.map((item, position) =>
      <button type="button" key={position} aria-pressed={position === index}
        className={position === index ? 'component-choice active' : 'component-choice'}
        onClick={() => onSelect(position)}>
        <strong title={item.name}>{item.name}</strong>{item.detail && <span title={item.detail}>{item.detail}</span>}
      </button>)}
  </div>
}

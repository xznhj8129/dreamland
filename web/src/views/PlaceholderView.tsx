import type { ReactElement } from 'react'

export function PlaceholderView({ name }: { name: string }): ReactElement {
  return (
    <section className="panel placeholder-view">
      <h1>{name}</h1>
      <p>Not migrated yet.</p>
    </section>
  )
}

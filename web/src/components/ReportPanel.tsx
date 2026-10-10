import { useSelection } from '../editor/selection'
import { useEffect, useId, useRef, useState } from 'react'
import type { KeyboardEvent, ReactElement } from 'react'
import type { ComponentSelection, LineLevel, LogEntry, ReportPages, ReportSection } from '../types'

type InspectComponent = (target: ComponentSelection) => void

const LEVEL_CLASS: Record<LineLevel, string> = {
  info: 'line-info',
  warning: 'line-warning',
  problem: 'line-problem',
}
const TABS = [
  { id: 'summary', label: 'Summary' },
  { id: 'mass', label: 'Mass' },
  { id: 'aero', label: 'Aero' },
  { id: 'propulsion', label: 'Propulsion' },
  { id: 'log', label: 'Log' },
] as const
type ReportTab = (typeof TABS)[number]['id']

function SectionContent({ section, onInspect }: { section: ReportSection; onInspect?: InspectComponent }): ReactElement {
  return <>
    {section.children.length > 0 && <ReportSections sections={section.children} onInspect={onInspect} />}
    {section.metrics.length > 0 && <dl className="report-metrics">
      {section.metrics.map(metric => <div key={metric.label} className="report-metric">
        <dt>{metric.label}</dt>
        <dd className={LEVEL_CLASS[metric.level]}>{metric.value}</dd>
      </div>)}
    </dl>}
    {section.notes.map((note, index) => <p key={index} className={`report-note ${LEVEL_CLASS[note.level]}`}>{note.text}{note.target && onInspect && <button type="button" className="inspect-problem" onClick={() => { if (note.target) onInspect(note.target) }}>Inspect {note.target.label}</button>}</p>)}
    {section.table && <div className="report-table-scroll" tabIndex={0} role="region" aria-label={`${section.title} table`}>
      <table className="report-table">
        <thead><tr>{section.table.columns.map(column => <th scope="col" key={column}>{column}</th>)}</tr></thead>
        <tbody>{section.table.rows.map((row, index) => <tr key={index} className={LEVEL_CLASS[row.level]}>
          {row.values.map((value, column) => column === 0
            ? <th scope="row" key={column}>{value}</th>
            : <td key={column}>{value}</td>)}
        </tr>)}</tbody>
      </table>
    </div>}
  </>
}

export function ReportSections({ sections, onInspect }: { sections: ReportSection[]; onInspect?: InspectComponent }): ReactElement {
  return <>{sections.map((section, index) => section.collapsible
    ? <details className="report-section" key={index}>
        <summary>{section.title}{section.table && <span className="report-count"> {section.table.rows.length} points</span>}</summary>
        <SectionContent section={section} onInspect={onInspect} />
      </details>
    : <section className="report-section" key={index}>
        <h3>{section.title}</h3>
        <SectionContent section={section} onInspect={onInspect} />
      </section>)}</>
}

export function ReportPanel({ pages, log, inspectEnabled = true }: { pages: ReportPages | null; log: LogEntry[]; inspectEnabled?: boolean }): ReactElement {
  const { select } = useSelection()
  const inspect: InspectComponent = target => select(target.kind, target.index)
  const [tab, setTab] = useState<ReportTab>('summary')
  const logLengthRef = useRef(0)
  const tabRefs = useRef<(HTMLButtonElement | null)[]>([])
  const id = useId()

  useEffect(() => {
    if (log.length > logLengthRef.current) setTab('log')
    logLengthRef.current = log.length
  }, [log.length])

  function navigateTab(event: KeyboardEvent<HTMLButtonElement>, index: number): void {
    let next = index
    switch (event.key) {
      case 'ArrowRight': next = (index + 1) % TABS.length; break
      case 'ArrowLeft': next = (index + TABS.length - 1) % TABS.length; break
      case 'Home': next = 0; break
      case 'End': next = TABS.length - 1; break
      default: return
    }
    event.preventDefault()
    setTab(TABS[next].id)
    tabRefs.current[next]?.focus()
  }

  return <div className="report-panel">
    <div className="tab-row report-tabs" role="tablist" aria-label="Aircraft report">
      {TABS.map((item, index) => <button type="button" role="tab" key={item.id}
        id={`${id}-${item.id}-tab`} aria-controls={`${id}-${item.id}-panel`}
        aria-selected={tab === item.id} tabIndex={tab === item.id ? 0 : -1}
        ref={element => { tabRefs.current[index] = element }}
        className={tab === item.id ? 'tab active' : 'tab'}
        onClick={() => setTab(item.id)} onKeyDown={event => navigateTab(event, index)}>
        {item.label}
      </button>)}
    </div>
    {TABS.map(item => <div key={item.id} role="tabpanel" id={`${id}-${item.id}-panel`}
      aria-labelledby={`${id}-${item.id}-tab`} hidden={tab !== item.id} tabIndex={0}
      className={item.id === 'log' ? 'report-content report-log' : 'report-content'}>
      {item.id === 'log'
        ? log.length === 0 ? <p className="report-empty">No errors logged.</p> : log.map((entry, index) =>
            <div key={index} className="log-entry line-problem">
              <div className="log-heading"><time>{entry.time}</time><strong>{entry.title}</strong></div>
              <div className="log-text">{entry.text}</div>
            </div>)
        : pages ? <ReportSections sections={pages[item.id]} onInspect={inspectEnabled ? inspect : undefined} /> : <p className="report-empty">No valid analysis yet. See Log for errors.</p>}
    </div>)}
  </div>
}

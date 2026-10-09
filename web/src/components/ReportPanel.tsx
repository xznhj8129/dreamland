import { useEffect, useRef, useState } from 'react'
import type { ReactElement } from 'react'
import type { LineLevel, LogEntry, ReportLine } from '../types'

const LEVEL_CLASS: Record<LineLevel, string> = {
  info: 'line-info',
  warning: 'line-warning',
  problem: 'line-problem',
}

export function ReportPanel({ report, log }: { report: ReportLine[]; log: LogEntry[] }): ReactElement {
  const [tab, setTab] = useState<'summary' | 'log'>('summary')
  const logLengthRef = useRef(0)

  // A new log entry brings the Log tab to the front (guibuilder.log_error selects the log tab); a shorter log
  // (a project switch clears it) leaves the current tab alone.
  useEffect(() => {
    if (log.length > logLengthRef.current) setTab('log')
    logLengthRef.current = log.length
  }, [log.length])

  return (
    <div className="report-panel">
      <div className="tab-row">
        <button type="button" className={tab === 'summary' ? 'tab active' : 'tab'} onClick={() => setTab('summary')}>
          Summary
        </button>
        <button type="button" className={tab === 'log' ? 'tab active' : 'tab'} onClick={() => setTab('log')}>
          Log
        </button>
      </div>
      <div className="report-text">
        {tab === 'summary'
          ? report.map((line, index) => (
              <div key={index} className={LEVEL_CLASS[line.level]}>
                {line.text || '\u00a0'}
              </div>
            ))
          : log.map((entry, index) => (
              <div key={index} className="log-entry line-problem">
                <div>
                  {entry.time}  {entry.title}
                </div>
                <div className="log-text">{entry.text}</div>
              </div>
            ))}
      </div>
    </div>
  )
}

import { useCallback, useEffect, useRef, useState } from 'react'
import type { ReactElement, ReactNode } from 'react'
import {
  cadState,
  exportDrawings,
  exportSections,
  exportStep,
  exportStl,
  exportVarsets,
  openFolder,
  setWorkerCpus,
} from '../cadApi'
import type { CadAnswer, CadOutcome, CadRun, CadState } from '../cadApi'
import { useSnapshot } from '../store'
import { useRememberedState } from '../viewMemory'

type AirframeFormat = 'step-parts' | 'step-solid' | 'stl'
type PartFormat = 'step' | 'stl'

/** One row of the list: the server tasks it starts and shows (Whole airframe is STEP or STL). */
interface ExportRow {
  key: string
  name: string
  tasks: readonly string[]
}

const ROWS = {
  airframe: { key: 'airframe', name: 'Whole airframe', tasks: ['step', 'stl'] },
  parts: { key: 'parts', name: 'Printed parts', tasks: ['sections'] },
  drawings: { key: 'drawings', name: 'Drawings', tasks: ['drawings'] },
  varsets: { key: 'varsets', name: 'FreeCAD varsets', tasks: ['varsets'] },
} satisfies Record<string, ExportRow>

const RUNNING_POLL_MS = 400
const IDLE_POLL_MS = 2000

const isAirframeFormat = (value: unknown): boolean => value === 'step-parts' || value === 'step-solid' || value === 'stl'
const isPartFormat = (value: unknown): boolean => value === 'step' || value === 'stl'
const isBoolean = (value: unknown): boolean => typeof value === 'boolean'

function duration(seconds: number): string {
  const whole = Math.max(0, Math.round(seconds))
  return `${Math.floor(whole / 60)}:${String(whole % 60).padStart(2, '0')}`
}

function clock(epochSeconds: number): string {
  return new Date(epochSeconds * 1000).toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })
}

function timeLeft(seconds: number): string {
  return seconds < 60 ? 'less than a minute left' : `about ${Math.round(seconds / 60)} min left`
}

function capitalized(text: string): string {
  return text.charAt(0).toUpperCase() + text.slice(1)
}

function plural(count: number, word: string): string {
  return `${count} ${word}${count === 1 ? '' : 's'}`
}

/** A running export in its row: the latest step, a bar, the time so far and left, and every step on request. */
function RowProgress({ run }: { run: CadRun }): ReactElement {
  const step = run.steps.at(-1) ?? null
  const percent = step !== null && step.total > 0 ? Math.min(100, Math.round((100 * step.done) / step.total)) : 0
  return (
    <>
      <div className="cad-status-line">
        {step === null ? 'Starting' : capitalized(step.label)}
        {step !== null && step.total > 0 && ` — ${step.done} of ${step.total}`}
      </div>
      <div className="cad-progress">
        <div className="build-bar" role="progressbar" aria-valuenow={percent} aria-valuemin={0} aria-valuemax={100}>
          <div className="build-bar-fill" style={{ width: `${percent}%` }} />
        </div>
        <span>
          {duration(run.elapsed_s)} so far{run.estimate !== null && `, ${timeLeft(run.estimate.left_s)}`}
        </span>
      </div>
      {run.steps.length > 0 && (
        <details className="cad-steps">
          <summary>Steps ({run.steps.length})</summary>
          <ol>
            {run.steps.map((entry, index) => (
              <li key={index}>
                <span>{duration(entry.at_s)}</span>
                <span>{entry.label}</span>
              </li>
            ))}
          </ol>
        </details>
      )}
    </>
  )
}

/** A finished export in its row: when, how many files, problems; or where it failed. Details are in the Log. */
function RowOutcome({ outcome, task }: { outcome: CadOutcome; task: string }): ReactElement {
  if (!outcome.ok) {
    return <div className="cad-status-line failed">Failed at {clock(outcome.finished_at)} while {outcome.message}, see Log</div>
  }
  return (
    <div className="cad-status-line">
      Done at {clock(outcome.finished_at)}, {plural(outcome.files.length, 'file')}
      {outcome.notes.length > 0 && <span className="cad-problems">, {plural(outcome.notes.length, 'problem')}, see Log</span>}
      <button type="button" onClick={() => void openFolder(task)}>
        Open folder
      </button>
    </div>
  )
}

interface RowProps {
  row: ExportRow
  state: CadState | null
  project: string | null
  notice: string
  busy: boolean
  onExport: () => void
  children?: ReactNode
}

/** One export: its name, its options, the Export button, and below them what it is doing or last did. */
function Row({ row, state, project, notice, busy, onExport, children }: RowProps): ReactElement {
  const run = state?.run ?? null
  const ownRun = run !== null && run.state === 'running' && row.tasks.includes(run.task) ? run : null
  const outcomes = row.tasks
    .map((task) => ({ task, outcome: state?.outcomes[task] ?? null }))
    .filter((entry): entry is { task: string; outcome: CadOutcome } => entry.outcome !== null && entry.outcome.project === project)
    .sort((a, b) => b.outcome.finished_at - a.outcome.finished_at)
  const latest = outcomes[0] ?? null
  return (
    <div className="cad-row">
      <div className="cad-row-name">{row.name}</div>
      <div className="cad-row-options">{children}</div>
      <button type="button" className="action" disabled={busy} onClick={onExport}>
        Export
      </button>
      <div className="cad-row-status">
        {ownRun !== null ? (
          <RowProgress run={ownRun} />
        ) : notice !== '' ? (
          <div className="cad-status-line failed">Not started: {notice}</div>
        ) : latest !== null ? (
          <RowOutcome outcome={latest.outcome} task={latest.task} />
        ) : (
          <div className="cad-status-line">Not exported yet</div>
        )}
      </div>
    </div>
  )
}

/** The CPUs the CAD work may use (applied at once, also to a running export) and its priority. */
function WorkerFooter({ state }: { state: CadState | null }): ReactElement | null {
  const [draft, setDraft] = useState('')
  const cpus = state?.worker.cpus ?? null
  useEffect(() => {
    if (cpus !== null) setDraft(String(cpus))
  }, [cpus])
  if (state === null) return null
  const max = state.worker.max_cpus
  return (
    <div className="cad-footer">
      <label>
        CPUs for CAD
        <input
          type="number"
          min={1}
          max={max}
          step={1}
          value={draft}
          title={`1 to ${max}; more is faster, fewer keeps the computer free`}
          onChange={(event) => {
            setDraft(event.target.value)
            const value = Number(event.target.value)
            if (Number.isInteger(value) && value >= 1 && value <= max) void setWorkerCpus(value)
          }}
        />
      </label>
      <span>Priority: {state.worker.nice >= 19 ? 'lowest' : 'low'}</span>
    </div>
  )
}

/**
 * The CAD tab (docs/ui/cad_view.md): one row per export, each showing its own progress while it runs and its last
 * result after. The server owns every run (webapp/cad.py); this view starts one and shows GET /api/cad/state, polled
 * fast while a run goes and slowly otherwise, so leaving the tab and coming back shows the run where it is.
 */
export function CadView(): ReactElement {
  const snapshot = useSnapshot()
  const [airframeFormat, setAirframeFormat] = useRememberedState<AirframeFormat>('cad.airframeFormat', 'step-parts', isAirframeFormat)
  const [propDiscs, setPropDiscs] = useRememberedState<boolean>('cad.propDiscs', false, isBoolean)
  const [partFormat, setPartFormat] = useRememberedState<PartFormat>('cad.partFormat', 'step', isPartFormat)
  const [state, setState] = useState<CadState | null>(null)
  const [starting, setStarting] = useState(false)
  const [notice, setNotice] = useState<{ row: string; text: string } | null>(null)
  const mounted = useRef(true)

  useEffect(() => {
    mounted.current = true
    let timer = 0
    const poll = async (): Promise<void> => {
      const answer = await cadState()
      if (!mounted.current) return
      if (answer !== null) setState(answer)
      timer = window.setTimeout(() => void poll(), answer?.run?.state === 'running' ? RUNNING_POLL_MS : IDLE_POLL_MS)
    }
    void poll()
    return () => {
      mounted.current = false
      window.clearTimeout(timer)
    }
  }, [])

  // A click shows at once, not at the next slow poll: refresh fast until the export has answered.
  useEffect(() => {
    if (!starting) return undefined
    const timer = window.setInterval(() => {
      void cadState().then((answer) => {
        if (mounted.current && answer !== null) setState(answer)
      })
    }, RUNNING_POLL_MS)
    return () => window.clearInterval(timer)
  }, [starting])

  const start = useCallback((row: string, task: () => Promise<CadAnswer>): void => {
    const clicked = Date.now() / 1000
    setStarting(true)
    setNotice(null)
    void task().then(async (answer) => {
      const latest = await cadState()
      if (!mounted.current) return
      if (latest !== null) setState(latest)
      // A refusal before the run began (no valid airframe) leaves no run behind; the row says why nothing happened.
      if (!answer.ok && (latest?.run == null || latest.run.started_at < clicked - 1)) setNotice({ row, text: answer.message })
      setStarting(false)
    })
  }, [])

  const busy = starting || state?.run?.state === 'running'
  const project = snapshot?.project ?? null
  const rowProps = (row: ExportRow) => ({ row, state, project, busy, notice: notice?.row === row.key ? notice.text : '' })

  const exportAirframe = (): Promise<CadAnswer> =>
    airframeFormat === 'stl' ? exportStl(propDiscs) : exportStep(airframeFormat === 'step-solid' ? 'fused' : 'assembly', propDiscs)

  return (
    <section className="panel cad-view">
      <div className="cad-list">
        <Row {...rowProps(ROWS.airframe)} onExport={() => start(ROWS.airframe.key, exportAirframe)}>
          <select value={airframeFormat} disabled={busy} onChange={(event) => setAirframeFormat(event.target.value as AirframeFormat)}>
            <option value="step-parts">STEP, separate parts</option>
            <option value="step-solid">STEP, one solid</option>
            <option value="stl">STL</option>
          </select>
          <label className="check-row">
            <input type="checkbox" checked={propDiscs} disabled={busy} onChange={(event) => setPropDiscs(event.target.checked)} />
            Propeller discs
          </label>
        </Row>
        <Row {...rowProps(ROWS.parts)} onExport={() => start(ROWS.parts.key, () => exportSections(partFormat))}>
          <select value={partFormat} disabled={busy} onChange={(event) => setPartFormat(event.target.value as PartFormat)}>
            <option value="step">STEP</option>
            <option value="stl">STL</option>
          </select>
        </Row>
        <Row {...rowProps(ROWS.drawings)} onExport={() => start(ROWS.drawings.key, exportDrawings)} />
        <Row {...rowProps(ROWS.varsets)} onExport={() => start(ROWS.varsets.key, exportVarsets)} />
      </div>
      <WorkerFooter state={state} />
    </section>
  )
}

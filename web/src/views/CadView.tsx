import { useCallback, useState } from 'react'
import type { ReactElement } from 'react'
import {
  cadProgress,
  exportDrawings,
  exportSections,
  exportStep,
  exportStl,
  exportVarsets,
  filesUrl,
} from '../cadApi'
import type { CadAnswer, CadProgress, CadResult, StepMode } from '../cadApi'

const MODE_OPTIONS: readonly { mode: StepMode; label: string }[] = [
  { mode: 'assembly', label: 'Assembly — named parts per section (CAD)' },
  { mode: 'fused', label: 'Fused — one watertight solid (CFD)' },
]

const SECTION_FORMAT_OPTIONS: readonly { format: 'step' | 'stl'; label: string }[] = [
  { format: 'step', label: 'STEP' },
  { format: 'stl', label: 'STL' },
]

const PROGRESS_INTERVAL_MS = 400

const TASK_LABELS: Record<string, string> = {
  step: 'STEP',
  stl: 'STL',
  sections: 'Sections',
  drawings: 'Drawings',
  varsets: 'Varsets',
  geometry: 'Compute geometry',
}

/** The busy line: the server's per-step label and count while the export runs (e.g. "Sections — writing S03 (3/12)"). */
function progressText(progress: CadProgress | null): string {
  if (progress === null || !progress.running || progress.task === null) return 'Exporting…'
  const task = TASK_LABELS[progress.task] ?? progress.task
  if (progress.label === null) return `${task}…`
  const count = progress.total > 0 ? ` (${progress.done}/${progress.total})` : ''
  return `${task} — ${progress.label}${count}`
}

/** One export's answer: the status detail, the written files as links, and any notes (problems, fallback). */
function ExportResult({ result }: { result: CadResult }): ReactElement {
  return (
    <div className="cad-result">
      <div className="cad-detail">{result.detail}</div>
      {result.files.length > 0 && (
        <ul className="cad-files">
          {result.files.map((file) => (
            <li key={file}>
              <a href={filesUrl(file)} target="_blank" rel="noreferrer">
                {file}
              </a>
            </li>
          ))}
        </ul>
      )}
      {result.notes.length > 0 && (
        <ul className="cad-notes">
          {result.notes.map((note, index) => (
            <li key={index}>{note}</li>
          ))}
        </ul>
      )}
    </div>
  )
}

/**
 * The web CAD view (libs/views/cad_view.py, docs/ui/cad_view.md): whole-airframe STEP/STL, printable sections,
 * drawings and FreeCAD varsets, each into its fixed place under the project's cad/. Exports are direct fetches;
 * a server-side lock serializes them, and the browser disables every button while one is in flight. Each card
 * shows the files its own export just wrote; a failed export's message appears at the bottom of the view. While
 * an export runs, the bottom line follows the server's progress record (GET /api/cad/progress, 400 ms poll) with
 * the step label and a small bar.
 */
export function CadView(): ReactElement {
  const [mode, setMode] = useState<StepMode>('assembly')
  const [sectionFormat, setSectionFormat] = useState<'step' | 'stl'>('step')
  const [propDiscs, setPropDiscs] = useState(false)
  const [busy, setBusy] = useState(false)
  const [error, setError] = useState('')
  const [stepResult, setStepResult] = useState<CadResult | null>(null)
  const [stlResult, setStlResult] = useState<CadResult | null>(null)
  const [sectionsResult, setSectionsResult] = useState<CadResult | null>(null)
  const [drawingsResult, setDrawingsResult] = useState<CadResult | null>(null)
  const [varsetsResult, setVarsetsResult] = useState<CadResult | null>(null)
  const [progress, setProgress] = useState<CadProgress | null>(null)

  const run = useCallback(
    async (task: () => Promise<CadAnswer>, apply: (result: CadResult) => void): Promise<void> => {
      setBusy(true)
      setError('')
      setProgress(null)
      let cancelled = false
      const poll = async (): Promise<void> => {
        const snapshot = await cadProgress()
        if (!cancelled && snapshot !== null) setProgress(snapshot)
      }
      const timer = window.setInterval(() => void poll(), PROGRESS_INTERVAL_MS)
      void poll()
      try {
        const answer = await task()
        if (answer.ok && answer.result !== null) apply(answer.result)
        else setError(answer.message)
      } finally {
        cancelled = true
        window.clearInterval(timer)
        setProgress(null)
        setBusy(false)
      }
    },
    [],
  )

  const cutaway = drawingsResult?.files.find((file) => file.endsWith('cutaway.svg')) ?? null

  return (
    <section className="panel cad-view">
      <div className="form-card">
        <div className="form-card-header">
          <h3>Whole airframe</h3>
        </div>
        <div className="cad-mode" role="radiogroup" aria-label="STEP mode">
          {MODE_OPTIONS.map((option) => (
            <button
              key={option.mode}
              type="button"
              role="radio"
              aria-checked={mode === option.mode}
              className={mode === option.mode ? 'cad-mode-option active' : 'cad-mode-option'}
              disabled={busy}
              onClick={() => setMode(option.mode)}
            >
              {option.label}
            </button>
          ))}
        </div>
        <p className="form-hint">The STEP mode applies to the STEP export only. STL is always a single fused mesh.</p>
        <div className="button-row">
          <button type="button" disabled={busy} onClick={() => void run(() => exportStep(mode, propDiscs), setStepResult)}>
            Export STEP
          </button>
          <button type="button" disabled={busy} onClick={() => void run(() => exportStl(propDiscs), setStlResult)}>
            Export STL (always a single fused mesh)
          </button>
        </div>
        <label className="check-row">
          <input
            type="checkbox"
            checked={propDiscs}
            disabled={busy}
            onChange={(event) => setPropDiscs(event.target.checked)}
          />
          Include propeller discs (actuator-disc markers)
        </label>
        {stepResult !== null && <ExportResult result={stepResult} />}
        {stlResult !== null && <ExportResult result={stlResult} />}
      </div>

      <div className="form-card">
        <div className="form-card-header">
          <h3>Printable parts</h3>
        </div>
        <div className="cad-mode" role="radiogroup" aria-label="Printable format">
          {SECTION_FORMAT_OPTIONS.map((option) => (
            <button
              key={option.format}
              type="button"
              role="radio"
              aria-checked={sectionFormat === option.format}
              className={sectionFormat === option.format ? 'cad-mode-option active' : 'cad-mode-option'}
              disabled={busy}
              onClick={() => setSectionFormat(option.format)}
            >
              {option.label}
            </button>
          ))}
        </div>
        <p className="form-hint">
          Fuselage sections S01…, couplers C01…, fixed wing segments W… and controls CS… as {sectionFormat.toUpperCase()} into
          cad/sections/. Independent of the STEP mode above.
        </p>
        <div className="button-row">
          <button type="button" disabled={busy} onClick={() => void run(() => exportSections(sectionFormat), setSectionsResult)}>
            Export printable sections
          </button>
        </div>
        {sectionsResult !== null && <ExportResult result={sectionsResult} />}
      </div>

      <div className="form-card">
        <div className="form-card-header">
          <h3>Drawings</h3>
        </div>
        <p className="form-hint">Cutaway, three-view and x-ray (.svg + .png) into cad/.</p>
        <div className="button-row">
          <button type="button" disabled={busy} onClick={() => void run(exportDrawings, setDrawingsResult)}>
            Export drawings
          </button>
        </div>
        {drawingsResult !== null && <ExportResult result={drawingsResult} />}
        {cutaway !== null && <img className="cad-preview" src={filesUrl(cutaway)} alt="Cutaway preview" />}
      </div>

      <div className="form-card">
        <div className="form-card-header">
          <h3>FreeCAD varsets</h3>
        </div>
        <p className="form-hint">One JSON per varset (Airframe, one per surface set, Propulsion, one per mount).</p>
        <div className="button-row">
          <button type="button" disabled={busy} onClick={() => void run(exportVarsets, setVarsetsResult)}>
            Export FreeCAD varsets
          </button>
        </div>
        {varsetsResult !== null && <ExportResult result={varsetsResult} />}
      </div>

      {busy && (
        <div className="status cad-progress">
          <span>{progressText(progress)}</span>
          {progress !== null && progress.total > 0 && (
            <span className="cad-progress-track" aria-hidden="true">
              <span
                className="cad-progress-fill"
                style={{ width: `${Math.min(100, Math.round((progress.done / progress.total) * 100))}%` }}
              />
            </span>
          )}
        </div>
      )}
      {error !== '' && <div className="status draw-error cad-error">{error}</div>}
    </section>
  )
}

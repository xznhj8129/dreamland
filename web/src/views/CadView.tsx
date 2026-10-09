import { useCallback, useState } from 'react'
import type { ReactElement } from 'react'
import {
  exportDrawings,
  exportSections,
  exportStep,
  exportStl,
  exportVarsets,
  filesUrl,
} from '../cadApi'
import type { CadAnswer, CadResult, StepMode } from '../cadApi'

const MODE_OPTIONS: readonly { mode: StepMode; label: string }[] = [
  { mode: 'assembly', label: 'Assembly — named parts per section (CAD)' },
  { mode: 'fused', label: 'Fused — one watertight solid (CFD)' },
]

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
 * shows the files its own export just wrote; a failed export's message appears at the bottom of the view.
 */
export function CadView(): ReactElement {
  const [mode, setMode] = useState<StepMode>('assembly')
  const [propDiscs, setPropDiscs] = useState(false)
  const [busy, setBusy] = useState(false)
  const [error, setError] = useState('')
  const [stepResult, setStepResult] = useState<CadResult | null>(null)
  const [stlResult, setStlResult] = useState<CadResult | null>(null)
  const [sectionsResult, setSectionsResult] = useState<CadResult | null>(null)
  const [drawingsResult, setDrawingsResult] = useState<CadResult | null>(null)
  const [varsetsResult, setVarsetsResult] = useState<CadResult | null>(null)

  const run = useCallback(
    async (task: () => Promise<CadAnswer>, apply: (result: CadResult) => void): Promise<void> => {
      setBusy(true)
      setError('')
      const answer = await task()
      setBusy(false)
      if (answer.ok && answer.result !== null) apply(answer.result)
      else setError(answer.message)
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
        <p className="form-hint">
          Sections S01… and couplers C01… as STEP + STL into cad/sections/. Independent of the STEP mode above.
        </p>
        <div className="button-row">
          <button type="button" disabled={busy} onClick={() => void run(exportSections, setSectionsResult)}>
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

      {busy && <div className="status">Exporting…</div>}
      {error !== '' && <div className="status draw-error">{error}</div>}
    </section>
  )
}

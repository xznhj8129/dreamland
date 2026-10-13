import type { ReactElement } from 'react'
import type { ScenarioOutline as Outline } from '../simApi'

export function ScenarioOutline({ plan, dirty }: { plan: Outline | null; dirty: boolean }): ReactElement {
  return <div className="scenario-outline">
    <h3>Scenario</h3>
    {plan && <p>{plan.description}</p>}
    {dirty && plan && <p className="form-hint">Save the Python edits to update this outline.</p>}
    {plan && Object.keys(plan.parameters).length > 0 && <details><summary>Parameters</summary><dl>{Object.entries(plan.parameters).map(([name, value]) => <div key={name}><dt>{name}</dt><dd>{JSON.stringify(value)}</dd></div>)}</dl></details>}
    {plan && Object.keys(plan.start).length > 0 && <details open><summary>Start (START)</summary><dl>{Object.entries(plan.start).map(([name, value]) => <div key={name}><dt>{name}</dt><dd>{String(value)}</dd></div>)}</dl></details>}
    {plan && Object.entries(plan.phases).map(([name, description]) => <section className="scenario-phase" key={name}>
      <h4>{name}</h4><p className="form-hint">{description}</p>
    </section>)}
    {plan?.telemetry && <p className="form-hint">Telemetry: {plan.telemetry.rate_hz} Hz maximum · {plan.telemetry.fields.join(', ')}</p>}
    <p className="form-hint">run(flight) executes top to bottom. Timeout, ground impact or Abort stops the run and retains logs.</p>
  </div>
}

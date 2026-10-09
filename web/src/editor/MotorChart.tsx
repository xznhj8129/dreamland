import { useEffect, useRef } from 'react'
import type { ReactElement } from 'react'
import uPlot from 'uplot'
import 'uplot/dist/uPlot.min.css'
import type { PropulsionChart } from '../types'

const HEIGHT = 260
const FALLBACK_WIDTH = 520
const AXIS_COLOR = '#67ff67'
const GRID_COLOR = 'rgba(10, 95, 10, 0.55)'
const HOVER_COLOR = '#67ff67'
// The Tk plot's series colors (editor_view.build_motor_info 1075-1076).
const SERIES_COLORS = ['#19ff19', '#33ccff', '#ffcc33']

/** A dashed line at the hover duty and its label, drawn by uPlot's draw hook (the Tk axvline, 1078-1079). */
function hoverPlugin(chartRef: { current: PropulsionChart }): uPlot.Plugin {
  return {
    hooks: {
      draw: (u: uPlot) => {
        const duty = chartRef.current.hover_duty
        if (duty === null) return
        const x = u.valToPos(duty, 'x', true)
        const { ctx } = u
        ctx.save()
        ctx.strokeStyle = HOVER_COLOR
        ctx.fillStyle = HOVER_COLOR
        ctx.setLineDash([4, 4])
        ctx.lineWidth = 1.0
        ctx.beginPath()
        ctx.moveTo(x, u.bbox.top)
        ctx.lineTo(x, u.bbox.top + u.bbox.height)
        ctx.stroke()
        ctx.setLineDash([])
        ctx.font = '10px "DejaVu Sans Mono", monospace'
        ctx.fillText('hover', x + 3, u.bbox.top + 10)
        ctx.restore()
      },
    },
  }
}

/** The three curves as uPlot wants them; rpm and power scaled as in Tk (per 1000 / per kW). */
function aligned(chart: PropulsionChart): uPlot.AlignedData {
  return [chart.duty, chart.thrust_n, chart.rpm.map((value) => value / 1000.0), chart.power_w.map((value) => value / 1000.0)]
}

function options(width: number, chartRef: { current: PropulsionChart }): uPlot.Options {
  return {
    width,
    height: HEIGHT,
    padding: [10, 10, 0, 0],
    cursor: { show: true, x: false, y: false, points: { show: false } },
    legend: { show: true },
    scales: { x: { time: false, min: 0, max: 1 } },
    axes: [
      {
        label: 'INAV motor output (ESC duty)',
        stroke: AXIS_COLOR,
        grid: { stroke: GRID_COLOR },
        ticks: { stroke: GRID_COLOR },
        values: (_u, splits) => splits.map((split) => split.toFixed(1)),
      },
      { label: 'per motor', stroke: AXIS_COLOR, grid: { stroke: GRID_COLOR }, ticks: { stroke: GRID_COLOR } },
    ],
    series: [
      {},
      { label: 'Thrust per motor (N)', stroke: SERIES_COLORS[0], width: 2, points: { show: false } },
      { label: 'RPM / 1000', stroke: SERIES_COLORS[1], width: 2, points: { show: false } },
      { label: 'kW per motor', stroke: SERIES_COLORS[2], width: 2, points: { show: false } },
    ],
    plugins: [hoverPlugin(chartRef)],
  }
}

/** The Motor info group's plot: uPlot, updated in place as the summary recomputes (no tab remount). */
export function MotorChart({ chart }: { chart: PropulsionChart }): ReactElement {
  const holderRef = useRef<HTMLDivElement | null>(null)
  const plotRef = useRef<uPlot | null>(null)
  const chartRef = useRef(chart)
  chartRef.current = chart

  useEffect(() => {
    const holder = holderRef.current
    if (holder === null) return undefined
    const width = holder.clientWidth > 0 ? holder.clientWidth : FALLBACK_WIDTH
    const plot = new uPlot(options(width, chartRef), aligned(chartRef.current), holder)
    plotRef.current = plot
    // A hidden tab has no width: when it becomes visible, give the plot the container's real size.
    const observer = new ResizeObserver(() => {
      const next = holder.clientWidth
      if (next > 0 && next !== plot.width) plot.setSize({ width: next, height: HEIGHT })
    })
    observer.observe(holder)
    return () => {
      observer.disconnect()
      plot.destroy()
      plotRef.current = null
    }
  }, [])

  useEffect(() => {
    plotRef.current?.setData(aligned(chart))
  }, [chart])

  return <div ref={holderRef} className="motor-chart" />
}

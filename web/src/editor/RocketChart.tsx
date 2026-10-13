import { useEffect, useRef } from 'react'
import type { ReactElement } from 'react'
import uPlot from 'uplot'
import 'uplot/dist/uPlot.min.css'
import { AXIS_COLOR, GRID_COLOR, SERIES_COLORS } from './MotorChart'

const HEIGHT = 180
const FALLBACK_WIDTH = 520

function aligned(curve: [number, number][]): uPlot.AlignedData {
  return [curve.map(([t]) => t), curve.map(([, thrust]) => thrust)]
}

function options(width: number): uPlot.Options {
  return {
    width,
    height: HEIGHT,
    padding: [10, 10, 0, 0],
    cursor: { show: true, x: false, y: false, points: { show: false } },
    legend: { show: false },
    scales: { x: { time: false } },
    axes: [
      { label: 'time (s)', stroke: AXIS_COLOR, grid: { stroke: GRID_COLOR }, ticks: { stroke: GRID_COLOR } },
      { label: 'thrust (N)', stroke: AXIS_COLOR, grid: { stroke: GRID_COLOR }, ticks: { stroke: GRID_COLOR } },
    ],
    series: [{}, { label: 'Thrust (N)', stroke: SERIES_COLORS[0], width: 2, points: { show: false } }],
  }
}

/** A rocket motor's published thrust curve (RASP), thrust against time from ignition; updated in place. */
export function RocketChart({ curve }: { curve: [number, number][] }): ReactElement {
  const holderRef = useRef<HTMLDivElement | null>(null)
  const plotRef = useRef<uPlot | null>(null)
  const curveRef = useRef(curve)
  curveRef.current = curve

  useEffect(() => {
    const holder = holderRef.current
    if (holder === null) return undefined
    const width = holder.clientWidth > 0 ? holder.clientWidth : FALLBACK_WIDTH
    const plot = new uPlot(options(width), aligned(curveRef.current), holder)
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
    plotRef.current?.setData(aligned(curve))
  }, [curve])

  return <div ref={holderRef} className="motor-chart" />
}

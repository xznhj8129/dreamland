import type { ReactElement, ReactNode } from 'react'

/** Cells are supplied by the caller: this layout never reads or changes an aircraft definition. */
export type PlacementCells = readonly [ReactNode, ReactNode, ReactNode]

interface PlacementTableProps {
  position: PlacementCells
  /** Roll about X, pitch about Y, yaw about Z. Omit when orientation is not editable. */
  angles?: PlacementCells
  label?: string
  positionLabel?: string
}

const AXES = ['X', 'Y', 'Z'] as const
const ANGLES = ['Roll', 'Pitch', 'Yaw'] as const

export function PlacementTable({ position, angles, label = 'Placement', positionLabel = 'Position' }: PlacementTableProps): ReactElement {
  return <table className="placement-table">
    <caption>{label}</caption>
    <colgroup><col className="placement-label-column" /><col /></colgroup>
    <tbody>
      {position.map((cell, index) => <tr key={AXES[index]}>
        <th scope="row">{positionLabel !== 'Position' ? `${positionLabel} ` : ''}{AXES[index]} <small>mm</small></th>
        <td>{cell}</td>
      </tr>)}
      {angles?.map((cell, index) => <tr key={ANGLES[index]}>
        <th scope="row">{ANGLES[index]} <small>deg</small></th>
        <td>{cell}</td>
      </tr>)}
    </tbody>
  </table>
}

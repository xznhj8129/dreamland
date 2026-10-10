import type { SceneItem } from './renderer'

export type LineLevel = 'info' | 'warning' | 'problem'

export type ComponentKind = 'surface' | 'part' | 'volume' | 'mount' | 'section' | 'battery' | 'fuselage'
export interface ComponentSelection { kind: ComponentKind; index: number; label?: string }

export interface ReportLine {
  text: string
  level: LineLevel
  target?: ComponentSelection | null
}

export interface ReportMetric {
  label: string
  value: string
  level: LineLevel
}

export interface ReportRow {
  values: string[]
  level: LineLevel
}

export interface ReportTable {
  columns: string[]
  rows: ReportRow[]
}

export interface ReportSection {
  title: string
  metrics: ReportMetric[]
  notes: ReportLine[]
  table: ReportTable | null
  collapsible: boolean
  children: ReportSection[]
}

export interface ReportPages {
  summary: ReportSection[]
  mass: ReportSection[]
  aero: ReportSection[]
  propulsion: ReportSection[]
}

export interface LogEntry {
  time: string
  title: string
  text: string
}

/** The airframe/2 definition as stored; unknown fields keep the JSON shape the server sent. */
export type Definition = Record<string, unknown>

/** A definition path: object keys and list indices (editor_view's data objects and lists). */
export type Path = readonly (string | number)[]

/** One atomic editor edit, as POST /api/editor/set applies it. */
export type PathEdit = { path: Path; value: unknown } | { path: Path; remove: true }

export interface ResolvedSection {
  x0_m: number
  x1_m: number
  solid: boolean
  empty_estimate_kg: number | null
  empty_kg: number
  complete_itemised_kg: [number, number]
  complete_kg: number | null
}

export interface ResolvedControl {
  number: number
  surface: string
  panel_index: number
  servo: string
}

/** webapp/serialize.py chart_payload: the Motor info chart curves (`editor_view.build_motor_info` 1069-1098). */
export interface PropulsionChart {
  duty: number[]
  thrust_n: number[]
  rpm: number[]
  power_w: number[]
  /** The hover point's duty and its nearest curve index; null when the aircraft cannot hover. */
  hover_duty: number | null
  hover_index: number | null
}

/** webapp/serialize.py pack_payload: the resolved battery label, size, mass and capacity (build_battery_group 1000-1005). */
export interface PackSummary {
  label: string
  size_mm: [number, number, number] | null
  mass_g: number
  capacity_mah: number
}

export interface ResolvedMassPoint {
  name: string
  mass_kg: [number, number]
  estimate_kg: number | null
}

export interface ResolvedSnapshot {
  mass_points: ResolvedMassPoint[]
  sections: ResolvedSection[]
  controls?: ResolvedControl[]
  [key: string]: unknown
}

/** One movable element's screen outline from POST /api/draw (webapp/drawing.py _handles). */
export interface DrawHandle {
  target: 'cut' | 'surface' | 'spars' | 'volume' | 'part' | 'battery' | 'rod'
  index: number
  kind: 'polygon' | 'line'
  /** Canvas px, flat x,y pairs: a closed loop for polygon, a polyline for line. */
  coords: number[]
  /** The definition axis a screen-horizontal drag edits; null when the view makes it ambiguous. */
  horizontal: 'x' | 'y' | 'z' | null
  /** The definition axis a screen-vertical drag edits; null when the view makes it ambiguous. */
  vertical: 'x' | 'y' | 'z' | null
  /** Draw order; a later handle is on top and wins the hit test. */
  order: number
}

/** One roll-slider detent from POST /api/draw (webapp/drawing_enrich.roll_detents): a surface's planform or tip angle. */
export interface RollDetent {
  surface: string
  kind: 'planform' | 'tip'
  roll_deg: number
}

/** The persistent server-side view state echoed by the draw payload (webapp/drawing.py _view_payload). */
export interface DrawView {
  mode: 'Longitudinal' | 'Front'
  roll_deg: number
  zoom_scale: number
  pan_x: number
  pan_y: number
}

/** One scrollbar thumb as fractions of the full travel (webapp/drawing.py scrollbar_fractions). */
export interface ScrollThumb {
  first: number
  last: number
}

/** The POST /api/draw answer, also returned as the drag fast path's response by POST /api/editor/move. */
export interface DrawPayload {
  scene: SceneItem[]
  handles: DrawHandle[]
  session_id: number
  metrics: Record<string, number | string>
  view: DrawView
  scrollbars: { x: ScrollThumb; y: ScrollThumb }
  roll_display: string
  roll_detents: RollDetent[]
  width: number
  height: number
}

/** One drag motion for POST /api/editor/move; the delta is in metres. */
export interface MoveBody {
  target: DrawHandle['target']
  index: number
  delta: { x?: number; y?: number; z?: number }
}

export interface Snapshot {
  project: string
  title: string
  session_id: number
  definition: Definition
  definition_revision: number
  /** Bumped on every accepted part-library save/delete (webapp/library.py); the editor-context cache follows it. */
  library_revision: number
  definition_valid: boolean
  dirty: boolean
  preset_status: string
  /** libs.presets.preset_spars for the definition's preset label: new spars prefilled from it, or null. */
  preset_spars: Record<string, unknown> | null
  preset_carry_through: Record<string, unknown> | null
  resolved: ResolvedSnapshot | null
  problems: string[]
  propulsion: ReportLine[]
  /** The Motor info chart data for the last valid propulsion summary; null when not powered or not resolved. */
  propulsion_chart: PropulsionChart | null
  /** The resolved pack summary shown under the battery fields; null without a propulsion summary. */
  pack_summary: PackSummary | null
  report: ReportLine[]
  report_pages: ReportPages | null
  log: LogEntry[]
  status: string
  aero_pending: boolean
}

export interface StateMessage extends Snapshot {
  type: 'state'
}

/** webapp/model3d.py build_model3d: one component as a flat triangle mesh in metres, airframe frame (x forward). */
export interface Model3DMesh {
  name: string
  kind: 'body' | 'pod' | 'panel' | 'control' | 'volume' | 'part' | 'motor' | 'prop' | 'rod' | 'spar'
  /** Flat x,y,z triples. */
  positions: number[]
  indices: number[]
  /** CSS hex colour, e.g. "#4fa3ff". */
  color: string
  opacity: number
}

export interface Model3DMarker {
  name: string
  /** x,y,z in metres, on the centreline. */
  position: [number, number, number]
  color: string
}

export interface Model3DBounds {
  min: [number, number, number]
  max: [number, number, number]
}

/** POST /api/model3d: the resolved airframe as renderable meshes plus CG/NP/Lift markers. */
export interface Model3DPayload {
  meshes: Model3DMesh[]
  markers: Model3DMarker[]
  bounds: Model3DBounds
}

/** One part-library category: a table file (data/parts/<name>.json) or a role directory (data/parts/<name>/). */
export interface LibraryCategory {
  name: string
  kind: 'table' | 'role'
}

/** The JSON name of an optional library field's type (webapp/library.py OPTIONAL_LIBRARY_FIELDS). */
export type LibraryOptionalKind = 'str' | 'int' | 'float'

/** GET /api/library: the whole part library plus the form metadata the browser editor needs. */
export interface LibraryPayload {
  parts: Record<string, Record<string, Record<string, unknown>>>
  categories: LibraryCategory[]
  field_order: string[]
  length_keys: string[]
  mass_keys: string[]
  optional_fields: Record<string, Record<string, LibraryOptionalKind>>
  defaults: Record<string, Record<string, unknown>>
  roles: string[]
}

/** POST /api/library/save: a New part (original null) or an edit, rename or category move. */
export interface LibrarySaveBody {
  category: string
  name: string
  original: { category: string; name: string } | null
  values: Record<string, unknown>
}

/** POST /api/library/delete: the entry to remove. */
export interface LibraryDeleteBody {
  category: string
  name: string
}

/** GET /api/editor/context: global editor data (parts, presets and the Tk editor constants ported to webapp/editor.py). */
export interface EditorContext {
  parts_data: Record<string, Record<string, Record<string, unknown>>>
  preset_names: string[]
  mm_keys: string[]
  gram_keys: string[]
  unit_suffixes: string[]
  choices: Record<string, string[]>
  type_flags: string[]
  profile_param_keys: Record<string, string[]>
  profile_param_defaults: Record<string, number>
  new_rod_pattern: Record<string, unknown>
  preset_airframe_rods: Record<string, Record<string, unknown>>
  rod_pattern_keys: string[]
  rod_extent_keys: string[]
  integer_keys: string[]
  cut_joints: string[]
  reinforcement_types: string[]
  no_preset: string
  /** webapp/editor.py context(): the ported Tk constants and add templates for the remaining editor tabs. */
  no_volume: string
  servo_outputs: string[]
  control_kinds: string[]
  control_roles: string[]
  control_kind_fields: Record<string, Record<string, number>>
  surface_prop_spins: string[]
  single_prop_spins: string[]
  volume_shapes: string[]
  volume_shape_fields: Record<string, Record<string, unknown>>
  part_categories: string[]
  layout_panel_counts: Record<string, number>
  placement_keys: string[]
  orientation_keys: string[]
  new_mount: Record<string, unknown>
  /** Propulsion and wiring (webapp/editor.py context()): model labels, add templates and motorlib combo lists. */
  battery_model_labels: Record<string, string>
  new_motor: Record<string, unknown>
  new_prop: string
  new_battery_run: Record<string, unknown>
  new_motor_run: Record<string, unknown>
  new_custom_pack: Record<string, unknown>
  remove_plug: string
  add_plug: string
  wiring_columns: string[]
  motor_manufacturers: string[]
  motors_by_manufacturer: Record<string, string[]>
  props: string[]
  cells: string[]
  connectors: string[]
  wire_gauges: number[]
}

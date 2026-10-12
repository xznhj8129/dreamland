import { useEffect, useMemo, useRef, useState } from 'react'
import type { ReactElement } from 'react'
import * as THREE from 'three'
import { OrbitControls } from 'three/addons/controls/OrbitControls.js'
import { buildAirframe, disposeAirframe, isModel3DPayload } from '../airframeMesh'
import type { AirframeGroups } from '../airframeMesh'
import type { Model3DPayload, Snapshot } from '../types'
import { aircraftQuaternion, controlDeflectionDeg } from '../flightPose'

type LogRows = (number | null)[][]
type CameraMode = 'orbit' | 'chase' | 'path'

/** The external flight geometry: fuselage and skins, control surfaces and propulsion; no interiors, cuts or spars. */
const FLIGHT_KINDS: ReadonlySet<string> = new Set(['body', 'panel', 'control', 'prop', 'pod', 'motor'])

// The retired path view's constants (the Tk view: sim_view.py PATH_COLUMNS/PATH_VIEW/colors/limits 43-49, draw_path 471-517).
const PATH_MIN_HALF_M = 50
const PATH_MIN_HEIGHT_M = 50
const PATH_BACKGROUND = '#141517'
const PATH_LINE_COLOR = '#19ff19'
const PATH_BOX_COLOR = '#14601a'
const PATH_FIT_MARGIN = 1.15
const PATH_DIRECTION = (() => {
  const elevation = THREE.MathUtils.degToRad(25) // PATH_VIEW elevation
  const azimuth = THREE.MathUtils.degToRad(-60) // PATH_VIEW azimuth
  return new THREE.Vector3(
    Math.cos(elevation) * Math.cos(azimuth),
    Math.cos(elevation) * Math.sin(azimuth),
    Math.sin(elevation),
  ).normalize()
})()

// The environment around the path: matte ground, a grid over the box footprint, a sky dome and horizon fog.
const GROUND_COLOR = '#46503f'
/** The solid ground at z = 0 follows the camera and is sized past the far plane, so it always fills the view
 *  under the aircraft and its edge never shows. */
const GRID_BASE_SIZE = 100
const GRID_DIVISIONS = 10
const GRID_Z_FRACTION = 2e-4
const SKY_TOP = '#5b7f9e'
const SKY_HORIZON = '#c9d6dd'
const FOG_MIN_FAR = 500
const FOG_FAR_FACTOR = 7

// Cameras: an iso framing of the model, and the chase camera's fixed offset from the heading.
const ISO_DIRECTION = new THREE.Vector3(1.0, -1.0, 0.6).normalize()
const ORBIT_MODEL_FACTOR = 3
const CHASE_LENGTH_FACTOR = 4
const CHASE_MIN_DISTANCE = 3
const CHASE_HEIGHT_FACTOR = 0.35
const SPEEDS = [0.5, 1, 2, 4] as const
const DEG2RAD = Math.PI / 180

/** One log row the aircraft can be placed at: t and every position/attitude field finite. */
interface FlightFrame {
  t: number
  east: number
  north: number
  agl: number
  roll: number
  pitch: number
  yaw: number
  /** Servo output in us, indexed by the servo<N> column number; null when that cell is absent or not finite. */
  servos: (number | null)[]
}

/** The compact valid frames, plus per frame whether it follows the previous one without a broken row between. */
interface FrameSet {
  frames: FlightFrame[]
  linked: boolean[]
}

interface PathFrame {
  cx: number
  cy: number
  half: number
  top: number
}

/** The scene's imperative surface: the React effects below drive it, the overlay's buttons and slider call it. */
interface FlightSceneHandle {
  setModel(payload: Model3DPayload | null): void
  setFrames(frameSet: FrameSet): void
  setMode(mode: CameraMode): void
  togglePlay(): void
  goLive(): void
  scrub(seconds: number): void
  selectSpeed(speed: number): void
  fit(): void
}

const REQUIRED_COLUMNS = ['t', 'east_m', 'north_m', 'agl_m', 'roll', 'pitch', 'yaw'] as const

function isFiniteNumber(value: number | null | undefined): value is number {
  return typeof value === 'number' && Number.isFinite(value)
}

/** The log's placcable frames: rows with every position, attitude and time column finite; broken rows are gaps. */
function readFrames(header: string[] | null, rows: LogRows): FrameSet {
  const empty: FrameSet = { frames: [], linked: [] }
  if (header === null) return empty
  const indices = REQUIRED_COLUMNS.map((name) => header.indexOf(name))
  if (indices.some((index) => index < 0)) return empty
  const servoColumns = new Map<number, number>()
  header.forEach((name, index) => {
    const match = /^servo(\d+)$/.exec(name)
    if (match !== null) servoColumns.set(Number(match[1]), index)
  })
  const frames: FlightFrame[] = []
  const linked: boolean[] = []
  let previousValid = false
  for (const row of rows) {
    // A row shorter than the header reads undefined here and is a gap, as is any non-finite cell.
    const t = row[indices[0]]
    const east = row[indices[1]]
    const north = row[indices[2]]
    const agl = row[indices[3]]
    const roll = row[indices[4]]
    const pitch = row[indices[5]]
    const yaw = row[indices[6]]
    if (!isFiniteNumber(t) || !isFiniteNumber(east) || !isFiniteNumber(north) || !isFiniteNumber(agl)
      || !isFiniteNumber(roll) || !isFiniteNumber(pitch) || !isFiniteNumber(yaw)) {
      previousValid = false
      continue
    }
    const servos: (number | null)[] = []
    for (const [servo, column] of servoColumns) {
      const value = row[column]
      servos[servo] = isFiniteNumber(value) ? value : null
    }
    frames.push({ t, east, north, agl, roll, pitch, yaw, servos })
    linked.push(previousValid)
    previousValid = true
  }
  return { frames, linked }
}

/** The ground box the path must fit: starts 50 m either side of the start and 50 m high, empty falls back to that. */
function boxFrame(frames: readonly FlightFrame[]): PathFrame {
  const first = frames[0]
  if (first === undefined) return { cx: 0, cy: 0, half: PATH_MIN_HALF_M, top: PATH_MIN_HEIGHT_M }
  let half = PATH_MIN_HALF_M
  let top = PATH_MIN_HEIGHT_M
  for (const frame of frames) {
    half = Math.max(half, Math.abs(frame.east - first.east), Math.abs(frame.north - first.north))
    top = Math.max(top, frame.agl)
  }
  return { cx: first.east, cy: first.north, half, top }
}

function frameRadius(frame: PathFrame): number {
  return Math.sqrt(2 * frame.half * frame.half + (frame.top / 2) * (frame.top / 2))
}

/** The 12 edges of the ground box as LineSegments vertices. */
function boxEdges(frame: PathFrame): Float32Array {
  const x0 = frame.cx - frame.half
  const x1 = frame.cx + frame.half
  const y0 = frame.cy - frame.half
  const y1 = frame.cy + frame.half
  const corners = [
    [x0, y0, 0], [x1, y0, 0], [x1, y1, 0], [x0, y1, 0],
    [x0, y0, frame.top], [x1, y0, frame.top], [x1, y1, frame.top], [x0, y1, frame.top],
  ]
  const edges = [0, 1, 1, 2, 2, 3, 3, 0, 4, 5, 5, 6, 6, 7, 7, 4, 0, 4, 1, 5, 2, 6, 3, 7]
  const positions = new Float32Array(edges.length * 3)
  edges.forEach((corner, index) => positions.set(corners[corner], index * 3))
  return positions
}

/**
 * The SIM view's 3D tab: the real airframe flying the logged path over ground, grid and sky.
 *
 * The path, its ground box and the start/end markers keep the retired path view's behaviour (green segments with
 * gaps at broken rows, a 50 m box that grows to fit). On top of it the model3d airframe is placed at each frame's
 * east/north/agl and attitude, with its control surfaces deflected by that frame's servo outputs, and the user can
 * follow it live or scrub the replay while choosing an orbit, chase or whole-box camera. The scene is built once
 * per mount and stays mounted while hidden (display: none); the model fetch and the log sampling update it in
 * place, and every geometry, material, renderer and observer is disposed on unmount.
 */
export function FlightScene({ snapshot, header, rows, runId, visible }: {
  snapshot: Snapshot | null
  header: string[] | null
  rows: LogRows
  runId: number | null
  visible: boolean
}): ReactElement {
  const hostRef = useRef<HTMLDivElement | null>(null)
  const handleRef = useRef<FlightSceneHandle | null>(null)
  const sliderRef = useRef<HTMLInputElement | null>(null)
  const timeRef = useRef<HTMLSpanElement | null>(null)
  const loadedKeyRef = useRef('')
  const [notice, setNotice] = useState('')
  const [model, setModel] = useState<Model3DPayload | null>(null)
  const [cameraMode, setCameraMode] = useState<CameraMode>('orbit')
  const [live, setLive] = useState(true)
  const [playing, setPlaying] = useState(false)
  const [speed, setSpeed] = useState(1)

  const session = snapshot?.session_id ?? -1
  const revision = snapshot?.definition_revision ?? -1
  const key = session >= 0 ? `${session}:${revision}` : ''

  const frameSet = useMemo(() => readFrames(header, rows), [header, rows])
  const lastFrame = frameSet.frames[frameSet.frames.length - 1]
  const totalTime = lastFrame === undefined ? 0 : lastFrame.t
  const hasFrames = frameSet.frames.length > 0

  // The renderer, scene, camera and controls live for the whole mount; the effects below only feed it data.
  useEffect(() => {
    const host = hostRef.current
    if (host === null) return undefined
    let renderer: THREE.WebGLRenderer
    try {
      renderer = new THREE.WebGLRenderer({ antialias: true })
    } catch {
      setNotice('WebGL is unavailable in this browser.')
      return undefined
    }
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2))
    host.appendChild(renderer.domElement)

    const scene = new THREE.Scene()
    scene.background = new THREE.Color(PATH_BACKGROUND)
    const fog = new THREE.Fog(new THREE.Color(SKY_HORIZON), FOG_MIN_FAR / 10, FOG_MIN_FAR)
    scene.fog = fog
    const camera = new THREE.PerspectiveCamera(45, 1, 0.1, 1e6)
    camera.up.set(0, 0, 1)
    const controls = new OrbitControls(camera, renderer.domElement)
    controls.enableDamping = true
    controls.dampingFactor = 0.12

    scene.add(new THREE.HemisphereLight(0xd8e8f0, 0x40503c, 1.5))
    const sun = new THREE.DirectionalLight(0xffffff, 2.0)
    sun.position.set(2, -3, 4)
    scene.add(sun)

    // The sky dome follows the camera; its gradient runs from the horizon colour at eye level to the top colour.
    const skyMaterial = new THREE.ShaderMaterial({
      uniforms: {
        topColor: { value: new THREE.Color(SKY_TOP) },
        horizonColor: { value: new THREE.Color(SKY_HORIZON) },
      },
      vertexShader: `
        varying float vHeight;
        void main() {
          vHeight = position.z;
          gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
        }
      `,
      fragmentShader: `
        uniform vec3 topColor;
        uniform vec3 horizonColor;
        varying float vHeight;
        void main() {
          float t = smoothstep(0.0, 0.7, clamp(vHeight, 0.0, 1.0));
          gl_FragColor = vec4(mix(horizonColor, topColor, t), 1.0);
          #include <colorspace_fragment>
        }
      `,
      side: THREE.BackSide,
      depthWrite: false,
      fog: false,
    })
    const skyGeometry = new THREE.SphereGeometry(1, 32, 16)
    const sky = new THREE.Mesh(skyGeometry, skyMaterial)
    sky.renderOrder = -1 // behind everything, without depth so the scene draws over it
    sky.frustumCulled = false
    scene.add(sky)

    const groundGeometry = new THREE.PlaneGeometry(1, 1)
    const groundMaterial = new THREE.MeshStandardMaterial({
      color: new THREE.Color(GROUND_COLOR), roughness: 1, metalness: 0,
      // A solid, unfogged surface at z = 0: the ground is there to be seen, not blended into the sky.
      fog: false,
      // Push the ground back in depth so the grid and the box's bottom edges (z = 0, coplanar with it) stay clean.
      polygonOffset: true, polygonOffsetFactor: 1, polygonOffsetUnits: 1,
    })
    const ground = new THREE.Mesh(groundGeometry, groundMaterial)
    ground.frustumCulled = false
    scene.add(ground)

    const grid = new THREE.GridHelper(GRID_BASE_SIZE, GRID_DIVISIONS, 0x6b7a68, 0x55624f)
    grid.rotation.x = Math.PI / 2 // GridHelper lies in XZ; this makes it the z=0 ground plane
    grid.material.transparent = true
    grid.material.opacity = 0.35
    grid.material.depthWrite = false
    grid.frustumCulled = false
    scene.add(grid)

    // The path line, the ground box and the start/end markers, as the retired path view drew them; fog only fades
    // the environment behind them, so the path keeps its colour across the whole box.
    let boxGeometry = new THREE.BufferGeometry()
    const boxMaterial = new THREE.LineBasicMaterial({ color: PATH_BOX_COLOR, fog: false })
    const box = new THREE.LineSegments(boxGeometry, boxMaterial)
    box.frustumCulled = false
    let pathGeometry = new THREE.BufferGeometry()
    const pathMaterial = new THREE.LineBasicMaterial({ color: PATH_LINE_COLOR, fog: false })
    const path = new THREE.LineSegments(pathGeometry, pathMaterial)
    path.frustumCulled = false

    /** Swap in a fresh position buffer: only geometry disposal frees the old GPU buffers (attribute.dispose() is WebGPU-only). */
    const setLinePositions = (line: THREE.LineSegments, vertices: Float32Array): THREE.BufferGeometry => {
      const geometry = new THREE.BufferGeometry()
      geometry.setAttribute('position', new THREE.BufferAttribute(vertices, 3))
      geometry.setDrawRange(0, vertices.length / 3)
      const previous = line.geometry
      line.geometry = geometry
      previous.dispose()
      return geometry
    }
    scene.add(box, path)

    let airframe: AirframeGroups | null = null
    const cg = new THREE.Vector3()
    const offset = new THREE.Vector3()
    const forward = new THREE.Vector3()
    const delta = new THREE.Vector3()
    const centre = new THREE.Vector3()
    const direction = new THREE.Vector3()
    const behind = new THREE.Vector3()
    let modelSize = 1

    let frames: FlightFrame[] = []
    let linked: boolean[] = []
    let mode: CameraMode = 'orbit'
    let fitting = true
    let live = true
    let playing = false
    let speed = 1
    let cursor = 0
    let total = 0
    let pathFrame: PathFrame | null = null
    let point: THREE.Vector3 | null = null
    let currentYaw = 0
    let framedOrbit = false
    let dirty = true
    const clock = new THREE.Clock()

    /** The distance at which the whole path and its ground still render, as the old update sized camera.far. */
    const pathFar = (target: PathFrame): number => camera.position.length() + target.half * 8 + target.top * 8

    const setCameraRange = (near: number, far: number): void => {
      camera.near = near
      camera.far = far
      camera.updateProjectionMatrix()
      sky.scale.setScalar(Math.max(far * 0.9, 1))
    }

    /** The matte ground, grid and fog follow the box as it grows; the box may extend the camera's far plane. */
    const applyEnvironment = (target: PathFrame): void => {
      const boxSize = 2 * target.half
      const fogFar = Math.max(FOG_MIN_FAR, frameRadius(target) * FOG_FAR_FACTOR)
      const far = Math.max(camera.far, pathFar(target))
      if (far !== camera.far) {
        camera.far = far
        camera.updateProjectionMatrix()
        sky.scale.setScalar(Math.max(far * 0.9, 1))
      }
      // The solid ground at z = 0: sized past the camera's far plane so its edge is never in view.
      ground.scale.set(far * 3, far * 3, 1)
      const gridScale = Math.max(boxSize / GRID_BASE_SIZE, 1e-6)
      grid.scale.set(gridScale, gridScale, 1)
      grid.position.set(target.cx, target.cy, Math.max(boxSize * GRID_Z_FRACTION, 0.002))
      fog.near = fogFar / 10
      fog.far = fogFar
      dirty = true
    }

    /** Fit the whole box, keeping the current orbit direction (the old view's Fit path). */
    const fitBox = (target: PathFrame): void => {
      centre.set(target.cx, target.cy, target.top / 2)
      const radius = frameRadius(target)
      const rect = host.getBoundingClientRect()
      const aspect = rect.width > 0 && rect.height > 0 ? rect.width / rect.height : camera.aspect
      const verticalFov = THREE.MathUtils.degToRad(camera.fov)
      const horizontalFov = 2 * Math.atan(Math.tan(verticalFov / 2) * Math.max(aspect, 1e-4))
      const distance = (radius / Math.sin(Math.min(verticalFov, horizontalFov) / 2)) * PATH_FIT_MARGIN
      direction.subVectors(camera.position, controls.target)
      if (direction.lengthSq() < 1e-12) direction.copy(PATH_DIRECTION)
      direction.normalize()
      camera.position.copy(centre).addScaledVector(direction, distance)
      controls.target.copy(centre)
      setCameraRange(Math.max(distance / 1000, 0.01), Math.max(distance + radius * 6, 1000))
      controls.update()
      dirty = true
    }

    /** Frame the aircraft itself from the iso direction at ~3x its largest dimension (orbit's first view). */
    const frameAircraft = (position: THREE.Vector3): void => {
      const distance = modelSize * ORBIT_MODEL_FACTOR
      camera.position.copy(position).addScaledVector(ISO_DIRECTION, distance)
      controls.target.copy(position)
      const pathRadius = pathFrame === null ? 0 : frameRadius(pathFrame)
      setCameraRange(Math.max(distance / 50, 0.01), Math.max(distance * 10, 1000, pathRadius * 14))
      controls.update()
      dirty = true
    }

    /** Chase: behind and above the aircraft using heading only, so roll and pitch do not swing the camera. */
    const applyChase = (position: THREE.Vector3): void => {
      const distance = Math.max(modelSize * CHASE_LENGTH_FACTOR, CHASE_MIN_DISTANCE)
      behind.set(-Math.sin(currentYaw), -Math.cos(currentYaw), 0)
      camera.position.copy(position).addScaledVector(behind, distance)
      camera.position.z += distance * CHASE_HEIGHT_FACTOR
      // OrbitControls.update() re-aims the camera at its target every frame, so chase must move the target too.
      controls.target.copy(position)
      camera.lookAt(position)
      const pathRadius = pathFrame === null ? 0 : frameRadius(pathFrame)
      setCameraRange(Math.max(distance / 50, 0.01), Math.max(distance * 10, 1000, pathRadius * 14))
      dirty = true
    }

    /** Place the airframe at one pose and let the current camera mode follow it. */
    const applyPose = (quaternion: THREE.Quaternion, east: number, north: number, agl: number, servos: (number | null)[]): void => {
      const previous = point
      const current = new THREE.Vector3(east, north, agl)
      point = current
      forward.set(-1, 0, 0).applyQuaternion(quaternion)
      currentYaw = Math.atan2(forward.x, forward.y)
      if (airframe !== null) {
        offset.copy(cg).applyQuaternion(quaternion)
        airframe.group.position.set(east - offset.x, north - offset.y, agl - offset.z)
        airframe.group.quaternion.copy(quaternion)
        airframe.group.visible = true
        for (const control of airframe.controls) {
          const us = servos[control.servo] ?? Number.NaN
          control.pivot.quaternion.setFromAxisAngle(control.axis, controlDeflectionDeg(us, control.minDeg, control.maxDeg) * DEG2RAD)
        }
      }
      if (mode === 'orbit') {
        if (!framedOrbit) {
          if (airframe !== null) {
            framedOrbit = true
            frameAircraft(current)
          }
        } else if (previous !== null && airframe !== null) {
          // Carry the user's orbit and zoom along with the aircraft.
          delta.subVectors(current, previous)
          camera.position.add(delta)
          controls.target.add(delta)
        }
      } else if (mode === 'chase' && airframe !== null) {
        applyChase(current)
      }
      dirty = true
    }

    const applyFrame = (frame: FlightFrame): void => {
      applyPose(aircraftQuaternion(frame.roll, frame.pitch, frame.yaw), frame.east, frame.north, frame.agl, frame.servos)
    }

    /** The servo outputs between two frames; a missing cell holds the other end, both missing stays neutral. */
    const lerpServos = (a: FlightFrame, b: FlightFrame, fraction: number): (number | null)[] => {
      const count = Math.max(a.servos.length, b.servos.length)
      const servos: (number | null)[] = []
      for (let servo = 0; servo < count; servo++) {
        const from = a.servos[servo] ?? null
        const to = b.servos[servo] ?? null
        if (from === null) servos[servo] = to
        else if (to === null) servos[servo] = from
        else servos[servo] = from + (to - from) * fraction
      }
      return servos
    }

    /** Replay: interpolate between the bracketing frames; across a gap the aircraft holds the last valid frame. */
    const applyTime = (seconds: number): void => {
      const lastIndex = frames.length - 1
      if (lastIndex < 0) return
      if (lastIndex === 0 || seconds <= frames[0].t) {
        applyFrame(frames[0])
        return
      }
      if (seconds >= total) {
        applyFrame(frames[lastIndex])
        return
      }
      let low = 0
      let high = lastIndex
      while (low < high) {
        const middle = (low + high) >> 1
        if (frames[middle].t < seconds) low = middle + 1
        else high = middle
      }
      const nextIndex = Math.max(low, 1)
      const previous = frames[nextIndex - 1]
      const next = frames[nextIndex]
      // Inside a gap (strictly before its far end) the aircraft holds the last valid frame.
      if (!linked[nextIndex] && seconds < next.t) {
        applyFrame(previous)
        return
      }
      const span = next.t - previous.t
      const fraction = span > 1e-9 ? (seconds - previous.t) / span : 0
      const quaternion = aircraftQuaternion(previous.roll, previous.pitch, previous.yaw)
      quaternion.slerp(aircraftQuaternion(next.roll, next.pitch, next.yaw), fraction)
      applyPose(
        quaternion,
        previous.east + (next.east - previous.east) * fraction,
        previous.north + (next.north - previous.north) * fraction,
        previous.agl + (next.agl - previous.agl) * fraction,
        lerpServos(previous, next, fraction),
      )
    }

    const updateTransport = (): void => {
      const slider = sliderRef.current
      // The slider is uncontrolled and scrub() keeps the cursor equal to its value, so writing it back is stable.
      if (slider !== null) slider.value = String(cursor)
      const label = timeRef.current
      if (label !== null) label.textContent = `${cursor.toFixed(1)} / ${total.toFixed(1)} s`
    }

    /** Live rows or a new run: rebuild the path and box, keep the aircraft at the newest frame unless replaying. */
    const setFrames = (frameSet: FrameSet): void => {
      frames = frameSet.frames
      linked = frameSet.linked
      const last = frames[frames.length - 1]
      total = last === undefined ? 0 : last.t
      cursor = Math.min(cursor, total)
      const positions: number[] = []
      for (let index = 1; index < frames.length; index++) {
        if (!linked[index]) continue
        const from = frames[index - 1]
        const to = frames[index]
        positions.push(from.east, from.north, from.agl, to.east, to.north, to.agl)
      }
      pathGeometry = setLinePositions(path, new Float32Array(positions))
      const next = boxFrame(frames)
      const changed = pathFrame === null || pathFrame.cx !== next.cx || pathFrame.cy !== next.cy
        || pathFrame.half !== next.half || pathFrame.top !== next.top
      if (changed) {
        pathFrame = next
        boxGeometry = setLinePositions(box, boxEdges(next))
        applyEnvironment(next)
        if (fitting && mode === 'path') fitBox(next)
      }
      if (last !== undefined) {
        if (live) {
          cursor = total
          applyFrame(last)
        } else {
          applyTime(cursor)
        }
      } else if (airframe !== null) {
        airframe.group.visible = false
      }
      updateTransport()
      dirty = true
    }

    const setModel = (payload: Model3DPayload | null): void => {
      if (airframe !== null) {
        scene.remove(airframe.group)
        disposeAirframe(airframe)
        airframe = null
      }
      if (payload === null) {
        // A definition that no longer resolves (model3d's 409): the path stays, the aircraft goes.
        dirty = true
        return
      }
      const groups = buildAirframe(payload, { transparentSkin: false, kinds: FLIGHT_KINDS })
      cg.set(payload.cg[0], payload.cg[1], payload.cg[2])
      modelSize = Math.max(payload.bounds.max[0] - payload.bounds.min[0], payload.bounds.max[1] - payload.bounds.min[1],
        payload.bounds.max[2] - payload.bounds.min[2], 1e-3)
      groups.group.visible = false
      scene.add(groups.group) // markers and the other buckets stay out of the flight scene
      airframe = groups
      framedOrbit = false
      if (frames.length > 0) applyTime(live ? total : cursor)
      dirty = true
    }

    const setMode = (next: CameraMode): void => {
      mode = next
      setCameraMode(next)
      if (next === 'chase') {
        controls.enabled = false
        if (point !== null && airframe !== null) applyChase(point)
      } else if (next === 'orbit') {
        controls.enabled = true
        if (point !== null) {
          if (!framedOrbit && airframe !== null) {
            framedOrbit = true
            frameAircraft(point)
          } else {
            controls.target.copy(point)
            controls.update()
          }
        }
      } else {
        controls.enabled = true
        fitting = true
        if (pathFrame !== null) fitBox(pathFrame)
      }
      dirty = true
    }

    const goLive = (): void => {
      live = true
      playing = false
      setLive(true)
      setPlaying(false)
      cursor = total
      const last = frames[frames.length - 1]
      if (last !== undefined) applyFrame(last)
      updateTransport()
      dirty = true
    }

    const togglePlay = (): void => {
      if (frames.length === 0) return
      if (playing) {
        playing = false
        setPlaying(false)
        return
      }
      if (live || cursor >= total - 1e-9) cursor = 0 // from the end, play runs the whole log again
      live = false
      playing = true
      setLive(false)
      setPlaying(true)
      applyTime(cursor)
      updateTransport()
      dirty = true
    }

    const scrub = (seconds: number): void => {
      if (frames.length === 0) return
      cursor = Math.min(Math.max(seconds, 0), total)
      live = false
      playing = false
      setLive(false)
      setPlaying(false)
      applyTime(cursor)
      updateTransport()
      dirty = true
    }

    const fit = (): void => {
      if (mode === 'chase') {
        if (point !== null && airframe !== null) applyChase(point)
      } else if (mode === 'path') {
        if (pathFrame !== null) fitBox(pathFrame)
      } else {
        if (point !== null && airframe !== null) {
          framedOrbit = true
          frameAircraft(point)
        }
      }
      dirty = true
    }

    const resize = (): void => {
      const rect = host.getBoundingClientRect()
      if (rect.width < 1 || rect.height < 1) return
      renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2))
      renderer.setSize(Math.floor(rect.width), Math.floor(rect.height))
      camera.aspect = rect.width / rect.height
      camera.updateProjectionMatrix()
      dirty = true
    }
    const observer = new ResizeObserver(() => resize())
    observer.observe(host)
    resize()

    const onStart = (): void => {
      if (mode === 'path') fitting = false // wheel, pan or orbit gives the user ownership of the box camera
    }
    // OrbitControls consumes a wheel zoom with its own update() (zoom-to-cursor), so by the animation frame the
    // update() there reports no movement; its change event is what marks that camera move for a repaint.
    const onControlsChange = (): void => {
      dirty = true
    }
    controls.addEventListener('start', onStart)
    controls.addEventListener('change', onControlsChange)

    // The minimum box before any data, framed once like the retired view did on its first update.
    pathFrame = boxFrame([])
    boxGeometry = setLinePositions(box, boxEdges(pathFrame))
    applyEnvironment(pathFrame)
    fitBox(pathFrame)

    // On-demand rendering: the playback cursor advances on the wall clock, everything else marks itself dirty.
    renderer.setAnimationLoop(() => {
      const elapsed = clock.getDelta()
      if (!live && playing && frames.length > 0) {
        cursor = Math.min(cursor + elapsed * speed, total)
        if (cursor >= total) {
          playing = false
          setPlaying(false)
        }
        applyTime(cursor)
        updateTransport()
      }
      const moved = controls.update()
      sky.position.copy(camera.position) // the dome always surrounds the camera
      ground.position.set(camera.position.x, camera.position.y, 0) // so does the fog-sized ground disc
      if (moved || dirty) {
        dirty = false
        renderer.render(scene, camera)
      }
    })

    handleRef.current = {
      setModel,
      setFrames,
      setMode,
      togglePlay,
      goLive,
      scrub,
      selectSpeed: (next) => {
        speed = next
        setSpeed(next)
      },
      fit,
    }

    return () => {
      handleRef.current = null
      observer.disconnect()
      renderer.setAnimationLoop(null)
      controls.removeEventListener('start', onStart)
      controls.removeEventListener('change', onControlsChange)
      controls.dispose()
      if (airframe !== null) {
        scene.remove(airframe.group)
        disposeAirframe(airframe)
        airframe = null
      }
      boxGeometry.dispose()
      boxMaterial.dispose()
      pathGeometry.dispose()
      pathMaterial.dispose()
      skyGeometry.dispose()
      skyMaterial.dispose()
      groundGeometry.dispose()
      groundMaterial.dispose()
      grid.geometry.dispose()
      grid.material.dispose()
      renderer.dispose()
      renderer.domElement.remove()
    }
  }, [])

  // Feed the log and the model into the scene; the path/pose only change with new data.
  useEffect(() => {
    handleRef.current?.setFrames(frameSet)
  }, [frameSet])

  useEffect(() => {
    handleRef.current?.setModel(model)
  }, [model])

  // A new run (the log's run id changes) starts live: a replay left over from the previous run must not freeze
  // at its old cursor. The id catches a run that starts and ends between two state polls, unlike a `running` edge.
  useEffect(() => {
    if (runId !== null) handleRef.current?.goLive()
  }, [runId])

  // Fetch the airframe once per session/definition revision while the tab is visible; a stale request is aborted.
  useEffect(() => {
    if (!visible || key === '' || key === loadedKeyRef.current) return undefined
    const controller = new AbortController()
    let cancelled = false
    void (async () => {
      try {
        const response = await fetch('/api/model3d', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ session_id: session }),
          signal: controller.signal,
        })
        const body: unknown = await response.json().catch(() => null)
        if (cancelled) return
        if (response.ok) {
          if (!isModel3DPayload(body)) {
            setNotice('3D model response was not understood.')
            return
          }
          loadedKeyRef.current = key
          setNotice('')
          setModel(body)
        } else if (response.status === 409) {
          // No resolved airframe for this session (model3d answers the snapshot with 409).
          loadedKeyRef.current = key
          setNotice('No valid airframe to display. Recompute after fixing the definition.')
          setModel(null)
        } else {
          setNotice(`3D model request failed (${response.status}).`)
        }
      } catch (error) {
        if (cancelled || (error instanceof DOMException && error.name === 'AbortError')) return
        setNotice('3D model request failed.')
      }
    })()
    return () => {
      cancelled = true
      controller.abort()
    }
  }, [visible, key, session])

  return (
    <div className="sim-path" style={{ display: visible ? 'flex' : 'none' }} ref={hostRef}>
      <div className="sim-path-tools">
        <div className="button-row" role="group" aria-label="Camera">
          <button type="button" aria-pressed={cameraMode === 'orbit'} onClick={() => handleRef.current?.setMode('orbit')}>Orbit</button>
          <button type="button" aria-pressed={cameraMode === 'chase'} onClick={() => handleRef.current?.setMode('chase')}>Chase</button>
          <button type="button" aria-pressed={cameraMode === 'path'} onClick={() => handleRef.current?.setMode('path')}>Path</button>
          <button type="button" onClick={() => handleRef.current?.fit()}>Fit</button>
        </div>
        <div className="button-row" role="group" aria-label="Playback">
          <button type="button" disabled={!hasFrames} onClick={() => handleRef.current?.togglePlay()}>{playing ? 'Pause' : 'Play'}</button>
          <button type="button" aria-pressed={live} disabled={!hasFrames} onClick={() => handleRef.current?.goLive()}>Live</button>
          <select className="field-input sim-path-speed" value={speed} onChange={(event) => handleRef.current?.selectSpeed(Number(event.target.value))} aria-label="Replay speed">
            {SPEEDS.map((value) => <option key={value} value={value}>{value}&#215;</option>)}
          </select>
          <span className="axes-label" ref={timeRef}>0.0 / 0.0 s</span>
        </div>
        <input
          className="sim-path-slider"
          ref={sliderRef}
          type="range"
          min={0}
          max={totalTime}
          step={0.1}
          defaultValue={0}
          disabled={!hasFrames}
          aria-label="Replay position (seconds)"
          onChange={(event) => handleRef.current?.scrub(Number(event.target.value))}
        />
      </div>
      {notice !== '' && <div className="model-notice">{notice}</div>}
    </div>
  )
}

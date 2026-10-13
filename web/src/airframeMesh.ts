import * as THREE from 'three'
import type { Model3DPayload } from './types'

/** Marker sphere radius as a fraction of the model's largest dimension. */
const MARKER_RADIUS_FRACTION = 0.02
/** A rocket flame at the motor's peak thrust; it scales linearly down to nothing at 0 N. */
export const FLAME_MAX_LENGTH_M = 1
const FLAME_COLOR = '#ff2a1a'
const FLAME_SEGMENTS = 16

/** One control surface's hinge pivot: rotating `pivot` about `axis` deflects it. */
export interface ControlPivot {
  /** Child of the returned group, positioned at the hinge point; its mesh geometry is hinge-relative. */
  pivot: THREE.Group
  /** Index of the servo<N> telemetry/log column. */
  servo: number
  minDeg: number
  maxDeg: number
  /** Unit hinge axis in the model frame; a right-hand rotation about it is a positive deflection. */
  axis: THREE.Vector3
}

/** One rocket motor's flame: a cone from the nozzle, pointing against the thrust, one metre long along local y. */
export interface RocketFlame {
  /** Child of the returned group; scale.y is the flame length in metres. */
  flame: THREE.Mesh
  /** N of the rocket<N>_thrust_n log column. */
  number: number
  peakThrustN: number
}

/** The airframe's scene buckets, as webapp/model3d.py build_model3d groups the meshes. */
export interface AirframeGroups {
  /** Body/panel/prop/parts/volumes/spars/rods AND the control pivots. */
  group: THREE.Group
  /** One per control mesh; empty when none. */
  controls: ControlPivot[]
  /** One per rocket motor when built with `flames`; hidden until a thrust is applied. */
  flames: RocketFlame[]
  markerGroup: THREE.Group
  guideGroup: THREE.Group
  sectionGroup: THREE.Group
  featureGroup: THREE.Group
}

function disposeObject(object: THREE.Object3D): void {
  if (object instanceof THREE.Mesh || object instanceof THREE.LineSegments) {
    object.geometry.dispose()
    const material = object.material
    if (Array.isArray(material)) {
      for (const entry of material) entry.dispose()
    } else {
      material.dispose()
    }
  }
  for (const child of object.children.slice()) disposeObject(child)
}

function disposeGroupContents(group: THREE.Group): void {
  for (const child of group.children.slice()) disposeObject(child)
  group.clear()
}

/** Dispose every geometry/material built by buildAirframe; the groups themselves carry no GPU resources. */
export function disposeAirframe(groups: AirframeGroups): void {
  disposeGroupContents(groups.group)
  disposeGroupContents(groups.markerGroup)
  disposeGroupContents(groups.guideGroup)
  disposeGroupContents(groups.sectionGroup)
  disposeGroupContents(groups.featureGroup)
}

export function isModel3DPayload(value: unknown): value is Model3DPayload {
  if (typeof value !== 'object' || value === null) return false
  const candidate = value as { meshes?: unknown; markers?: unknown; bounds?: unknown }
  return Array.isArray(candidate.meshes) && Array.isArray(candidate.markers) && typeof candidate.bounds === 'object' && candidate.bounds !== null
}

/**
 * Build the Three.js scene buckets for one model3d payload (webapp/model3d.py build_model3d).
 *
 * `kinds` restricts the built meshes (markers are always built); undefined builds every kind. A control mesh with
 * metadata gets its geometry translated to be hinge-relative and is wrapped in a pivot group at the hinge point,
 * so rotating the pivot about the hinge axis deflects the surface. With `flames`, each rocket motor also gets a
 * hidden flame cone at its nozzle. Everything else matches the payload one-to-one.
 */
export function buildAirframe(
  payload: Model3DPayload,
  options: { transparentSkin: boolean; kinds?: ReadonlySet<string>; flames?: boolean },
): AirframeGroups {
  const group = new THREE.Group()
  const markerGroup = new THREE.Group()
  const guideGroup = new THREE.Group()
  const sectionGroup = new THREE.Group()
  const featureGroup = new THREE.Group()
  const controls: ControlPivot[] = []
  const flames: RocketFlame[] = []
  const minimum = new THREE.Vector3(payload.bounds.min[0], payload.bounds.min[1], payload.bounds.min[2])
  const maximum = new THREE.Vector3(payload.bounds.max[0], payload.bounds.max[1], payload.bounds.max[2])
  const size = new THREE.Vector3().subVectors(maximum, minimum)
  const markerRadius = Math.max(Math.max(size.x, size.y, size.z) * MARKER_RADIUS_FRACTION, 0.005)
  for (const mesh of payload.meshes) {
    if (options.kinds !== undefined && !options.kinds.has(mesh.kind)) continue
    const geometry = new THREE.BufferGeometry()
    geometry.setAttribute('position', new THREE.Float32BufferAttribute(mesh.positions, 3))
    geometry.setIndex(mesh.indices)
    geometry.computeVertexNormals()
    if (mesh.kind === 'feature') {
      const cut = new THREE.Mesh(geometry, new THREE.MeshBasicMaterial({
        color: mesh.color, side: THREE.DoubleSide, transparent: true, opacity: mesh.opacity,
        depthWrite: false, depthTest: false,
      }))
      cut.renderOrder = 4
      const outline = new THREE.LineSegments(new THREE.EdgesGeometry(geometry),
        new THREE.LineBasicMaterial({ color: mesh.color, depthTest: false, transparent: true, opacity: 0.9 }))
      outline.renderOrder = 5
      featureGroup.add(cut, outline)
      continue
    }
    if (mesh.kind === 'section' || mesh.kind === 'loft') {
      const plane = new THREE.Mesh(geometry, new THREE.MeshBasicMaterial({
        color: mesh.color, side: THREE.DoubleSide, transparent: true, opacity: mesh.opacity,
        depthWrite: false, depthTest: false,
      }))
      plane.renderOrder = 2
      const border = new THREE.LineSegments(new THREE.EdgesGeometry(geometry),
        new THREE.LineBasicMaterial({ color: mesh.color, transparent: true, opacity: 0.8, depthTest: false }))
      border.renderOrder = 3
      ;(mesh.kind === 'loft' ? guideGroup : sectionGroup).add(plane, border)
      continue
    }
    // A control mesh is hinge-relative and pivoted at its hinge; its edges must come from the translated geometry.
    const control = mesh.kind === 'control' ? mesh.control : undefined
    if (control !== undefined) geometry.translate(-control.hinge[0], -control.hinge[1], -control.hinge[2])
    const doubleSided = mesh.kind === 'panel' || mesh.kind === 'control'
    const seeThrough = mesh.kind === 'volume' || mesh.kind === 'prop'
    const skin = options.transparentSkin && ['body', 'pod', 'panel', 'control', 'hatch'].includes(mesh.kind)
    const material = new THREE.MeshStandardMaterial({
      color: new THREE.Color(mesh.color),
      side: doubleSided || skin ? THREE.DoubleSide : THREE.FrontSide,
      transparent: skin || mesh.opacity < 1,
      opacity: skin ? 0.18 : mesh.opacity,
      depthWrite: !seeThrough && !skin,
      roughness: 0.7,
      metalness: 0.05,
    })
    const object = new THREE.Mesh(geometry, material)
    object.userData.kind = mesh.kind
    if (doubleSided) {
      // A flat panel/control shell seen edge-on (the Side view) renders nothing; its edges stay visible.
      object.add(new THREE.LineSegments(new THREE.EdgesGeometry(geometry), new THREE.LineBasicMaterial({ color: mesh.color })))
    }
    if (control !== undefined) {
      const pivot = new THREE.Group()
      pivot.position.set(control.hinge[0], control.hinge[1], control.hinge[2])
      pivot.add(object)
      group.add(pivot)
      controls.push({
        pivot,
        servo: control.servo,
        minDeg: control.min_deg,
        maxDeg: control.max_deg,
        axis: new THREE.Vector3(control.axis[0], control.axis[1], control.axis[2]),
      })
    } else {
      group.add(object)
    }
    const rocket = mesh.rocket
    if (options.flames === true && rocket !== undefined) {
      // Base on the nozzle, apex one metre out along local +y, turned to point against the thrust.
      const cone = new THREE.ConeGeometry(rocket.radius_m, 1, FLAME_SEGMENTS)
      cone.translate(0, 0.5, 0)
      const flame = new THREE.Mesh(cone, new THREE.MeshBasicMaterial({ color: FLAME_COLOR }))
      flame.position.set(rocket.nozzle[0], rocket.nozzle[1], rocket.nozzle[2])
      const exhaust = new THREE.Vector3(-rocket.thrust_dir[0], -rocket.thrust_dir[1], -rocket.thrust_dir[2])
      flame.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), exhaust)
      flame.visible = false
      group.add(flame)
      flames.push({ flame, number: rocket.number, peakThrustN: rocket.peak_thrust_n })
    }
  }
  for (const marker of payload.markers) {
    const sphere = new THREE.Mesh(
      new THREE.SphereGeometry(markerRadius, 20, 14),
      new THREE.MeshBasicMaterial({ color: new THREE.Color(marker.color) }),
    )
    sphere.position.set(marker.position[0], marker.position[1], marker.position[2])
    markerGroup.add(sphere)
  }
  return { group, controls, flames, markerGroup, guideGroup, sectionGroup, featureGroup }
}

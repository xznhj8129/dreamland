import * as THREE from 'three'
import type { Model3DPayload } from './types'

/** Marker sphere radius as a fraction of the model's largest dimension. */
const MARKER_RADIUS_FRACTION = 0.02

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

/** The airframe's scene buckets, as webapp/model3d.py build_model3d groups the meshes. */
export interface AirframeGroups {
  /** Body/panel/prop/parts/volumes/spars/rods AND the control pivots. */
  group: THREE.Group
  /** One per control mesh; empty when none. */
  controls: ControlPivot[]
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
 * so rotating the pivot about the hinge axis deflects the surface. Everything else matches the payload one-to-one.
 */
export function buildAirframe(
  payload: Model3DPayload,
  options: { transparentSkin: boolean; kinds?: ReadonlySet<string> },
): AirframeGroups {
  const group = new THREE.Group()
  const markerGroup = new THREE.Group()
  const guideGroup = new THREE.Group()
  const sectionGroup = new THREE.Group()
  const featureGroup = new THREE.Group()
  const controls: ControlPivot[] = []
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
    const skin = options.transparentSkin && ['body', 'pod', 'panel', 'control'].includes(mesh.kind)
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
  }
  for (const marker of payload.markers) {
    const sphere = new THREE.Mesh(
      new THREE.SphereGeometry(markerRadius, 20, 14),
      new THREE.MeshBasicMaterial({ color: new THREE.Color(marker.color) }),
    )
    sphere.position.set(marker.position[0], marker.position[1], marker.position[2])
    markerGroup.add(sphere)
  }
  return { group, controls, markerGroup, guideGroup, sectionGroup, featureGroup }
}

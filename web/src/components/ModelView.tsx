import { useCallback, useEffect, useRef, useState } from 'react'
import type { ReactElement } from 'react'
import * as THREE from 'three'
import { OrbitControls } from 'three/addons/controls/OrbitControls.js'
import type { Model3DBounds, Model3DPayload, Snapshot } from '../types'

/** The 2D canvas colour (guibuilder.py UI_CANVAS) keeps the 3D view on the app's dark theme. */
const BACKGROUND = '#001000'
const FIT_MARGIN = 1.15
/** Marker sphere radius as a fraction of the model's largest dimension. */
const MARKER_RADIUS_FRACTION = 0.02
/** Looking straight down a preset axis makes the (0, 0, 1) up vector degenerate; nudge the axis. */
const AXIS_NUDGE = 1e-3
const ISO_DIRECTION = new THREE.Vector3(1.0, -1.0, 0.6).normalize()
const TOP_DIRECTION = new THREE.Vector3(AXIS_NUDGE, 0.0, 1.0).normalize()
const FRONT_DIRECTION = new THREE.Vector3(1.0, 0.0, 0.0)
const SIDE_DIRECTION = new THREE.Vector3(0.0, -1.0, 0.0)

interface LoadedModel {
  key: string
  session: number
  payload: Model3DPayload
}

function disposeGroupContents(group: THREE.Group): void {
  const children = group.children.slice()
  group.clear()
  for (const child of children) {
    if (child instanceof THREE.Mesh) {
      child.geometry.dispose()
      const material = child.material
      if (Array.isArray(material)) {
        for (const entry of material) entry.dispose()
      } else {
        material.dispose()
      }
    }
  }
}

function isModel3DPayload(value: unknown): value is Model3DPayload {
  if (typeof value !== 'object' || value === null) return false
  const candidate = value as { meshes?: unknown; markers?: unknown; bounds?: unknown }
  return Array.isArray(candidate.meshes) && Array.isArray(candidate.markers) && typeof candidate.bounds === 'object' && candidate.bounds !== null
}

/**
 * The live Three.js view of the resolved airframe (POST /api/model3d, webapp/model3d.py).
 *
 * The scene is built once per mount; the model group is rebuilt only when a new payload arrives, disposing the
 * old geometries/materials. The fetch is fenced to the session/revision/aero key of the current snapshot and is
 * aborted when that key changes, so a response from a project the browser has left is never drawn. The 3D tab
 * keeps the component mounted while hidden (display: none); on reveal the ResizeObserver re-sizes and repaints.
 */
export function ModelView({ snapshot, visible }: { snapshot: Snapshot | null; visible: boolean }): ReactElement {
  const canvasHostRef = useRef<HTMLDivElement | null>(null)
  const cameraRef = useRef<THREE.PerspectiveCamera | null>(null)
  const controlsRef = useRef<OrbitControls | null>(null)
  const groupRef = useRef<THREE.Group | null>(null)
  const markerGroupRef = useRef<THREE.Group | null>(null)
  const boundsRef = useRef<Model3DBounds | null>(null)
  const lastCentreRef = useRef<THREE.Vector3 | null>(null)
  const framedSessionRef = useRef<number | null>(null)
  const dirtyRef = useRef(true)
  const loadedKeyRef = useRef('')
  const [model, setModel] = useState<LoadedModel | null>(null)
  const [notice, setNotice] = useState('')

  const session = snapshot?.session_id ?? -1
  const revision = snapshot?.definition_revision ?? -1
  const aeroReady = snapshot !== null && !snapshot.aero_pending
  const key = session >= 0 ? `${session}:${revision}:${aeroReady ? 1 : 0}` : ''

  /** Frame the model: keep the current view direction (or take the given one) and back off to fit the bounds. */
  const placeCamera = useCallback((direction: THREE.Vector3 | null): void => {
    const camera = cameraRef.current
    const controls = controlsRef.current
    const host = canvasHostRef.current
    const bounds = boundsRef.current
    if (camera === null || controls === null || host === null || bounds === null) return
    const minimum = new THREE.Vector3(bounds.min[0], bounds.min[1], bounds.min[2])
    const maximum = new THREE.Vector3(bounds.max[0], bounds.max[1], bounds.max[2])
    const size = new THREE.Vector3().subVectors(maximum, minimum)
    const centre = new THREE.Vector3().addVectors(minimum, maximum).multiplyScalar(0.5)
    const radius = Math.max(size.length() / 2, 0.01)
    const rect = host.getBoundingClientRect()
    const aspect = rect.width > 0 && rect.height > 0 ? rect.width / rect.height : camera.aspect
    const verticalFov = THREE.MathUtils.degToRad(camera.fov)
    const horizontalFov = 2 * Math.atan(Math.tan(verticalFov / 2) * Math.max(aspect, 1e-4))
    const distance = (radius / Math.sin(Math.min(verticalFov, horizontalFov) / 2)) * FIT_MARGIN
    let viewDirection = direction
    if (viewDirection === null) {
      viewDirection = camera.position.clone().sub(controls.target)
      if (viewDirection.lengthSq() < 1e-12) viewDirection.copy(ISO_DIRECTION)
      viewDirection.normalize()
    }
    camera.position.copy(centre).addScaledVector(viewDirection, distance)
    controls.target.copy(centre)
    camera.near = Math.max(radius / 100, 0.001)
    camera.far = Math.max(distance + radius * 6, 100)
    camera.updateProjectionMatrix()
    controls.update()
    dirtyRef.current = true
  }, [])

  // The renderer, scene, camera and controls live for the whole mount; only the payload rebuilds geometry.
  useEffect(() => {
    const host = canvasHostRef.current
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
    scene.background = new THREE.Color(BACKGROUND)
    const camera = new THREE.PerspectiveCamera(45, 1, 0.05, 500)
    camera.up.set(0, 0, 1)
    camera.position.copy(ISO_DIRECTION).multiplyScalar(3)
    const controls = new OrbitControls(camera, renderer.domElement)
    controls.enableDamping = true
    controls.dampingFactor = 0.12
    controls.target.set(0, 0, 0)
    scene.add(new THREE.HemisphereLight(0xd8ffe0, 0x0a2a0a, 1.4))
    const sun = new THREE.DirectionalLight(0xffffff, 2.0)
    sun.position.set(2, -3, 4)
    scene.add(sun)
    const group = new THREE.Group()
    const markerGroup = new THREE.Group()
    scene.add(group, markerGroup)
    cameraRef.current = camera
    controlsRef.current = controls
    groupRef.current = group
    markerGroupRef.current = markerGroup

    const resize = (width: number, height: number): void => {
      if (width < 1 || height < 1) return
      renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2))
      renderer.setSize(width, height)
      camera.aspect = width / height
      camera.updateProjectionMatrix()
      dirtyRef.current = true
    }
    const observer = new ResizeObserver((entries) => {
      const rect = entries[0]?.contentRect
      if (rect === undefined) return
      resize(Math.floor(rect.width), Math.floor(rect.height))
    })
    observer.observe(host)
    const initial = host.getBoundingClientRect()
    resize(Math.floor(initial.width), Math.floor(initial.height))

    // On-demand rendering: damping keeps update() returning true until the camera settles.
    renderer.setAnimationLoop(() => {
      const moved = controls.update()
      if (moved || dirtyRef.current) {
        dirtyRef.current = false
        renderer.render(scene, camera)
      }
    })

    return () => {
      observer.disconnect()
      renderer.setAnimationLoop(null)
      controls.dispose()
      disposeGroupContents(group)
      disposeGroupContents(markerGroup)
      renderer.dispose()
      renderer.domElement.remove()
      cameraRef.current = null
      controlsRef.current = null
      groupRef.current = null
      markerGroupRef.current = null
      boundsRef.current = null
      lastCentreRef.current = null
      framedSessionRef.current = null
    }
  }, [])

  // Rebuild only when the payload changes; the first payload of a session frames the camera, later ones keep it.
  useEffect(() => {
    const group = groupRef.current
    const markerGroup = markerGroupRef.current
    if (group === null || markerGroup === null) return
    disposeGroupContents(group)
    disposeGroupContents(markerGroup)
    if (model === null) {
      boundsRef.current = null
      dirtyRef.current = true
      return
    }
    const data = model.payload
    boundsRef.current = data.bounds
    const minimum = new THREE.Vector3(data.bounds.min[0], data.bounds.min[1], data.bounds.min[2])
    const maximum = new THREE.Vector3(data.bounds.max[0], data.bounds.max[1], data.bounds.max[2])
    const size = new THREE.Vector3().subVectors(maximum, minimum)
    const markerRadius = Math.max(Math.max(size.x, size.y, size.z) * MARKER_RADIUS_FRACTION, 0.005)
    for (const mesh of data.meshes) {
      const geometry = new THREE.BufferGeometry()
      geometry.setAttribute('position', new THREE.Float32BufferAttribute(mesh.positions, 3))
      geometry.setIndex(mesh.indices)
      geometry.computeVertexNormals()
      const doubleSided = mesh.kind === 'panel' || mesh.kind === 'control'
      const seeThrough = mesh.kind === 'volume' || mesh.kind === 'prop'
      const material = new THREE.MeshStandardMaterial({
        color: new THREE.Color(mesh.color),
        side: doubleSided ? THREE.DoubleSide : THREE.FrontSide,
        transparent: mesh.opacity < 1,
        opacity: mesh.opacity,
        depthWrite: !seeThrough,
        roughness: 0.7,
        metalness: 0.05,
      })
      group.add(new THREE.Mesh(geometry, material))
    }
    for (const marker of data.markers) {
      const sphere = new THREE.Mesh(
        new THREE.SphereGeometry(markerRadius, 20, 14),
        new THREE.MeshBasicMaterial({ color: new THREE.Color(marker.color) }),
      )
      sphere.position.set(marker.position[0], marker.position[1], marker.position[2])
      markerGroup.add(sphere)
    }
    const centre = new THREE.Vector3().addVectors(minimum, maximum).multiplyScalar(0.5)
    if (framedSessionRef.current !== model.session) {
      framedSessionRef.current = model.session
      lastCentreRef.current = centre
      placeCamera(ISO_DIRECTION)
    } else {
      // A definition edit moved the model; carry the camera the same way so the view stays centred.
      const previous = lastCentreRef.current
      const camera = cameraRef.current
      const controls = controlsRef.current
      if (previous !== null && camera !== null && controls !== null) {
        const delta = new THREE.Vector3().subVectors(centre, previous)
        camera.position.add(delta)
        controls.target.copy(centre)
        controls.update()
      }
      lastCentreRef.current = centre
    }
    dirtyRef.current = true
  }, [model, placeCamera])

  // Fetch once per session/revision/aero state while the tab is visible; abort a request whose key goes stale.
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
          setModel({ key, session, payload: body })
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

  const empty = model !== null && model.payload.meshes.length === 0 && model.payload.markers.length === 0

  return (
    <div
      className="model-view"
      style={{ display: visible ? 'flex' : 'none' }}
      data-model-meshes={model?.payload.meshes.length ?? 0}
      data-model-markers={model?.payload.markers.length ?? 0}
    >
      <div className="graphics-controls">
        <span className="axes-label">X = Front &nbsp; Y = Right &nbsp; Z = Up</span>
        <button type="button" onClick={() => placeCamera(null)} disabled={model === null}>
          Fit view
        </button>
        <button type="button" onClick={() => placeCamera(TOP_DIRECTION)} disabled={model === null}>
          Top
        </button>
        <button type="button" onClick={() => placeCamera(FRONT_DIRECTION)} disabled={model === null}>
          Front
        </button>
        <button type="button" onClick={() => placeCamera(SIDE_DIRECTION)} disabled={model === null}>
          Side
        </button>
        <button type="button" onClick={() => placeCamera(ISO_DIRECTION)} disabled={model === null}>
          Iso
        </button>
        {notice !== '' && <span className="draw-error">{notice}</span>}
      </div>
      <div className="model-canvas" ref={canvasHostRef}>
        {empty && <div className="model-notice">No geometry to display.</div>}
      </div>
    </div>
  )
}

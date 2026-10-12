import * as THREE from 'three'

/**
 * The logged-airframe pose helpers of the SIM view's 3D flight scene.
 *
 * The model3d frame is the airframe's: x from the nose tip aft, y right, z up. JSBSim logs the true attitude as
 * roll/pitch/yaw in the NED body frame; the verified world rotation of the model frame in ENU is
 * R = C . R_ned_body(roll, pitch, yaw) . diag(-1, 1, -1), C mapping NED (N, E, D) to ENU (E, N, U). It was checked
 * against aircraft/m62v1/logs/mcu_experiment_air.csv: reconstructing the body velocities from the logged
 * ve/vn/vd and these attitudes reproduces the log's own aoa_deg and sideslip_deg.
 */

/** The airframe model's world orientation from JSBSim roll/pitch/yaw (degrees), as the SIM log records them. */
export function aircraftQuaternion(rollDeg: number, pitchDeg: number, yawDeg: number): THREE.Quaternion {
  const roll = THREE.MathUtils.degToRad(rollDeg)
  const pitch = THREE.MathUtils.degToRad(pitchDeg)
  const yaw = THREE.MathUtils.degToRad(yawDeg)
  const sr = Math.sin(roll)
  const cr = Math.cos(roll)
  const sp = Math.sin(pitch)
  const cp = Math.cos(pitch)
  const sy = Math.sin(yaw)
  const cy = Math.cos(yaw)
  const matrix = new THREE.Matrix4().set(
    -sy * cp, sy * sp * sr + cy * cr, -sy * sp * cr + cy * sr, 0,
    -cy * cp, cy * sp * sr - sy * cr, -cy * sp * cr - sy * sr, 0,
    -sp, -cp * sr, cp * cr, 0,
    0, 0, 0, 1,
  )
  return new THREE.Quaternion().setFromRotationMatrix(matrix)
}

/**
 * The command path's own servo-to-deflection mapping (libs/simrun.py plot_pitch_time): 1500 us is neutral, and
 * min/max degrees scale the two sides. A missing or non-finite output reads as neutral.
 */
export function controlDeflectionDeg(servoUs: number, minDeg: number, maxDeg: number): number {
  if (!Number.isFinite(servoUs)) return 0
  const norm = Math.min(1, Math.max(-1, (servoUs - 1500) / 500))
  return norm > 0 ? norm * maxDeg : -norm * minDeg
}

import { type PerspectiveCamera, Plane, Raycaster, Vector2, Vector3 } from "three";

/** Pick the nearest drawn square sprite, otherwise intersect the z=0 floor.
 * Viewport dimensions and the hardware point-size range are in device pixels.
 * Positions are world coordinates, as in the scene's untransformed Points. */
export function pickVoxelGoal(
  camera: PerspectiveCamera,
  positions: Float32Array,
  pointer: Vector2,
  size: number,
  viewport: Vector2,
  pointSizeRange: Float32Array,
): [number, number] | null {
  camera.updateMatrixWorld();
  const projected = new Vector3();
  let nearest = Infinity;
  let hit = -1;
  for (let i = 0; i < positions.length; i += 3) {
    projected.fromArray(positions, i).applyMatrix4(camera.matrixWorldInverse);
    const depth = -projected.z;
    if (depth < camera.near || depth > camera.far || depth > nearest) continue;
    projected.applyMatrix4(camera.projectionMatrix);
    // WebGL clips point primitives by their centres, before expanding sprites.
    if (Math.abs(projected.x) > 1 || Math.abs(projected.y) > 1) continue;
    // Match PointsMaterial's size attenuation and the GPU's point-size clamp.
    const pixels = Math.max(
      pointSizeRange[0],
      Math.min(pointSizeRange[1], size * viewport.y / (2 * depth)),
    );
    if (
      Math.abs(projected.x - pointer.x) <= pixels / viewport.x &&
      Math.abs(projected.y - pointer.y) <= pixels / viewport.y
    ) {
      nearest = depth;
      hit = i;
    }
  }
  if (hit !== -1) return [positions[hit], positions[hit + 1]];

  const raycaster = new Raycaster();
  raycaster.setFromCamera(pointer, camera);
  const ground = raycaster.ray.intersectPlane(new Plane(new Vector3(0, 0, 1), 0), projected);
  return ground === null ? null : [ground.x, ground.y];
}

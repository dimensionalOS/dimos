import * as THREE from "three";
import { mergeGeometries } from "three/addons/utils/BufferGeometryUtils.js";

/** Merge fixed geometry with identical materials; scoreboard lamps stay addressable. */
export function batchStaticMeshes(group: THREE.Group): THREE.BufferGeometry[] {
  const batches = new Map<string, THREE.Mesh<THREE.BufferGeometry, THREE.MeshStandardMaterial>[]>();
  for (const child of group.children) {
    if (
      !(child instanceof THREE.Mesh) || !(child.material instanceof THREE.MeshStandardMaterial) ||
      child.name.startsWith("score_") || child.material.transparent
    ) continue;
    const material = child.material;
    const key = [
      material.color.getHex(),
      material.roughness,
      material.metalness,
      material.opacity,
      material.map?.uuid ?? "",
      child.castShadow,
    ].join(":");
    const batch = batches.get(key) ?? [];
    batch.push(child);
    batches.set(key, batch);
  }
  const geometries: THREE.BufferGeometry[] = [];
  for (const meshes of batches.values()) {
    if (meshes.length < 2) continue;
    const parts = meshes.map((mesh) => {
      mesh.updateMatrix();
      return mesh.geometry.clone().applyMatrix4(mesh.matrix);
    });
    const merged = mergeGeometries(parts);
    for (const part of parts) part.dispose();
    if (!merged) throw new Error("Static room geometry could not be batched.");
    const result = new THREE.Mesh(merged, meshes[0].material);
    result.castShadow = meshes[0].castShadow;
    result.receiveShadow = true;
    group.add(result);
    for (const mesh of meshes) group.remove(mesh);
    geometries.push(merged);
  }
  return geometries;
}

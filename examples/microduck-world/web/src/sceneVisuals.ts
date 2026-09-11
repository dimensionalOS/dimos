import * as THREE from "three";
import type { VisualGeom, WorldDefinition } from "./worldModel.ts";
import { unpack } from "./worldModel.ts";

export function readMeshes(model: WorldDefinition): Map<string, THREE.BufferGeometry> {
  return new Map(
    Object.entries(model.meshes).map(([id, mesh]) => {
      const buffer = new THREE.BufferGeometry();
      buffer.setAttribute(
        "position",
        new THREE.BufferAttribute(new Float32Array(unpack(mesh.positions)), 3),
      );
      buffer.setIndex(new THREE.BufferAttribute(new Uint32Array(unpack(mesh.indices)), 1));
      buffer.computeVertexNormals();
      buffer.computeBoundingSphere();
      return [id, buffer];
    }),
  );
}

export function geometry(
  geom: VisualGeom,
  meshes: Map<string, THREE.BufferGeometry>,
): THREE.BufferGeometry {
  const [x, y, z] = geom.size;
  switch (geom.kind) {
    case "mesh":
      return meshes.get(geom.mesh!)!;
    case "box":
      return new THREE.BoxGeometry(x * 2, y * 2, z * 2);
    case "plane":
      return new THREE.PlaneGeometry(Math.max(24, x * 2), Math.max(24, y * 2));
    case "sphere":
      return new THREE.SphereGeometry(x, 24, 16);
    case "ellipsoid":
      return new THREE.SphereGeometry(1, 24, 16).scale(x, y, z);
    case "cylinder":
      return new THREE.CylinderGeometry(x, x, y * 2, 24).rotateX(Math.PI / 2);
    case "capsule":
      return new THREE.CapsuleGeometry(x, y * 2, 6, 16).rotateX(Math.PI / 2);
  }
}

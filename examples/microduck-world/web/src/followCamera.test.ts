import * as THREE from "three";
import { expect, it } from "vitest";
import { DuckFollowCamera } from "./followCamera.ts";

function rig() {
  const follow = new DuckFollowCamera();
  const camera = new THREE.PerspectiveCamera();
  camera.up.set(0, 0, 1);
  const target = new THREE.Vector3();
  return { follow, camera, target };
}
it("keeps a close fixed offset while translating and turning with the duck", () => {
  const { follow, camera, target } = rig();
  const position = new THREE.Vector3(2, 3, .12);
  const rotation = new THREE.Quaternion();
  follow.update(camera, target, position, rotation, []);
  const firstOffset = camera.position.clone().sub(target);
  expect(firstOffset.length()).toBeLessThan(.85);
  expect(firstOffset.x).toBeLessThan(0);
  position.add(new THREE.Vector3(1, 2, 0));
  follow.update(camera, target, position, rotation, []);
  expect(camera.position.clone().sub(target).distanceTo(firstOffset)).toBeLessThan(1e-10);
  rotation.setFromAxisAngle(new THREE.Vector3(0, 0, 1), Math.PI / 2);
  follow.update(camera, target, position, rotation, []);
  const turned = firstOffset.clone().applyQuaternion(rotation);
  expect(camera.position.clone().sub(target).distanceTo(turned)).toBeLessThan(1e-10);
  expect(
    camera.getWorldDirection(new THREE.Vector3()).dot(
      target.clone().sub(camera.position).normalize(),
    ),
  ).toBeCloseTo(1);
});
it("does not copy body roll and pitch into the camera", () => {
  const { follow, camera, target } = rig();
  const position = new THREE.Vector3();
  const yaw = .6;
  follow.update(
    camera,
    target,
    position,
    new THREE.Quaternion().setFromEuler(new THREE.Euler(0, 0, yaw, "ZYX")),
    [],
  );
  const original = camera.position.clone();
  follow.update(
    camera,
    target,
    position,
    new THREE.Quaternion().setFromEuler(new THREE.Euler(.3, -.2, yaw, "ZYX")),
    [],
  );
  expect(camera.position.distanceTo(original)).toBeLessThan(1e-10);
});
it("stays on the duck's side of an intervening wall and recovers in clear space", () => {
  const { follow, camera, target } = rig();
  const wall = new THREE.Mesh(new THREE.BoxGeometry(.05, 2, 2), new THREE.MeshBasicMaterial());
  wall.position.set(-.35, 0, .5);
  const position = new THREE.Vector3(0, 0, .12);
  const rotation = new THREE.Quaternion();
  follow.update(camera, target, position, rotation, [wall]);
  expect(camera.position.x).toBeGreaterThan(-.325);
  expect(camera.position.x).toBeLessThan(0);
  const closeDistance = camera.position.distanceTo(target);
  follow.update(camera, target, position, rotation, []);
  expect(camera.position.distanceTo(target)).toBeGreaterThan(closeDistance);
  wall.geometry.dispose();
  wall.material.dispose();
});

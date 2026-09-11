import * as THREE from "three";

/** A fixed shoulder view using heading only, without the walking body's roll/pitch. */
export class DuckFollowCamera {
  private readonly forward = new THREE.Vector3();
  private readonly offset = new THREE.Vector3();
  private readonly ray = new THREE.Raycaster();
  private readonly hits: THREE.Intersection[] = [];

  update(
    camera: THREE.PerspectiveCamera,
    target: THREE.Vector3,
    position: THREE.Vector3,
    rotation: THREE.Quaternion,
    obstacles: THREE.Object3D[],
  ): void {
    target.copy(position);
    target.z += .11;
    this.forward.set(1, 0, 0).applyQuaternion(rotation);
    const yaw = Math.atan2(this.forward.y, this.forward.x);
    const c = Math.cos(yaw), s = Math.sin(yaw);
    // Behind and slightly to the right of the duck, less than a metre away.
    this.offset.set(-.62 * c + .22 * s, -.62 * s - .22 * c, .45);
    const distance = this.offset.length();
    this.offset.divideScalar(distance);
    this.ray.set(target, this.offset);
    this.ray.far = distance;
    this.hits.length = 0;
    for (const object of obstacles) object.updateWorldMatrix(true, true);
    this.ray.intersectObjects(obstacles, true, this.hits);
    const clearDistance = this.hits.length ? Math.max(.06, this.hits[0].distance - .04) : distance;
    camera.position.copy(target).addScaledVector(this.offset, clearDistance);
    camera.lookAt(target);
  }
}

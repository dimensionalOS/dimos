// Copyright 2026 Dimensional Inc. Licensed under the Apache License, Version 2.0.
import * as THREE from 'three';
const opticalToEye=new THREE.Matrix4().makeRotationX(Math.PI);
const unit=new THREE.Vector3(1,1,1);
export function eyeMatrix(pose) {
  return new THREE.Matrix4().compose(new THREE.Vector3().fromArray(pose.position),
    new THREE.Quaternion().fromArray(pose.orientation),unit);
}
export class RobotEyesView {
  constructor() { this.anchor=null; }
  enter(viewer) {
    const q=new THREE.Quaternion().fromArray(viewer.orientation);
    const forward=new THREE.Vector3(0,0,-1).applyQuaternion(q);
    const yaw=Math.atan2(-forward.x,-forward.z);
    this.anchor=new THREE.Matrix4().compose(new THREE.Vector3().fromArray(viewer.position),
      new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0,1,0),yaw),unit);
  }
  placement(robotEye) {
    return this.anchor.clone().multiply(opticalToEye).multiply(eyeMatrix(robotEye).invert());
  }
  desktopCamera(stage,robotEye) { return stage.clone().multiply(eyeMatrix(robotEye)).multiply(opticalToEye); }
}

// Copyright 2026 Dimensional Inc. Licensed under the Apache License, Version 2.0.
import * as THREE from "three";
import { TransformControls } from "three/addons/controls/TransformControls.js";

export class SpatialOperator {
  constructor(
    { scene, stage, camera, renderer, orbit, arms, endpoint, onTarget },
  ) {
    this.endpoint = endpoint;
    this.arms = arms;
    this.stage = stage;
    this.revision = 0;
    this.orientation = [0, 0, 0, 1];
    this.target = new THREE.Group();
    this.target.visible = false;
    stage.add(this.target);
    this.axes = new THREE.AxesHelper(.09);
    this.target.add(this.axes);
    this.controls = new TransformControls(camera, renderer.domElement);
    this.controls.setSpace("local");
    this.controls.setSize(.7);
    this.controls.setMode("translate");
    scene.add(this.controls.getHelper());
    this.controls.addEventListener("dragging-changed", (event) => {
      orbit.enabled = !event.value;
    });
    this.controls.addEventListener(
      "objectChange",
      () =>
        onTarget({
          position: this.target.position.toArray(),
          orientation: this.orientation,
          frame: "world",
        }),
    );
    this.ghosts = {};
    for (const [side, arm] of Object.entries(arms)) {
      const ghost = new THREE.Group();
      stage.add(ghost);
      ghost.visible = false;
      this.ghosts[side] = ghost;
      for (const mesh of arm.children) {
        const copy = new THREE.Mesh(
          mesh.geometry,
          new THREE.MeshBasicMaterial({
            color: 0x8fe6d0,
            transparent: true,
            opacity: .28,
            depthWrite: false,
          }),
        );
        copy.matrixAutoUpdate = false;
        ghost.add(copy);
      }
    }
  }
  setTarget({ pose, editable }) {
    if (!pose) {
      this.controls.detach();
      this.target.visible = false;
      return;
    }
    this.target.visible = true;
    this.orientation = [...pose.orientation];
    this.target.position.fromArray(pose.position);
    this.axes.quaternion.fromArray(pose.orientation);
    if (editable) this.controls.attach(this.target);
    else this.controls.detach();
  }
  async preview(joints) {
    const revision = ++this.revision;
    this.abort?.abort();
    this.abort = new AbortController();
    if (!joints) {
      for (const ghost of Object.values(this.ghosts)) ghost.visible = false;
      return;
    }
    try {
      const response = await fetch(this.endpoint("preview"), {
        method: "POST",
        headers: { "content-type": "application/json" },
        body: JSON.stringify({ joints }),
        signal: this.abort.signal,
      });
      if (!response.ok) throw new Error(`Preview HTTP ${response.status}`);
      const result = await response.json();
      if (revision !== this.revision) return;
      for (const [side, value] of Object.entries(result)) {
        const ghost = this.ghosts[side];
        ghost.visible = true;
        value.transforms.forEach((matrix, i) =>
          ghost.children[i].matrix.fromArray(matrix)
        );
      }
    } catch (error) {
      if (error.name !== "AbortError") {
        parent.postMessage({
          type: "dimos.spatial.error",
          value: error.message,
        }, location.origin);
      }
    }
  }
  dispose() {
    this.abort?.abort();
    this.controls.dispose();
    for (const ghost of Object.values(this.ghosts)) {
      for (const mesh of ghost.children) {
        mesh.material.dispose();
      }
    }
  }
}

// Copyright 2026 Dimensional Inc. Licensed under the Apache License, Version 2.0.
import * as THREE from 'three';

// Placement only. The caller owns content, hit tests and button/gesture routing.
export class XRPanelLayout {
  constructor(workspace, scene) {
    this.workspace = workspace; this.scene = scene;
    this.head = new THREE.Group(); scene.add(this.head);
    this.panels = new Map(); this.drags = new Map();
  }

  add(id, mesh) { this.panels.set(id, {mesh, pinned: false}); }
  pinned(id) { return this.panels.get(id).pinned; }

  updateHead(position, orientation) {
    this.head.position.fromArray(position); this.head.quaternion.fromArray(orientation);
    this.head.updateMatrixWorld(true);
  }

  togglePin(id) {
    const panel = this.panels.get(id);
    if ([...this.drags.values()].some(drag => drag.id === id)) return;
    panel.pinned = !panel.pinned;
    // attach preserves world pose and apparent size, even in a scaled workspace.
    (panel.pinned ? this.head : this.workspace).attach(panel.mesh);
  }

  begin(id, owner, pose) {
    if (this.drags.has(owner) || [...this.drags.values()].some(drag => drag.id === id)) return false;
    const panel = this.panels.get(id);
    this.scene.attach(panel.mesh); panel.pinned = false;
    panel.mesh.updateWorldMatrix(true, false);
    this.drags.set(owner, {id, offset: pose.clone().invert().multiply(panel.mesh.matrixWorld)});
    return true;
  }

  move(owner, pose) {
    const drag = this.drags.get(owner);
    if (!drag) return;
    const matrix = pose.clone().multiply(drag.offset);
    const mesh = this.panels.get(drag.id).mesh;
    this.scene.updateWorldMatrix(true, false);
    matrix.premultiply(this.scene.matrixWorld.clone().invert());
    matrix.decompose(mesh.position, mesh.quaternion, mesh.scale);
    mesh.updateMatrixWorld(true);
  }

  end(owner) {
    const drag = this.drags.get(owner);
    if (!drag) return;
    this.workspace.attach(this.panels.get(drag.id).mesh);
    this.drags.delete(owner);
  }

  cancel() { for (const owner of [...this.drags.keys()]) this.end(owner); }
  dispose() { this.cancel(); this.head.removeFromParent(); }
}

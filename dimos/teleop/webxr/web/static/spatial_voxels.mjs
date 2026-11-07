// Copyright 2026 Dimensional Inc. Licensed under the Apache License, Version 2.0.
import * as THREE from 'three';

// The mapper publishes occupied voxel centres. Draw cubes in one instanced
// batch, reusing GPU storage between map updates (no object per point).
export class VoxelMap extends THREE.Group {
  constructor() {
    super();
    this.geometry=new THREE.BoxGeometry(1,1,1);
    this.material=new THREE.MeshLambertMaterial({color:0x77939b});
    this.mesh=null; this.capacity=0; this.matrix=new THREE.Matrix4();
  }

  update(points, size) {
    if(!Number.isFinite(size)||size<=0) throw new Error('Invalid voxel size');
    const count=Math.floor(points.length/3);
    if(count>this.capacity) {
      this.mesh?.removeFromParent(); this.mesh?.dispose();
      this.capacity=Math.max(count,this.capacity*2,1024);
      this.mesh=new THREE.InstancedMesh(this.geometry,this.material,this.capacity);
      this.mesh.instanceMatrix.setUsage(THREE.DynamicDrawUsage); this.add(this.mesh);
    }
    if(!this.mesh) return;
    this.matrix.makeScale(size,size,size);
    let valid=0;
    for(let i=0;i<count;i++) {
      const x=points[i*3],y=points[i*3+1],z=points[i*3+2];
      if(!Number.isFinite(x)||!Number.isFinite(y)||!Number.isFinite(z)) continue;
      this.matrix.setPosition(x,y,z); this.mesh.setMatrixAt(valid++,this.matrix);
    }
    this.mesh.count=valid; this.mesh.instanceMatrix.needsUpdate=true;
    this.mesh.computeBoundingBox(); this.mesh.computeBoundingSphere();
  }

  dispose() { this.mesh?.dispose(); this.geometry.dispose(); this.material.dispose(); }
}

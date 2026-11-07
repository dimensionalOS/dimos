// Copyright 2026 Dimensional Inc. Licensed under the Apache License, Version 2.0.
import {test} from 'node:test';
import assert from 'node:assert/strict';
import {SurfaceGrid,decodeSurface} from './spatial_surface.mjs';
const header={width:4,height:2,step:1,image_width:4,image_height:2,
  K:[200,0,1,0,200,0,0,0,1],max_edge_m:.035};
test('surfaces preserve depth discontinuities and reuse buffers across frames',()=>{
  const grid=new SurfaceGrid(4,2), positions=grid.positions, indices=grid.indices;
  grid.update(header,new Uint16Array([1000,1000,2000,2000,1000,1000,2000,2000]));
  assert.equal(grid.count,12);
  for(let i=0;i<grid.count;i+=3) {
    const triangle=[...grid.indices.slice(i,i+3)];
    assert.ok(triangle.every(v=>v%4<2) || triangle.every(v=>v%4>=2));
  }
  grid.update(header,new Uint16Array(8).fill(1000));
  assert.equal(grid.count,18); assert.equal(grid.positions,positions); assert.equal(grid.indices,indices);
});
test('missing depth leaves a hole; valid neighbours remain',()=>{
  const grid=new SurfaceGrid(2,2);
  grid.update({...header,width:2},new Uint16Array([0,1000,1000,1000]));
  assert.equal(grid.count,3); assert.equal(grid.valid,3);
  assert.deepEqual([...grid.indices.slice(0,3)],[1,2,3]);
  grid.update({...header,width:2},new Uint16Array(4)); assert.equal(grid.count,0);
});
test('malformed or oversized observations are rejected before allocation',()=>{
  assert.throws(()=>decodeSurface(new ArrayBuffer(2)),/Incomplete/);
  const bytes=new ArrayBuffer(8); new DataView(bytes).setUint32(0,20000,true);
  assert.throws(()=>decodeSurface(bytes),/header/);
});

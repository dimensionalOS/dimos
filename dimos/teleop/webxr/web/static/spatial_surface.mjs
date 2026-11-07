// Copyright 2026 Dimensional Inc. Licensed under the Apache License, Version 2.0.

export function decodeSurface(buffer) {
  if (buffer.byteLength < 4) throw new Error('Incomplete RGB-D frame');
  const bytes = new DataView(buffer), length = bytes.getUint32(0, true);
  if (length > 16384 || 4+length > buffer.byteLength) throw new Error('Invalid RGB-D header');
  const header = JSON.parse(new TextDecoder().decode(new Uint8Array(buffer, 4, length)));
  const n = header.width * header.height;
  if (header.version !== 1 || !Number.isInteger(header.width) || !Number.isInteger(header.height)
      || n < 4 || n > 200000
      || header.width < 2 || header.height < 2 || length%2
      || 4+length+2*n >= buffer.byteLength) throw new Error('Invalid RGB-D dimensions');
  if (!Array.isArray(header.K) || header.K.length!==9 || !header.K.every(Number.isFinite)
      || header.K[0]<=0 || header.K[4]<=0 || !Number.isFinite(header.ts)
      || !Number.isInteger(header.step) || header.step<1
      || !Number.isFinite(header.max_edge_m) || header.max_edge_m<=0
      || !Array.isArray(header.world_from_optical) || header.world_from_optical.length!==16
      || !header.world_from_optical.every(Number.isFinite)) throw new Error('Invalid RGB-D calibration');
  const depth = new Uint16Array(n);
  for (let i = 0; i < n; i++) depth[i] = bytes.getUint16(4+length+2*i, true);
  return {header, depth, jpeg: buffer.slice(4+length+2*n)};
}

export class SurfaceGrid {
  constructor(width, height) {
    this.width = width; this.height = height;
    this.positions = new Float32Array(width*height*3);
    this.uv = new Float32Array(width*height*2);
    this.indices = new Uint32Array((width-1)*(height-1)*6);
    this.count = 0; this.valid = 0;
  }
  update(h, depth) {
    const {width:w, height:height} = this, p = this.positions;
    if (depth.length !== w*height) throw new Error('Depth size changed');
    this.valid = 0;
    for (let y=0, i=0; y<height; y++) for (let x=0; x<w; x++, i++) {
      const z = depth[i]*0.001;
      p[3*i] = (x*h.step-h.K[2])*z/h.K[0];
      p[3*i+1] = (y*h.step-h.K[5])*z/h.K[4]; p[3*i+2] = z;
      this.uv[2*i] = (x*h.step+0.5)/h.image_width;
      this.uv[2*i+1] = 1-(y*h.step+0.5)/h.image_height;
      if (z>0) this.valid++;
    }
    const max2 = h.max_edge_m ** 2;
    const edge = (a,b) => (p[3*a]-p[3*b])**2 + (p[3*a+1]-p[3*b+1])**2 + (p[3*a+2]-p[3*b+2])**2 <= max2;
    this.count = 0;
    const triangle = (a,b,c) => {
      if (!depth[a] || !depth[b] || !depth[c] || !edge(a,b) || !edge(b,c) || !edge(c,a)) return;
      this.indices[this.count++] = a; this.indices[this.count++] = b; this.indices[this.count++] = c;
    };
    for (let y=0; y<height-1; y++) for (let x=0; x<w-1; x++) {
      const a=y*w+x, b=a+1, c=a+w, d=c+1;
      triangle(a,c,b); triangle(b,c,d);
    }
    return this;
  }
}

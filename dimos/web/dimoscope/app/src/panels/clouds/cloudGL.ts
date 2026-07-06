// Minimal dependency-free WebGL point-cloud renderer + orbit camera (no three.js — the repo keeps
// heavy deps out; raw gl.POINTS is plenty for a few thousand points). Renders an interleaved xyz
// Float32Array height-coloured (blue→amber) to match the 2D drawCloud. One instance per <canvas>;
// several instances share ONE OrbitCam so dragging any cell rotates them all together.

export type OrbitCam = {
  target: [number, number, number];
  azimuth: number; // radians, around world-up (z)
  elevation: number; // radians, above the xy plane
  distance: number; // metres from target
};

export function defaultCam(): OrbitCam {
  return { target: [0, 0, 0], azimuth: 0.9, elevation: 0.5, distance: 12 };
}

// --- tiny column-major mat4 -------------------------------------------------------------------
type Vec3 = [number, number, number];
const sub = (a: Vec3, b: Vec3): Vec3 => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const cross = (a: Vec3, b: Vec3): Vec3 => [
  a[1] * b[2] - a[2] * b[1],
  a[2] * b[0] - a[0] * b[2],
  a[0] * b[1] - a[1] * b[0],
];
const dot = (a: Vec3, b: Vec3) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const norm = (a: Vec3): Vec3 => {
  const l = Math.hypot(a[0], a[1], a[2]) || 1;
  return [a[0] / l, a[1] / l, a[2] / l];
};

function perspective(fovy: number, aspect: number, near: number, far: number): Float32Array {
  const f = 1 / Math.tan(fovy / 2);
  const nf = 1 / (near - far);
  return new Float32Array([
    f / aspect, 0, 0, 0,
    0, f, 0, 0,
    0, 0, (far + near) * nf, -1,
    0, 0, 2 * far * near * nf, 0,
  ]);
}

function lookAt(eye: Vec3, center: Vec3, up: Vec3): Float32Array {
  const z = norm(sub(eye, center));
  const x = norm(cross(up, z));
  const y = cross(z, x);
  return new Float32Array([
    x[0], y[0], z[0], 0,
    x[1], y[1], z[1], 0,
    x[2], y[2], z[2], 0,
    -dot(x, eye), -dot(y, eye), -dot(z, eye), 1,
  ]);
}

function multiply(a: Float32Array, b: Float32Array): Float32Array {
  const o = new Float32Array(16);
  for (let c = 0; c < 4; c++) {
    for (let r = 0; r < 4; r++) {
      o[c * 4 + r] = a[0 * 4 + r] * b[c * 4 + 0] +
        a[1 * 4 + r] * b[c * 4 + 1] +
        a[2 * 4 + r] * b[c * 4 + 2] +
        a[3 * 4 + r] * b[c * 4 + 3];
    }
  }
  return o;
}

/** eye position for an orbit cam (world is z-up). */
export function camEye(cam: OrbitCam): Vec3 {
  const ce = Math.cos(cam.elevation), se = Math.sin(cam.elevation);
  return [
    cam.target[0] + cam.distance * ce * Math.cos(cam.azimuth),
    cam.target[1] + cam.distance * ce * Math.sin(cam.azimuth),
    cam.target[2] + cam.distance * se,
  ];
}

const VERT = `
attribute vec3 aPos;
uniform mat4 uMVP;
uniform float uPointSize;
uniform vec2 uZ;
varying float vT;
void main() {
  gl_Position = uMVP * vec4(aPos, 1.0);
  gl_PointSize = uPointSize;
  vT = clamp((aPos.z - uZ.x) / max(0.001, uZ.y - uZ.x), 0.0, 1.0);
}`;

const FRAG = `
precision mediump float;
varying float vT;
void main() {
  vec2 d = gl_PointCoord - vec2(0.5);
  if (dot(d, d) > 0.25) discard;           // round dot
  vec3 lo = vec3(0.16, 0.42, 0.86);        // blue (low)
  vec3 hi = vec3(0.98, 0.75, 0.20);        // amber (high)
  gl_FragColor = vec4(mix(lo, hi, vT), 1.0);
}`;

export interface CloudGL {
  ok: boolean;
  setPoints(xyz: Float32Array | null, zRange: [number, number]): void;
  render(cam: OrbitCam): void;
  resize(): void;
  dispose(): void;
}

function compile(gl: WebGLRenderingContext, type: number, src: string): WebGLShader | null {
  const s = gl.createShader(type);
  if (!s) return null;
  gl.shaderSource(s, src);
  gl.compileShader(s);
  if (!gl.getShaderParameter(s, gl.COMPILE_STATUS)) {
    console.warn("[cloudGL] shader compile failed", gl.getShaderInfoLog(s));
    gl.deleteShader(s);
    return null;
  }
  return s;
}

/** Create a point renderer bound to `canvas`, or a no-op ({ok:false}) if WebGL is unavailable
 *  (the caller falls back to the 2D path). */
export function createCloudGL(canvas: HTMLCanvasElement): CloudGL {
  const noop: CloudGL = {
    ok: false,
    setPoints() {},
    render() {},
    resize() {},
    dispose() {},
  };
  const gl = (canvas.getContext("webgl", { antialias: true, alpha: false }) ??
    canvas.getContext("experimental-webgl")) as WebGLRenderingContext | null;
  if (!gl) return noop;

  const vs = compile(gl, gl.VERTEX_SHADER, VERT);
  const fs = compile(gl, gl.FRAGMENT_SHADER, FRAG);
  const prog = gl.createProgram();
  if (!vs || !fs || !prog) return noop;
  gl.attachShader(prog, vs);
  gl.attachShader(prog, fs);
  gl.linkProgram(prog);
  if (!gl.getProgramParameter(prog, gl.LINK_STATUS)) {
    console.warn("[cloudGL] link failed", gl.getProgramInfoLog(prog));
    return noop;
  }
  const aPos = gl.getAttribLocation(prog, "aPos");
  const uMVP = gl.getUniformLocation(prog, "uMVP");
  const uPointSize = gl.getUniformLocation(prog, "uPointSize");
  const uZ = gl.getUniformLocation(prog, "uZ");
  const buf = gl.createBuffer();
  let count = 0;
  let zRange: [number, number] = [0, 1];

  gl.clearColor(0x0b / 255, 0x0e / 255, 0x14 / 255, 1);
  gl.enable(gl.DEPTH_TEST);

  const resize = () => {
    const dpr = window.devicePixelRatio || 1;
    const w = Math.max(1, Math.round(canvas.clientWidth * dpr));
    const h = Math.max(1, Math.round(canvas.clientHeight * dpr));
    if (canvas.width !== w || canvas.height !== h) {
      canvas.width = w;
      canvas.height = h;
    }
    gl.viewport(0, 0, canvas.width, canvas.height);
  };

  return {
    ok: true,
    setPoints(xyz, zr) {
      if (!xyz || xyz.length === 0) {
        count = 0;
        return;
      }
      gl.bindBuffer(gl.ARRAY_BUFFER, buf);
      gl.bufferData(gl.ARRAY_BUFFER, xyz, gl.DYNAMIC_DRAW);
      count = xyz.length / 3;
      zRange = zr;
    },
    render(cam) {
      resize();
      gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
      if (count === 0) return;
      const aspect = canvas.width / Math.max(1, canvas.height);
      const proj = perspective((50 * Math.PI) / 180, aspect, 0.05, cam.distance * 20 + 100);
      const view = lookAt(camEye(cam), cam.target, [0, 0, 1]);
      const mvp = multiply(proj, view);
      gl.useProgram(prog);
      gl.uniformMatrix4fv(uMVP, false, mvp);
      gl.uniform1f(uPointSize, 2.5 * (window.devicePixelRatio || 1));
      gl.uniform2f(uZ, zRange[0], zRange[1]);
      gl.bindBuffer(gl.ARRAY_BUFFER, buf);
      gl.enableVertexAttribArray(aPos);
      gl.vertexAttribPointer(aPos, 3, gl.FLOAT, false, 0, 0);
      gl.drawArrays(gl.POINTS, 0, count);
    },
    resize,
    dispose() {
      gl.deleteBuffer(buf);
      gl.deleteProgram(prog);
      gl.deleteShader(vs);
      gl.deleteShader(fs);
    },
  };
}

/** Centroid + a fit distance from an interleaved xyz buffer, for auto-framing the orbit cam. */
export function cloudExtent(
  xyz: Float32Array,
): { center: Vec3; radius: number; zMin: number; zMax: number } {
  let minX = Infinity, minY = Infinity, minZ = Infinity, maxX = -Infinity, maxY = -Infinity, maxZ = -Infinity;
  for (let i = 0; i < xyz.length; i += 3) {
    const x = xyz[i], y = xyz[i + 1], z = xyz[i + 2];
    if (x < minX) minX = x;
    if (x > maxX) maxX = x;
    if (y < minY) minY = y;
    if (y > maxY) maxY = y;
    if (z < minZ) minZ = z;
    if (z > maxZ) maxZ = z;
  }
  if (minX === Infinity) return { center: [0, 0, 0], radius: 5, zMin: 0, zMax: 1 };
  const center: Vec3 = [(minX + maxX) / 2, (minY + maxY) / 2, (minZ + maxZ) / 2];
  const radius = Math.max(0.5, Math.hypot(maxX - minX, maxY - minY, maxZ - minZ) / 2);
  return { center, radius, zMin: minZ, zMax: maxZ };
}

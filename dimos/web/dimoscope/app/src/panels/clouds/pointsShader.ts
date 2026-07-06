// Shared height-coloured point shader for the three.js cloud views (CloudCompare + WorldView 3D).
// World is z-up; z drives the blue→amber ramp over the uZ range. Round dots via gl_PointCoord discard,
// with a small size + mild depth cue. `uSize` is a per-material pixel scale (typically 2.6 * dpr).
export const POINTS_VERT = `
uniform vec2 uZ;
uniform float uSize;
varying float vT;
void main() {
  vT = clamp((position.z - uZ.x) / max(0.001, uZ.y - uZ.x), 0.0, 1.0);
  vec4 mv = modelViewMatrix * vec4(position, 1.0);
  gl_PointSize = clamp(uSize * (12.0 / max(0.5, -mv.z)), 1.5, uSize + 1.5);
  gl_Position = projectionMatrix * mv;
}`;

export const POINTS_FRAG = `
varying float vT;
void main() {
  vec2 d = gl_PointCoord - vec2(0.5);
  if (dot(d, d) > 0.25) discard;
  vec3 lo = vec3(0.16, 0.42, 0.86);   // blue (low)
  vec3 hi = vec3(0.98, 0.75, 0.20);   // amber (high)
  gl_FragColor = vec4(mix(lo, hi, vT), 1.0);
}`;

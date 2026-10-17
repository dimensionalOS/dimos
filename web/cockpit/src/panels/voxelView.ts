// Pure pieces of the map3d panel: the scene contract the sink drives, the
// height colour ramp, the camera fit and follow, the clip planes and the
// point size.
// Nothing here imports three.js, so it all unit-tests on node and the
// three.js half (voxelScene.ts) stays a lazy chunk.

export interface Pose3d {
  x: number;
  y: number;
  z: number;
  yaw: number;
}

export interface Bounds {
  min: [number, number, number];
  max: [number, number, number];
}

/** What the sink drives; voxelScene.ts implements it with three.js. */
export interface VoxelScene {
  /** Replace the drawn voxels: xyz triplets, sRGB bytes per voxel (the scene
   * may rewrite them in place), voxel edge in m. */
  setVoxels(positions: Float32Array, colors: Uint8Array, res: number): void;
  setPose(pose: Pose3d | null): void;
  /** Frame the current voxels. */
  fit(): void;
  /** Keep the camera behind the robot as it moves (on), or leave it be (off). */
  setFollow(on: boolean): void;
  resize(cssWidth: number, cssHeight: number, dpr: number): void;
  render(): void;
  dispose(): void;
}

export function boundsOf(positions: Float32Array): Bounds | null {
  if (positions.length < 3) return null;
  const min: [number, number, number] = [Infinity, Infinity, Infinity];
  const max: [number, number, number] = [-Infinity, -Infinity, -Infinity];
  for (let i = 0; i < positions.length; i += 3) {
    for (let axis = 0; axis < 3; axis++) {
      const v = positions[i + axis];
      if (v < min[axis]) min[axis] = v;
      if (v > max[axis]) max[axis] = v;
    }
  }
  return { min, max };
}

// Turbo, the colormap the Rerun bridge paints point clouds with
// (register_colormap_annotation in PointCloud2.py): purple at the floor,
// blue, green and yellow through the mid heights, dark red at the top.
// Every 16th entry of matplotlib's 256-entry table, interpolated linearly,
// stays within 7/255 of it.
export const HEIGHT_RAMP: [number, number, number][] = [
  [0x30, 0x12, 0x3b],
  [0x40, 0x40, 0xa1],
  [0x46, 0x6b, 0xe3],
  [0x41, 0x93, 0xfe],
  [0x28, 0xbb, 0xeb],
  [0x17, 0xdc, 0xc2],
  [0x32, 0xf1, 0x97],
  [0x6d, 0xfd, 0x62],
  [0xa4, 0xfc, 0x3b],
  [0xca, 0xed, 0x33],
  [0xec, 0xd1, 0x39],
  [0xfc, 0xae, 0x34],
  [0xfb, 0x80, 0x22],
  [0xec, 0x52, 0x0e],
  [0xd2, 0x30, 0x05],
  [0xac, 0x16, 0x01],
  [0x7a, 0x04, 0x02],
];

// The ramp spans the 2nd to 98th percentile of height, so a few stray voxels
// under the floor or above the ceiling do not wash out the floor and walls.
const RAMP_LOW = 0.02;
const RAMP_HIGH = 0.98;
const RAMP_BINS = 256;

/** [low, high] height for the ramp: percentile cuts from a histogram, no sort. */
export function heightRange(positions: Float32Array): [number, number] {
  let zMin = Infinity;
  let zMax = -Infinity;
  for (let i = 2; i < positions.length; i += 3) {
    const z = positions[i];
    if (z < zMin) zMin = z;
    if (z > zMax) zMax = z;
  }
  if (!(zMax > zMin)) return [zMin, zMax];
  const counts = new Uint32Array(RAMP_BINS);
  const scale = RAMP_BINS / (zMax - zMin);
  for (let i = 2; i < positions.length; i += 3) {
    counts[Math.min(RAMP_BINS - 1, Math.floor((positions[i] - zMin) * scale))]++;
  }
  const n = positions.length / 3;
  let low = zMin;
  let high = zMax;
  let seen = 0;
  for (let b = 0; b < RAMP_BINS; b++) {
    const before = seen;
    seen += counts[b];
    if (before < RAMP_LOW * n && seen >= RAMP_LOW * n) low = zMin + b / scale;
    if (before < RAMP_HIGH * n && seen >= RAMP_HIGH * n) {
      high = zMin + (b + 1) / scale;
      break;
    }
  }
  return [low, high];
}

/**
 * RGB bytes per voxel from its height within heightRange. A flat cloud is
 * all mid colour. Pass the previous frame's array back in to reuse it while
 * the voxel count holds.
 */
export function heightColors(positions: Float32Array, reuse?: Uint8Array): Uint8Array {
  const n = positions.length / 3;
  const colors = reuse !== undefined && reuse.length === n * 3 ? reuse : new Uint8Array(n * 3);
  const [low, high] = heightRange(positions);
  const span = high - low;
  const last = HEIGHT_RAMP.length - 1;
  for (let i = 0; i < n; i++) {
    const t = span > 0 ? Math.min(1, Math.max(0, (positions[i * 3 + 2] - low) / span)) : 0.5;
    const s = t * last;
    const k = Math.min(Math.floor(s), last - 1);
    const f = s - k;
    const a = HEIGHT_RAMP[k];
    const b = HEIGHT_RAMP[k + 1];
    colors[i * 3] = Math.round(a[0] + (b[0] - a[0]) * f);
    colors[i * 3 + 1] = Math.round(a[1] + (b[1] - a[1]) * f);
    colors[i * 3 + 2] = Math.round(a[2] + (b[2] - a[2]) * f);
  }
  return colors;
}

export interface View {
  target: [number, number, number];
  position: [number, number, number];
}

// Where the camera sits relative to the target: in front (+x), to the right
// (-y) and above (+z) in the z-up world, so a map is seen from a corner.
const VIEW_DIRECTION = [1, -1, 0.8];
// The fitted bounding sphere is never smaller than this, so a lone voxel is
// framed at a sensible distance.
const MIN_RADIUS = 0.5;
const FIT_MARGIN = 1.1;

/**
 * Camera placement that frames the bounds: looking at the box centre from
 * VIEW_DIRECTION, far enough that the bounding sphere fits the narrower of
 * the vertical and horizontal fields of view.
 */
export function fitView(bounds: Bounds, fovDeg: number, aspect: number): View {
  const target: [number, number, number] = [0, 0, 0];
  let radius = 0;
  for (let axis = 0; axis < 3; axis++) {
    target[axis] = (bounds.min[axis] + bounds.max[axis]) / 2;
    const half = (bounds.max[axis] - bounds.min[axis]) / 2;
    radius += half * half;
  }
  radius = Math.max(Math.sqrt(radius), MIN_RADIUS);
  const halfFov = (fovDeg * Math.PI) / 360;
  const fitFov = aspect >= 1 ? halfFov : Math.atan(Math.tan(halfFov) * aspect);
  const distance = (radius / Math.sin(fitFov)) * FIT_MARGIN;
  const norm = Math.hypot(...VIEW_DIRECTION);
  const position = VIEW_DIRECTION.map((d, axis) => target[axis] + (d / norm) * distance);
  return { target, position: position as [number, number, number] };
}

// The follow camera sits this far behind the robot along its heading and
// this far above it: further back than a game's over-the-shoulder view, so
// the room around the robot stays in the frame.
export const FOLLOW_BACK_M = 5;
export const FOLLOW_UP_M = 2.5;

/** The camera behind and above the robot, looking at it. */
export function followView(pose: Pose3d): View {
  return {
    target: [pose.x, pose.y, pose.z],
    position: [
      pose.x - FOLLOW_BACK_M * Math.cos(pose.yaw),
      pose.y - FOLLOW_BACK_M * Math.sin(pose.yaw),
      pose.z + FOLLOW_UP_M,
    ],
  };
}

/**
 * The view carried along with the robot from `from` to `to`: the target and
 * the camera keep their offsets from the robot in its own frame, so an orbit,
 * zoom or pan made while following holds as the robot drives and turns.
 */
export function carryView(view: View, from: Pose3d, to: Pose3d): View {
  const c = Math.cos(to.yaw - from.yaw);
  const s = Math.sin(to.yaw - from.yaw);
  const carry = (p: [number, number, number]): [number, number, number] => {
    const dx = p[0] - from.x;
    const dy = p[1] - from.y;
    return [to.x + dx * c - dy * s, to.y + dx * s + dy * c, to.z + p[2] - from.z];
  };
  return { target: carry(view.target), position: carry(view.position) };
}

export interface ClipPlanes {
  near: number;
  far: number;
}

// The near plane never comes closer than this, so a voxel orbited from
// centimetres is still drawn.
export const NEAR_MIN = 0.05;
const FAR_MARGIN = 1.05;
// A 24-bit depth buffer resolves points fine at this near-to-far ratio.
const NEAR_FAR_RATIO = 1e4;

/**
 * Clip planes for a camera at `position` drawing `bounds`: the far plane
 * reaches past the box's farthest corner, so a map fitted from kilometres
 * away, zoomed out further or panned to stays inside the frustum, and the
 * near plane keeps a fixed ratio under it. The scene passes a box that
 * includes the floor grid, so it never collapses to a point.
 */
export function clipPlanes(bounds: Bounds, position: [number, number, number]): ClipPlanes {
  let d2 = 0;
  for (let axis = 0; axis < 3; axis++) {
    const d = Math.max(
      Math.abs(position[axis] - bounds.min[axis]),
      Math.abs(position[axis] - bounds.max[axis]),
    );
    d2 += d * d;
  }
  const far = Math.sqrt(d2) * FAR_MARGIN;
  return { near: Math.max(NEAR_MIN, far / NEAR_FAR_RATIO), far };
}

/** The box enclosing both. */
export function unionBounds(a: Bounds, b: Bounds): Bounds {
  const min = a.min.map((v, i) => Math.min(v, b.min[i])) as [number, number, number];
  const max = a.max.map((v, i) => Math.max(v, b.max[i])) as [number, number, number];
  return { min, max };
}

/**
 * PointsMaterial size that makes a point cover its voxel: three.js draws a
 * size-attenuated point at size * (viewportHeight / 2) / depth pixels, and a
 * voxel edge projects to res * (viewportHeight / 2) / (depth * tan(fov / 2)).
 */
export function pointSize(res: number, fovDeg: number): number {
  return res / Math.tan((fovDeg * Math.PI) / 360);
}

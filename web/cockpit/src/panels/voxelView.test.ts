import { PerspectiveCamera, Vector3 } from "three";
import { describe, expect, it } from "vitest";
import {
  type Bounds,
  boundsOf,
  carryView,
  clipPlanes,
  fitView,
  FOLLOW_BACK_M,
  FOLLOW_UP_M,
  followView,
  HEIGHT_RAMP,
  heightColors,
  heightRange,
  NEAR_MIN,
  pointSize,
  type Pose3d,
  unionBounds,
  type View,
} from "./voxelView.ts";

describe("boundsOf", () => {
  it("is null for no voxels and the per-axis extremes otherwise", () => {
    expect(boundsOf(new Float32Array(0))).toBeNull();
    const positions = Float32Array.from([1, 5, -2, -3, 4, 0, 2, 6, -1]);
    expect(boundsOf(positions)).toEqual({ min: [-3, 4, -2], max: [2, 6, 0] });
  });
});

describe("heightColors", () => {
  const rgb = (colors: Uint8Array, i: number) => [...colors.slice(i * 3, i * 3 + 3)];
  const TOP = HEIGHT_RAMP.length - 1;
  const MID = TOP / 2; // the ramp has an odd length, so mid height is a sample

  it("ramps from purple at the floor to red at the top over the z range", () => {
    const positions = Float32Array.from([0, 0, 0, 0, 0, 1, 0, 0, 2]);
    const colors = heightColors(positions);
    expect(rgb(colors, 0)).toEqual(HEIGHT_RAMP[0]);
    expect(rgb(colors, 1)).toEqual(HEIGHT_RAMP[MID]);
    expect(rgb(colors, 2)).toEqual(HEIGHT_RAMP[TOP]);
  });

  it("paints a flat cloud in the mid colour", () => {
    const colors = heightColors(Float32Array.from([0, 0, 0.5, 1, 1, 0.5]));
    expect(rgb(colors, 0)).toEqual(HEIGHT_RAMP[MID]);
    expect(rgb(colors, 1)).toEqual(HEIGHT_RAMP[MID]);
  });

  it("keeps stray voxels far below or above the map from washing out the ramp", () => {
    // 100 floor voxels at z=0, 100 wall-top voxels at z=1, one outlier each way.
    const zs = [-50, ...Array(100).fill(0), ...Array(100).fill(1), 50];
    const positions = new Float32Array(zs.length * 3);
    zs.forEach((z, i) => (positions[i * 3 + 2] = z));
    const [low, high] = heightRange(positions);
    expect(low).toBeCloseTo(0, 6);
    expect(high).toBeGreaterThan(1);
    expect(high).toBeLessThan(1.5);
    const colors = heightColors(positions);
    expect(rgb(colors, 1)).toEqual(HEIGHT_RAMP[0]); // the floor is purple
    // Walls land past mid, in the orange band: more red than the mid green.
    expect(rgb(colors, 101)[0]).toBeGreaterThan(HEIGHT_RAMP[MID][0]);
    expect(rgb(colors, 0)).toEqual(HEIGHT_RAMP[0]); // outliers clamp to the ends
    expect(rgb(colors, 201)).toEqual(HEIGHT_RAMP[TOP]);
  });

  it("reuses the previous array only while the voxel count holds", () => {
    const positions = Float32Array.from([0, 0, 0, 0, 0, 1]);
    const first = heightColors(positions);
    expect(heightColors(positions, first)).toBe(first);
    expect(heightColors(Float32Array.from([0, 0, 0]), first)).not.toBe(first);
  });
});

describe("fitView", () => {
  const bounds = {
    min: [-2, -4, 0] as [number, number, number],
    max: [2, 4, 2] as [number, number, number],
  };

  it("targets the box centre from in front, to the right and above", () => {
    const view = fitView(bounds, 50, 1.5);
    expect(view.target).toEqual([0, 0, 1]);
    expect(view.position[0]).toBeGreaterThan(0);
    expect(view.position[1]).toBeLessThan(0);
    expect(view.position[2]).toBeGreaterThan(1);
  });

  it("backs off far enough for the bounding sphere, further for a narrow panel", () => {
    const radius = Math.hypot(2, 4, 1);
    const wide = fitView(bounds, 50, 1.5);
    const distance = Math.hypot(...wide.position.map((p, i) => p - wide.target[i]));
    expect(distance).toBeCloseTo((radius / Math.sin((25 * Math.PI) / 180)) * 1.1, 6);
    const narrow = fitView(bounds, 50, 0.5);
    expect(Math.hypot(...narrow.position.map((p, i) => p - narrow.target[i]))).toBeGreaterThan(
      distance,
    );
  });

  it("frames a lone voxel from a sensible distance", () => {
    const one = {
      min: [1, 1, 1] as [number, number, number],
      max: [1, 1, 1] as [number, number, number],
    };
    const view = fitView(one, 50, 1);
    expect(Math.hypot(...view.position.map((p, i) => p - view.target[i]))).toBeGreaterThan(1);
  });
});

describe("follow", () => {
  const at = (x: number, y: number, yaw: number): Pose3d => ({ x, y, z: 0.3, yaw });
  const close = (a: number[], b: number[]) => a.forEach((v, i) => expect(v).toBeCloseTo(b[i], 6));

  it("puts the camera behind the robot along its heading and above it, looking at it", () => {
    const view = followView(at(1, 2, Math.PI / 2)); // facing +y
    expect(view.target).toEqual([1, 2, 0.3]);
    close(view.position, [1, 2 - FOLLOW_BACK_M, 0.3 + FOLLOW_UP_M]);
  });

  it("carries the default view along as the default view", () => {
    const from = at(0, 0, 0);
    const to = at(3, -1, 2.5);
    const carried = carryView(followView(from), from, to);
    close(carried.target, followView(to).target);
    close(carried.position, followView(to).position);
  });

  it("holds an orbit, zoom or pan relative to the robot as it turns", () => {
    // Facing +x, the viewer orbited to the robot's left, zoomed in and panned
    // the target a metre ahead of it.
    const view: View = { target: [1, 0, 0.3], position: [0, 2, 1.3] };
    const carried = carryView(view, at(0, 0, 0), at(10, 10, Math.PI / 2)); // a quarter turn left
    close(carried.target, [10, 11, 0.3]); // still a metre ahead
    close(carried.position, [8, 10, 1.3]); // still 2 m to its left
  });
});

describe("clipPlanes", () => {
  // The camera as voxelScene.ts sets it up: z-up, looking at the target.
  function cameraAt(view: View, near: number, far: number): PerspectiveCamera {
    const camera = new PerspectiveCamera(50, 1, near, far);
    camera.up.set(0, 0, 1);
    camera.position.set(...view.position);
    camera.lookAt(...view.target);
    camera.updateMatrixWorld();
    return camera;
  }
  const inFrustum = (camera: PerspectiveCamera, p: [number, number, number]): boolean => {
    const ndc = new Vector3(...p).project(camera);
    return Math.abs(ndc.x) <= 1 && Math.abs(ndc.y) <= 1 && Math.abs(ndc.z) <= 1;
  };
  const distance = (view: View) => Math.hypot(...view.position.map((p, i) => p - view.target[i]));
  // Voxels a kilometre apart: the fit backs the camera off ~2.6 km.
  const POINTS: [number, number, number][] = [[-1000, 0, 0], [0, 0, 0], [1000, 0, 0]];
  const FAR_BOUNDS = boundsOf(Float32Array.from(POINTS.flat()))!;

  it("keeps a map fitted from kilometres away inside the frustum", () => {
    const view = fitView(FAR_BOUNDS, 50, 1);
    expect(distance(view)).toBeGreaterThan(2000);
    const fixed = cameraAt(view, NEAR_MIN, 1000); // a fixed far plane clips it all
    expect(POINTS.some((p) => inFrustum(fixed, p))).toBe(false);
    const { near, far } = clipPlanes(FAR_BOUNDS, view.position);
    const fitted = cameraAt(view, near, far);
    for (const p of POINTS) expect(inFrustum(fitted, p)).toBe(true);
  });

  it("follows the camera as it zooms out and in", () => {
    const view = fitView(FAR_BOUNDS, 50, 1);
    const zoomed = (factor: number): View => ({
      target: view.target,
      position: view.position.map((p, i) => view.target[i] + (p - view.target[i]) * factor) as [
        number,
        number,
        number,
      ],
    });
    // Wheeled out 20x: the fit-time planes end long before the map ...
    const out = zoomed(20);
    const atFit = clipPlanes(FAR_BOUNDS, view.position);
    expect(POINTS.some((p) => inFrustum(cameraAt(out, atFit.near, atFit.far), p))).toBe(false);
    // ... the recomputed ones reach it.
    const outPlanes = clipPlanes(FAR_BOUNDS, out.position);
    for (const p of POINTS) {
      expect(inFrustum(cameraAt(out, outPlanes.near, outPlanes.far), p)).toBe(true);
    }
    // Wheeled in 20x: the target voxel is still between the planes.
    const close = zoomed(1 / 20);
    const closePlanes = clipPlanes(FAR_BOUNDS, close.position);
    expect(inFrustum(cameraAt(close, closePlanes.near, closePlanes.far), [0, 0, 0])).toBe(true);
  });

  it("floors the near plane for a small map and clears the farthest corner", () => {
    const bounds: Bounds = { min: [-2, -4, 0], max: [2, 4, 2] };
    const view = fitView(bounds, 50, 1.5);
    const { near, far } = clipPlanes(bounds, view.position);
    expect(near).toBe(NEAR_MIN);
    const corners = [bounds.min[0], bounds.max[0]].flatMap((x) =>
      [bounds.min[1], bounds.max[1]].flatMap((y) =>
        [bounds.min[2], bounds.max[2]].map((z) => [x, y, z])
      )
    );
    const farthest = Math.max(
      ...corners.map((c) => Math.hypot(...c.map((v, i) => v - view.position[i]))),
    );
    expect(far).toBeCloseTo(farthest * 1.05, 6);
  });

  it("unions boxes", () => {
    const a: Bounds = { min: [-1, 0, 5], max: [1, 2, 6] };
    const b: Bounds = { min: [0, -3, 0], max: [4, 1, 0] };
    expect(unionBounds(a, b)).toEqual({ min: [-1, -3, 0], max: [4, 2, 6] });
  });
});

describe("pointSize", () => {
  it("is the voxel edge over tan(fov / 2)", () => {
    expect(pointSize(0.05, 90)).toBeCloseTo(0.05, 6);
    expect(pointSize(0.1, 50)).toBeCloseTo(0.1 / Math.tan((25 * Math.PI) / 180), 6);
  });
});

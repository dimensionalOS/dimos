import { PerspectiveCamera, Vector2, Vector3 } from "three";
import { beforeEach, describe, expect, it } from "vitest";
import { pickVoxelGoal } from "./voxelPicking.ts";

describe("pickVoxelGoal", () => {
  let camera: PerspectiveCamera;
  const viewport = new Vector2(800, 400);
  const range = Float32Array.of(1, 256);
  const nearVoxel = [2, -2, 5];
  const farVoxel = [2, 0, 3];
  const pick = (positions: number[], x = 0, y = 0, size = 2) =>
    pickVoxelGoal(camera, Float32Array.from(positions), new Vector2(x, y), size, viewport, range);

  beforeEach(() => {
    camera = new PerspectiveCamera(90, 2, 0.1, 100);
    camera.up.set(0, 0, 1);
    camera.position.set(2, -7, 10);
    camera.lookAt(2, 3, 0);
  });

  it.each([
    [...farVoxel, ...nearVoxel],
    [...nearVoxel, ...farVoxel],
  ])("uses the nearest overlapping voxel's x/y regardless of storage order: %j", (...points) => {
    expect(pick(points)).toEqual([2, -2]);
  });

  it.each([1, 2])("picks sprite corners at pixel ratio %s", (dpr) => {
    // The nearer voxel is sqrt(50) m from the camera. Its square is ~57 px
    // wide at 800x400; the corner is outside an inscribed circular hit area.
    const depth = Math.sqrt(50);
    const pointer = new Vector2(0.9 / (depth * 2), 0.9 / depth);
    expect(pickVoxelGoal(
      camera,
      Float32Array.from(nearVoxel),
      pointer,
      2,
      viewport.clone().multiplyScalar(dpr),
      range,
    )).toEqual([2, -2]);
  });

  it("uses the ground intersection for a miss, including an empty cloud", () => {
    for (const points of [[], [30, 30, 5]]) {
      const goal = pick(points)!;
      expect(goal[0]).toBeCloseTo(2);
      expect(goal[1]).toBeCloseTo(3);
    }
    const goal = pick(nearVoxel, 0.2, -0.2)!;
    expect(goal[0]).toBeCloseTo(20 / (3 * Math.sqrt(2)) + 2);
    expect(goal[1]).toBeCloseTo(-1 / 3);
  });

  it("honours the hardware maximum point size", () => {
    // At this zoom the unclamped sprite would fill the panel, but the GPU
    // limits it to 16 pixels. A click 12 pixels from its centre misses it.
    const goal = pickVoxelGoal(
      camera,
      Float32Array.from(nearVoxel),
      new Vector2(0.03, 0),
      100,
      viewport,
      Float32Array.of(1, 16),
    )!;
    expect(goal[0]).toBeCloseTo(2 + 0.6 * Math.sqrt(2));
    expect(goal[1]).toBeCloseTo(3);
  });

  it("does not pick voxels outside the camera's clip planes or behind it", () => {
    camera.near = 8;
    camera.far = 12;
    camera.updateProjectionMatrix();
    // A giant sprite does not make a clipped point selectable.
    const goal = pick([...nearVoxel, 2, -8, 11, 2, 6, -3, 1000, 0, 3], 0, 0, 10000)!;
    expect(goal[0]).toBeCloseTo(2);
    expect(goal[1]).toBeCloseTo(3);
  });

  it.each([new Vector3(2, 3, 10), new Vector3(2, 3, 20)])(
    "sends no goal for rays parallel to or pointing away from the ground: %j",
    (target) => {
      camera.lookAt(target);
      expect(pick([])).toBeNull();
    },
  );
});

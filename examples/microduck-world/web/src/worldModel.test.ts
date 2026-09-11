import { describe, expect, it } from "vitest";
import { readDefinition, readSnapshot, unpack } from "./worldModel.ts";

const snapshot = {
  model: "/world-assets/scene-0123456789abcdef0123.json",
  t: 10,
  poses: [[1, 2, 3, 1, 0, 0, 0]],
};
const definition = {
  version: 1,
  up: "z",
  bodyIds: [0],
  focusBody: 0,
  geoms: [],
  meshes: {},
  appearance: {},
};

describe("world protocol", () => {
  it("accepts finite poses and same-origin versioned models only", () => {
    expect(readSnapshot(snapshot)).toEqual(snapshot);
    expect(readSnapshot({ ...snapshot, model: "https://foreign.test/model.json" })).toBeNull();
    expect(readSnapshot({ ...snapshot, poses: [[1, 2, 3, 1, 0, 0]] })).toBeNull();
    expect(readSnapshot({ ...snapshot, poses: [[NaN, 2, 3, 1, 0, 0, 0]] })).toBeNull();
  });
  it("rejects incompatible model versions and malformed camera settings", () => {
    expect(readDefinition(definition)).toEqual(definition);
    expect(() => readDefinition({ ...definition, version: 2 })).toThrow("unsupported");
    expect(() =>
      readDefinition({
        ...definition,
        appearance: { camera: { position: [1], target: [0, 0, 0] } },
      })
    ).toThrow("camera");
    expect(() => readDefinition({ ...definition, geoms: [null] })).toThrow("geometry");
  });
  it("preserves binary mesh data without changing byte order", () => {
    const bytes = new Uint8Array(new Float32Array([0.25, -2, 4]).buffer);
    const encoded = btoa(String.fromCharCode(...bytes));
    expect(Array.from(new Float32Array(unpack(encoded)))).toEqual([0.25, -2, 4]);
  });
});

it("rejects a camera with invalid clipping planes", () => {
  expect(() =>
    readDefinition({
      ...definition,
      camera: {
        body: 0,
        position: [0, 0, 0],
        quaternion: [1, 0, 0, 0],
        fovy: 70,
        near: 1,
        far: 0.5,
      },
    })
  ).toThrow("robot camera");
});

it("accepts authoritative scores and rejects malformed lamp updates", () => {
  const football = { scores: [12, 3], lit: [7, 9, 12] };
  expect(readSnapshot({ ...snapshot, football })?.football).toEqual(football);
  for (const scores of [[-1, 0], [1], [Infinity, 0], [0.5, 0]]) {
    expect(readSnapshot({ ...snapshot, football: { ...football, scores } })).toBeNull();
  }
  expect(readSnapshot({ ...snapshot, football: { ...football, lit: [-1] } })).toBeNull();
});

it("validates room camera views before using them", () => {
  const views = { football: { position: [4, 0, 5], target: [0, 4.3, 0] } };
  expect(readDefinition({ ...definition, appearance: { views } }).appearance.views).toEqual(views);
  expect(() => readDefinition({ ...definition, appearance: { views: { football: null } } }))
    .toThrow("room camera");
});

it("rejects invalid room extents and incomplete texture references", () => {
  for (const extent of [[1, 2], [1, -2, 3], [1, NaN, 3]]) {
    expect(() =>
      readDefinition({
        ...definition,
        appearance: {
          views: {
            football: { position: [1, 2, 3], target: [0, 0, 0], extent },
          },
        },
      })
    ).toThrow("room camera");
  }
  const geom = {
    id: 0,
    name: "field",
    body: 0,
    kind: "box",
    size: [1, 1, 1],
    position: [0, 0, 0],
    quaternion: [1, 0, 0, 0],
    rgba: [1, 1, 1, 1],
  };
  const textured = { ...geom, texture: { id: "0", repeat: [1, 1] } };
  expect(() => readDefinition({ ...definition, geoms: [textured] })).toThrow("geometry");
  expect(() =>
    readDefinition({
      ...definition,
      textures: { "0": "png" },
      geoms: [{ ...geom, texture: { id: "0" } }],
    })
  ).toThrow("geometry");
  expect(
    readDefinition({ ...definition, textures: { "0": "png" }, geoms: [textured] })
      .geoms[0].texture,
  ).toEqual(textured.texture);
});

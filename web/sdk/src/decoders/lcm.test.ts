import { describe, expect, it } from "vitest";
import type { FrameHeader } from "@dimos/shared";
import lcmFrames from "../../../shared/fixtures/lcm_frames.json";
import {
  compileLcmDecoder,
  isLcmSchema,
  LCM_PREVIEW_MAX_CHARS,
  lcmDecoderFor,
  LcmOversizedError,
  lcmPreview,
  type LcmSchema,
  type LcmValue,
  MAX_LCM_ARRAY_ELEMENTS,
} from "./lcm.ts";

const HEADER: FrameHeader = { ch: "p", seq: 1, ts: 0, delivery: "reliable" };

interface LcmVector {
  name: string;
  encoding: string;
  schema: LcmSchema;
  payload_b64: string;
  value: unknown;
}

// Python is the reference (dimos/web/relay_bridge/gen_lcm_fixtures.py):
// pytest re-encodes these, vitest decodes them.
const VECTORS = (lcmFrames as unknown as { vectors: LcmVector[] }).vectors;

function b64ToBytes(b64: string): Uint8Array {
  return Uint8Array.from(atob(b64), (c) => c.charCodeAt(0));
}

/** The fixture's JSON form of a decoded value: byte views as base64, other
 * typed arrays as plain arrays, bigints as decimal strings. */
function plain(v: unknown): unknown {
  if (v instanceof Uint8Array) return btoa(String.fromCharCode(...v));
  if (typeof v === "bigint") return v.toString();
  if (v instanceof BigInt64Array) return Array.from(v, (x) => x.toString());
  if (ArrayBuffer.isView(v)) return Array.from(v as unknown as ArrayLike<number>);
  if (Array.isArray(v)) return v.map(plain);
  if (typeof v === "object" && v !== null) {
    return Object.fromEntries(Object.entries(v).map(([k, x]) => [k, plain(x)]));
  }
  return v;
}

function vector(name: string): LcmVector {
  const vec = VECTORS.find((v) => v.name === name);
  if (vec === undefined) throw new Error(`no fixture vector ${name}`);
  return vec;
}

function decodeVector(name: string): { payload: Uint8Array; value: LcmValue } {
  const vec = vector(name);
  const payload = b64ToBytes(vec.payload_b64);
  return { payload, value: compileLcmDecoder(vec.schema)(payload) };
}

// A hand-built schema for the error cases: x, then n strings and n floats.
const POINT: LcmSchema = {
  type: "t.P",
  fp: "0011223344556677",
  structs: {
    "t.P": [["x", "double", null], ["n", "int32_t", null], ["xs", "float", ["n"]], [
      "tags",
      "string",
      ["n"],
    ]],
  },
};

/** A t.P frame: fingerprint, x = 1.5, n, two floats, two "a" strings. */
function pointFrame(fp = POINT.fp, n = 2, strLen = 2): Uint8Array {
  const bytes = new Uint8Array(8 + 8 + 4 + 8 + 12);
  for (let i = 0; i < 8; i++) bytes[i] = parseInt(fp.slice(i * 2, i * 2 + 2), 16);
  const view = new DataView(bytes.buffer, 8);
  view.setFloat64(0, 1.5, false);
  view.setInt32(8, n, false);
  view.setFloat32(12, 0.5, false);
  view.setFloat32(16, -0.25, false);
  for (const at of [20, 26]) {
    view.setInt32(at, strLen, false);
    bytes[8 + at + 4] = 0x61; // "a"
    bytes[8 + at + 5] = 0;
  }
  return bytes;
}

describe("lcm golden vectors", () => {
  for (const vec of VECTORS) {
    it(`decodes ${vec.name} (${vec.encoding})`, () => {
      const decode = compileLcmDecoder(vec.schema);
      expect(plain(decode(b64ToBytes(vec.payload_b64)))).toEqual(vec.value);
    });
  }
});

describe("lcm value shape", () => {
  it("keeps the LCM field names in wire order, count fields included", () => {
    expect(Object.keys(decodeVector("laser_scan").value)).toEqual([
      "ranges_length",
      "intensities_length",
      "header",
      "angle_min",
      "angle_max",
      "angle_increment",
      "time_increment",
      "scan_time",
      "range_min",
      "range_max",
      "ranges",
      "intensities",
    ]);
  });

  it("views byte[] and int8_t[] into the frame, copies other arrays as typed arrays", () => {
    const cloud = decodeVector("point_cloud");
    expect(cloud.value.data).toBeInstanceOf(Uint8Array);
    expect((cloud.value.data as Uint8Array).buffer).toBe(cloud.payload.buffer);
    expect(cloud.value.is_dense).toBe(true);
    expect(cloud.value.fields).toHaveLength(3);
    const grid = decodeVector("occupancy_grid");
    expect(grid.value.data).toBeInstanceOf(Int8Array);
    expect((grid.value.data as Int8Array).buffer).toBe(grid.payload.buffer);
    expect(Array.from(grid.value.data as Int8Array)).toEqual([0, 100, -1, 50, 0, 7]);
    const scan = decodeVector("laser_scan").value;
    expect(scan.ranges).toBeInstanceOf(Float32Array);
    expect(scan.intensities).toBeInstanceOf(Float32Array);
    expect((scan.intensities as Float32Array).length).toBe(0);
    expect(decodeVector("imu").value.orientation_covariance).toBeInstanceOf(Float64Array);
    expect(decodeVector("joint_state").value.name).toEqual(["shoulder", "elbow", "wrist_é"]);
  });

  it("decodes int64_t to a bigint, losslessly", () => {
    expect(decodeVector("int64").value.data).toBe(-9007199254740993n);
  });
});

describe("lcm decoder errors", () => {
  const decode = compileLcmDecoder(POINT);

  it("decodes the hand-built frame", () => {
    expect(plain(decode(pointFrame()))).toEqual({
      x: 1.5,
      n: 2,
      xs: [0.5, -0.25],
      tags: ["a", "a"],
    });
  });

  it("rejects the wrong fingerprint and a frame shorter than one", () => {
    expect(() => decode(pointFrame("0011223344556678"))).toThrow(/fingerprint mismatch for t\.P/);
    expect(() => decode(new Uint8Array(4))).toThrow(/shorter than the fingerprint/);
  });

  it("rejects truncated, overlong and malformed frames", () => {
    expect(() => decode(pointFrame().subarray(0, 24))).toThrow();
    expect(() => decode(pointFrame().subarray(0, 8 + 8 + 4 + 8 + 4))).toThrow(/bad string length/);
    const trailing = new Uint8Array(pointFrame().byteLength + 1);
    trailing.set(pointFrame());
    expect(() => decode(trailing)).toThrow(/trailing bytes: 1/);
    expect(() => decode(pointFrame(POINT.fp, 2, 0))).toThrow(/bad string length 0/);
  });

  it("rejects negative and oversized counts before allocating", () => {
    expect(() => decode(pointFrame(POINT.fp, -1))).toThrow(/bad array length -1/);
    expect(() => decode(pointFrame(POINT.fp, 1_000_000))).toThrow(/overruns the payload/);
  });

  it("refuses schemas it cannot serve at compile time", () => {
    const withRoot = (rows: unknown): LcmSchema =>
      ({ ...POINT, structs: { "t.P": rows } }) as LcmSchema;
    // A type that is neither a primitive nor a struct key reads as a missing struct.
    expect(() => compileLcmDecoder(withRoot([["x", "int128_t", null]]))).toThrow(
      /schema has no struct int128_t/,
    );
    expect(() => compileLcmDecoder(withRoot([["h", "t.H", null]]))).toThrow(
      /schema has no struct t\.H/,
    );
    expect(() => compileLcmDecoder(withRoot([["m", "double", [3, 3]]]))).toThrow(
      /multi-dimensional arrays are not supported \(t\.P\.m\)/,
    );
  });
});

describe("lcm schema gate and decoder factory", () => {
  it("accepts the fixture schemas and rejects the rest", () => {
    for (const vec of VECTORS) expect(isLcmSchema(vec.schema)).toBe(true);
    for (
      const bad of [
        null,
        undefined,
        "t.P",
        {},
        { type: "t.P", fp: "zz", structs: {} },
        { type: "t.P", fp: POINT.fp, structs: null },
        { type: 1, fp: POINT.fp, structs: {} },
      ]
    ) {
      expect(isLcmSchema(bad)).toBe(false);
    }
  });

  it("returns null for an unusable schema and a previewing decoder otherwise", () => {
    expect(lcmDecoderFor(undefined)).toBeNull();
    expect(lcmDecoderFor({ ...POINT, structs: {} })).toBeNull();
    const decoder = lcmDecoderFor(POINT);
    expect(decoder).not.toBeNull();
    const decoded = decoder!(pointFrame(), HEADER);
    expect(plain(decoded.value)).toEqual({ x: 1.5, n: 2, xs: [0.5, -0.25], tags: ["a", "a"] });
    expect(decoded.preview).toBe(
      '{x: 1.5, n: 2, xs: Float32Array(2)[0.5, -0.25], tags: ["a", "a"]}',
    );
  });
});

// Two struct arrays and a string array, each with its own count field.
const CLOUD: LcmSchema = {
  type: "t.C",
  fp: POINT.fp,
  structs: {
    "t.C": [
      ["n", "int32_t", null],
      ["m", "int32_t", null],
      ["k", "int32_t", null],
      ["a", "t.X", ["n"]],
      ["b", "t.X", ["m"]],
      ["tags", "string", ["k"]],
    ],
    "t.X": [["x", "float", null]],
  },
};

function cloudFrame(n: number, m: number, k: number): Uint8Array {
  const bytes = new Uint8Array(8 + 12 + 4 * (n + m) + 6 * k);
  for (let i = 0; i < 8; i++) bytes[i] = parseInt(CLOUD.fp.slice(i * 2, i * 2 + 2), 16);
  const view = new DataView(bytes.buffer, 8);
  view.setInt32(0, n, false);
  view.setInt32(4, m, false);
  view.setInt32(8, k, false);
  // Floats stay zero; every tag is "a".
  for (let i = 0, at = 12 + 4 * (n + m); i < k; i++, at += 6) {
    view.setInt32(at, 2, false);
    bytes[8 + at + 4] = 0x61;
  }
  return bytes;
}

describe("lcm element budget", () => {
  const decode = compileLcmDecoder(CLOUD);
  const MAX = MAX_LCM_ARRAY_ELEMENTS;

  it("decodes a frame at the budget", () => {
    expect((decode(cloudFrame(MAX, 0, 0)).a as unknown[]).length).toBe(MAX);
    expect((decode(cloudFrame(0, 0, MAX)).tags as string[]).length).toBe(MAX);
  });

  it("reports a frame over the budget as oversized instead of expanding it", () => {
    expect(() => decode(cloudFrame(MAX + 1, 0, 0))).toThrow(LcmOversizedError);
    expect(() => decode(cloudFrame(0, 0, MAX + 1))).toThrow(/over 100000 array elements \(tags\)/);
    // The budget is per frame: sibling arrays add up.
    expect(() => decode(cloudFrame(MAX / 2, MAX / 2 + 1, 0))).toThrow(/\(b\)/);
    const frame = cloudFrame(MAX + 1, 0, 0);
    expect(lcmDecoderFor(CLOUD)!(frame, HEADER)).toEqual({
      value: undefined,
      preview: `(oversized lcm message: over 100000 array elements (a), ${frame.byteLength} B)`,
    });
  });
});

describe("lcmPreview", () => {
  it("prints nested fields, typed arrays and bigints", () => {
    const value = {
      a: 1,
      s: "x",
      b: true,
      big: 5n,
      arr: new Float64Array([1.5, 2]),
      nested: { q: null },
    };
    expect(lcmPreview(value)).toBe(
      '{a: 1, s: "x", b: true, big: 5, arr: Float64Array(2)[1.5, 2], nested: {q: null}}',
    );
  });

  it("shows the head of long arrays and cuts long strings", () => {
    expect(lcmPreview({ r: new Float32Array(20) })).toBe(
      "{r: Float32Array(20)[0, 0, 0, 0, 0, 0, 0, 0, ...]}",
    );
    expect(lcmPreview(["y".repeat(100)])).toBe(`["${"y".repeat(64)}..."]`);
  });

  it("stays short on a huge array and stops at its budget on a wide message", () => {
    const poses = Array.from({ length: 100_000 }, (_, i) => ({ x: i, y: -i }));
    const long = lcmPreview({ poses_length: poses.length, poses });
    expect(long.startsWith("{poses_length: 100000, poses: [{x: 0, y: 0}, {x: 1, y: -1}, ")).toBe(
      true,
    );
    expect(long.endsWith(", ...]}")).toBe(true);
    expect(long.length).toBeLessThan(200);
    const wide = Object.fromEntries(
      Array.from({ length: 200 }, (_, i) => [`field_${i}`, "v".repeat(20)]),
    );
    const text = lcmPreview(wide);
    expect(text.endsWith(" ... (truncated)")).toBe(true);
    expect(text.length).toBeLessThanOrEqual(LCM_PREVIEW_MAX_CHARS + 20);
  });
});

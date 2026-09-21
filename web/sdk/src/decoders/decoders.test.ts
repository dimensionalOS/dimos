import { describe, expect, it } from "vitest";
import type { FrameHeader } from "@dimos/shared";
import costmapFrames from "../../../shared/fixtures/costmap_frames.json";
import lcmFrames from "../../../shared/fixtures/lcm_frames.json";
import voxelFrames from "../../../shared/fixtures/voxel_frames.json";
import { spec } from "../testing/fakeRelay.ts";
import {
  type CostmapValue,
  inflateCostmap,
  MAX_COSTMAP_DIM,
  MAX_COSTMAP_PAYLOAD_BYTES,
} from "./costmap.ts";
import { createDecoderRegistry, type Decoder } from "./index.ts";
import { MAX_JPEG_DIM, MAX_JPEG_PAYLOAD_BYTES } from "./jpeg.ts";
import { JSON_PREVIEW_MAX_CHARS, MAX_JSON_PAYLOAD_BYTES } from "./json.ts";
import {
  inflateVoxels,
  MAX_VOXEL_CHUNKS,
  MAX_VOXEL_PAYLOAD_BYTES,
  MAX_VOXELS,
  type VoxelsValue,
} from "./voxels.ts";

const HEADER: FrameHeader = { ch: "x", seq: 1, ts: 0, delivery: "latest" };

// Read-only lookups share one built-in registry; registration tests build
// their own.
const registry = createDecoderRegistry();

function b64ToBytes(b64: string): Uint8Array {
  return Uint8Array.from(atob(b64), (c) => c.charCodeAt(0));
}

async function deflate(bytes: Uint8Array): Promise<Uint8Array> {
  const stream = new Blob([bytes as BlobPart]).stream()
    .pipeThrough(new CompressionStream("deflate"));
  return new Uint8Array(await new Response(stream).arrayBuffer());
}

// The pose_stamped golden vector (Python-generated, see lcm.test.ts).
const POSE = (lcmFrames as { vectors: { name: string; schema: unknown; payload_b64: string }[] })
  .vectors[0];
const poseSpec = () =>
  spec({
    ch: "lcm_pose",
    encoding: "geometry_msgs.PoseStamped.lcm.v1",
    params: { lcm: POSE.schema },
  });

/** Minimal scannable JPEG: SOI + SOF0 declaring w x h (no scan data). */
function jpegBytes(w: number, h: number): Uint8Array {
  // SOI, then SOF0 (FF C0) with length 11: precision 8, height BE, width BE,
  // one component (id 1, sampling 0x11, quant table 0).
  const bytes = [0xff, 0xd8, 0xff, 0xc0, 0x00, 0x0b, 0x08];
  bytes.push((h >> 8) & 0xff, h & 0xff, (w >> 8) & 0xff, w & 0xff, 0x01, 0x01, 0x11, 0x00);
  return new Uint8Array(bytes);
}

describe("decoder registry", () => {
  it("resolves any *.json.vN encoding to the JSON decoder", () => {
    const decode = registry.get("pose.json.v1");
    expect(decode).toBeDefined();
    const payload = new TextEncoder().encode('{"x": 1.5, "yaw": -0.25}');
    expect(decode!(payload, HEADER)).toEqual({
      value: { x: 1.5, yaw: -0.25 },
      preview: '{"x": 1.5, "yaw": -0.25}',
    });
    expect(registry.get("future.json.v7")).toBeDefined();
  });

  it("returns undefined for unknown encodings (unsupported, not an error)", () => {
    expect(registry.get("h264.v1")).toBeUndefined();
    expect(registry.get(undefined)).toBeUndefined();
  });

  it("resolve() compiles *.lcm.v1 from params.lcm and caches per manifest record", () => {
    expect(POSE.name).toBe("pose_stamped");
    const s = poseSpec();
    const decode = registry.resolve(s);
    expect(decode).toBeDefined();
    expect(registry.resolve(s)).toBe(decode);
    expect(registry.resolve({ ...s })).not.toBe(decode); // a re-adopted manifest compiles anew
    const decoded = decode!(b64ToBytes(POSE.payload_b64), HEADER);
    const value = decoded.value as {
      pose: { position: { x: number } };
      header: { frame_id: string };
    };
    expect(value.pose.position.x).toBe(1.5);
    expect(value.header.frame_id).toBe("map");
    expect(decoded.preview).toContain("position: {x: 1.5, y: -2.5, z: 0.25}");
    // get() alone knows nothing about the schema.
    expect(registry.get(s.encoding)).toBeUndefined();
  });

  it("resolve() lets an exact registration beat the lcm family rule", () => {
    const own = createDecoderRegistry();
    const mine: Decoder = () => ({ value: "mine" });
    own.register("geometry_msgs.PoseStamped.lcm.v1", mine);
    expect(own.resolve(poseSpec())).toBe(mine);
  });

  it("resolve() is undefined, repeatably, for a missing or unusable schema", () => {
    for (
      const params of [
        {},
        { lcm: null },
        { lcm: { type: "t.P", fp: "zz", structs: {} } },
        { lcm: { type: "t.P", fp: "0011223344556677", structs: {} } },
      ]
    ) {
      const s = spec({ ch: "x", encoding: "t.P.lcm.v1", params });
      expect(registry.resolve(s)).toBeUndefined();
      expect(registry.resolve(s)).toBeUndefined();
    }
  });

  it("resolve() keeps get() semantics for json and unknown encodings", () => {
    expect(registry.resolve(spec())).toBe(registry.get("pose.json.v1"));
    expect(registry.resolve(spec({ encoding: "h264.v1" }))).toBeUndefined();
  });

  it("passes jpeg payloads through with dimensions scanned from the bytes", () => {
    const decode = registry.get("jpeg.v1");
    expect(decode).toBeDefined();
    const payload = jpegBytes(320, 240);
    // header.meta is robot-controlled and ignored: the scan wins.
    const decoded = decode!(payload, { ...HEADER, meta: { w: 1, h: 1 } });
    expect(decoded.value).toBe(payload); // the bytes ARE the value, no copy
    expect(decoded.preview).toBe(`(jpeg 320x240, ${payload.byteLength} B)`);
  });

  it("skips APPn segments to find the SOF", () => {
    const sof = jpegBytes(64, 48).subarray(2);
    const payload = new Uint8Array([0xff, 0xd8, 0xff, 0xe0, 0x00, 0x04, 0x4a, 0x46, ...sof]);
    const decode = registry.get("jpeg.v1")!;
    expect(decode(payload, HEADER).preview).toBe(`(jpeg 64x48, ${payload.byteLength} B)`);
  });

  it("throws on a payload without a JPEG SOI", () => {
    const decode = registry.get("jpeg.v1")!;
    expect(() => decode(new Uint8Array([1, 2, 3, 4, 5]), HEADER)).toThrow(/SOI/);
    expect(() => decode(new Uint8Array([0xff, 0xd8, 0xff]), HEADER)).toThrow(/SOI/);
  });

  it("throws when the header is truncated before the SOF", () => {
    const decode = registry.get("jpeg.v1")!;
    expect(() => decode(new Uint8Array([0xff, 0xd8]), HEADER)).toThrow();
    expect(() => decode(jpegBytes(320, 240).subarray(0, 8), HEADER)).toThrow(/truncated|overruns/);
  });

  it("throws when SOS arrives before any SOF", () => {
    const decode = registry.get("jpeg.v1")!;
    const payload = new Uint8Array([0xff, 0xd8, 0xff, 0xda, 0x00, 0x02]);
    expect(() => decode(payload, HEADER)).toThrow(/SOS/);
  });

  it("throws on an oversized payload before scanning it", () => {
    const decode = registry.get("jpeg.v1")!;
    // All zeros (no SOI): the cap message proves the size check ran first.
    const payload = new Uint8Array(MAX_JPEG_PAYLOAD_BYTES + 1);
    expect(() => decode(payload, HEADER)).toThrow(/oversized/);
  });

  it("throws on dimensions past MAX_JPEG_DIM and on zero width", () => {
    const decode = registry.get("jpeg.v1")!;
    expect(() => decode(jpegBytes(MAX_JPEG_DIM + 1, 100), HEADER)).toThrow(/out of bounds/);
    expect(() => decode(jpegBytes(0, 100), HEADER)).toThrow(/out of bounds/);
  });

  it("prefers an exact registration over the JSON fallback", () => {
    const own = createDecoderRegistry();
    own.register("special.json.v1", () => ({ value: "exact" }));
    expect(own.get("special.json.v1")!(new Uint8Array(), HEADER).value).toBe("exact");
  });

  it("decodes the bare json.v1 encoding out of the box", () => {
    const decode = registry.get("json.v1");
    expect(decode).toBeDefined();
    expect(decode!(new TextEncoder().encode("[1.5]"), HEADER).value).toEqual([1.5]);
  });

  it("keeps registries independent: same encoding id, different decoders", () => {
    const a = createDecoderRegistry();
    const b = createDecoderRegistry();
    a.register("foo.v1", () => ({ value: "a" }));
    b.register("foo.v1", () => ({ value: "b" }));
    expect(a.get("foo.v1")!(new Uint8Array(), HEADER).value).toBe("a");
    expect(b.get("foo.v1")!(new Uint8Array(), HEADER).value).toBe("b");
    expect(createDecoderRegistry().get("foo.v1")).toBeUndefined();
  });

  it("rejects duplicate registration unless replace is passed", () => {
    const own = createDecoderRegistry();
    own.register("foo.v1", () => ({ value: 1 }));
    expect(() => own.register("foo.v1", () => ({ value: 2 }))).toThrow(/already registered/);
    // Built-ins count as registrations too.
    expect(() => own.register("jpeg.v1", () => ({ value: 2 }))).toThrow(/already registered/);
    own.register("foo.v1", () => ({ value: 2 }), { replace: true });
    expect(own.get("foo.v1")!(new Uint8Array(), HEADER).value).toBe(2);
  });

  it("throws on invalid UTF-8 so the caller can count a decode error", () => {
    const decode = registry.get("pose.json.v1")!;
    expect(() => decode(new Uint8Array([0xff, 0xfe, 0x22]), HEADER)).toThrow();
  });

  it("reports oversized json instead of parsing it", () => {
    const decode = registry.get("pose.json.v1")!;
    // 0x31 = "1": would be valid JSON, but must never reach the parser.
    const payload = new Uint8Array(MAX_JSON_PAYLOAD_BYTES + 1).fill(0x31);
    const { value, preview } = decode(payload, HEADER);
    expect(value).toBeUndefined();
    expect(preview).toContain("oversized");
    expect(preview!.length).toBeLessThan(200);
  });

  it("bounds the preview of large-but-valid json", () => {
    const decode = registry.get("pose.json.v1")!;
    const long = JSON.stringify({ data: "x".repeat(10_000) });
    const { value, preview } = decode(new TextEncoder().encode(long), HEADER);
    expect(value).toEqual({ data: "x".repeat(10_000) });
    expect(preview).toContain("truncated");
    expect(preview!.length).toBeLessThan(JSON_PREVIEW_MAX_CHARS + 50);
  });
});

describe("costmap decoder", () => {
  const decode = registry.get("costmap.zlib.v1")!;
  const header = (meta: Record<string, unknown>): FrameHeader => ({ ...HEADER, meta });
  const META = { w: 3, h: 2, res: 0.05, origin: [-1.25, 2.5, 0.25] };

  it("decodes and inflates every golden vector byte-exactly", async () => {
    // The pytest mirror (test_costmap_encoding.py) re-encodes these same
    // vectors; together they pin the Python-zlib -> DecompressionStream pair.
    for (const vec of costmapFrames.vectors) {
      const payload = b64ToBytes(vec.payload_b64);
      const decoded = decode(payload, header(vec.meta));
      const value = decoded.value as CostmapValue;
      expect(decoded.preview).toBe(
        `(costmap ${vec.meta.w}x${vec.meta.h}, ${payload.byteLength} B)`,
      );
      expect(value.bytes).toBe(payload); // the bytes stay deflated, no copy
      expect({ w: value.w, h: value.h, res: value.res, origin: value.origin }).toEqual(vec.meta);
      expect(await inflateCostmap(value)).toEqual(b64ToBytes(vec.grid_b64));
    }
  });

  it("rejects missing or malformed meta", () => {
    const payload = new Uint8Array([1, 2, 3]);
    expect(() => decode(payload, HEADER)).toThrow(/no meta/);
    expect(() => decode(payload, header({ ...META, w: 2.5 }))).toThrow(/positive integers/);
    expect(() => decode(payload, header({ ...META, h: 0 }))).toThrow(/positive integers/);
    expect(() => decode(payload, header({ ...META, res: -0.5 }))).toThrow(/positive/);
    expect(() => decode(payload, header({ ...META, res: "0.05" }))).toThrow(/finite/);
    expect(() => decode(payload, header({ ...META, origin: [1.5, 2.5] }))).toThrow(/origin/);
    expect(() => decode(payload, header({ ...META, origin: [1.5, 2.5, Infinity] }))).toThrow(
      /origin/,
    );
  });

  it("rejects out-of-bounds dimensions and oversized payloads", () => {
    const payload = new Uint8Array(8);
    expect(() => decode(payload, header({ ...META, w: MAX_COSTMAP_DIM + 1, h: 1 }))).toThrow(
      /out of bounds/,
    );
    expect(() => decode(payload, header({ ...META, w: 1, h: MAX_COSTMAP_DIM + 1 }))).toThrow(
      /out of bounds/,
    );
    expect(() => decode(new Uint8Array(MAX_COSTMAP_PAYLOAD_BYTES + 1), header(META))).toThrow(
      /oversized/,
    );
  });

  it("rejects an inflate length mismatch in either direction", async () => {
    const deflated = await deflate(new Uint8Array(24).fill(7));
    // Declared 3x2=6 cells but the stream inflates to 24: the bomb guard
    // fires mid-stream instead of allocating past the declaration.
    const bomb = decode(deflated, header({ ...META, w: 3, h: 2 })).value as CostmapValue;
    await expect(inflateCostmap(bomb)).rejects.toThrow(/beyond/);
    const short = decode(deflated, header({ ...META, w: 5, h: 5 })).value as CostmapValue;
    await expect(inflateCostmap(short)).rejects.toThrow(/expected/);
  });

  it("rejects a truncated deflate stream", async () => {
    const vec = costmapFrames.vectors[0];
    const payload = b64ToBytes(vec.payload_b64).slice(0, 6);
    const value = decode(payload, header(vec.meta)).value as CostmapValue;
    await expect(inflateCostmap(value)).rejects.toThrow();
  });
});

describe("voxels decoder", () => {
  const decode = registry.get("voxels.zlib.v1")!;
  const header = (meta: Record<string, unknown>): FrameHeader => ({ ...HEADER, meta });
  const META = { res: 0.05, n: 3, chunks: 2 };
  const RECORD_BYTES = 12 + 512;

  /** Voxel indices from decoded centres, sorted like the fixture's `voxels`. */
  function indicesOf(positions: Float32Array, res: number): number[][] {
    const out: number[][] = [];
    for (let i = 0; i < positions.length; i += 3) {
      out.push([0, 1, 2].map((axis) => Math.round(positions[i + axis] / res - 0.5)));
    }
    return out.sort((a, b) => a[0] - b[0] || a[1] - b[1] || a[2] - b[2]);
  }

  it("decodes and inflates every golden vector to the pinned voxels", async () => {
    // The pytest mirror (test_voxel_encoding.py) re-encodes these same
    // vectors; together they pin the Python-zlib -> DecompressionStream pair
    // and the chunk record layout.
    for (const vec of voxelFrames.vectors) {
      const payload = b64ToBytes(vec.payload_b64);
      const decoded = decode(payload, header(vec.meta));
      const value = decoded.value as VoxelsValue;
      expect(decoded.preview).toBe(
        `(voxels ${vec.meta.n}, ${vec.meta.chunks} chunks, ${payload.byteLength} B)`,
      );
      expect(value.bytes).toBe(payload); // the bytes stay deflated, no copy
      expect({ res: value.res, n: value.n, chunks: value.chunks }).toEqual(vec.meta);
      const positions = await inflateVoxels(value);
      expect(positions.length).toBe(vec.meta.n * 3);
      expect(indicesOf(positions, vec.meta.res)).toEqual(vec.voxels);
    }
  });

  it("rejects missing or malformed meta", () => {
    const payload = new Uint8Array([1, 2, 3]);
    expect(() => decode(payload, HEADER)).toThrow(/no meta/);
    expect(() => decode(payload, header({ ...META, res: 0 }))).toThrow(/positive/);
    expect(() => decode(payload, header({ ...META, res: "0.05" }))).toThrow(/finite/);
    expect(() => decode(payload, header({ ...META, n: 2.5 }))).toThrow(/whole number/);
    expect(() => decode(payload, header({ ...META, n: -1 }))).toThrow(/whole number/);
    expect(() => decode(payload, header({ ...META, chunks: 0 }))).toThrow(/chunks/);
    expect(() => decode(payload, header({ ...META, n: 1, chunks: 2 }))).toThrow(/chunks/);
    expect(() => decode(payload, header({ ...META, n: 0, chunks: 1 }))).toThrow(/chunks/);
    expect(() => decode(payload, header({ ...META, n: MAX_VOXELS + 1 }))).toThrow(/exceeds/);
    expect(() => decode(payload, header({ ...META, n: MAX_VOXELS, chunks: MAX_VOXEL_CHUNKS + 1 })))
      .toThrow(/exceeds/);
  });

  it("rejects oversized payloads", () => {
    expect(() => decode(new Uint8Array(MAX_VOXEL_PAYLOAD_BYTES + 1), header(META))).toThrow(
      /oversized/,
    );
  });

  it("accepts the empty frame that clears a map", async () => {
    const deflated = await deflate(new Uint8Array(0));
    const empty = decode(deflated, header({ res: 0.05, n: 0, chunks: 0 })).value as VoxelsValue;
    expect(await inflateVoxels(empty)).toEqual(new Float32Array(0));
  });

  it("rejects a payload whose bits disagree with n", async () => {
    // One record with two bits set: chunk (0, 0, 0), voxels 0 and 1.
    const record = new Uint8Array(RECORD_BYTES);
    record[12] = 0b11;
    const deflated = await deflate(record);
    const over = decode(deflated, header({ res: 0.05, n: 1, chunks: 1 })).value as VoxelsValue;
    await expect(inflateVoxels(over)).rejects.toThrow(/more than/);
    const under = decode(deflated, header({ res: 0.05, n: 3, chunks: 1 })).value as VoxelsValue;
    await expect(inflateVoxels(under)).rejects.toThrow(/expected/);
    const exact = decode(deflated, header({ res: 0.05, n: 2, chunks: 1 })).value as VoxelsValue;
    expect(await inflateVoxels(exact)).toEqual(
      Float32Array.from([0.025, 0.025, 0.025, 0.075, 0.025, 0.025]),
    );
  });

  it("rejects an inflate length mismatch in either direction", async () => {
    // Declared one chunk but the stream inflates to two: the bomb guard fires
    // mid-stream instead of allocating past the declaration.
    const deflated = await deflate(new Uint8Array(RECORD_BYTES * 2));
    const bomb = decode(deflated, header({ res: 0.05, n: 1, chunks: 1 })).value as VoxelsValue;
    await expect(inflateVoxels(bomb)).rejects.toThrow(/beyond/);
    const short = decode(deflated, header({ res: 0.05, n: 3, chunks: 3 })).value as VoxelsValue;
    await expect(inflateVoxels(short)).rejects.toThrow(/expected/);
  });
});

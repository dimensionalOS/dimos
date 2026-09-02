import { describe, expect, it } from "vitest";
import type { FrameHeader } from "@dimos/shared";
import costmapFrames from "../../../shared/fixtures/costmap_frames.json";
import {
  type CostmapValue,
  inflateCostmap,
  MAX_COSTMAP_DIM,
  MAX_COSTMAP_PAYLOAD_BYTES,
} from "./costmap.ts";
import { createDecoderRegistry } from "./index.ts";
import { MAX_JPEG_DIM, MAX_JPEG_PAYLOAD_BYTES } from "./jpeg.ts";
import { JSON_PREVIEW_MAX_CHARS, MAX_JSON_PAYLOAD_BYTES } from "./json.ts";

const HEADER: FrameHeader = { ch: "x", seq: 1, ts: 0, delivery: "latest" };

// Read-only lookups share one built-in registry; registration tests build
// their own.
const registry = createDecoderRegistry();

function b64ToBytes(b64: string): Uint8Array {
  return Uint8Array.from(atob(b64), (c) => c.charCodeAt(0));
}

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

  async function deflate(bytes: Uint8Array): Promise<Uint8Array> {
    const stream = new Blob([bytes as BlobPart]).stream()
      .pipeThrough(new CompressionStream("deflate"));
    return new Uint8Array(await new Response(stream).arrayBuffer());
  }

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

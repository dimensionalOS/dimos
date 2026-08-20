import { describe, expect, it } from "vitest";
import type { FrameHeader } from "@dimos/shared";
import { getDecoder, registerDecoder } from "./index.ts";
import { MAX_JPEG_DIM, MAX_JPEG_PAYLOAD_BYTES } from "./jpeg.ts";
import { JSON_PREVIEW_MAX_CHARS, MAX_JSON_PAYLOAD_BYTES } from "./json.ts";

const HEADER: FrameHeader = { ch: "x", seq: 1, ts: 0, delivery: "latest" };

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
    const decode = getDecoder("pose.json.v1");
    expect(decode).toBeDefined();
    const payload = new TextEncoder().encode('{"x": 1.5, "yaw": -0.25}');
    expect(decode!(payload, HEADER)).toEqual({
      value: { x: 1.5, yaw: -0.25 },
      preview: '{"x": 1.5, "yaw": -0.25}',
    });
    expect(getDecoder("future.json.v7")).toBeDefined();
  });

  it("returns undefined for unknown encodings (unsupported, not an error)", () => {
    expect(getDecoder("costmap.zlib.v1")).toBeUndefined();
    expect(getDecoder("h264.v1")).toBeUndefined();
    expect(getDecoder(undefined)).toBeUndefined();
  });

  it("passes jpeg payloads through with dimensions scanned from the bytes", () => {
    const decode = getDecoder("jpeg.v1");
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
    const decode = getDecoder("jpeg.v1")!;
    expect(decode(payload, HEADER).preview).toBe(`(jpeg 64x48, ${payload.byteLength} B)`);
  });

  it("throws on a payload without a JPEG SOI", () => {
    const decode = getDecoder("jpeg.v1")!;
    expect(() => decode(new Uint8Array([1, 2, 3, 4, 5]), HEADER)).toThrow(/SOI/);
    expect(() => decode(new Uint8Array([0xff, 0xd8, 0xff]), HEADER)).toThrow(/SOI/);
  });

  it("throws when the header is truncated before the SOF", () => {
    const decode = getDecoder("jpeg.v1")!;
    expect(() => decode(new Uint8Array([0xff, 0xd8]), HEADER)).toThrow();
    expect(() => decode(jpegBytes(320, 240).subarray(0, 8), HEADER)).toThrow(/truncated|overruns/);
  });

  it("throws when SOS arrives before any SOF", () => {
    const decode = getDecoder("jpeg.v1")!;
    const payload = new Uint8Array([0xff, 0xd8, 0xff, 0xda, 0x00, 0x02]);
    expect(() => decode(payload, HEADER)).toThrow(/SOS/);
  });

  it("throws on an oversized payload before scanning it", () => {
    const decode = getDecoder("jpeg.v1")!;
    // All zeros (no SOI): the cap message proves the size check ran first.
    const payload = new Uint8Array(MAX_JPEG_PAYLOAD_BYTES + 1);
    expect(() => decode(payload, HEADER)).toThrow(/oversized/);
  });

  it("throws on dimensions past MAX_JPEG_DIM and on zero width", () => {
    const decode = getDecoder("jpeg.v1")!;
    expect(() => decode(jpegBytes(MAX_JPEG_DIM + 1, 100), HEADER)).toThrow(/out of bounds/);
    expect(() => decode(jpegBytes(0, 100), HEADER)).toThrow(/out of bounds/);
  });

  it("prefers an exact registration over the JSON fallback", () => {
    registerDecoder("special.json.v1", () => ({ value: "exact" }));
    expect(getDecoder("special.json.v1")!(new Uint8Array(), HEADER).value).toBe("exact");
  });

  it("throws on invalid UTF-8 so the caller can count a decode error", () => {
    const decode = getDecoder("pose.json.v1")!;
    expect(() => decode(new Uint8Array([0xff, 0xfe, 0x22]), HEADER)).toThrow();
  });

  it("reports oversized json instead of parsing it", () => {
    const decode = getDecoder("pose.json.v1")!;
    // 0x31 = "1": would be valid JSON, but must never reach the parser.
    const payload = new Uint8Array(MAX_JSON_PAYLOAD_BYTES + 1).fill(0x31);
    const { value, preview } = decode(payload, HEADER);
    expect(value).toBeUndefined();
    expect(preview).toContain("oversized");
    expect(preview!.length).toBeLessThan(200);
  });

  it("bounds the preview of large-but-valid json", () => {
    const decode = getDecoder("pose.json.v1")!;
    const long = JSON.stringify({ data: "x".repeat(10_000) });
    const { value, preview } = decode(new TextEncoder().encode(long), HEADER);
    expect(value).toEqual({ data: "x".repeat(10_000) });
    expect(preview).toContain("truncated");
    expect(preview!.length).toBeLessThan(JSON_PREVIEW_MAX_CHARS + 50);
  });
});

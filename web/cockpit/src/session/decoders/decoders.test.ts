import { describe, expect, it } from "vitest";
import type { FrameHeader } from "@dimos/shared";
import { getDecoder, registerDecoder } from "./index.ts";
import { JSON_PREVIEW_MAX_CHARS, MAX_JSON_PAYLOAD_BYTES } from "./json.ts";

const HEADER: FrameHeader = { ch: "x", seq: 1, ts: 0, delivery: "latest" };

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
    expect(getDecoder("jpeg.v1")).toBeUndefined();
    expect(getDecoder("costmap.zlib.v1")).toBeUndefined();
    expect(getDecoder(undefined)).toBeUndefined();
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

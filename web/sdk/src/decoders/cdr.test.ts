import { describe, expect, it } from "vitest";
import frames from "../../../shared/fixtures/cdr_frames.json";
import { cdrDecoder, cdrDecoderFor, cdrPreview } from "./cdr.ts";
const HEADER = { ch: "message", seq: 1, ts: 0, delivery: "latest" as const };
const bytes = (value: string) => Uint8Array.from(atob(value), (c) => c.charCodeAt(0));
function plain(value: unknown): unknown {
  if (typeof value === "bigint") return String(value);
  if (ArrayBuffer.isView(value)) return Array.from(value as unknown as ArrayLike<unknown>, plain);
  if (Array.isArray(value)) return value.map(plain);
  if (value !== null && typeof value === "object") {
    return Object.fromEntries(
      Object.entries(value).map(([key, item]) => [key, plain(item)]),
    );
  }
  return value;
}
describe("generated CDR browser decoding", () => {
  for (const vector of frames.vectors) {
    it(`decodes ${vector.name} in both byte orders from its embedded schema`, () => {
      const decode = cdrDecoder(vector.schema);
      for (const encoded of [vector.payload_b64, vector.big_endian_b64]) {
        expect(plain(decode(bytes(encoded), HEADER).value)).toEqual(vector.value);
      }
    });
  }
  it("rejects truncation, invalid encapsulation and trailing bytes", () => {
    const vector = frames.vectors[0];
    const decode = cdrDecoder(vector.schema);
    const payload = bytes(vector.payload_b64);
    expect(() => decode(payload.subarray(0, payload.length - 1), HEADER)).toThrow();
    expect(() => decode(new Uint8Array([...payload, 0]), HEADER)).toThrow(/trailing/);
    payload[0] = 255;
    expect(() => decode(payload, HEADER)).toThrow(/header/);
  });
  it("declines missing and malformed schemas", () => {
    expect(cdrDecoderFor(undefined)).toBeNull();
    expect(cdrDecoderFor({ type: "p/msg/T", definition: "not a definition !" })).toBeNull();
    expect(cdrDecoderFor({ type: "p.T", definition: "float64 x" })).toBeNull();
  });
  it("bounds previews of large images and arrays", () => {
    const text = cdrPreview({ data: new Uint8Array(1000000), label: "x".repeat(10000) });
    expect(text.length).toBeLessThan(550);
    expect(text).toContain("Uint8Array(1000000)");
  });
});

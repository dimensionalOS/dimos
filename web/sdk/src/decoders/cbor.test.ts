import { describe, expect, it } from "vitest";
import vectors from "../../../shared/fixtures/topic_cbor.json";
import { projectCbor } from "./cbor.ts";

const bytes = (hex: string) =>
  Uint8Array.from(hex.match(/../g) ?? [], (part) => parseInt(part, 16));

describe("data-only DimOS CBOR projection", () => {
  for (const vector of vectors) {
    it(`reads the Python wire fixture: ${vector.name}`, () => {
      const payload = bytes(vector.hex);
      const result = projectCbor(payload);
      if (vector.opaque) {
        expect(result).toMatchObject({ kind: "opaque", bytes: payload });
      } else {
        expect(result).toEqual({ kind: "value", value: vector.value });
      }
    });
  }

  it("preserves bytes, 64-bit integers and numeric special values", () => {
    expect(projectCbor(bytes("4300ff80"))).toEqual({ kind: "value", value: bytes("00ff80") });
    expect(projectCbor(bytes("1bffffffffffffffff"))).toEqual({
      kind: "value",
      value: 2n ** 64n - 1n,
    });
    expect(projectCbor(bytes("3bffffffffffffffff"))).toEqual({
      kind: "value",
      value: -(2n ** 64n),
    });
    expect(projectCbor(bytes("f97c00"))).toEqual({ kind: "value", value: Infinity });
    expect(projectCbor(bytes("f98000"))).toEqual({ kind: "value", value: -0 });
    expect(projectCbor(bytes("f97e00"))).toEqual({ kind: "value", value: NaN });
  });

  it("treats prototype names as data, without changing an object's prototype", () => {
    const result = projectCbor(bytes("a1695f5f70726f746f5f5f01"));
    expect(result.kind).toBe("value");
    if (result.kind !== "value") throw new Error("expected map");
    expect(Object.getPrototypeOf(result.value)).toBeNull();
    expect(Object.hasOwn(result.value as object, "__proto__")).toBe(true);
  });

  it.each([
    ["truncated bytes", "5affffffff"],
    ["oversized array", "9affffffff"],
    ["oversized map", "baffffffff"],
    ["duplicate keys", "a2616101616102"],
    ["non-string keys", "a10102"],
    ["invalid UTF8", "62ffff"],
    ["extra bytes", "0102"],
    ["unknown nested tag", "81d90fd401"],
    ["deep recursion", "81".repeat(34) + "01"],
    ["invalid dataclass", "d90fd001"],
  ])("keeps %s opaque", (_name, hex) => {
    const payload = bytes(hex);
    expect(projectCbor(payload)).toMatchObject({ kind: "opaque", bytes: payload });
  });
});

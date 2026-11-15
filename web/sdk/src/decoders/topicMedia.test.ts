import { describe, expect, it } from "vitest";
import vectors from "../../../shared/fixtures/topic_media.json";
import type { TopicSample } from "../topics.ts";
import { decodeTopicMedia } from "./topicMedia.ts";

const bytes = (hex: string) =>
  Uint8Array.from(hex.match(/../g) ?? [], (part) => parseInt(part, 16));
function sample(name: string, metadata: Record<string, unknown> = {}): TopicSample {
  const vector = vectors.find((v) => v.name === name)!;
  return {
    payload: bytes(vector.payload),
    metadata: { ...vector.metadata, ...metadata },
    encoding: vector.encoding,
    key: "camera",
    source: "workspace",
    generation: 1,
    sequence: 1,
    ingressTs: 200,
    sentTs: 201,
  };
}

describe("fixed topic media", () => {
  it.each(vectors.filter((v) => v.expectedDepth).map((v) => [v.name, v.expectedDepth]))(
    "preserves every numeric bit in %s",
    async (name, expected) => {
      const decoded = await decodeTopicMedia(sample(name));
      expect(decoded.kind).toBe("depth");
      if (decoded.kind !== "depth") throw new Error("expected depth");
      expect(new Uint8Array(decoded.data.buffer)).toEqual(bytes(expected));
      expect([decoded.width, decoded.height]).toEqual([3, 2]);
      expect(decoded.header).toEqual({ frame_id: "camera/β", stamp: 123.456, seq: 9 });
    },
  );

  it.each(["cloud", "cloud_big"])("decodes %s into arrays without point objects", async (name) => {
    const decoded = await decodeTopicMedia(sample(name));
    if (decoded.kind !== "cloud") throw new Error("expected cloud");
    expect(decoded.positions).toBeInstanceOf(Float32Array);
    expect([decoded.count, decoded.sourceCount]).toEqual([3, 4]);
    const expected = [1, 2, 3, 10, -20, 30, -5, 6, 7];
    for (let i = 0; i < expected.length; i++) {
      expect(Math.abs(decoded.positions[i] - expected[i])).toBeLessThan(decoded.maxError + 1e-5);
    }
    expect(decoded.intensity).toEqual(new Float32Array([4, 40, 80]));
    expect(decoded.color).toEqual(bytes("112233ffeedd123456"));
  });

  it("applies fixed cloud decimation without overwriting the full profile", async () => {
    const [full, low] = await Promise.all([
      decodeTopicMedia(sample("cloud")),
      decodeTopicMedia(sample("cloud_low")),
    ]);
    if (full.kind !== "cloud" || low.kind !== "cloud") throw new Error("expected clouds");
    expect([full.count, low.count]).toEqual([3, 1]);
    expect(low.positions).toEqual(new Float32Array([1, 2, 3]));
  });

  it.each(["color_rgb", "color_bgr"])(
    "validates %s JPEG dimensions without creating a bitmap",
    async (name) => {
      const original = sample(name);
      const decoded = await decodeTopicMedia(original);
      expect(decoded).toMatchObject({ kind: "image", encoding: "jpeg", width: 16, height: 8 });
      if (decoded.kind !== "image") throw new Error("expected image");
      expect(decoded.data).toBe(original.payload);
    },
  );

  it.each(
    [
      ["depth16", { w: 4097 }],
      ["depth32", { w: 0 }],
      ["depth16", { units: "m" }],
      ["depth32", { littleEndian: false }],
      ["depth32", { pixelEncoding: "rgb8" }],
      ["cloud", { count: 1_000_001 }],
      ["cloud", { scale: 0 }],
      ["cloud", { origin: [0, NaN, 0] }],
      ["cloud", { intensity: 1 }],
      ["cloud", { sourceCount: 1 }],
      ["color_rgb", { w: 15 }],
      ["depth16", { frameId: "x".repeat(1025) }],
    ] as const,
  )("rejects invalid %s metadata before expanding it", async (name, meta) => {
    await expect(decodeTopicMedia(sample(name, meta))).rejects.toThrow();
  });

  it("rejects compressed data whose expansion exceeds its declared shape", async () => {
    await expect(decodeTopicMedia(sample("depth16", { w: 1, h: 1 }))).rejects.toThrow(
      "inflates beyond",
    );
  });
});

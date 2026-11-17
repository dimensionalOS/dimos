import type { TopicSample } from "../topics.ts";
import { inflateExact } from "./inflate.ts";
import { jpegDimensions, MAX_JPEG_PAYLOAD_BYTES } from "./jpeg.ts";

const MAX_PIXELS = 4_194_304;
const MAX_POINTS = 1_000_000;
const MAX_INFLATED = 16 * 1024 * 1024;
const littleEndian = new Uint8Array(new Uint16Array([1]).buffer)[0] === 1;

export interface MediaHeader {
  frame_id: string;
  /** Original LCM acquisition time, separate from provider ingress. */
  stamp: number;
  seq: number;
}
export interface TopicImage {
  kind: "image";
  width: number;
  height: number;
  encoding: "jpeg";
  /** Encoded bytes; callers control bitmap lifetime and rendering rate. */
  data: Uint8Array;
  header: MediaHeader;
}
export interface TopicDepth {
  kind: "depth";
  width: number;
  height: number;
  encoding: "16UC1" | "32FC1" | "mono16";
  data: Uint16Array | Float32Array;
  units: "m" | "mm";
  unitScale: number;
  header: MediaHeader;
}
export interface TopicCloud {
  kind: "cloud";
  count: number;
  sourceCount: number;
  positions: Float32Array;
  intensity?: Float32Array;
  color?: Uint8Array;
  /** Quantization error per axis, before the final float32 conversion. */
  maxError: number;
  header: MediaHeader;
}
export type TopicMedia = TopicImage | TopicDepth | TopicCloud;

function number(meta: Readonly<Record<string, unknown>>, name: string): number {
  const value = meta[name];
  if (typeof value !== "number" || !Number.isFinite(value)) {
    throw new Error(`invalid media ${name}`);
  }
  return value;
}
function count(meta: Readonly<Record<string, unknown>>, name: string, cap: number): number {
  const value = number(meta, name);
  if (!Number.isInteger(value) || value < 0 || value > cap) {
    throw new Error(`media ${name} exceeds budget`);
  }
  return value;
}
function header(meta: Readonly<Record<string, unknown>>): MediaHeader {
  if (typeof meta.frameId !== "string" || meta.frameId.length > 1024) {
    throw new Error("invalid media frame id");
  }
  return {
    frame_id: meta.frameId,
    stamp: number(meta, "acquisitionTs"),
    seq: number(meta, "headerSeq"),
  };
}
function boolean(meta: Readonly<Record<string, unknown>>, name: string): boolean {
  if (typeof meta[name] !== "boolean") throw new Error(`invalid media ${name}`);
  return meta[name];
}

/** Explicit, bounded decoder for the advertised fixed profiles. No type-name
 * imports and no Python object construction. Call after latest selection;
 * compressed input must inflate to exactly the validated shape. */
export async function decodeTopicMedia(sample: TopicSample): Promise<TopicMedia> {
  if (sample.payload.byteLength > MAX_JPEG_PAYLOAD_BYTES) {
    throw new Error("media payload exceeds budget");
  }
  const meta = sample.metadata;
  const acquisition = header(meta);
  if (sample.encoding === "application/vnd.dimos.cloud.q16.zlib.v1") {
    const n = count(meta, "count", MAX_POINTS);
    const sourceCount = count(meta, "sourceCount", MAX_POINTS);
    const hasIntensity = boolean(meta, "intensity"), hasColor = boolean(meta, "color");
    if (n > sourceCount) throw new Error("cloud count exceeds source count");
    const origin = meta.origin;
    const scale = number(meta, "scale");
    if (
      !Array.isArray(origin) || origin.length !== 3 || !origin.every((x) =>
        typeof x === "number" && Number.isFinite(x) && Math.abs(x) <= 1e30
      ) || scale <= 0 || scale > 1e30
    ) {
      throw new Error("invalid cloud quantization");
    }
    const expected = n * (6 + (hasIntensity ? 4 : 0) + (hasColor ? 3 : 0));
    if (expected > MAX_INFLATED) throw new Error("cloud expansion exceeds budget");
    const bytes = await inflateExact(sample.payload, expected);
    const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
    const positions = new Float32Array(n * 3);
    for (let i = 0; i < positions.length; i++) {
      positions[i] = origin[i % 3] + view.getInt16(i * 2, true) * scale;
    }
    let offset = n * 6;
    let intensity: Float32Array | undefined;
    if (hasIntensity) {
      if (littleEndian && offset % 4 === 0) intensity = new Float32Array(bytes.buffer, offset, n);
      else {
        intensity = new Float32Array(n);
        for (let i = 0; i < n; i++) intensity[i] = view.getFloat32(offset + i * 4, true);
      }
      offset += n * 4;
    }
    return {
      kind: "cloud",
      count: n,
      sourceCount,
      positions,
      intensity,
      color: hasColor ? bytes.subarray(offset) : undefined,
      maxError: scale / 2,
      header: acquisition,
    };
  }
  const width = count(meta, "w", 4096), height = count(meta, "h", 4096);
  if (!width || !height || width * height > MAX_PIXELS) {
    throw new Error("image shape exceeds budget");
  }
  if (sample.encoding === "image/jpeg") {
    const actual = jpegDimensions(sample.payload);
    if (actual.w !== width || actual.h !== height) throw new Error("JPEG shape mismatch");
    return {
      kind: "image",
      width,
      height,
      encoding: "jpeg",
      data: sample.payload,
      header: acquisition,
    };
  }
  if (sample.encoding !== "application/vnd.dimos.depth.zlib.v1") {
    throw new Error("unsupported media encoding");
  }
  const encoding = meta.pixelEncoding;
  if (encoding !== "16UC1" && encoding !== "32FC1" && encoding !== "mono16") {
    throw new Error("unsupported depth encoding");
  }
  const float = encoding === "32FC1", stride = float ? 4 : 2;
  const units = float ? "m" : "mm", unitScale = float ? 1 : 0.001;
  if (meta.units !== units || meta.unitScale !== unitScale || meta.littleEndian !== true) {
    throw new Error("invalid depth units or endianness");
  }
  const expected = width * height * stride;
  if (expected > MAX_INFLATED) throw new Error("depth expansion exceeds budget");
  const bytes = await inflateExact(sample.payload, expected);
  if (!littleEndian) {
    for (let at = 0; at < bytes.length; at += stride) bytes.subarray(at, at + stride).reverse();
  }
  const data = float ? new Float32Array(bytes.buffer) : new Uint16Array(bytes.buffer);
  return { kind: "depth", width, height, encoding, data, units, unitScale, header: acquisition };
}

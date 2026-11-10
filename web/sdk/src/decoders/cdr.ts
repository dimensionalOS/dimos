// Decode generated messages from the manifest's complete ROS2 definition.
import { parse } from "@foxglove/rosmsg";
import { MessageReader } from "@foxglove/rosmsg2-serialization";
import type { Decoder } from "./index.ts";

export interface CdrSchema {
  type: string;
  definition: string;
}
export const CDR_ENCODING_RE = /\.cdr\.v1$/;
export const CDR_PREVIEW_MAX_CHARS = 512;
const PREVIEW_MAX_ITEMS = 8;
const PREVIEW_MAX_STRING = 64;

export function isCdrSchema(value: unknown): value is CdrSchema {
  if (value === null || typeof value !== "object") return false;
  const schema = value as Record<string, unknown>;
  return typeof schema.type === "string" &&
    /^[A-Za-z][A-Za-z0-9_]*\/msg\/[A-Za-z][A-Za-z0-9_]*$/.test(schema.type) &&
    typeof schema.definition === "string";
}

export function cdrDecoder(schema: CdrSchema): Decoder {
  const definitions = parse(schema.definition, { ros2: true });
  const reader = new MessageReader(definitions);
  return (payload) => {
    // Generated codecs use encapsulated XCDR1 with no trailing bytes.
    if (
      payload.length < 4 || payload[0] !== 0 || payload[1] > 1 ||
      payload[2] !== 0 || payload[3] !== 0
    ) throw new Error("invalid XCDR1 header");
    const value = reader.readMessage(payload);
    if (reader.lastReadByteLength() !== payload.byteLength) throw new Error("trailing CDR bytes");
    return { value, preview: cdrPreview(value) };
  };
}

export function cdrDecoderFor(value: unknown): Decoder | null {
  if (!isCdrSchema(value)) return null;
  try {
    return cdrDecoder(value);
  } catch {
    return null;
  }
}

/** Variable-length arrays require an explicit panel subscription in the cockpit. */
export function cdrHasSequences(schema: CdrSchema): boolean {
  return parse(schema.definition, { ros2: true }).some((definition) =>
    definition.definitions.some((field) => field.isArray && field.arrayLength === undefined)
  );
}

export function cdrPreview(value: unknown, budget = CDR_PREVIEW_MAX_CHARS): string {
  const parts: string[] = [];
  let len = 0;
  let full = false;
  const push = (s: string) => {
    parts.push(s);
    len += s.length;
    if (len >= budget) full = true;
  };
  const items = (n: number, at: (i: number) => unknown) => {
    for (let i = 0; i < n && i < PREVIEW_MAX_ITEMS && !full; i++) {
      if (i > 0) push(", ");
      walk(at(i));
    }
    if (n > PREVIEW_MAX_ITEMS && !full) push(", ...");
    push("]");
  };
  const walk = (v: unknown): void => {
    if (full) return;
    if (v === null || v === undefined) {
      push("null");
    } else if (typeof v === "string") {
      push(
        JSON.stringify(v.length > PREVIEW_MAX_STRING ? `${v.slice(0, PREVIEW_MAX_STRING)}...` : v),
      );
    } else if (typeof v === "number" || typeof v === "boolean" || typeof v === "bigint") {
      push(String(v));
    } else if (ArrayBuffer.isView(v)) {
      const a = v as unknown as ArrayLike<unknown>;
      push(`${v.constructor.name}(${a.length})[`);
      items(a.length, (i) => a[i]);
    } else if (Array.isArray(v)) {
      push("[");
      items(v.length, (i) => v[i]);
    } else if (typeof v === "object") {
      push("{");
      let first = true;
      for (const [key, item] of Object.entries(v)) {
        if (full) break;
        if (!first) push(", ");
        first = false;
        push(`${key}: `);
        walk(item);
      }
      push("}");
    } else {
      push(String(v));
    }
  };
  walk(value);
  const text = parts.join("");
  return full ? `${text.slice(0, budget)} ... (truncated)` : text;
}

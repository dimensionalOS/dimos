// Schema-driven LCM decoder for `<msg_name>.lcm.v1` channels. The robot ships
// each channel's schema (exported from the generated dimos_lcm class) in the
// manifest record's params.lcm; compileLcmDecoder turns it into closures once
// per adopted manifest, so any DimOS message decodes with no generated code.
// Values are plain objects with the LCM fields in wire order (including the
// *_length count fields); byte[] and int8_t[] are zero-copy views into the
// frame, other primitive arrays are fresh typed arrays, int64_t is a bigint.
// Decoding runs on the ingest path, so a frame that would expand into more
// than MAX_LCM_ARRAY_ELEMENTS objects is reported as oversized instead (the
// json.v1 decoder's approach to a payload above its cap).

import type { Decoder } from "./index.ts";

export type LcmDim = number | string;
export type LcmField = [name: string, type: string, dims: LcmDim[] | null];
export interface LcmSchema {
  type: string;
  /** 16 hex chars: the big-endian packed fingerprint every frame starts with. */
  fp: string;
  structs: Record<string, LcmField[]>;
}
export type LcmValue = Record<string, unknown>;

export const LCM_ENCODING_RE = /\.lcm\.v1$/;
// Struct and string array elements one frame may expand into JS objects
// (typed arrays and views cost what the payload weighs and need no budget).
// A Path with 1000 poses is 1000; a page that needs more registers its own
// decoder for that encoding.
export const MAX_LCM_ARRAY_ELEMENTS = 100_000;

/** Thrown mid-decode by a compiled decoder for a frame over MAX_LCM_ARRAY_ELEMENTS. */
export class LcmOversizedError extends Error {}

interface Budget {
  left: number;
}
// Longest preview the raw-value UI mounts for an LCM channel.
export const LCM_PREVIEW_MAX_CHARS = 512;
const PREVIEW_MAX_ITEMS = 8;
const PREVIEW_MAX_STRING = 64;

type FieldDecoder = (view: DataView, bytes: Uint8Array, off: number, out: LcmValue) => number;

// fatal: corrupted bytes must fail decode, not U+FFFD their way onto screen.
const utf8 = new TextDecoder("utf-8", { fatal: true });
const SIZE: Record<string, number> = {
  int8_t: 1,
  int16_t: 2,
  int32_t: 4,
  int64_t: 8,
  float: 4,
  double: 8,
  boolean: 1,
  byte: 1,
};

/** Shape check for a manifest's params.lcm; the compiler validates the rows. */
export function isLcmSchema(x: unknown): x is LcmSchema {
  if (typeof x !== "object" || x === null) return false;
  const s = x as Record<string, unknown>;
  return typeof s.type === "string" && typeof s.fp === "string" &&
    /^[0-9a-f]{16}$/.test(s.fp) && typeof s.structs === "object" && s.structs !== null;
}

function readString(view: DataView, bytes: Uint8Array, off: number): [string, number] {
  const len = view.getInt32(off, false); // counts the trailing NUL
  off += 4;
  if (len < 1 || off + len > bytes.byteLength) {
    throw new Error(`bad string length ${len} at ${off}`);
  }
  return [utf8.decode(bytes.subarray(off, off + len - 1)), off + len];
}

function scalar(name: string, type: string): FieldDecoder {
  switch (type) {
    case "double":
      return (v, _b, off, out) => {
        out[name] = v.getFloat64(off, false);
        return off + 8;
      };
    case "float":
      return (v, _b, off, out) => {
        out[name] = v.getFloat32(off, false);
        return off + 4;
      };
    case "int32_t":
      return (v, _b, off, out) => {
        out[name] = v.getInt32(off, false);
        return off + 4;
      };
    case "int16_t":
      return (v, _b, off, out) => {
        out[name] = v.getInt16(off, false);
        return off + 2;
      };
    case "int8_t":
      return (v, _b, off, out) => {
        out[name] = v.getInt8(off);
        return off + 1;
      };
    case "int64_t":
      return (v, _b, off, out) => {
        out[name] = v.getBigInt64(off, false);
        return off + 8;
      };
    case "byte":
      return (v, _b, off, out) => {
        out[name] = v.getUint8(off);
        return off + 1;
      };
    case "boolean":
      return (v, _b, off, out) => {
        out[name] = v.getUint8(off) !== 0;
        return off + 1;
      };
    case "string":
      return (v, b, off, out) => {
        const [text, next] = readString(v, b, off);
        out[name] = text;
        return next;
      };
    default:
      throw new Error(`unknown primitive ${type}`);
  }
}

/** Array length: the fixed size, or the value of the count field already
 * decoded into the same struct (LCM puts the count before the array). */
function count(dim: LcmDim, out: LcmValue): number {
  const n = typeof dim === "number" ? dim : out[dim];
  if (typeof n !== "number" || n < 0 || !Number.isInteger(n)) {
    throw new Error(`bad array length ${String(n)}`);
  }
  return n;
}

function reserve(budget: Budget, name: string, n: number): void {
  if (n > budget.left) {
    throw new LcmOversizedError(`over ${MAX_LCM_ARRAY_ELEMENTS} array elements (${name})`);
  }
  budget.left -= n;
}

function checked(name: string, type: string, size: number) {
  return (bytes: Uint8Array, off: number, n: number) => {
    if (off + n * size > bytes.byteLength) {
      throw new Error(`${name}: ${n} x ${type} overruns the payload`);
    }
  };
}

function primitiveArray(name: string, type: string, dim: LcmDim, budget: Budget): FieldDecoder {
  const check = checked(name, type, SIZE[type] ?? 1);
  switch (type) {
    case "byte":
      return (_v, b, off, out) => {
        const n = count(dim, out);
        check(b, off, n);
        out[name] = b.subarray(off, off + n);
        return off + n;
      };
    case "int8_t":
      return (_v, b, off, out) => {
        const n = count(dim, out);
        check(b, off, n);
        out[name] = new Int8Array(b.buffer, b.byteOffset + off, n);
        return off + n;
      };
    case "boolean":
      return (_v, b, off, out) => {
        const n = count(dim, out);
        check(b, off, n);
        const a = new Array<boolean>(n);
        for (let i = 0; i < n; i++) a[i] = b[off + i] !== 0;
        out[name] = a;
        return off + n;
      };
    case "float":
      return (v, b, off, out) => {
        const n = count(dim, out);
        check(b, off, n);
        const a = new Float32Array(n);
        for (let i = 0; i < n; i++, off += 4) a[i] = v.getFloat32(off, false);
        out[name] = a;
        return off;
      };
    case "double":
      return (v, b, off, out) => {
        const n = count(dim, out);
        check(b, off, n);
        const a = new Float64Array(n);
        for (let i = 0; i < n; i++, off += 8) a[i] = v.getFloat64(off, false);
        out[name] = a;
        return off;
      };
    case "int16_t":
      return (v, b, off, out) => {
        const n = count(dim, out);
        check(b, off, n);
        const a = new Int16Array(n);
        for (let i = 0; i < n; i++, off += 2) a[i] = v.getInt16(off, false);
        out[name] = a;
        return off;
      };
    case "int32_t":
      return (v, b, off, out) => {
        const n = count(dim, out);
        check(b, off, n);
        const a = new Int32Array(n);
        for (let i = 0; i < n; i++, off += 4) a[i] = v.getInt32(off, false);
        out[name] = a;
        return off;
      };
    case "int64_t":
      return (v, b, off, out) => {
        const n = count(dim, out);
        check(b, off, n);
        const a = new BigInt64Array(n);
        for (let i = 0; i < n; i++, off += 8) a[i] = v.getBigInt64(off, false);
        out[name] = a;
        return off;
      };
    case "string":
      return (v, b, off, out) => {
        const n = count(dim, out);
        check(b, off, n); // every string is at least 5 bytes: no giant Array from a bad count
        reserve(budget, name, n);
        const a = new Array<string>(n);
        for (let i = 0; i < n; i++) [a[i], off] = readString(v, b, off);
        out[name] = a;
        return off;
      };
    default:
      throw new Error(`unknown primitive ${type}`);
  }
}

function struct(name: string, fields: FieldDecoder[]): FieldDecoder {
  return (v, b, off, out) => {
    const o: LcmValue = {};
    for (let i = 0; i < fields.length; i++) off = fields[i](v, b, off, o);
    out[name] = o;
    return off;
  };
}

function structArray(
  name: string,
  fields: FieldDecoder[],
  dim: LcmDim,
  budget: Budget,
): FieldDecoder {
  const check = checked(name, "struct", 1);
  return (v, b, off, out) => {
    const n = count(dim, out);
    check(b, off, n); // a corrupt count must not allocate a giant Array first
    reserve(budget, name, n);
    const a = new Array<LcmValue>(n);
    for (let j = 0; j < n; j++) {
      const o: LcmValue = {};
      for (let i = 0; i < fields.length; i++) off = fields[i](v, b, off, o);
      a[j] = o;
    }
    out[name] = a;
    return off;
  };
}

/** Compile a schema into a decoder of whole frames (fingerprint included).
 * Throws on a schema it cannot serve: unknown primitives, missing structs,
 * multi-dimensional arrays. The decoder throws LcmOversizedError, before
 * expanding the array that crosses it, for a frame over the element budget. */
export function compileLcmDecoder(schema: LcmSchema): (payload: Uint8Array) => LcmValue {
  const compiled = new Map<string, FieldDecoder[]>();
  const budget: Budget = { left: 0 };
  const structDecoders = (name: string): FieldDecoder[] => {
    let fields = compiled.get(name);
    if (fields !== undefined) return fields;
    const rows = schema.structs[name];
    if (!Array.isArray(rows)) throw new Error(`schema has no struct ${name}`);
    fields = [];
    compiled.set(name, fields);
    for (const [fname, ftype, dims] of rows) {
      const prim = ftype === "string" || Object.hasOwn(SIZE, ftype);
      if (dims === null) {
        fields.push(prim ? scalar(fname, ftype) : struct(fname, structDecoders(ftype)));
      } else if (dims.length === 1) {
        fields.push(
          prim
            ? primitiveArray(fname, ftype, dims[0], budget)
            : structArray(fname, structDecoders(ftype), dims[0], budget),
        );
      } else {
        throw new Error(`multi-dimensional arrays are not supported (${name}.${fname})`);
      }
    }
    return fields;
  };
  const root = structDecoders(schema.type);
  const fp = Uint8Array.from(
    { length: 8 },
    (_, i) => parseInt(schema.fp.slice(i * 2, i * 2 + 2), 16),
  );
  return (payload: Uint8Array): LcmValue => {
    if (payload.byteLength < 8) throw new Error("payload shorter than the fingerprint");
    for (let i = 0; i < 8; i++) {
      if (payload[i] !== fp[i]) throw new Error(`fingerprint mismatch for ${schema.type}`);
    }
    const view = new DataView(payload.buffer, payload.byteOffset, payload.byteLength);
    const out: LcmValue = {};
    budget.left = MAX_LCM_ARRAY_ELEMENTS;
    let off = 8;
    for (let i = 0; i < root.length; i++) off = root[i](view, payload, off, out);
    if (off !== payload.byteLength) throw new Error(`trailing bytes: ${payload.byteLength - off}`);
    return out;
  };
}

/** Bounded text form of a decoded value for the raw channel table: the first
 * elements of arrays and typed arrays, cut strings, bigint digits. Cost is
 * proportional to the budget, never to the message. */
export function lcmPreview(value: unknown, budget = LCM_PREVIEW_MAX_CHARS): string {
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

/** The Decoder for a channel's params.lcm, or null when the schema is missing
 * or unusable (the channel then renders as one without a decoder). */
export function lcmDecoderFor(lcm: unknown): Decoder | null {
  if (!isLcmSchema(lcm)) return null;
  let decode: (payload: Uint8Array) => LcmValue;
  try {
    decode = compileLcmDecoder(lcm);
  } catch {
    return null;
  }
  return (payload) => {
    let value: LcmValue;
    try {
      value = decode(payload);
    } catch (e) {
      if (!(e instanceof LcmOversizedError)) throw e;
      return {
        value: undefined,
        preview: `(oversized lcm message: ${e.message}, ${payload.byteLength} B)`,
      };
    }
    return { value, preview: lcmPreview(value) };
  };
}

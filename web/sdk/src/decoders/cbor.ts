/** A data-only projection of DimOS CBOR. No Python class resolution or hooks.
 * Unknown tags leave the whole message opaque for an application decoder. */
export type CborData =
  | null
  | boolean
  | number
  | bigint
  | string
  | Uint8Array
  | CborData[]
  | { [name: string]: CborData };

export type CborProjection =
  | { kind: "value"; value: CborData }
  | { kind: "opaque"; bytes: Uint8Array; reason: string };

const MAX_BYTES = 8 * 1024 * 1024;
const MAX_ITEMS = 100_000;
const MAX_DEPTH = 32;
const utf8 = new TextDecoder("utf-8", { fatal: true });

class Reader {
  readonly view: DataView;
  offset = 0;
  items = 0;
  constructor(readonly bytes: Uint8Array) {
    this.view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  }

  take(size: number): number {
    const start = this.offset;
    if (size < 0 || size > this.bytes.byteLength - start) throw new Error("truncated CBOR");
    this.offset += size;
    return start;
  }

  argument(info: number): number | bigint {
    if (info < 24) return info;
    if (info === 24) return this.view.getUint8(this.take(1));
    if (info === 25) return this.view.getUint16(this.take(2));
    if (info === 26) return this.view.getUint32(this.take(4));
    if (info === 27) {
      const value = this.view.getBigUint64(this.take(8));
      return value <= BigInt(Number.MAX_SAFE_INTEGER) ? Number(value) : value;
    }
    throw new Error("indefinite or reserved CBOR argument");
  }

  value(depth = 0): CborData {
    if (depth > MAX_DEPTH || ++this.items > MAX_ITEMS) {
      throw new Error("CBOR complexity bound exceeded");
    }
    const initial = this.view.getUint8(this.take(1));
    const major = initial >> 5;
    const info = initial & 31;
    if (major === 7) {
      if (info === 20) return false;
      if (info === 21) return true;
      if (info === 22) return null;
      if (info === 25) {
        const bits = this.view.getUint16(this.take(2));
        const sign = bits & 0x8000 ? -1 : 1;
        const exponent = (bits >> 10) & 31;
        const mantissa = bits & 1023;
        return sign *
          (exponent === 31
            ? (mantissa ? NaN : Infinity)
            : exponent === 0
            ? mantissa * 2 ** -24
            : (1 + mantissa / 1024) * 2 ** (exponent - 15));
      }
      if (info === 26) return this.view.getFloat32(this.take(4));
      if (info === 27) return this.view.getFloat64(this.take(8));
      throw new Error("unsupported CBOR simple value");
    }
    const argument = this.argument(info);
    if (major === 0) return argument;
    if (major === 1) return typeof argument === "bigint" ? -1n - argument : -1 - argument;
    if (typeof argument !== "number") throw new Error("CBOR size or tag exceeds bound");
    if (major === 2 || major === 3) {
      const start = this.take(argument);
      const bytes = this.bytes.subarray(start, start + argument);
      return major === 2 ? bytes : utf8.decode(bytes);
    }
    if (major === 4) {
      if (argument > MAX_ITEMS - this.items) throw new Error("CBOR array exceeds bound");
      const values: CborData[] = [];
      for (let i = 0; i < argument; i++) values.push(this.value(depth + 1));
      return values;
    }
    if (major === 5) {
      if (argument > (MAX_ITEMS - this.items) / 2) throw new Error("CBOR map exceeds bound");
      // Null prototype preserves keys such as __proto__ as ordinary data.
      const values: Record<string, CborData> = Object.create(null);
      for (let i = 0; i < argument; i++) {
        const key = this.value(depth + 1);
        if (typeof key !== "string" || Object.hasOwn(values, key)) {
          throw new Error("CBOR map needs distinct string keys");
        }
        values[key] = this.value(depth + 1);
      }
      return values;
    }
    if (major === 6) {
      // Constants mirror dimos/protocol/encode/cbor.py, not its object loader.
      if (argument !== 4040 && argument !== 4048) throw new Error(`opaque CBOR tag ${argument}`);
      const data = this.value(depth + 1);
      if (argument === 4040 && Array.isArray(data)) return data;
      if (
        argument === 4048 && Array.isArray(data) && data.length === 2 &&
        typeof data[0] === "string" && data[0].length <= 256 &&
        typeof data[1] === "object" && data[1] !== null && !Array.isArray(data[1]) &&
        !(data[1] instanceof Uint8Array)
      ) {
        return { $type: data[0], fields: data[1] };
      }
      throw new Error("invalid DimOS CBOR data tag");
    }
    throw new Error("unsupported CBOR type");
  }
}

/** Explicit decoder for pZenohTransport bytes. The dataclass projection is
 * `{ $type: "module:Class", fields: {...} }`; its name is inert metadata.
 * Bytes in either result are borrowed and must be copied before transferring.
 */
export function projectCbor(bytes: Uint8Array): CborProjection {
  try {
    if (bytes.byteLength > MAX_BYTES) throw new Error("CBOR payload exceeds byte bound");
    const reader = new Reader(bytes);
    const value = reader.value();
    if (reader.offset !== bytes.byteLength) throw new Error("trailing CBOR bytes");
    return { kind: "value", value };
  } catch (error) {
    return { kind: "opaque", bytes, reason: (error as Error).message };
  }
}

/** Envelope only. Payloads are the unchanged DimOS control and data frames. */
export const MAX_FRAME_BYTES = 2 * 1024 * 1024;
export const MAX_CONTROL_BYTES = 64 * 1024;
export const MAX_CLIENTS = 26;
// A cockpit publishes roughly 80 small frames/s across its channels. Allow
// normal Internet acknowledgment latency while retaining the 2 MiB byte bound.
export const MAX_PENDING_DOWNLOADS = 64;
export const MAX_PENDING_UPLOADS = 32;
const encoder = new TextEncoder();
const decoder = new TextDecoder("utf-8", { fatal: true });
export type Identity = { id: string; login: string };
export type Envelope = { to: string[]; kind: 0 | 1 | 2; sentAt?: number };

export function pack(header: Envelope, payload: Uint8Array): Uint8Array {
  const json = encoder.encode(JSON.stringify(header));
  if (json.length > 8192 || payload.length > MAX_FRAME_BYTES) {
    throw new Error("Frame too large");
  }
  const bytes = new Uint8Array(4 + json.length + payload.length);
  new DataView(bytes.buffer).setUint32(0, json.length);
  bytes.set(json, 4);
  bytes.set(payload, 4 + json.length);
  return bytes;
}

export function unpack(
  bytes: Uint8Array,
): { header: Envelope; payload: Uint8Array } {
  if (bytes.length < 5 || bytes.length > MAX_FRAME_BYTES + 8196) {
    throw new Error("Invalid frame");
  }
  const length = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength)
    .getUint32(0);
  if (length > 8192 || length + 4 > bytes.length) {
    throw new Error("Invalid header");
  }
  const h = JSON.parse(decoder.decode(bytes.subarray(4, length + 4)));
  if (
    !h || !Array.isArray(h.to) || h.to.length > MAX_CLIENTS ||
    !h.to.every((s: unknown) =>
      typeof s === "string" && /^[a-zA-Z0-9-]{1,80}$/.test(s)
    ) ||
    ![0, 1, 2].includes(h.kind) ||
    (h.sentAt !== undefined && !Number.isFinite(h.sentAt))
  ) {
    throw new Error("Invalid routing header");
  }
  return { header: h, payload: bytes.subarray(length + 4) };
}

export function serverFrame(
  kind: number,
  sequence: number,
  payload: Uint8Array,
): Uint8Array {
  const bytes = new Uint8Array(5 + payload.length);
  bytes[0] = kind;
  new DataView(bytes.buffer).setUint32(1, sequence);
  bytes.set(payload, 5);
  return bytes;
}

export function clientFrame(
  kind: 0 | 2,
  payload: Uint8Array,
  now = Date.now(),
): Uint8Array {
  if (payload.length > MAX_CONTROL_BYTES) throw new Error("Control too large");
  const bytes = new Uint8Array(9 + payload.length);
  bytes[0] = kind;
  new DataView(bytes.buffer).setFloat64(1, now);
  bytes.set(payload, 9);
  return bytes;
}

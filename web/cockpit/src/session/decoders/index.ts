// Payload decoder registry, keyed by the manifest's encoding id. An encoding
// without a decoder is not an error: the channel renders as "unsupported"
// (forward compatibility with newer bridges). Binary decoders (jpeg.v1,
// costmap.zlib.v1, ...) arrive with their panels from T4 on.

import type { FrameHeader } from "@dimos/shared";
import { jsonDecoder } from "./json.ts";

export interface Decoded {
  value: unknown;
  /** Bounded text form of `value` for the raw channel UI; binary decoders leave it unset. */
  preview?: string;
}

export type Decoder = (payload: Uint8Array, header: FrameHeader) => Decoded;

const registry = new Map<string, Decoder>();

export function registerDecoder(encoding: string, decoder: Decoder): void {
  registry.set(encoding, decoder);
}

export function getDecoder(encoding: string | undefined): Decoder | undefined {
  if (encoding === undefined) return undefined;
  const exact = registry.get(encoding);
  if (exact !== undefined) return exact;
  if (/\.json\.v\d+$/.test(encoding)) return jsonDecoder;
  return undefined;
}

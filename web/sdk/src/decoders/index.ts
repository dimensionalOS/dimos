// Payload decoder registry, keyed by the manifest's encoding id. An encoding
// without a decoder is not an error: the channel renders as "unsupported"
// (forward compatibility with newer bridges). Binary decoders (h264.v1, ...)
// arrive with their panels. Each session captures its own registry
// (ConnectOptions.decoders), so two independently embedded frontends cannot
// mutate one another's decoder tables.

import type { FrameHeader } from "@dimos/shared";
import { costmapDecoder } from "./costmap.ts";
import { jpegDecoder } from "./jpeg.ts";
import { jsonDecoder } from "./json.ts";

export interface Decoded {
  value: unknown;
  /** Bounded text form of `value` for the raw channel UI; binary decoders leave it unset. */
  preview?: string;
}

export type Decoder = (payload: Uint8Array, header: FrameHeader) => Decoded;

export class DecoderRegistry {
  #decoders = new Map<string, Decoder>();

  /** Duplicate registration is an error unless `replace` is set. */
  register(encoding: string, decoder: Decoder, opts: { replace?: boolean } = {}): void {
    if (opts.replace !== true && this.#decoders.has(encoding)) {
      throw new Error(`decoder for ${encoding} already registered (pass replace: true)`);
    }
    this.#decoders.set(encoding, decoder);
  }

  get(encoding: string | undefined): Decoder | undefined {
    if (encoding === undefined) return undefined;
    const exact = this.#decoders.get(encoding);
    if (exact !== undefined) return exact;
    // The documented *.json.vN convention decodes without registration.
    if (/\.json\.v\d+$/.test(encoding)) return jsonDecoder;
    return undefined;
  }
}

/** A fresh registry preloaded with the built-in codecs. */
export function createDecoderRegistry(): DecoderRegistry {
  const registry = new DecoderRegistry();
  registry.register("jpeg.v1", jpegDecoder);
  registry.register("costmap.zlib.v1", costmapDecoder);
  registry.register("json.v1", jsonDecoder);
  return registry;
}

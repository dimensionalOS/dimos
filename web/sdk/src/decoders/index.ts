// Payload decoder registry. resolve(spec) finds a manifest channel's decoder:
// a registered encoding id, the *.json.vN convention, or a *.cdr.v1 decoder
// compiled from the schema the robot put in the channel's params. An encoding
// without a decoder is not an error: the channel renders as "unsupported"
// (forward compatibility with newer bridges). Binary decoders (h264.v1, ...)
// arrive with their panels. Each session captures its own registry
// (ConnectOptions.decoders), so two independently embedded frontends cannot
// mutate one another's decoder tables.

import type { ChannelSpec, FrameHeader } from "@dimos/shared";
import { costmapDecoder } from "./costmap.ts";
import { jpegDecoder } from "./jpeg.ts";
import { jsonDecoder } from "./json.ts";
import { CDR_ENCODING_RE, cdrDecoderFor } from "./cdr.ts";

export interface Decoded {
  value: unknown;
  /** Bounded text form of `value` for the raw channel UI; binary decoders leave it unset. */
  preview?: string;
}

export type Decoder = (payload: Uint8Array, header: FrameHeader) => Decoded;

export class DecoderRegistry {
  #decoders = new Map<string, Decoder>();
  // Compiled *.cdr.v1 decoders per adopted manifest record (null: params.cdr
  // unusable); entries die with the manifest object that carried them.
  #cdr = new WeakMap<ChannelSpec, Decoder | null>();

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

  /** The decoder for a manifest channel: get(spec.encoding) first (registered
   * ids and the JSON convention win), else a *.cdr.v1 decoder compiled once
   * from the schema in spec.params.cdr. */
  resolve(spec: ChannelSpec): Decoder | undefined {
    const known = this.get(spec.encoding);
    if (known !== undefined) return known;
    if (!CDR_ENCODING_RE.test(spec.encoding)) return undefined;
    let compiled = this.#cdr.get(spec);
    if (compiled === undefined) {
      compiled = cdrDecoderFor(spec.params.cdr);
      this.#cdr.set(spec, compiled);
    }
    return compiled ?? undefined;
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

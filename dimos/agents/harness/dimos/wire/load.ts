import type { WireDecoder } from "./base.ts";
import { LoadResultSchema } from "../../../shared/protocol/runtime/load.ts";
import { wireDecoder } from "./decoder.ts";

export const loadResultDecoder = wireDecoder(
  "dimos agent load output",
  LoadResultSchema,
) satisfies WireDecoder<unknown>;

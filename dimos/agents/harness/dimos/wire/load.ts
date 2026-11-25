import type { WireDecoder } from "#dimos/agents/harness/dimos/wire/base.ts";
import { LoadResultSchema } from "#dimos/agents/shared/protocol/runtime/load.ts";
import { wireDecoder } from "#dimos/agents/harness/dimos/wire/decoder.ts";

export const loadResultDecoder = wireDecoder(
  "dimos agent load output",
  LoadResultSchema,
) satisfies WireDecoder<unknown>;

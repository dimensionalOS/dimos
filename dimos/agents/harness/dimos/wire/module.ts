import type { WireDecoder } from "#dimos/agents/harness/dimos/wire/base.ts";
import { ModuleOutcomeSchema } from "#dimos/agents/shared/protocol/runtime/module.ts";
import { wireDecoder } from "#dimos/agents/harness/dimos/wire/decoder.ts";

export const moduleOutcomeDecoder = wireDecoder(
  "agent_module outcome",
  ModuleOutcomeSchema,
) satisfies WireDecoder<unknown>;

import type { WireDecoder } from "./base.ts";
import { ModuleOutcomeSchema } from "../../../shared/protocol/runtime/module.ts";
import { wireDecoder } from "./decoder.ts";

export const moduleOutcomeDecoder = wireDecoder(
  "agent_module outcome",
  ModuleOutcomeSchema,
) satisfies WireDecoder<unknown>;

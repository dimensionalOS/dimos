import type { WireDecoder } from "#dimos/agents/harness/dimos/wire/base.ts";
import { EventSchema } from "#dimos/agents/shared/protocol/runtime/event.ts";
import { wireDecoder } from "#dimos/agents/harness/dimos/wire/decoder.ts";

/** The `params` of a `dimos/harness_event` notification. */
export const harnessEventDecoder = wireDecoder("harness event", EventSchema) satisfies WireDecoder<unknown>;

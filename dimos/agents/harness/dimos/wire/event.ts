import type { WireDecoder } from "./base.ts";
import { EventSchema } from "../../../shared/protocol/runtime/event.ts";
import { wireDecoder } from "./decoder.ts";

/** The `params` of a `dimos/harness_event` notification. */
export const harnessEventDecoder = wireDecoder("harness event", EventSchema) satisfies WireDecoder<unknown>;

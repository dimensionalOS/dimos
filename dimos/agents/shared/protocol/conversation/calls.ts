import type { EpochMs } from "../time.ts";
import type { JsonObject } from "@earendil-works/pi-durable";

/**
 * A model-issued tool call's id (pi-ai `ToolCall.id`), as `pi.live.tools[].callId` names it. Defined here, on the side
 * that publishes it; the harness's detacher and input source type-import it.
 */
export type CallId = string & { readonly __brand: "CallId" };

/** A running call whose work can move to a background task now. */
export type Detachable = { readonly call: CallId; readonly tool: string; readonly since: EpochMs };

/** A call whose work was moved to the background and still runs. */
export type MovedCall = {
  readonly call: CallId;
  readonly tool: string;
  /** The arguments the model called it with. */
  readonly arguments: JsonObject;
  /** When the call started. */
  readonly since: EpochMs;
  /** The end of the work's output so far. */
  readonly output: string;
  /** Earlier output was cut from `output`. */
  readonly truncated: boolean;
};

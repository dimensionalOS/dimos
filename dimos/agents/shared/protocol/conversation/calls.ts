import type { EpochMs } from "../time.ts";

/**
 * A model-issued tool call's id (pi-ai `ToolCall.id`), as `pi.live.tools[].callId` names it. Defined here, on the side
 * that publishes it; the harness's detacher and input source type-import it.
 */
export type CallId = string & { readonly __brand: "CallId" };

/** A running call whose work can move to a background task now. */
export type Detachable = { readonly call: CallId; readonly tool: string; readonly since: EpochMs };

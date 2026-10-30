import type { CallId } from "./calls.ts";
import type { EntryId, TaskId } from "@earendil-works/pi-durable";
import type { HarnessEvent } from "../runtime/event.ts";
import type { JobName, JobRun } from "../jobs/state.ts";
import type { LimitRecord } from "../tools/limit.ts";
import type { RenderDetails } from "../media.ts";
import type { RuntimeChange } from "../runtime/context.ts";

/** Chosen by the terminal for one typed input and reused when it retries, so a retry submits nothing new. */
export type UserRequestId = string & { readonly __brand: "UserRequestId" };

/** Who put an input into a conversation. Its request id, its display and its trajectory step follow from it. */
export type InputSource =
  | { readonly kind: "user"; readonly request: UserRequestId }
  | { readonly kind: "event"; readonly event: HarnessEvent }
  | { readonly kind: "job"; readonly job: JobName; readonly run: JobRun }
  /** The steer after a tool round that may have changed the system; it carries only a change. */
  | { readonly kind: "runtime"; readonly after: EntryId }
  /** A tool call sent to the background has finished, failed, was stopped, or was lost to a gateway restart. */
  | {
      readonly kind: "background";
      readonly task: TaskId;
      readonly call: CallId;
      readonly tool: string;
      readonly end: "finished" | "failed" | "stopped" | "interrupted";
    };

/** The data of every input the harness places. One kind for all sources, so one fold, one view and one step mapping cover them. */
export type DimosInput = {
  readonly source: InputSource;
  /** The text the model received, after the limit. */
  readonly text: string;
  readonly change: RuntimeChange | null;
  /** Exports and files rendered into the input's images. */
  readonly render: RenderDetails | null;
  readonly limit: LimitRecord | null;
};

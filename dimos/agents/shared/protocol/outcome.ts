import type { AgentModuleName } from "#dimos/agents/shared/protocol/runtime/event.ts";
import type { ConversationId, EntryId } from "@earendil-works/pi-durable";
import type { EpochMs } from "#dimos/agents/shared/protocol/time.ts";

/**
 * Why an action did not happen, as data. Executors on the gateway and the client's transport layer both produce these;
 * `refusalText` is the only place any of them becomes a sentence, so every frontend says the same thing.
 */
export type Refusal =
  /** A method of the owner variant on a viewer: attached read-only, or a view-only browser token. */
  | { readonly code: "viewing"; readonly conversation: ConversationId | null }
  /** The DimOS link is not up. `detail` is `describeLink` of the gateway's LinkStatus. */
  | { readonly code: "dimos_unavailable"; readonly detail: string }
  /** An agent module op that ran and failed; HarnessRuntime's one-line error, its traceback kept for a detail view. */
  | {
      readonly code: "module_failed";
      readonly module: AgentModuleName;
      readonly op: "create" | "restart" | "unload" | "delete";
      readonly error: string;
      readonly traceback: string | null;
    }
  /** Edit on a message that is not one of the user's inputs. */
  | { readonly code: "not_a_user_message"; readonly conversation: ConversationId; readonly entry: EntryId }
  /** Up or withdraw with no user inputs queued. */
  | { readonly code: "nothing_queued"; readonly conversation: ConversationId }
  /** A model change while a run is in progress. */
  | { readonly code: "running"; readonly conversation: ConversationId }
  | {
      readonly code: "not_found";
      readonly what: "conversation" | "message" | "call" | "module" | "job" | "subscription" | "provider" | "model";
      readonly name: string;
    }
  /** An argument the executor rejected: a bad cron expression, an unknown time zone, a job that already exists. */
  | { readonly code: "invalid"; readonly field: string; readonly problem: string }
  /** The message left the active range (a compaction or reset moved on), or a summary head pi-durable settled `stale`. */
  | { readonly code: "stale"; readonly conversation: ConversationId }
  /** Background asked for a call that already finished, or one that cannot be adopted (it was not started yet). */
  | { readonly code: "not_running"; readonly call: string }
  | { readonly code: "login_failed"; readonly provider: string; readonly error: string }
  /** The model call behind a summary failed. */
  | { readonly code: "model_failed"; readonly error: string }
  /** Client side: the link dropped while the call was in flight; it may have happened. */
  | { readonly code: "unknown_outcome"; readonly since: EpochMs }
  /** Client side: a contract this client was built with does not match the gateway's. */
  | { readonly code: "other_build"; readonly gatewayBuild: string | null }
  | { readonly code: "stopping" };

/** Every method result: done with its value, or refused with the reason. */
export type Outcome<T> = { readonly ok: true; readonly value: T } | { readonly ok: false; readonly refusal: Refusal };

export function done<T>(value: T): Outcome<T> {
  return { ok: true, value };
}

export function refused(refusal: Refusal): Outcome<never> {
  return { ok: false, refusal };
}

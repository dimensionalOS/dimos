import type { Refusal } from "#dimos/agents/shared/protocol/outcome.ts";
import type { Rule, Standing } from "#dimos/agents/shared/protocol/actions/standing.ts";
import type { Static, TSchema, TUnsafe } from "typebox";

/** Who carries an action out, so a reader knows where a refusal or a delay comes from. */
export type Executor = "gateway" | "pi-durable" | "harness-runtime" | "model-runtime";

/** `owner`: on the owner variant only. `any`: on both variants (reads). */
export type Access = "owner" | "any";

/**
 * What the client does when the link drops mid-call. `by_request_id`: send it again with the same `request` once the
 * link is live, and the executor answers with the first call's result. `unknown`: record the outcome as unknown and let
 * state show what happened. Only arguments that carry a `request` may retry, which the type enforces.
 */
export type Retry = "by_request_id" | "unknown";

type RetryFor<A> = A extends { readonly request: string } ? Retry : "unknown";

/** Server action schemas, access, validation and request deduplication policy. */
export type ActionDef<A extends TSchema, V, X extends Access = Access> = {
  readonly args: A;
  readonly value: TUnsafe<V>;
  readonly executor: Executor;
  /** Kept as the entry's own literal, so `ViewerMethods` can pick the `any` entries by type. */
  readonly access: X;
  readonly retry: RetryFor<Static<A>>;
  readonly rule: Rule<Static<A>>;
};

/** Any entry, with its argument type erased for catalog iteration. */
export type AnyAction = {
  readonly args: TSchema;
  readonly value: TSchema;
  readonly executor: Executor;
  readonly access: Access;
  readonly retry: Retry;
  readonly rule: (args: never, standing: Standing) => Refusal | null;
};

export type ActionGroup = { readonly [name: string]: AnyAction };

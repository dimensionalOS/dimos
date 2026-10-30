import type { ActionGroup, AnyAction } from "../../../../shared/protocol/actions/base.ts";
import type { ActionObject } from "../../../../shared/protocol/actions/catalog.ts";
import type { Context } from "@earendil-works/chord/context";
import type { Ctx } from "@earendil-works/chord";
import type { Outcome } from "../../../../shared/protocol/outcome.ts";
import type { Principal } from "../../../auth/principal.ts";
import type { Replies } from "../../replies.ts";
import type { Standing } from "../../../../shared/protocol/actions/standing.ts";
import type { Static } from "typebox";
import { BACKGROUND_CONTEXT, withAbortSignal } from "@earendil-works/chord/context";
import { DimosUnavailable } from "../../../../harness/dimos/connection.ts";
import { describeLink } from "../../../../shared/protocol/gateway/link.ts";
import { refused } from "../../../../shared/protocol/outcome.ts";

/**
 * What an executor knows of its call: the pi-durable context, from Chord's or, for a `by_request_id` action, the
 * gateway's from `replies`. Who asked is the factory's principal; inside an object, Chord's ctx acts as the gateway.
 */
export type HandlerCall = { readonly context: Context };

/**
 * An executor per action of a catalog group, checked exhaustive by the compiler: a new catalog entry does not compile
 * until its executor exists. It returns a refusal for what it finds wrong; an unexpected throw propagates to Chord.
 */
export type Handlers<G extends ActionGroup> = {
  readonly [N in keyof G]: (args: Static<G[N]["args"]>, call: HandlerCall) => Promise<Outcome<Static<G[N]["value"]>>>;
};

type BoundMethod<D extends AnyAction> = (
  args: Static<D["args"]>,
  ctx: Ctx<Principal>,
) => Promise<Outcome<Static<D["value"]>>>;

export type OwnerBound<G extends ActionGroup> = { readonly [N in keyof G]: BoundMethod<G[N]> };

export type ViewerBound<G extends ActionGroup> = {
  readonly [N in keyof G as G[N]["access"] extends "any" ? N : never]: BoundMethod<G[N]>;
};

type Executor = (args: unknown, call: HandlerCall) => Promise<Outcome<unknown>>;

/**
 * Each Chord method runs the catalog's `rule` over server state, then the executor. An action whose entry retries
 * `by_request_id` answers a repeated `request` from `replies` with its first outcome before the rule sees the state
 * that outcome left, and runs in a context the caller's link does not abort, so a client's retry after a dropped link
 * neither acts twice nor loses texts that exist nowhere else. `DimosUnavailable` thrown by an executor becomes
 * `dimos_unavailable`; any other throw propagates to Chord.
 */
export function bind<G extends ActionGroup>(
  object: ActionObject,
  group: G,
  handlers: Handlers<G>,
  standing: () => Standing,
  replies: Replies,
  variant: "owner",
): OwnerBound<G>;

export function bind<G extends ActionGroup>(
  object: ActionObject,
  group: G,
  handlers: Handlers<G>,
  standing: () => Standing,
  replies: Replies,
  variant: "viewer",
): ViewerBound<G>;

export function bind<G extends ActionGroup>(
  object: ActionObject,
  group: G,
  handlers: Handlers<G>,
  standing: () => Standing,
  replies: Replies,
  variant: "owner" | "viewer",
): OwnerBound<G> | ViewerBound<G> {
  const bound: { [name: string]: BoundMethod<AnyAction> } = {};
  for (const [name, def] of Object.entries(group)) {
    if (variant === "viewer" && def.access !== "any") continue;
    // The catalog types each executor by its own entry; this loop sees every entry as one erased shape.
    const execute = handlers[name as keyof G] as Executor;
    bound[name] = async (args, ctx) => {
      const run = async (context: Context): Promise<Outcome<unknown>> => {
        const refusal = def.rule(args as never, standing());
        if (refusal !== null) return refused(refusal);
        return guarded(() => execute(args, { context }));
      };
      if (def.retry !== "by_request_id") return run(withAbortSignal(ctx.signal, BACKGROUND_CONTEXT));
      const { request } = args as { readonly request: string };
      return replies.once(`${object}.${name}:${request}`, run);
    };
  }
  return bound as OwnerBound<G> | ViewerBound<G>;
}

async function guarded<T>(run: () => Promise<Outcome<T>>): Promise<Outcome<T>> {
  try {
    return await run();
  } catch (error) {
    if (error instanceof DimosUnavailable)
      return refused({ code: "dimos_unavailable", detail: describeLink(error.status) });
    throw error;
  }
}

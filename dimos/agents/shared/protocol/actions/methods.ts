import type { ActionGroup } from "./base.ts";
import type { Outcome } from "../outcome.ts";
import type { Static, TUnsafe } from "typebox";
import { carried } from "../schema.ts";
import { method } from "@earendil-works/chord";

/** One action as a Chord method spec: its arguments, and its Outcome as a trusted result. */
type MethodOf<D extends ActionGroup[string]> = ReturnType<
  typeof method<D["args"], TUnsafe<Outcome<Static<D["value"]>>>>
>;

/** The owner variant carries every action of the group. */
export type OwnerMethods<G extends ActionGroup> = { readonly [N in keyof G]: MethodOf<G[N]> };

/** The viewer variant carries only the actions whose access is `any`. */
export type ViewerMethods<G extends ActionGroup> = {
  readonly [N in keyof G as G[N]["access"] extends "any" ? N : never]: MethodOf<G[N]>;
};

/**
 * The Chord method specs of a catalog group for one variant, derived so the contracts list no method by hand. The
 * result is trusted like every gateway-produced value: a result schema that inlined the refusal union would cost
 * about 3 KB per method in every hold frame.
 */
export function methodsOf<G extends ActionGroup>(group: G, variant: "owner"): OwnerMethods<G>;

export function methodsOf<G extends ActionGroup>(group: G, variant: "viewer"): ViewerMethods<G>;

export function methodsOf<G extends ActionGroup>(
  group: G,
  variant: "owner" | "viewer",
): OwnerMethods<G> | ViewerMethods<G> {
  const specs: { [name: string]: unknown } = {};
  for (const [name, def] of Object.entries(group))
    if (variant === "owner" || def.access === "any") specs[name] = method(def.args, carried());
  return specs as OwnerMethods<G> | ViewerMethods<G>;
}

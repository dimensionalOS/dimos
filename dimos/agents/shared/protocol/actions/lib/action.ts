import type { Access, ActionDef } from "#dimos/agents/shared/protocol/actions/base.ts";
import type { TSchema } from "typebox";

export function action<A extends TSchema, V, X extends Access>(def: ActionDef<A, V, X>): ActionDef<A, V, X> {
  return def;
}

import type { ActionGroup } from "./base.ts";
import { Type } from "typebox";
import { action } from "./lib/action.ts";
import { carried } from "../schema.ts";
import { rules } from "./standing.ts";

/** The gateway process's one action; it lives on the frozen Lifecycle object, so it works against any build. */
export const LIFECYCLE_ACTIONS = {
  stop: action({
    args: Type.Object({}),
    value: carried<null>(),
    executor: "gateway",
    access: "owner",
    retry: "unknown",
    rule: rules.open,
  }),
} as const satisfies ActionGroup;

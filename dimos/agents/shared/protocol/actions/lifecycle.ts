import type { ActionGroup } from "#dimos/agents/shared/protocol/actions/base.ts";
import { Type } from "typebox";
import { action } from "#dimos/agents/shared/protocol/actions/lib/action.ts";
import { carried } from "#dimos/agents/shared/protocol/schema.ts";
import { rules } from "#dimos/agents/shared/protocol/actions/standing.ts";

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

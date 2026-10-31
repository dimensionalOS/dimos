import { CONVERSATION_ACTIONS } from "#dimos/agents/shared/protocol/actions/conversation.ts";
import { GATEWAY_ACTIONS } from "#dimos/agents/shared/protocol/actions/gateway.ts";
import { LIFECYCLE_ACTIONS } from "#dimos/agents/shared/protocol/actions/lifecycle.ts";

export const CATALOG = {
  conversation: CONVERSATION_ACTIONS,
  gateway: GATEWAY_ACTIONS,
  lifecycle: LIFECYCLE_ACTIONS,
} as const;

/** The served objects that have actions. */
export type ActionObject = keyof typeof CATALOG;

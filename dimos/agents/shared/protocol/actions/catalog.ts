import { CONVERSATION_ACTIONS } from "./conversation.ts";
import { GATEWAY_ACTIONS } from "./gateway.ts";
import { LIFECYCLE_ACTIONS } from "./lifecycle.ts";

export const CATALOG = {
  conversation: CONVERSATION_ACTIONS,
  gateway: GATEWAY_ACTIONS,
  lifecycle: LIFECYCLE_ACTIONS,
} as const;

/** The served objects that have actions. */
export type ActionObject = keyof typeof CATALOG;

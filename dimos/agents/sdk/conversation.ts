import type { ConversationId } from "@earendil-works/pi-durable";
import type { Refusal } from "../shared/protocol/outcome.ts";

/** Why an owner request came back as a viewer, or null when the requested role was granted. */
export function attachRefusal(
  conversation: ConversationId,
  asked: "owner" | "viewer",
  got: "owner" | "viewer",
): Refusal | null {
  return asked === "owner" && got === "viewer" ? viewingRefusal(conversation) : null;
}

/** Why a viewer cannot change this conversation. */
export function viewingRefusal(conversation: ConversationId): Refusal {
  return { code: "viewing", conversation };
}

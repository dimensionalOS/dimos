import type { ConversationId } from "@earendil-works/pi-durable";
import type { UserRequestId } from "#dimos/agents/shared/protocol/conversation/input.ts";
import type { EpochMs } from "#dimos/agents/shared/protocol/time.ts";
import type { Refusal } from "#dimos/agents/shared/protocol/outcome.ts";
import { refusalText } from "#dimos/agents/shared/view-models/outcome.ts";
export type ActionStatus = "pending" | "done" | "refused" | "unknown";

/**
 * One action as every frontend shows it: pending until its result, then done, refused (with the reason) or unknown. A
 * refusal remains visible with its reason.
 */
export type ActionRecord = {
  readonly seq: number;
  readonly action: "action" | "conversation.attach" | "conversation.send";
  readonly conversation: ConversationId | null;
  /** The UI label for this action. */
  readonly text: string;
  /** For sends: what the timeline shows greyed until the gateway queues or places it. */
  readonly sending: { readonly request: UserRequestId; readonly text: string } | null;
  readonly status: ActionStatus;
  readonly refusal: Refusal | null;
  readonly startedAt: EpochMs;
  readonly endedAt: EpochMs | null;
};

/** "⟳ restart follower…", "✓ restart follower", "✖ restart follower: restart follower failed: SyntaxError …". */
export function actionLine(record: ActionRecord): string {
  switch (record.status) {
    case "pending":
      return `⟳ ${record.text}…`;
    case "done":
      return `✓ ${record.text}`;
    case "refused":
    case "unknown":
      return record.refusal === null ? `✖ ${record.text}` : `✖ ${record.text}: ${refusalText(record.refusal)}`;
  }
}

/** The sends of one conversation that are still pending, for `timelineOf`. */
export function pendingSends(
  actions: readonly ActionRecord[],
  conversation: ConversationId,
): readonly { readonly request: UserRequestId; readonly text: string }[] {
  const sends: { readonly request: UserRequestId; readonly text: string }[] = [];
  for (const record of actions) {
    if (record.status !== "pending" || record.conversation !== conversation || record.sending === null) continue;
    sends.push(record.sending);
  }
  return sends;
}

import type { CallId, Detachable, MovedCall } from "#dimos/agents/shared/protocol/conversation/calls.ts";
import type { ConversationId, EntryId, InboxState, LiveState } from "@earendil-works/pi-durable";
import type { LinkStatus } from "#dimos/agents/shared/protocol/gateway/link.ts";
import type { Refusal } from "#dimos/agents/shared/protocol/outcome.ts";
import type { Transcript } from "#dimos/agents/shared/protocol/conversation/state.ts";
import { describeLink } from "#dimos/agents/shared/protocol/gateway/link.ts";
import { inputOf } from "#dimos/agents/shared/protocol/conversation/queue.ts";

/** Server state used to decide whether an action can run. */
export type Standing = {
  /** The variant the caller holds on the object the action is addressed to. */
  readonly role: "owner" | "viewer";
  readonly link: LinkStatus;
  /** Present when the action is addressed to a conversation. */
  readonly conversation: ConversationStanding | null;
};

export type ConversationStanding = {
  readonly id: ConversationId;
  /** `pi.live.run` exists: the one definition of busy. */
  readonly busy: boolean;
  /** User inputs in `pi.inbox`. */
  readonly userQueued: number;
  readonly detachable: readonly CallId[];
  readonly moved: readonly CallId[];
  /** "user" for a user input in the active range, "other" for any other item there, null when it left the range. */
  readonly messageKind: (entry: EntryId) => "user" | "other" | null;
};

/** The conversation's standing from its members' values. */
export function conversationStanding(
  id: ConversationId,
  values: {
    readonly transcript: Transcript;
    readonly live: LiveState;
    readonly queue: InboxState;
    readonly detachable: readonly Detachable[];
    readonly moved: readonly MovedCall[];
  },
): ConversationStanding {
  let userQueued = 0;
  for (const item of values.queue.items) if (inputOf(item)?.source.kind === "user") userQueued += 1;
  return {
    id,
    busy: values.live.run !== undefined,
    userQueued,
    detachable: values.detachable.map((call) => call.call),
    moved: values.moved.map((call) => call.call),
    messageKind: (entry) => {
      const item = values.transcript.items.find((candidate) => candidate.entry === entry);
      if (item === undefined) return null;
      if (item.kind === "user" || (item.kind === "input" && item.input.source.kind === "user")) return "user";
      return "other";
    },
  };
}

/** Why an action cannot run now, or null. Pure, total over `Standing`. */
export type Rule<Args> = (args: Args, standing: Standing) => Refusal | null;

/** The rules the catalog names. The object's variant determines which methods a client can call. */
export const rules = {
  open: (): Refusal | null => null,
  /** The McpServer link is up; otherwise `dimos_unavailable` with `describeLink`'s clause. */
  dimosUp: (_args: unknown, standing: Standing): Refusal | null =>
    standing.link.state === "up" ? null : { code: "dimos_unavailable", detail: describeLink(standing.link) },
  /** No run in progress: a model change mid-run is `running`. */
  idle: (_args: unknown, standing: Standing): Refusal | null =>
    standing.conversation !== null && standing.conversation.busy
      ? { code: "running", conversation: standing.conversation.id }
      : null,
  /** At least one user input is queued; otherwise `nothing_queued`. */
  queued: (_args: unknown, standing: Standing): Refusal | null =>
    standing.conversation !== null && standing.conversation.userQueued === 0
      ? { code: "nothing_queued", conversation: standing.conversation.id }
      : null,
  /** The message is a user input still in the active range: `not_a_user_message`, or `stale` once it left the range. */
  userMessage: (args: { readonly message: EntryId }, standing: Standing): Refusal | null => {
    if (standing.conversation === null) return null;
    const kind = standing.conversation.messageKind(args.message);
    if (kind === null) return { code: "stale", conversation: standing.conversation.id };
    if (kind === "other")
      return { code: "not_a_user_message", conversation: standing.conversation.id, entry: args.message };
    return null;
  },
  /** The message is still in the active range; otherwise `stale`. */
  inRange: (args: { readonly message: EntryId }, standing: Standing): Refusal | null =>
    standing.conversation !== null && standing.conversation.messageKind(args.message) === null
      ? { code: "stale", conversation: standing.conversation.id }
      : null,
  /** The call is in `detachable`; otherwise `not_running`. */
  detachable: (args: { readonly call: CallId }, standing: Standing): Refusal | null =>
    standing.conversation !== null && !standing.conversation.detachable.includes(args.call)
      ? { code: "not_running", call: args.call }
      : null,
  /** The call is in `moved`; otherwise `not_running`. */
  moved: (args: { readonly call: CallId }, standing: Standing): Refusal | null =>
    standing.conversation !== null && !standing.conversation.moved.includes(args.call)
      ? { code: "not_running", call: args.call }
      : null,
} as const;

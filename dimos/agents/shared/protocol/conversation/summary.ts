import type { ConversationId, EntryId } from "@earendil-works/pi-durable";
import type { EpochMs } from "../time.ts";

/** One switcher row, folded on the gateway from commit publications; never built from a conversation's whole view. */
export type ConversationRow = {
  readonly id: ConversationId;
  /** pi-durable's fork record: this conversation inherits `conversation` through entry `at`. An edit of a first message forks at it too. */
  readonly parent: { readonly conversation: ConversationId; readonly at: EntryId } | null;
  /** The first line of this conversation's own first user input; a fork's own, so forks differ from their source. */
  readonly title: string;
  readonly cwd: string | null;
  readonly running: Running | null;
  /** User items in `pi.inbox`. */
  readonly queued: number;
  /** Notice inputs placed since anyone last showed this conversation. */
  readonly unread: number;
  readonly costUsd: number;
  readonly updatedAt: EpochMs;
};

/** What a busy conversation is doing, as the switcher names it. Never "task". */
export type Running =
  | { readonly kind: "answering"; readonly since: EpochMs }
  | { readonly kind: "tool"; readonly name: string; readonly since: EpochMs }
  | { readonly kind: "compaction"; readonly since: EpochMs }
  | { readonly kind: "summary"; readonly since: EpochMs };

import type { InboxState } from "@earendil-works/pi-durable";
import type { Transcript, TranscriptItem } from "#dimos/agents/shared/protocol/conversation/state.ts";
import type { QueuedRow } from "#dimos/agents/shared/view-models/conversation/queue.ts";
import { queuedRows } from "#dimos/agents/shared/view-models/conversation/queue.ts";

/** Requirement 6: the notices that woke this conversation, and the ones still on their way. */
export type Notices = {
  /** Input items whose source is a module, a job or a background call, oldest first. */
  readonly delivered: readonly TranscriptItem[];
  /** Admitted, waiting in `pi.inbox` for the next boundary. */
  readonly queued: readonly QueuedRow[];
};

function isNotice(item: TranscriptItem): boolean {
  if (item.kind !== "input") return false;
  const { kind } = item.input.source;
  return kind === "event" || kind === "job" || kind === "background";
}

function isQueuedNotice(row: QueuedRow): boolean {
  return row.from === "module" || row.from === "job" || row.from === "background";
}

export function noticesOf(transcript: Transcript, queue: InboxState): Notices {
  return { delivered: transcript.items.filter(isNotice), queued: queuedRows(queue).filter(isQueuedNotice) };
}

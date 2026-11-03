import type { AssistantMessage, ToolCall } from "@earendil-works/pi-ai";
import type { Detachable } from "#dimos/agents/shared/protocol/conversation/calls.ts";
import type { EpochMs } from "#dimos/agents/shared/protocol/time.ts";
import type { InboxState, LiveState, ToolSlot } from "@earendil-works/pi-durable";
import type { InputOutcome, Transcript, TranscriptItem } from "#dimos/agents/shared/protocol/conversation/state.ts";
import type { QueuedRow } from "#dimos/agents/shared/view-models/conversation/queue.ts";
import type { UserRequestId } from "#dimos/agents/shared/protocol/conversation/input.ts";
import { inputOf } from "#dimos/agents/shared/protocol/conversation/queue.ts";
import { queuedRows } from "#dimos/agents/shared/view-models/conversation/queue.ts";

/** One row of the chat, keyed so a renderer reconciles its components instead of rebuilding them. */
export type TimelineRow =
  | {
      readonly kind: "item";
      readonly key: string;
      readonly item: TranscriptItem;
      readonly outcome: InputOutcome | null;
    }
  /** The partial of the response being generated, from `pi.live.generation.message`. */
  | { readonly kind: "streaming"; readonly key: "streaming"; readonly message: AssistantMessage }
  | {
      readonly kind: "tool";
      readonly key: string;
      readonly call: ToolCall;
      readonly slot: ToolSlot;
      /** The detach registry holds it: Ctrl+B or the web page's button may send it to the background. */
      readonly backgroundable: boolean;
      /** When the detach registry took the call, for its duration; null while it is not backgroundable. */
      readonly since: EpochMs | null;
    }
  | { readonly kind: "queued"; readonly key: string; readonly row: QueuedRow; readonly position: number }
  /** A `send` the client made that the gateway has not answered: shown greyed until it is queued or placed. */
  | { readonly kind: "sending"; readonly key: string; readonly request: UserRequestId; readonly text: string }
  /** A retry, a failed compaction or a stale summary that `pi.live` reports; transient, not stored. */
  | { readonly kind: "live_notice"; readonly key: string; readonly text: string };

export type TimelineInput = {
  readonly transcript: Transcript;
  readonly live: LiveState;
  readonly queue: InboxState;
  readonly outcomes: readonly InputOutcome[];
  readonly detachable: readonly Detachable[];
};

/** The request a user input carries; null for every other item. */
function requestOf(item: TranscriptItem): UserRequestId | null {
  return item.kind === "input" && item.input.source.kind === "user" ? item.input.source.request : null;
}

function newestAssistant(items: readonly TranscriptItem[]): AssistantMessage | null {
  for (let index = items.length - 1; index >= 0; index -= 1) {
    const item = items[index];
    if (item.kind === "assistant") return item.message;
  }
  return null;
}

function toolCallIn(message: AssistantMessage, callId: string): ToolCall | null {
  for (const block of message.content) if (block.type === "toolCall" && block.id === callId) return block;
  return null;
}

/**
 * Items in transcript order with each input's outcome beside it, then the streamed partial, the running calls with
 * their arguments taken from the newest assistant item, the queue in order, and the sends still in flight. A send
 * disappears once an item or a queued row carries its request id.
 */
export function timelineOf(
  input: TimelineInput,
  sending: readonly { readonly request: UserRequestId; readonly text: string }[],
): readonly TimelineRow[] {
  const { transcript, live, queue, outcomes, detachable } = input;
  const rows: TimelineRow[] = [];
  const placed = new Set<UserRequestId>();
  for (const item of transcript.items) {
    const request = requestOf(item);
    if (request !== null) placed.add(request);
    const outcome = request === null ? undefined : outcomes.find((candidate) => candidate.request === request);
    rows.push({ kind: "item", key: `e${item.entry}`, item, outcome: outcome === undefined ? null : outcome });
  }
  const message = live.generation?.message;
  if (message !== undefined) rows.push({ kind: "streaming", key: "streaming", message });
  const newest = newestAssistant(transcript.items);
  const tools = live.tools === undefined ? [] : live.tools;
  for (const slot of tools) {
    if (slot.status === "done") continue;
    const call = newest === null ? null : toolCallIn(newest, slot.callId);
    if (call === null) continue;
    const detached = detachable.find((candidate) => candidate.call === slot.callId);
    rows.push({
      kind: "tool",
      key: `t${slot.callId}`,
      call,
      slot,
      backgroundable: detached !== undefined,
      since: detached === undefined ? null : detached.since,
    });
  }
  queuedRows(queue).forEach((row, position) => rows.push({ kind: "queued", key: `q${row.submission}`, row, position }));
  for (const item of queue.items) {
    const queued = inputOf(item);
    if (queued !== null && queued.source.kind === "user") placed.add(queued.source.request);
  }
  const retry = live.generation?.retry;
  if (retry !== undefined)
    rows.push({ kind: "live_notice", key: "retry", text: `Retrying after an error: ${retry.error}` });
  const failure = live.compactionFailure;
  if (failure !== undefined)
    rows.push({ kind: "live_notice", key: "compaction", text: `Compaction failed: ${failure.error}` });
  for (const send of sending) {
    if (placed.has(send.request)) continue;
    rows.push({ kind: "sending", key: `s${send.request}`, request: send.request, text: send.text });
  }
  return rows;
}

/** Whether a run is in progress: `pi.live.run` exists exactly while busy. The one definition every frontend uses. */
export function isRunning(live: LiveState): boolean {
  return live.run !== undefined;
}

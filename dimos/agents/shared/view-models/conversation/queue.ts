import type { DimosInput } from "../../protocol/conversation/input.ts";
import type { InboxState, SubmissionId } from "@earendil-works/pi-durable";
import type { QueuedText } from "../../protocol/conversation/queue.ts";
import { inputOf } from "../../protocol/conversation/queue.ts";

/** One queued item as the chat shows it: in queue order, with its mode, and whose it is. Never "event" or "task". */
export type QueuedRow = {
  readonly submission: SubmissionId;
  readonly mode: "steer" | "followUp";
  /** The user's own text (`dimos.input` data.text), or the notice's text for module, job and background notices. */
  readonly text: string;
  readonly from: "you" | "module" | "job" | "background" | "runtime";
};

function fromOf(input: DimosInput): QueuedRow["from"] {
  switch (input.source.kind) {
    case "user":
      return "you";
    case "event":
      return "module";
    case "job":
      return "job";
    case "runtime":
      return "runtime";
    default:
      return "background";
  }
}

function textOf(input: DimosInput): string {
  const { source } = input;
  if (source.kind === "event") return `${source.event.module}: ${source.event.text}`;
  if (source.kind === "runtime") return input.change === null ? input.text : input.change.summary;
  return input.text;
}

/** `pi.inbox` items in submission order; writes (summaries, kicks) are not shown. */
export function queuedRows(queue: InboxState): readonly QueuedRow[] {
  const rows: QueuedRow[] = [];
  for (const item of queue.items) {
    if (item.mode === "write") continue;
    const input = inputOf(item);
    if (input === null) continue;
    rows.push({ submission: item.id, mode: item.mode, text: textOf(input), from: fromOf(input) });
  }
  return rows;
}

/** What Up would bring back, as `withdraw` will: the user's own items, oldest first. */
export function userQueued(rows: readonly QueuedRow[]): readonly QueuedText[] {
  return rows.filter((row) => row.from === "you").map((row) => ({ mode: row.mode, text: row.text }));
}

/** The editor text after a withdraw: the texts joined by blank lines, as Claude Code does. */
export function editorTextOf(texts: readonly QueuedText[]): string {
  return texts.map((item) => item.text).join("\n\n");
}

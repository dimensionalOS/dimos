import type { ConversationView, EntryId, EntryRecord } from "@earendil-works/pi-durable";
import type { ImageAddress, ImageRef } from "../../../../shared/protocol/conversation/images.ts";
import type { ImageContent, Message, TextContent, ToolResultMessage, UserMessage } from "@earendil-works/pi-ai";
import type { Outcome } from "../../../../shared/protocol/outcome.ts";
import type { ToolResult, Transcript, TranscriptItem } from "../../../../shared/protocol/conversation/state.ts";
import { AssistantEntry, CompactionEntry, ResetEntry, ToolResultEntry, UserEntry } from "@earendil-works/pi-durable";
import { BranchSummaryEntry, OriginEntry, SummaryEntry } from "../../../../harness/history/entries.ts";
import { DimosInputEntry } from "../../../../harness/inputs/source.ts";
import { done, refused } from "../../../../shared/protocol/outcome.ts";
import { parseToolDetails } from "../../../../harness/tools/details.ts";

/** At most this much of the active range crosses in one snapshot; `omitted` counts the older rest. */
export type TranscriptBudget = { readonly items: number; readonly bytes: number };

export const TRANSCRIPT_BUDGET: TranscriptBudget = { items: 2_000, bytes: 8 * 2 ** 20 };

/** An entry's item with its wire size, kept per entry record so consecutive revisions share item objects. */
export type TranscriptMemo = WeakMap<EntryRecord, { readonly item: TranscriptItem; readonly bytes: number } | null>;

function blocksWithoutImageData<B extends TextContent | ImageContent>(blocks: readonly B[]): readonly B[] {
  let changed = false;
  const stripped = blocks.map((block) => {
    if (block.type !== "image" || block.data === "") return block;
    changed = true;
    return { ...block, data: "" };
  });
  return changed ? stripped : blocks;
}

function userWithoutImageData(message: UserMessage): UserMessage {
  if (typeof message.content === "string") return message;
  const content = blocksWithoutImageData(message.content);
  return content === message.content ? message : { ...message, content: [...content] };
}

function toolResultWithoutImageData(message: ToolResultMessage): ToolResultMessage {
  const content = blocksWithoutImageData(message.content);
  return content === message.content ? message : { ...message, content: [...content] };
}

function imageRefs(entry: EntryId, message: Message | undefined): readonly ImageRef[] {
  if (message?.role !== "user" || typeof message.content === "string") return [];
  return message.content.flatMap((block, index) =>
    block.type === "image" ? [{ at: { entry, message: 0, block: index }, mimeType: block.mimeType }] : [],
  );
}

function textOf(message: Message | undefined): string {
  if (message === undefined || message.role === "assistant") return "";
  if (typeof message.content === "string") return message.content;
  return message.content.flatMap((block) => (block.type === "text" ? [block.text] : [])).join("\n");
}

/**
 * What one stored entry looks like to a frontend; null for entries no frontend shows (`pi.system`, kicks).
 * `dimos.summary` and `dimos.branch_summary` become `summary` items, `dimos.origin` (an edited first message's empty
 * start) a `reset` marker; every image block is stripped of its `data`, and an input's images become refs.
 */
export function transcriptItem(entry: EntryRecord): TranscriptItem | null {
  const message = entry.model?.[0];
  if (DimosInputEntry.is(entry))
    return { kind: "input", entry: entry.id, input: entry.data, images: imageRefs(entry.id, message) };
  if (SummaryEntry.is(entry))
    return { kind: "summary", entry: entry.id, of: "up_to", from: entry.data.from, text: entry.data.text };
  if (BranchSummaryEntry.is(entry))
    return { kind: "summary", entry: entry.id, of: "branch", from: entry.data.from, text: entry.data.text };
  if (OriginEntry.is(entry)) return { kind: "reset", entry: entry.id };
  if (UserEntry.is(entry) && message?.role === "user")
    return { kind: "user", entry: entry.id, message: userWithoutImageData(message) };
  if (AssistantEntry.is(entry) && message?.role === "assistant") return { kind: "assistant", entry: entry.id, message };
  if (ToolResultEntry.is(entry) && message?.role === "toolResult")
    return { kind: "tool_result", entry: entry.id, result: toolResult(toolResultWithoutImageData(message)) };
  if (CompactionEntry.is(entry)) return { kind: "compaction", entry: entry.id, summary: textOf(message) };
  if (ResetEntry.is(entry)) return { kind: "reset", entry: entry.id };
  return null;
}

/** A stored tool result with its `details` parsed by `parseToolDetails`: the only place stored details are read. */
export function toolResult(message: ToolResultMessage): ToolResult {
  const { details, ...result } = message;
  return { ...result, details: parseToolDetails(message.toolName, details) };
}

function projected(
  entry: EntryRecord,
  memo: TranscriptMemo,
): { readonly item: TranscriptItem; readonly bytes: number } | null {
  const known = memo.get(entry);
  if (known !== undefined) return known;
  const item = transcriptItem(entry);
  const value = item === null ? null : { item, bytes: JSON.stringify(item).length };
  memo.set(entry, value);
  return value;
}

/**
 * The view's entries projected newest first into the budget, then returned oldest first. The first item that does not
 * fit stops the projection, so the items shown are contiguous; `omitted` counts the older items left out.
 */
export function projectTranscript(view: ConversationView, memo: TranscriptMemo, budget: TranscriptBudget): Transcript {
  const kept: TranscriptItem[] = [];
  let bytes = 0;
  let omitted = 0;
  let full = false;
  for (let index = view.entries.length - 1; index >= 0; index -= 1) {
    const entry = view.entries[index];
    if (entry === undefined) continue;
    const next = projected(entry, memo);
    if (next === null) continue;
    if (!full && kept.length < budget.items && bytes + next.bytes <= budget.bytes) {
      kept.push(next.item);
      bytes += next.bytes;
    } else {
      full = true;
      omitted += 1;
    }
  }
  kept.reverse();
  return { items: kept, omitted };
}

/** The stored image block at `at`; `not_found` when the entry, the message or the block is not an image. */
export function imageAt(entry: EntryRecord | undefined, at: ImageAddress): Outcome<ImageContent> {
  const message = entry?.model?.[at.message];
  const block = message === undefined || typeof message.content === "string" ? undefined : message.content[at.block];
  if (block?.type === "image") return done(block);
  return refused({ code: "not_found", what: "message", name: `${at.entry}/${at.message}/${at.block}` });
}

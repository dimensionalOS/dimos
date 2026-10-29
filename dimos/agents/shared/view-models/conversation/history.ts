import type { EntryId } from "@earendil-works/pi-durable";
import type { ImageAddress } from "#dimos/agents/shared/protocol/conversation/images.ts";
import type { Transcript, TranscriptItem } from "#dimos/agents/shared/protocol/conversation/state.ts";

/** A message Esc-then-arrows can select, and what can be done with it. */
export type Selectable = {
  readonly entry: EntryId;
  readonly item: TranscriptItem;
  /** User inputs only. Every edit forks, the first message included (its fork starts with a `dimos.origin` write). */
  readonly edit: "fork" | null;
  /** Nothing comes before the first visible entry, so it has nothing to summarize up to. */
  readonly summarizeUpTo: boolean;
  readonly summarizeFrom: boolean;
};

function isUserMessage(item: TranscriptItem): boolean {
  return item.kind === "user" || (item.kind === "input" && item.input.source.kind === "user");
}

/** User and assistant items, oldest first; tool results, compactions and resets are skipped by the arrows. */
export function selectables(transcript: Transcript): readonly Selectable[] {
  const found: Selectable[] = [];
  transcript.items.forEach((item, index) => {
    const user = isUserMessage(item);
    if (!user && item.kind !== "assistant") return;
    const first = index === 0 && transcript.omitted === 0;
    found.push({ entry: item.entry, item, edit: user ? "fork" : null, summarizeUpTo: !first, summarizeFrom: true });
  });
  return found;
}

/** Where the images an edit of `item` keeps are stored: a user input's image refs, or a user message's image blocks. */
export function editImages(item: TranscriptItem): readonly ImageAddress[] {
  if (item.kind === "input") return item.images.map((image) => image.at);
  if (item.kind !== "user" || typeof item.message.content === "string") return [];
  return item.message.content.flatMap((block, index) =>
    block.type === "image" ? [{ entry: item.entry, message: 0, block: index }] : [],
  );
}

/** The text an edit of `item` starts from: a user input's text, or a user message's text blocks joined. */
export function editText(item: TranscriptItem): string {
  if (item.kind === "input") return item.input.text;
  if (item.kind !== "user") return "";
  const content = item.message.content;
  return typeof content === "string"
    ? content
    : content.flatMap((block) => (block.type === "text" ? [block.text] : [])).join("\n");
}

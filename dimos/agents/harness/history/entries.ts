import type { ConversationId, Entry, EntryId } from "@earendil-works/pi-durable";
import { defineEntry } from "@earendil-works/pi-durable";

/** The message a history action was taken on, in the conversation it was taken from. */
export type Selected = { readonly conversation: ConversationId; readonly entry: EntryId };

export type SummaryData = {
  readonly from: Selected;
  /** The summary as the summarizer wrote it; the model gets it wrapped in the entry's user message. */
  readonly text: string;
};

/** A head at the selected message whose model message summarizes everything before it. */
export const SummaryEntry: Entry<SummaryData> = defineEntry<SummaryData>("dimos.summary");

/** pi's summary of the branch from the selected message to the tail, which the fork left behind. */
export const BranchSummaryEntry: Entry<SummaryData> = defineEntry<SummaryData>("dimos.branch_summary");

/** Starts an empty context in a fork of a conversation's first message, which has no entry before it to fork at. */
export const OriginEntry: Entry<{ readonly from: Selected }> = defineEntry<{ readonly from: Selected }>("dimos.origin");

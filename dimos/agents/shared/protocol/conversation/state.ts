import type { AssistantMessage, ToolResultMessage, UserMessage } from "@earendil-works/pi-ai";
import type { ConversationId, EntryId, SubmissionId } from "@earendil-works/pi-durable";
import type { DimosInput, UserRequestId } from "#dimos/agents/shared/protocol/conversation/input.ts";
import type { ImageRef } from "#dimos/agents/shared/protocol/conversation/images.ts";
import type { ModuleRow } from "#dimos/agents/shared/protocol/runtime/module.ts";
import type { StreamRef } from "#dimos/agents/shared/protocol/streams.ts";
import type { ToolDetails } from "#dimos/agents/shared/protocol/tools/details.ts";

/** A finished tool call with its stored `details` parsed once, on the gateway. */
export type ToolResult = Omit<ToolResultMessage, "details"> & { readonly details: ToolDetails };

/**
 * One stored entry as every frontend shows it. Image blocks inside `message` and `result` content arrive with empty
 * `data` (see images.ts); an input's images are refs.
 */
export type TranscriptItem =
  | {
      readonly kind: "input";
      readonly entry: EntryId;
      readonly input: DimosInput;
      readonly images: readonly ImageRef[];
    }
  | { readonly kind: "user"; readonly entry: EntryId; readonly message: UserMessage }
  | { readonly kind: "assistant"; readonly entry: EntryId; readonly message: AssistantMessage }
  | { readonly kind: "tool_result"; readonly entry: EntryId; readonly result: ToolResult }
  | { readonly kind: "compaction"; readonly entry: EntryId; readonly summary: string }
  /** `dimos.summary` (a head that summarizes everything before `from`) or `dimos.branch_summary` (a branch left behind). */
  | {
      readonly kind: "summary";
      readonly entry: EntryId;
      readonly of: "up_to" | "branch";
      readonly from: { readonly conversation: ConversationId; readonly entry: EntryId };
      readonly text: string;
    }
  | { readonly kind: "reset"; readonly entry: EntryId };

/**
 * The conversation's active range, projected entry by entry with image bytes stripped, newest items first into a budget
 * of 2,000 items and 8 MiB, so a snapshot on attach or reconnect is bounded. The projection is memoized by entry, so
 * consecutive values share items and Chord's diff of a replace is a splice of the new ones.
 */
export type Transcript = {
  readonly items: readonly TranscriptItem[];
  /** Older items of the active range the budget left out; scrollback past them is not offered. */
  readonly omitted: number;
};

/** How one input ended. Fed from the submission changes of commit publications, newest 50 kept. */
export type InputOutcome =
  | {
      readonly submission: SubmissionId;
      readonly request: UserRequestId | null;
      readonly status: "done";
      readonly answer: EntryId;
    }
  | {
      readonly submission: SubmissionId;
      readonly request: UserRequestId | null;
      readonly status: "unanswered";
      readonly reason: string;
    };

/** One stream this conversation shows, and why it is this conversation's. */
export type OwnedStream = {
  readonly ref: StreamRef;
  readonly via: { readonly kind: "module"; readonly module: ModuleRow["name"] };
};

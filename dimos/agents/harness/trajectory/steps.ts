import type { ConversationId, EntryId, EntryRecord } from "@earendil-works/pi-durable";
import type { EpochMs } from "#dimos/agents/shared/protocol/time.ts";
import type { Message } from "@earendil-works/pi-ai";
import { AssistantEntry, CompactionEntry, ToolResultEntry } from "@earendil-works/pi-durable";
import { DimosInputEntry } from "#dimos/agents/harness/inputs/source.ts";
import { epochMs } from "#dimos/agents/shared/protocol/time.ts";

type StepKind = "user" | "event" | "job" | "runtime" | "assistant" | "tool_result" | "compaction";

/** One thing the agent did or was told, published on HarnessRuntime's agent_step output as an AgentStep. */
export type Step = {
  readonly conversation: ConversationId;
  readonly entry: EntryId;
  readonly kind: StepKind;
  readonly at: EpochMs;
  /** A one-paragraph summary: the input text, the answer, or the tool name and arguments with the result's first lines. */
  readonly text: string;
};

const TEXT_CHARS = 2_000;

/** Null for entries a recording has no use for: system prompt entries, kicks and display-only writes. */
export function stepOf(entry: EntryRecord, fallbackAt: EpochMs): Step | null {
  const first = entry.model?.[0];
  const at = first === undefined ? fallbackAt : epochMs(first.timestamp);
  const step = (kind: StepKind, text: string): Step => ({
    conversation: entry.conversationId,
    entry: entry.id,
    kind,
    at,
    text: text.length > TEXT_CHARS ? `${text.slice(0, TEXT_CHARS)}…` : text,
  });
  if (DimosInputEntry.is(entry)) {
    const { text, change } = entry.data;
    const { kind } = entry.data.source;
    return step(
      kind === "background" ? "event" : kind,
      text !== "" || change === null ? text : `runtime change: ${change.summary}`,
    );
  }
  if (AssistantEntry.is(entry)) return step("assistant", (entry.model ?? []).map(messageText).join("\n"));
  if (ToolResultEntry.is(entry)) return step("tool_result", (entry.model ?? []).map(messageText).join("\n"));
  if (CompactionEntry.is(entry)) return step("compaction", (entry.model ?? []).map(messageText).join("\n"));
  return null;
}

function messageText(message: Message): string {
  switch (message.role) {
    case "user":
      return typeof message.content === "string" ? message.content : message.content.map(blockText).join("\n");
    case "assistant":
      return message.content
        .map((block) =>
          block.type === "text"
            ? block.text
            : block.type === "toolCall"
              ? `${block.name}(${JSON.stringify(block.arguments)})`
              : "",
        )
        .filter(Boolean)
        .join("\n");
    case "toolResult":
      return `${message.toolName}${message.isError ? " (error)" : ""}: ${message.content.map(blockText).join("\n")}`;
    case "system":
      return typeof message.content === "string" ? message.content : message.content.map(blockText).join("\n");
  }
}

function blockText(block: { readonly type: "text"; readonly text: string } | { readonly type: "image" }): string {
  return block.type === "text" ? block.text : "[image]";
}

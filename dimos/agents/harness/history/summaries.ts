import type { EntryRecord, ModelRef } from "@earendil-works/pi-durable";
import type { Message, Models, Usage, UserMessage } from "@earendil-works/pi-ai";
import type { SessionMessageEntry } from "@earendil-works/pi-coding-agent";
import {
  DEFAULT_COMPACTION_SETTINGS,
  convertToLlm,
  estimateTokens,
  generateBranchSummary,
  generateSummaryWithUsage,
} from "@earendil-works/pi-coding-agent";

/** A summary and the user message that carries it into a fork's context, or why the model call failed. */
export type Summary =
  | { readonly kind: "summary"; readonly text: string; readonly message: UserMessage; readonly usage: Usage | null }
  | { readonly kind: "failed"; readonly error: string };

/** pi-coding-agent's compaction summary of `messages`, wrapped as a compaction summary is. */
export async function summarizeMessages(
  messages: readonly Message[],
  model: ModelRef,
  models: Models,
  signal: AbortSignal,
): Promise<Summary> {
  const resolved = models.getModel(model.provider, model.modelId);
  if (resolved === undefined) return unavailable(model);
  try {
    const { text, usage } = await generateSummaryWithUsage(
      [...messages],
      resolved,
      DEFAULT_COMPACTION_SETTINGS.reserveTokens,
      undefined,
      undefined,
      signal,
      undefined,
      undefined,
      undefined,
      (resolvedModel, context, options) => models.streamSimple(resolvedModel, context, options),
    );
    const tokensBefore = messages.reduce((sum, message) => sum + estimateTokens(message), 0);
    return summary(
      text,
      convertToLlm([{ role: "compactionSummary", summary: text, tokensBefore, timestamp: Date.now() }]),
      usage,
    );
  } catch (error) {
    return { kind: "failed", error: error instanceof Error ? error.message : String(error) };
  }
}

/**
 * pi's branch summary of `entries`, the source's entries from the selected message to its tail, oldest first. pi drops
 * tool results itself and keeps the newest messages that fit the model's window.
 */
export async function summarizeBranch(
  entries: readonly EntryRecord[],
  model: ModelRef,
  models: Models,
  signal: AbortSignal,
): Promise<Summary> {
  const resolved = models.getModel(model.provider, model.modelId);
  if (resolved === undefined) return unavailable(model);
  const session = entries.flatMap((entry) =>
    (entry.model ?? []).map(
      (message, index): SessionMessageEntry => ({
        type: "message",
        id: `${entry.id}.${index}`,
        parentId: null,
        timestamp: new Date(message.timestamp).toISOString(),
        message,
      }),
    ),
  );
  const result = await generateBranchSummary(session, {
    model: resolved,
    signal,
    streamFn: (resolvedModel, context, options) => models.streamSimple(resolvedModel, context, options),
  });
  if (result.summary === undefined)
    return { kind: "failed", error: result.aborted === true ? "aborted" : (result.error ?? "no summary") };
  const message = convertToLlm([
    { role: "branchSummary", summary: result.summary, fromId: null, timestamp: Date.now() },
  ]);
  return summary(result.summary, message, result.usage ?? null);
}

function summary(text: string, messages: readonly Message[], usage: Usage | null): Summary {
  const [message] = messages;
  if (message?.role !== "user")
    throw new Error(`convertToLlm gave a ${message?.role ?? "missing"} message for a summary`);
  return { kind: "summary", text, message, usage };
}

function unavailable(model: ModelRef): Summary {
  return { kind: "failed", error: `model ${model.provider}/${model.modelId} is not available` };
}

import type { Accounts } from "#dimos/agents/shared/protocol/auth.ts";
import type { AgentState, InboxState, LiveState, ModelRef, UsageState } from "@earendil-works/pi-durable";
import type { Level } from "#dimos/agents/shared/view-models/levels.ts";
import type { ModelThinkingLevel } from "@earendil-works/pi-ai";
import type { Transcript, TranscriptItem } from "#dimos/agents/shared/protocol/conversation/state.ts";
import type { Settings } from "#dimos/agents/shared/protocol/settings.ts";
import { isRunning } from "#dimos/agents/shared/view-models/conversation/timeline.ts";
import { queuedRows } from "#dimos/agents/shared/view-models/conversation/queue.ts";

/** Requirement 7 as data: the conversation's model, its queue, cost and context left. */
export type Footer = {
  /** The stored model, else the gateway's default; null when neither is set. */
  readonly model: ModelRef | null;
  readonly thinking: ModelThinkingLevel | null;
  /** `pi.live.run` exists. Its start is the gateway's `ConversationRow.running.since` in `gateway.conversations`. */
  readonly running: boolean;
  readonly queued: { readonly steer: number; readonly followUp: number; readonly notices: number };
  readonly costUsd: number;
  /** From the newest answered assistant item's usage and the model's context window; null before the first answer. */
  readonly context: { readonly leftTokens: number; readonly fraction: number; readonly level: Level } | null;
};

export type FooterInput = {
  readonly transcript: Transcript;
  readonly live: LiveState;
  readonly queue: InboxState;
  readonly usage: UsageState;
  readonly agent: AgentState;
  readonly settings: Settings;
  readonly accounts: Accounts;
};

const CONTEXT_WARN_FRACTION = 0.25;

const CONTEXT_CRITICAL_FRACTION = 0.1;

/** A failed or aborted response keeps the usage it had when it stopped, usually zero, not the context it used. */
function newestAssistantTokens(items: readonly TranscriptItem[]): number | null {
  for (let index = items.length - 1; index >= 0; index -= 1) {
    const item = items[index];
    if (item.kind === "assistant" && item.message.stopReason !== "error" && item.message.stopReason !== "aborted")
      return item.message.usage.totalTokens;
  }
  return null;
}

function contextOf(input: FooterInput, model: ModelRef | null): Footer["context"] {
  if (model === null) return null;
  const used = newestAssistantTokens(input.transcript.items);
  if (used === null) return null;
  const choice = input.accounts.models.find(
    (candidate) => candidate.provider === model.provider && candidate.id === model.modelId,
  );
  if (choice === undefined || choice.contextWindow <= 0) return null;
  const leftTokens = Math.max(0, choice.contextWindow - used);
  const fraction = leftTokens / choice.contextWindow;
  const level: Level =
    fraction < CONTEXT_CRITICAL_FRACTION ? "critical" : fraction < CONTEXT_WARN_FRACTION ? "warn" : "ok";
  return { leftTokens, fraction, level };
}

/** Context left is warn under 25 % and critical under 10 %; cost sums `usage.models[*].cost.total`. */
export function footerOf(input: FooterInput): Footer {
  const model = input.agent.model !== undefined ? input.agent.model : input.settings.model;
  const thinking = input.agent.thinkingLevel !== undefined ? input.agent.thinkingLevel : input.settings.thinkingLevel;
  const queued = { steer: 0, followUp: 0, notices: 0 };
  for (const row of queuedRows(input.queue)) {
    if (row.from !== "you") queued.notices += 1;
    else if (row.mode === "steer") queued.steer += 1;
    else queued.followUp += 1;
  }
  let costUsd = 0;
  for (const usage of Object.values(input.usage.models)) costUsd += usage.cost.total;
  return {
    model,
    thinking,
    running: isRunning(input.live),
    queued,
    costUsd,
    context: contextOf(input, model),
  };
}

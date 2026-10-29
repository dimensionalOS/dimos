import type { ConversationId, Harness, SubmissionId } from "@earendil-works/pi-durable";
import type { InputGate } from "#dimos/agents/harness/inputs/gate.ts";
import { BACKGROUND_CONTEXT } from "@earendil-works/chord/context";

type QueueKickerOptions = {
  readonly harness: Harness;
  readonly inputs: InputGate;
  /** Delay before the kick that follows the n-th consecutive failed run of one conversation. */
  readonly backoffMs: (attempt: number) => number;
  readonly report: (error: unknown) => void;
};

/**
 * pi-durable leaves inputs queued behind a failed run until the next input arrives. After a run fails with
 * `model_error`, this waits, and if the conversation's inbox still holds inputs it submits a write that places them.
 */
export class QueueKicker {
  private readonly options: QueueKickerOptions;
  private readonly failures = new Map<ConversationId, number>();
  private readonly timers = new Map<ConversationId, NodeJS.Timeout>();
  private unsubscribe: (() => void) | null = null;

  constructor(options: QueueKickerOptions) {
    this.options = options;
  }

  start(): void {
    this.unsubscribe = this.options.harness.subscribeCommits((publication) => {
      // A failed run settles all its inputs in one commit, so one publication is one failure per conversation.
      const failed = new Map<ConversationId, SubmissionId>();
      for (const change of publication.changes) {
        if (change.type !== "submission" || change.value.type !== "input") continue;
        const { conversationId, status, id } = change.value;
        if (status === "done") this.failures.delete(conversationId);
        else if (status === "unanswered" && change.value.reason === "model_error") failed.set(conversationId, id);
      }
      for (const [conversation, id] of failed) this.schedule(conversation, id);
    });
  }

  stop(): void {
    this.unsubscribe?.();
    this.unsubscribe = null;
    for (const timer of this.timers.values()) clearTimeout(timer);
    this.timers.clear();
  }

  private schedule(conversation: ConversationId, failed: SubmissionId): void {
    const attempt = this.failures.get(conversation) ?? 0;
    this.failures.set(conversation, attempt + 1);
    const earlier = this.timers.get(conversation);
    if (earlier !== undefined) clearTimeout(earlier);
    const timer = setTimeout(() => {
      this.timers.delete(conversation);
      this.options.inputs.kick(conversation, failed, BACKGROUND_CONTEXT).catch(this.options.report);
    }, this.options.backoffMs(attempt));
    timer.unref();
    this.timers.set(conversation, timer);
  }
}

import type { ActionRecord } from "../../shared/view-models/actions.ts";
import type { Outcome, Refusal } from "../../shared/protocol/outcome.ts";
import { Status } from "../../shared/protocol/status.ts";
import { epochMs } from "../../shared/protocol/time.ts";
import { refusalOfFailure } from "../../sdk/failures.ts";
import { refused } from "../../shared/protocol/outcome.ts";

type Context = Partial<Pick<ActionRecord, "action" | "conversation" | "sending">>;

/** Presentation history of promises the UI already started. Never invokes or retries an RPC. */
export class Feedback extends Status<readonly ActionRecord[]> {
  private seq = 0;
  private readonly ended: Promise<never>;
  constructor(ended: Promise<never>) {
    super([], (error) => {
      throw error;
    });
    this.ended = ended;
  }

  async observe<T>(text: string, work: Promise<Outcome<T>>, context: Context = {}): Promise<Outcome<T>> {
    const record: ActionRecord = {
      seq: ++this.seq,
      text,
      action: context.action ?? "action",
      conversation: context.conversation ?? null,
      sending: context.sending ?? null,
      status: "pending",
      refusal: null,
      startedAt: epochMs(Date.now()),
      endedAt: null,
    };
    this.set([...this.value, record].slice(-50));
    let outcome: Outcome<T>;
    try {
      outcome = await Promise.race([work, this.ended]);
    } catch (error) {
      const refusal = refusalOfFailure(error, record.startedAt);
      outcome = refused(refusal.code === "viewing" ? { ...refusal, conversation: record.conversation } : refusal);
    }
    const refusal = outcome.ok ? null : outcome.refusal;
    this.set(
      this.value.map((entry) =>
        entry.seq === record.seq
          ? {
              ...record,
              refusal,
              endedAt: epochMs(Date.now()),
              status: refusal === null ? "done" : refusal.code === "unknown_outcome" ? "unknown" : "refused",
            }
          : entry,
      ),
    );
    return outcome;
  }

  refuse(text: string, refusal: Refusal): void {
    void this.observe(text, Promise.resolve(refused(refusal)));
  }
}

import type { Context } from "@earendil-works/chord/context";
import type {
  ConversationId,
  ConversationRecord,
  Cursor,
  EntryId,
  EntryRecord,
  Harness,
} from "@earendil-works/pi-durable";
import type { DimosConnection } from "../dimos/connection.ts";
import type { RuntimeClient } from "../dimos/client.ts";
import type { Step } from "./steps.ts";
import { DimosUnavailable } from "../dimos/connection.ts";
import { defineDoc } from "@earendil-works/pi-durable";
import { epochMs } from "../../shared/protocol/time.ts";
import { stepOf } from "./steps.ts";

/** The newest entry sent, per conversation id. Written only by TrajectorySender. */
const TrajectoryCursorDoc = defineDoc<{ sent: { [conversation: string]: number } }>({
  kind: "dimos.trajectory_cursor",
  version: 1,
  scope: "session",
  initial: () => ({ sent: {} }),
});

type TrajectorySenderOptions = {
  readonly harness: Harness;
  readonly connection: DimosConnection;
  readonly runtime: RuntimeClient;
  /** Entries read from the conversation store at once. */
  readonly batch: number;
  readonly report: (error: unknown) => void;
};

/** Publishes committed entries through HarnessRuntime, then advances the conversation cursor. */
export class TrajectorySender {
  private readonly options: TrajectorySenderOptions;
  private wanted = false;
  private draining: Promise<void> | null = null;
  private context: Context | null = null;
  private readonly unsubscribers: (() => void)[] = [];
  private stopped = false;

  constructor(options: TrajectorySenderOptions) {
    this.options = options;
  }

  /** Wakes on commits that append entries and when the link comes up; idle while DimOS is unreachable. */
  start(context: Context): void {
    this.context = context;
    this.unsubscribers.push(
      this.options.harness.subscribeCommits((publication) => {
        if (publication.changes.some((change) => change.type === "entry")) this.wake();
      }),
    );
    this.unsubscribers.push(
      this.options.connection.status.subscribe((status) => {
        if (status.state === "up") this.wake();
      }),
    );
  }

  wake(): void {
    if (this.stopped || this.context === null) return;
    this.wanted = true;
    if (this.draining === null) {
      const context = this.context;
      this.draining = this.loop(context).finally(() => {
        this.draining = null;
      });
    }
  }

  async stop(): Promise<void> {
    this.stopped = true;
    for (const unsubscribe of this.unsubscribers.splice(0)) unsubscribe();
    await this.draining;
  }

  private async loop(context: Context): Promise<void> {
    while (this.wanted && !this.stopped) {
      this.wanted = false;
      try {
        await this.drain(context);
      } catch (error) {
        if (!(error instanceof DimosUnavailable)) this.options.report(error);
      }
    }
  }

  private async drain(context: Context): Promise<void> {
    const cursor = (await this.options.harness.snapshot(TrajectoryCursorDoc, context))?.sent ?? {};
    for (const record of await this.conversations(context)) {
      const conversation = record.id;
      // A fork's entries through its fork point are its parent's, sent under the parent's cursor.
      const inherited = record.parent === undefined ? 0 : record.parent.at;
      let sent = cursor[String(conversation)] ?? inherited;
      while (!this.stopped) {
        const entries = await this.entriesAfter(conversation, sent, context);
        if (entries.length === 0) break;
        const now = epochMs(Date.now());
        const steps: Step[] = [];
        for (const entry of entries) {
          const step = stepOf(entry, now);
          if (step !== null) steps.push(step);
        }
        if (steps.length > 0) await this.options.runtime.publishSteps(steps);
        sent = entries[entries.length - 1]?.id ?? sent;
        await this.commitCursor(conversation, sent, context);
      }
    }
  }

  private async conversations(context: Context): Promise<ConversationRecord[]> {
    const records: ConversationRecord[] = [];
    let cursor: Cursor | undefined;
    for (;;) {
      const page = await this.options.harness.commit(
        (tx) => tx.scanConversations({ order: "ascending" }, 100, cursor),
        context,
      );
      records.push(...page.items);
      if (page.next === undefined) return records;
      cursor = page.next;
    }
  }

  private entriesAfter(conversation: ConversationId, sent: number, context: Context): Promise<readonly EntryRecord[]> {
    return this.options.harness.commit(
      async (tx) =>
        (
          await tx.scanEntries(
            { conversationId: conversation, minEntryId: (sent + 1) as EntryId, order: "ascending" },
            this.options.batch,
          )
        ).items,
      context,
    );
  }

  private commitCursor(conversation: ConversationId, sent: number, context: Context): Promise<void> {
    return this.options.harness.commit(async (tx) => {
      const doc = await tx.doc(TrajectoryCursorDoc);
      doc.sent[String(conversation)] = sent;
    }, context);
  }
}

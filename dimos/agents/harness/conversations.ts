import type { Context } from "@earendil-works/chord/context";
import type { ConversationId, EntryId, Harness, ModelRef } from "@earendil-works/pi-durable";
import type { Facts } from "../shared/protocol/runtime/context.ts";
import type { ModelThinkingLevel } from "@earendil-works/pi-ai";
import type { RuntimeClient } from "./dimos/client.ts";
import { defineDoc } from "@earendil-works/pi-durable";
import { writeBaseline } from "./context/baseline.ts";

/** Read at each creation, so a changed default model applies to the next conversation. */
export type ConversationDefaults = {
  readonly model: ModelRef | null;
  readonly thinkingLevel: ModelThinkingLevel | null;
  readonly workspace: string;
};

type ConversationsOptions = {
  readonly harness: Harness;
  readonly runtime: RuntimeClient;
  /** The gateway's own facts, its jobs and the running systems, which the baseline records beside the snapshot. */
  readonly gatewayFacts: (context: Context) => Promise<Facts>;
  readonly defaults: () => ConversationDefaults;
  readonly snapshotTimeoutMs: number;
};

/** Each create request's conversation, written in the commit that creates it. */
const CreatedDoc = defineDoc<{ created: { [request: string]: number } }>({
  kind: "dimos.created",
  version: 1,
  scope: "session",
  initial: () => ({ created: {} }),
});

/** The seen document's kind, which the gateway's conversation fold watches for in commit publications. */
export const SEEN_DOC_KIND = "dimos.seen";

/** Per conversation, the newest entry anyone was shown. */
const SeenDoc = defineDoc<{ seen: { [conversation: string]: number } }>({
  kind: SEEN_DOC_KIND,
  version: 1,
  scope: "session",
  initial: () => ({ seen: {} }),
});

/**
 * Creating conversations and the two session documents the UI needs beside pi-durable's records: `dimos.created`
 * (request id to the conversation it made, so a retried create or edit makes one) and `dimos.seen` (per conversation,
 * the newest entry anyone was shown, from which the switcher counts unread notices). The gateway folds switcher rows
 * from commit publications instead of listing here.
 */
export class Conversations {
  private readonly options: ConversationsOptions;
  private readonly inFlight = new Map<string, Promise<ConversationId>>();

  constructor(options: ConversationsOptions) {
    this.options = options;
  }

  /**
   * Takes the runtime snapshot and the gateway's facts first, then creates the conversation with its agent and
   * baseline in one commit, recorded under `request`; a repeated request returns the conversation it made, and one in
   * flight is joined.
   */
  start(cwd: string | null, request: string, context: Context): Promise<ConversationId> {
    const flying = this.inFlight.get(request);
    if (flying !== undefined) return flying;
    const made = (async () => {
      const done = await this.created(request, context);
      if (done !== null) return done;
      const [system, gateway] = await Promise.all([this.snapshot(), this.options.gatewayFacts(context)]);
      const { model, thinkingLevel, workspace } = this.options.defaults();
      const conversation = await this.options.harness.createConversation(
        {
          ownership: { kind: "ownerless" },
          agent: { model, thinkingLevel, cwd: cwd ?? workspace },
          init: async (tx, id) => {
            await writeBaseline(tx, id, { ...system, ...gateway });
            const doc = await tx.doc(CreatedDoc);
            doc.created[request] = id;
          },
        },
        context,
      );
      return conversation.id;
    })().finally(() => this.inFlight.delete(request));
    this.inFlight.set(request, made);
    return made;
  }

  /** The conversation a request made, if it finished. */
  async created(request: string, context: Context): Promise<ConversationId | null> {
    const doc = await this.options.harness.snapshot(CreatedDoc, context);
    const conversation = doc?.created[request];
    return conversation === undefined ? null : (conversation as ConversationId);
  }

  /** Moves the seen cursor forward only; a commit only when it moves. */
  async markSeen(conversation: ConversationId, entry: EntryId, context: Context): Promise<void> {
    const key = String(conversation);
    const stored = (await this.options.harness.snapshot(SeenDoc, context))?.seen[key];
    if (stored !== undefined && stored >= entry) return;
    await this.options.harness.commit(async (tx) => {
      const doc = await tx.doc(SeenDoc);
      const current = doc.seen[key];
      if (current === undefined || current < entry) doc.seen[key] = entry;
    }, context);
  }

  /** The newest entry anyone was shown, per conversation. */
  async seen(context: Context): Promise<ReadonlyMap<ConversationId, EntryId>> {
    const doc = await this.options.harness.snapshot(SeenDoc, context);
    return new Map(
      Object.entries(doc?.seen ?? {}).map(([conversation, entry]) => [
        Number(conversation) as ConversationId,
        entry as EntryId,
      ]),
    );
  }

  /** The system's facts, none when DimOS does not answer in time. */
  private async snapshot(): Promise<Facts> {
    const abort = new AbortController();
    const timer = setTimeout(() => abort.abort(), this.options.snapshotTimeoutMs);
    try {
      return await this.options.runtime.snapshot(abort.signal);
    } catch {
      return {};
    } finally {
      clearTimeout(timer);
    }
  }
}

import type { Context } from "@earendil-works/chord/context";
import type {
  Conversation,
  ConversationId,
  Cursor,
  EntryId,
  EntryRecord,
  Harness,
  ModelRef,
  Page,
} from "@earendil-works/pi-durable";
import type { ImageContent, Models } from "@earendil-works/pi-ai";
import type { InputGate, WriteResult } from "../inputs/gate.ts";
import type { Selected } from "./entries.ts";
import type { UserRequestId } from "../../shared/protocol/conversation/input.ts";
import { BranchSummaryEntry, OriginEntry, SummaryEntry } from "./entries.ts";
import { DimosInputEntry } from "../inputs/source.ts";
import { UserEntry, defineDocFamily } from "@earendil-works/pi-durable";
import { summarizeBranch, summarizeMessages } from "./summaries.ts";

/**
 * What an edit or a summary made. `new` is a fork of a conversation's first message, which has no history before it:
 * the fork starts an empty context there.
 */
export type Branch =
  | { readonly kind: "fork" | "new"; readonly conversation: ConversationId }
  | { readonly kind: "not_found"; readonly what: "conversation" | "message" }
  | { readonly kind: "not_a_user_message" }
  /** Summarize up to a conversation's first message: nothing comes before it. */
  | { readonly kind: "nothing_before" }
  /** Summarize up to a message above the newest compaction or summary head. */
  | { readonly kind: "stale" }
  | { readonly kind: "model_failed"; readonly error: string };

type Made = Extract<Branch, { readonly kind: "fork" | "new" }>;

/** The fork a request made, written in the commit that forks; `complete` once the request's last step landed. */
type BranchRecord = {
  readonly conversation: ConversationId;
  readonly kind: Made["kind"];
  readonly complete: boolean;
};

const BranchRecordDoc = defineDocFamily<BranchRecord, BranchRecord>({
  kind: "dimos.branch",
  version: 1,
  scope: "session",
  family: true,
  initial: (record) => record,
});

type Located = {
  readonly kind: "located";
  readonly source: Conversation;
  readonly from: Selected;
  readonly selected: EntryRecord;
  /** The visible entry just before the selected one, where a fork that leaves it out starts; null for the first. */
  readonly before: EntryRecord | null;
};

/**
 * Requirement 13's actions, each idempotent by its request: the fork is recorded under the request in the commit that
 * makes it, its writes and input carry request ids, and a request in flight is joined. The source is never changed.
 * The summary text is made before anything is forked, so a failed model call leaves nothing behind.
 */
export class ConversationHistory {
  private readonly harness: Harness;
  private readonly inputs: InputGate;
  private readonly models: Models;
  private readonly inFlight = new Map<string, Promise<Branch>>();

  constructor(harness: Harness, inputs: InputGate, models: Models) {
    this.harness = harness;
    this.inputs = inputs;
    this.models = models;
  }

  /** Forks just before `message` and sends `text` there as the user's input `request`. */
  edit(
    source: ConversationId,
    message: EntryId,
    text: string,
    images: readonly ImageContent[],
    request: UserRequestId,
    context: Context,
  ): Promise<Branch> {
    return this.once(request, source, message, context, async (located) => {
      if (!isUserInput(located.selected)) return { kind: "not_a_user_message" };
      const fork = await this.forkBefore(request, located, context);
      if (located.before === null) {
        const origin = await this.inputs.write(
          fork.conversation,
          { entry: OriginEntry, head: "self", model: [], data: { from: located.from } },
          `origin:${request}`,
          context,
        );
        if (origin.kind === "unknown_conversation") return { kind: "not_found", what: "conversation" };
      }
      const sent = await this.inputs.submit(
        fork.conversation,
        { source: { kind: "user", request }, text, images, render: null, whenBusy: "followUp" },
        context,
      );
      if (sent.kind === "unknown_conversation") return { kind: "not_found", what: "conversation" };
      return this.complete(request, fork, context);
    });
  }

  /**
   * Forks at the tail and writes a head at `message` whose model message is pi's compaction summary of the context
   * before it, so the fork continues from `message` with everything earlier summarized.
   */
  summarizeUpTo(source: ConversationId, message: EntryId, request: string, context: Context): Promise<Branch> {
    return this.once(request, source, message, context, async (located) => {
      if (located.before === null) return { kind: "nothing_before" };
      const marker = await located.source.commit((tx) => tx.latestHeadMarker(located.source.id), context);
      if (marker !== undefined && message < marker.head) return { kind: "stale" };
      const model = await this.modelOf(located.source, context);
      if (model === null) return { kind: "model_failed", error: "the conversation has no model" };
      const before = await located.source.context(context, { at: located.before.id });
      const summary = await summarizeMessages(before.messages, model, this.models, signalOf(context));
      if (summary.kind === "failed") return { kind: "model_failed", error: summary.error };
      const fork = await this.forkAt(request, located.source, await tailOf(located.source, context), "fork", context);
      const written = await this.inputs.write(
        fork.conversation,
        {
          entry: SummaryEntry,
          head: message,
          model: [summary.message],
          data: { from: located.from, text: summary.text },
        },
        `summary:${request}`,
        context,
      );
      return this.landed(written, request, fork, context);
    });
  }

  /** pi's branch summary: forks just before `message` and writes the summary of `message` through the tail there. */
  summarizeFrom(source: ConversationId, message: EntryId, request: string, context: Context): Promise<Branch> {
    return this.once(request, source, message, context, async (located) => {
      const model = await this.modelOf(located.source, context);
      if (model === null) return { kind: "model_failed", error: "the conversation has no model" };
      const branch = await entriesFrom(located.source, message, await tailOf(located.source, context), context);
      const summary = await summarizeBranch(branch, model, this.models, signalOf(context));
      if (summary.kind === "failed") return { kind: "model_failed", error: summary.error };
      const fork = await this.forkBefore(request, located, context);
      const written = await this.inputs.write(
        fork.conversation,
        {
          entry: BranchSummaryEntry,
          head: located.before === null ? "self" : null,
          model: [summary.message],
          data: { from: located.from, text: summary.text },
        },
        `branch-summary:${request}`,
        context,
      );
      return this.landed(written, request, fork, context);
    });
  }

  private once(
    request: string,
    source: ConversationId,
    message: EntryId,
    context: Context,
    act: (located: Located) => Promise<Branch>,
  ): Promise<Branch> {
    const running = this.inFlight.get(request);
    if (running !== undefined) return running;
    const started = this.run(request, source, message, context, act).finally(() => this.inFlight.delete(request));
    this.inFlight.set(request, started);
    return started;
  }

  private async run(
    request: string,
    source: ConversationId,
    message: EntryId,
    context: Context,
    act: (located: Located) => Promise<Branch>,
  ): Promise<Branch> {
    const record = await this.harness.snapshot(BranchRecordDoc, request, context);
    if (record?.complete === true) return { kind: record.kind, conversation: record.conversation };
    const conversation = await this.harness.conversation(source, context);
    if (conversation === undefined) return { kind: "not_found", what: "conversation" };
    const { items } = await conversation.entries({ order: "descending", maxEntryId: message }, 2, undefined, context);
    const selected = items.at(0);
    if (selected?.id !== message) return { kind: "not_found", what: "message" };
    return act({
      kind: "located",
      source: conversation,
      from: { conversation: source, entry: message },
      selected,
      before: items.at(1) ?? null,
    });
  }

  /** A fork without the selected message, or for a first message a fork of it whose context starts over. */
  private forkBefore(request: string, located: Located, context: Context): Promise<Made> {
    return located.before === null
      ? this.forkAt(request, located.source, located.selected.id, "new", context)
      : this.forkAt(request, located.source, located.before.id, "fork", context);
  }

  private async forkAt(
    request: string,
    source: Conversation,
    at: EntryId,
    kind: Made["kind"],
    context: Context,
  ): Promise<Made> {
    const record = await this.harness.snapshot(BranchRecordDoc, request, context);
    if (record !== undefined) return { kind: record.kind, conversation: record.conversation };
    const fork = await source.fork(
      at,
      {
        ownership: { kind: "ownerless" },
        init: async (tx, conversation) => {
          await tx.doc(BranchRecordDoc, request, { conversation, kind, complete: false });
        },
      },
      context,
    );
    return { kind, conversation: fork.id };
  }

  private async landed(written: WriteResult, request: string, fork: Made, context: Context): Promise<Branch> {
    switch (written.kind) {
      case "done":
        return this.complete(request, fork, context);
      case "stale":
        return { kind: "stale" };
      case "unknown_conversation":
        return { kind: "not_found", what: "conversation" };
    }
  }

  private async complete(request: string, fork: Made, context: Context): Promise<Made> {
    await this.harness.commit(async (tx) => {
      const record = await tx.doc(BranchRecordDoc, request, { ...fork, complete: false });
      record.complete = true;
    }, context);
    return fork;
  }

  /** Summaries run on the source's resolved model. */
  private async modelOf(source: Conversation, context: Context): Promise<ModelRef | null> {
    return (await source.agent(context)).model ?? null;
  }
}

function isUserInput(entry: EntryRecord): boolean {
  return UserEntry.is(entry) || (DimosInputEntry.is(entry) && entry.data.source.kind === "user");
}

/** The newest entry visible from `conversation`. */
async function tailOf(conversation: Conversation, context: Context): Promise<EntryId> {
  const [newest] = (await conversation.entries({ order: "descending" }, 1, undefined, context)).items;
  return newest.id;
}

/** Visible entries from `first` through `last`, oldest first. */
async function entriesFrom(
  conversation: Conversation,
  first: EntryId,
  last: EntryId,
  context: Context,
): Promise<readonly EntryRecord[]> {
  const entries: EntryRecord[] = [];
  let cursor: Cursor | undefined = undefined;
  do {
    const page: Page<EntryRecord, Cursor> = await conversation.entries(
      { order: "ascending", minEntryId: first, maxEntryId: last },
      200,
      cursor,
      context,
    );
    entries.push(...page.items);
    cursor = page.next;
  } while (cursor !== undefined);
  return entries;
}

/** The model calls stop when the caller does; a context without a signal never cancels them. */
function signalOf(context: Context): AbortSignal {
  return context.abortSignal ?? new AbortController().signal;
}

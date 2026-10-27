import type { Context } from "@earendil-works/chord/context";
import type {
  ConversationId,
  Entry,
  EntryId,
  Harness,
  InboxItem,
  SubmissionId,
  TaskId,
} from "@earendil-works/pi-durable";
import type { DimosInput, InputSource } from "../../shared/protocol/conversation/input.ts";
import type { ImageContent, TextContent, UserMessage } from "@earendil-works/pi-ai";
import type { JsonValue } from "@earendil-works/chord";
import type { OutputLimiter } from "../limit/limit.ts";
import type { QueuedText } from "../../shared/protocol/conversation/queue.ts";
import type { RenderDetails } from "../../shared/protocol/media.ts";
import type { RuntimeObserver } from "../context/observer.ts";
import { ConversationBusy, InboxDoc, LiveDoc, defineDoc } from "@earendil-works/pi-durable";
import { DimosInputEntry, requestIdFor } from "./source.ts";
import { changeMessage } from "../context/messages.ts";
import { readFile } from "node:fs/promises";

export type InputDraft = {
  readonly source: InputSource;
  readonly text: string;
  readonly images: readonly ImageContent[];
  readonly render: RenderDetails | null;
  readonly whenBusy: "steer" | "followUp" | "reject";
};

/** A write that starts no run. `head` starts the active context at that entry, or at the write itself for `"self"`. */
export type WriteDraft<D extends JsonValue> = {
  readonly entry: Entry<D>;
  readonly head: EntryId | "self" | null;
  readonly model: readonly UserMessage[];
  readonly data: D;
};

export type WriteResult =
  | { readonly kind: "done" }
  /** `head` is before the conversation's active range. */
  | { readonly kind: "stale" }
  | { readonly kind: "unknown_conversation" };

export type SubmitResult =
  | { readonly kind: "submitted"; readonly submission: SubmissionId; readonly repeated: boolean }
  | { readonly kind: "unknown_conversation" }
  /** `whenBusy: "reject"` while a run is in progress. */
  | { readonly kind: "busy" }
  /** A runtime steer with no change to report. */
  | { readonly kind: "nothing_to_say" };

/** A withdrawn input; `fullText` is the spill file with its whole text when the input was cut. */
type Taken = { readonly mode: QueuedText["mode"]; readonly text: string; readonly fullText: string | null };

/** What each withdraw request took and the run it named to abort, written in the commit that withdraws. */
const WithdrawnDoc = defineDoc<{ requests: { [request: string]: { taken: Taken[]; aborts: TaskId | null } } }>({
  kind: "dimos.withdrawn",
  version: 1,
  scope: "session",
  initial: () => ({ requests: {} }),
});

export type Withdrawn = {
  readonly texts: readonly QueuedText[];
  /** The run live when the request first withdrew with `"abort"`, for the caller to abort; null otherwise. */
  readonly aborts: TaskId | null;
};

type InputGateOptions = {
  readonly harness: Harness;
  readonly observer: RuntimeObserver;
  /** The `max_chars` cut tool results get, at its default. */
  readonly limiter: OutputLimiter;
};

/** The one door into a conversation: each input becomes one `dimos.input` entry (runtime change, text cut to the limit, images) with a request id from its source. */
export class InputGate {
  private readonly options: InputGateOptions;

  constructor(options: InputGateOptions) {
    this.options = options;
  }

  async submit(conversation: ConversationId, draft: InputDraft, context: Context): Promise<SubmitResult> {
    const handle = await this.options.harness.conversation(conversation, context);
    if (handle === undefined) return { kind: "unknown_conversation" };
    const requestId = requestIdFor(draft.source);
    const prior = await handle.commit((tx) => tx.submissionByRequest(conversation, requestId), context);
    if (prior !== undefined) return { kind: "submitted", submission: prior.id, repeated: true };
    const { change, told } = await this.options.observer.changeFor(handle, context);
    if (draft.source.kind === "runtime" && change === null) return { kind: "nothing_to_say" };
    const limited = await this.options.limiter.limit(draft.text, {
      conversation,
      requested: null,
      spillName: `${conversation}/${requestId}`,
      fullTextAt: null,
      argument: null,
    });
    const content: (TextContent | ImageContent)[] = [];
    if (change !== null) content.push({ type: "text", text: changeMessage(change, told) });
    if (limited.text !== "") content.push({ type: "text", text: limited.text });
    content.push(...draft.images);
    const data: DimosInput = {
      source: draft.source,
      text: limited.text,
      change,
      render: draft.render,
      limit: limited.record,
    };
    try {
      const submission = await handle.submit(
        { type: "input", content, entry: { kind: DimosInputEntry.kind, data }, whenBusy: draft.whenBusy, requestId },
        context,
      );
      return { kind: "submitted", submission: submission.id, repeated: false };
    } catch (error) {
      if (error instanceof ConversationBusy) return { kind: "busy" };
      throw error;
    }
  }

  /**
   * The user's queued inputs, removed from `pi.inbox` and settled `unanswered/withdrawn` in one commit so no boundary
   * places some of them meanwhile, with their whole texts in queue order. Queued notices and writes stay. With
   * `"abort"` the same commit names the live run. It records both under `request`; a repeated `request` takes nothing
   * and answers with that record, so a retry neither takes inputs queued since nor names a run started since.
   */
  async withdrawUser(
    conversation: ConversationId,
    request: string,
    run: "keep" | "abort",
    context: Context,
  ): Promise<Withdrawn> {
    const handle = await this.options.harness.conversation(conversation, context);
    if (handle === undefined) return { texts: [], aborts: null };
    const record = await handle.commit(async (tx) => {
      const withdrawn = await tx.doc(WithdrawnDoc);
      const kept = withdrawn.requests[request];
      if (kept !== undefined)
        return { taken: kept.taken.map(({ mode, text, fullText }) => ({ mode, text, fullText })), aborts: kept.aborts };
      const aborts = run === "abort" ? ((await tx.doc(LiveDoc, conversation)).run?.taskId ?? null) : null;
      const inbox = await tx.doc(InboxDoc, conversation);
      const taken: Taken[] = [];
      for (let index = inbox.items.length - 1; index >= 0; index -= 1) {
        const item = inbox.items[index];
        const input = userInputOf(item);
        if (item.mode === "write" || input === null) continue;
        taken.unshift({
          mode: item.mode,
          text: input.text,
          fullText: input.limit === null ? null : input.limit.fullText,
        });
        tx.settleSubmission(item.id, { status: "unanswered", reason: "withdrawn" });
        inbox.items.splice(index, 1);
      }
      withdrawn.requests[request] = { taken, aborts };
      return { taken, aborts };
    }, context);
    const texts = await Promise.all(
      record.taken.map(async ({ mode, text, fullText }) => ({
        mode,
        text: fullText === null ? text : await readFile(fullText, "utf8"),
      })),
    );
    return { texts, aborts: record.aborts };
  }

  /** Admits a write and waits for it to be placed; a repeated `requestId` answers with the first write's settlement. */
  async write<D extends JsonValue>(
    conversation: ConversationId,
    draft: WriteDraft<D>,
    requestId: string,
    context: Context,
  ): Promise<WriteResult> {
    const handle = await this.options.harness.conversation(conversation, context);
    if (handle === undefined) return { kind: "unknown_conversation" };
    const submission = await handle.submit(
      {
        type: "write",
        requestId,
        entry: {
          kind: draft.entry.kind,
          ...(draft.head === null ? {} : { head: draft.head }),
          model: draft.model,
          data: draft.data,
        },
      },
      context,
    );
    const settled = await submission.wait(context);
    if (settled.status === "done") return { kind: "done" };
    if (settled.reason === "stale") return { kind: "stale" };
    throw new Error(`Write ${requestId} to conversation ${conversation} was not placed: ${settled.reason}`);
  }

  /**
   * Places inputs left queued behind the failed submission `after`: a write `dimos.kick` with an empty `model`,
   * request id `kick:<after>`. Nothing when the conversation's inbox is empty.
   */
  async kick(conversation: ConversationId, after: SubmissionId, context: Context): Promise<void> {
    const handle = await this.options.harness.conversation(conversation, context);
    if (handle === undefined) return;
    const inspection = await this.options.harness.inspect(context);
    if (!inspection.submissions.some((record) => record.conversationId === conversation && record.status === "queued"))
      return;
    await handle.submit(
      { type: "write", entry: { kind: "dimos.kick", model: [], data: { after } }, requestId: `kick:${after}` },
      context,
    );
  }
}

/** The data of a queued user input. Only this gate writes `dimos.input`, so its data is a DimosInput. */
function userInputOf(item: InboxItem): DimosInput | null {
  if (item.mode === "write" || item.entry?.kind !== DimosInputEntry.kind) return null;
  const input = item.entry.data as DimosInput;
  return input.source.kind === "user" ? input : null;
}

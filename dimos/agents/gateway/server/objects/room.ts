import type {
  CommitPublication,
  CommonDocDefinition,
  ConversationId,
  ConversationView,
  ConversationWatch,
  Cursor,
  JsonObject,
  Storage,
  SubmissionRecord,
} from "@earendil-works/pi-durable";
import type { Context } from "@earendil-works/chord/context";
import type { InputOutcome, OwnedStream } from "../../../shared/protocol/conversation/state.ts";
import type { JobRow } from "../../../shared/protocol/jobs/state.ts";
import type { ModuleRow } from "../../../shared/protocol/runtime/module.ts";
import type { ServeObject } from "./base.ts";
import type { StreamRef } from "../../../shared/protocol/streams.ts";
import type { Subscribable } from "@earendil-works/chord";
import type { TranscriptMemo } from "./lib/items.ts";
import type { UserRequestId } from "../../../shared/protocol/conversation/input.ts";
import { AgentDoc, InboxDoc, LiveDoc, UsageDoc } from "@earendil-works/pi-durable";
import { BACKGROUND_CONTEXT, withAbortSignal } from "@earendil-works/chord/context";
import { ConversationIdSchema } from "../../../shared/protocol/args.ts";
import { ConversationState } from "../../../shared/protocol/conversation/contract.ts";
import { GatewayStore } from "./store.ts";
import { TRANSCRIPT_BUDGET, projectTranscript } from "./lib/items.ts";
import { Type } from "typebox";
import { commitEvents } from "./lib/commits.ts";
import { feed } from "./lib/feed.ts";
import { invalid, local } from "@earendil-works/chord";

const KEPT_OUTCOMES = 50;

const USER_REQUEST = /^user:(.+)$/;

/**
 * One conversation's cells, shared by every client's view of it. Referenced lifetime with a 30 s linger, so a client
 * switching away and back, or reconnecting, does not mount pi-durable's view again.
 */
export const ConversationRoom = local("dimos.conversation.room", {
  version: 1,
  input: Type.Object({ conversation: ConversationIdSchema }),
  members: ConversationState,
});

/** A built-in document of the view as its token types it; the definition's initial value when the view lacks it. */
function docOf<T extends JsonObject>(
  view: ConversationView,
  token: { readonly definition: CommonDocDefinition<T> },
): T {
  const doc = view.docs[token.definition.kind];
  return doc === undefined ? token.definition.initial() : (doc as T);
}

/** Each frame of the watch as a Chord event source; the watch is stopped by its resource, so unsubscribing is a no-op. */
function frames(watch: ConversationWatch): Subscribable<ConversationView> {
  return {
    subscribe: (listener) => {
      watch.start(async (value) => listener(value));
      return () => undefined;
    },
  };
}

function outcomeOf(record: SubmissionRecord & { readonly type: "input" }): InputOutcome | null {
  const matched = record.requestId === undefined ? null : USER_REQUEST.exec(record.requestId);
  const request = matched === null ? null : (matched[1] as UserRequestId);
  if (record.status === "done") return { submission: record.id, request, status: "done", answer: record.answer };
  if (record.status === "unanswered")
    return { submission: record.id, request, status: "unanswered", reason: record.reason };
  return null;
}

/** The newest `KEPT_OUTCOMES` settled input submissions of the conversation in the store, oldest first. */
async function storedOutcomes(
  storage: Pick<Storage, "scanSubmissions">,
  conversation: ConversationId,
  context: Context,
): Promise<readonly InputOutcome[]> {
  const found: InputOutcome[] = [];
  let cursor: Cursor | undefined;
  do {
    const page = await storage.scanSubmissions(
      { conversationId: conversation, order: "descending" },
      KEPT_OUTCOMES,
      cursor,
      context,
    );
    for (const record of page.items) {
      const outcome = record.type === "input" ? outcomeOf(record) : null;
      if (outcome !== null) found.push(outcome);
    }
    cursor = page.next;
  } while (cursor !== undefined && found.length < KEPT_OUTCOMES);
  return found.slice(0, KEPT_OUTCOMES).reverse();
}

/** The settled input submissions of this conversation in a publication folded into the newest `KEPT_OUTCOMES`. */
function foldOutcomes(
  rows: readonly InputOutcome[],
  publication: CommitPublication,
  conversation: ConversationId,
): readonly InputOutcome[] {
  let next = rows;
  for (const change of publication.changes) {
    if (change.type !== "submission") continue;
    const record = change.value;
    if (record.conversationId !== conversation || record.type !== "input") continue;
    const outcome = outcomeOf(record);
    if (outcome === null) continue;
    next = [...next.filter((row) => row.submission !== outcome.submission), outcome].slice(-KEPT_OUTCOMES);
  }
  return next;
}

function ownedModules(rows: readonly ModuleRow[], conversation: ConversationId): readonly ModuleRow[] {
  return rows.filter((row) => row.owner === conversation);
}

function ownedJobs(rows: readonly JobRow[], conversation: ConversationId): readonly JobRow[] {
  return rows.filter((row) => row.conversation === conversation);
}

function ownedStreams(modules: readonly ModuleRow[], streams: readonly StreamRef[]): readonly OwnedStream[] {
  const owned: OwnedStream[] = [];
  for (const module of modules) {
    if (module.instance === null) continue;
    for (const ref of streams)
      if (ref.source?.module === module.instance) owned.push({ ref, via: { kind: "module", module: module.name } });
  }
  return owned;
}

/**
 * Feeds from pi-durable's `watch()`: on each frame, `record`, `live`, `queue`, `usage` and `agent` are replaced with the
 * view's own subtrees (shared containers, so Chord sends ops), and `transcript` with `projectTranscript`, diffed, as
 * its new array holds the previous frame's items. `outcomes` starts from the settled inputs in the store, so a reopened
 * room still has the ones that settled while it was closed, and folds in the submission changes of later commit
 * publications. `detachable` and `moved` are fed from the Detacher. The DimOS members are derivations over the store,
 * filtered to this conversation. The watch resource opens before the store reference goes live, so its `open` touches
 * only the harness.
 */
export const serveRoom: ServeObject = (host, deps) =>
  host.serve(
    ConversationRoom,
    ({ conversation }, _principal, self) => {
      const store = self.get(GatewayStore, {});
      const { agent } = deps;
      const watch = self.resource("watch", {
        version: 1,
        open: async ({ signal }) => {
          const handle = await agent.harness.conversation(conversation, withAbortSignal(signal, BACKGROUND_CONTEXT));
          if (handle === undefined) throw invalid("conversation", `No conversation ${conversation}`);
          return handle.watch(BACKGROUND_CONTEXT);
        },
        close: async (opened) => {
          await opened.stop();
        },
      });
      const memo: TranscriptMemo = new WeakMap();
      const record = self.state("record", ConversationState.record, { init: () => watch.value.value.conversation });
      const transcript = self.state("transcript", ConversationState.transcript, {
        init: () => projectTranscript(watch.value.value, memo, TRANSCRIPT_BUDGET),
      });
      const live = self.state("live", ConversationState.live, { init: () => docOf(watch.value.value, LiveDoc) });
      const queue = self.state("queue", ConversationState.queue, { init: () => docOf(watch.value.value, InboxDoc) });
      const usage = self.state("usage", ConversationState.usage, { init: () => docOf(watch.value.value, UsageDoc) });
      const agentState = self.state("agent", ConversationState.agent, {
        init: () => docOf(watch.value.value, AgentDoc),
      });
      self.on("view", watch, frames, (view, ctx) => {
        record.replace(ctx, view.conversation);
        transcript.replace(ctx, projectTranscript(view, memo, TRANSCRIPT_BUDGET), { diff: true });
        live.replace(ctx, docOf(view, LiveDoc));
        queue.replace(ctx, docOf(view, InboxDoc));
        usage.replace(ctx, docOf(view, UsageDoc));
        agentState.replace(ctx, docOf(view, AgentDoc));
      });
      const commits = self.resource("commits", {
        version: 1,
        open: async ({ signal }) => {
          // Subscribed before the scan, so an input settling meanwhile is in the scan or in the events.
          const events = commitEvents(agent.harness);
          try {
            const stored = await storedOutcomes(
              agent.storage,
              conversation,
              withAbortSignal(signal, BACKGROUND_CONTEXT),
            );
            return { events, stored };
          } catch (error) {
            events.close();
            throw error;
          }
        },
        close: ({ events }) => events.close(),
      });
      const outcomes = self.state("outcomes", ConversationState.outcomes, { init: () => commits.value.stored });
      self.on(
        "outcomes",
        commits,
        ({ events }) => events,
        (publication, ctx) => {
          const next = foldOutcomes(outcomes.value, publication, conversation);
          if (next !== outcomes.value) outcomes.replace(ctx, next);
        },
      );
      const modules = self.derive([store.modules], (rows) => ownedModules(rows, conversation));
      return {
        record,
        transcript,
        live,
        queue,
        usage,
        agent: agentState,
        outcomes,
        detachable: feed(self, "detachable", ConversationState.detachable, agent.detacher.of(conversation)),
        moved: feed(self, "moved", ConversationState.moved, agent.detacher.moved(conversation), { diff: true }),
        modules,
        jobs: self.derive([store.jobs], (rows) => ownedJobs(rows, conversation)),
        streams: self.derive([modules, store.streams], (mine, streams) => ownedStreams(mine, streams)),
      };
    },
    { linger: 30_000 },
  );

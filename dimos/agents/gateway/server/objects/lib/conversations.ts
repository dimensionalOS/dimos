import type {
  AgentState,
  CommitPublication,
  CommonDocDefinition,
  ConversationId,
  ConversationRecord,
  Cursor,
  EntryId,
  EntryRecord,
  Harness,
  InboxState,
  JsonObject,
  LiveState,
  Tx,
  UsageState,
} from "@earendil-works/pi-durable";
import type { Context } from "@earendil-works/chord/context";
import type { ConversationRow, Running } from "#dimos/agents/shared/protocol/conversation/summary.ts";
import type { Conversations } from "#dimos/agents/harness/conversations.ts";
import type { EpochMs } from "#dimos/agents/shared/protocol/time.ts";
import type { Cell, Ctx, Self } from "@earendil-works/chord";
import type { Principal } from "#dimos/agents/gateway/auth/principal.ts";
import { BACKGROUND_CONTEXT, withAbortSignal } from "@earendil-works/chord/context";
import { GatewayState } from "#dimos/agents/shared/protocol/gateway/contract.ts";
import { commitEvents } from "#dimos/agents/gateway/server/objects/lib/commits.ts";
import { AgentDoc, InboxDoc, LiveDoc, UsageDoc } from "@earendil-works/pi-durable";
import { DimosInputEntry } from "#dimos/agents/harness/inputs/source.ts";
import { SEEN_DOC_KIND } from "#dimos/agents/harness/conversations.ts";
import { epochMs } from "#dimos/agents/shared/protocol/time.ts";
import { inputOf } from "#dimos/agents/shared/protocol/conversation/queue.ts";

const EMPTY_TITLE = "(empty)";

const SCAN_PAGE = 200;

/** The session document `Conversations.markSeen` writes; a change to it re-reads the cursors. */

type Seen = ReadonlyMap<ConversationId, EntryId>;

function titleOf(text: string): string {
  const line = text.split("\n")[0].trim();
  return line === "" ? EMPTY_TITLE : line;
}

/** A notice input: anything a module, a job or background work placed; not the user and not the runtime steer. */
function noticeOf(entry: EntryRecord): EntryId | null {
  if (!DimosInputEntry.is(entry)) return null;
  const { kind } = entry.data.source;
  return kind === "user" || kind === "runtime" ? null : entry.id;
}

function unseen(cursor: EntryId | undefined, entry: EntryId): boolean {
  return cursor === undefined || entry > cursor;
}

function parentOf(record: ConversationRecord): ConversationRow["parent"] {
  return record.parent === undefined ? null : { conversation: record.parent.conversationId, at: record.parent.at };
}

function freshRow(record: ConversationRecord, now: EpochMs): ConversationRow {
  return {
    id: record.id,
    parent: parentOf(record),
    title: EMPTY_TITLE,
    cwd: null,
    running: null,
    queued: 0,
    unread: 0,
    costUsd: 0,
    updatedAt: now,
  };
}

function sameRunning(a: Running, b: Running): boolean {
  if (a.kind === "tool" && b.kind === "tool") return a.name === b.name;
  return a.kind === b.kind;
}

/** What `pi.live` says the conversation is doing; the previous value when it says the same, so `since` stays. */
function runningOf(live: LiveState, previous: Running | null, now: EpochMs): Running | null {
  let next: Running;
  if (live.run !== undefined) {
    const tool = live.tools?.find((slot) => slot.status !== "done");
    if (tool !== undefined) next = { kind: "tool", name: tool.name, since: now };
    else if (live.compactions?.some((compaction) => compaction.blocking) === true)
      next = { kind: "compaction", since: now };
    else next = { kind: "answering", since: now };
  } else if (live.compactions !== undefined && live.compactions.length > 0) next = { kind: "summary", since: now };
  else return null;
  return previous !== null && sameRunning(previous, next) ? previous : next;
}

function queuedOf(inbox: InboxState): number {
  let queued = 0;
  for (const item of inbox.items) if (inputOf(item)?.source.kind === "user") queued += 1;
  return queued;
}

function costOf(usage: UsageState): number {
  let total = 0;
  for (const model of Object.values(usage.models)) total += model.cost.total;
  return total;
}

function cwdOf(agent: AgentState): string | null {
  return agent.cwd === undefined ? null : agent.cwd;
}

function withLive(row: ConversationRow, live: LiveState, now: EpochMs): ConversationRow {
  const running = runningOf(live, row.running, now);
  return running === row.running ? row : { ...row, running, updatedAt: now };
}

function withInbox(row: ConversationRow, inbox: InboxState, now: EpochMs): ConversationRow {
  const queued = queuedOf(inbox);
  return queued === row.queued ? row : { ...row, queued, updatedAt: now };
}

function withUsage(row: ConversationRow, usage: UsageState, now: EpochMs): ConversationRow {
  const costUsd = costOf(usage);
  return costUsd === row.costUsd ? row : { ...row, costUsd, updatedAt: now };
}

function withAgent(row: ConversationRow, agent: AgentState): ConversationRow {
  const cwd = cwdOf(agent);
  return cwd === row.cwd ? row : { ...row, cwd };
}

function withEntry(row: ConversationRow, entry: EntryRecord, seen: Seen, now: EpochMs): ConversationRow {
  let next: ConversationRow = { ...row, updatedAt: now };
  if (DimosInputEntry.is(entry) && entry.data.source.kind === "user" && row.title === EMPTY_TITLE)
    next = { ...next, title: titleOf(entry.data.text) };
  const notice = noticeOf(entry);
  if (notice !== null && unseen(seen.get(row.id), notice)) next = { ...next, unread: row.unread + 1 };
  return next;
}

/** A conversation document's value as its token types it. */
function valueOf<T extends JsonObject>(value: JsonObject, _token: { readonly definition: CommonDocDefinition<T> }): T {
  return value as T;
}

/**
 * One publication applied to the rows. A row that did not change keeps its object, so the cell's diff is empty for the
 * `pi.live` partials that arrive every 100 ms and change nothing a row shows. Records come first, whatever the
 * publication's order, so an entry or a document of a conversation created in the same commit finds its row.
 */
export function foldPublication(
  rows: readonly ConversationRow[],
  publication: CommitPublication,
  seen: Seen,
  now: EpochMs,
): readonly ConversationRow[] {
  const byId = new Map<ConversationId, ConversationRow>(rows.map((row) => [row.id, row]));
  let added = false;
  for (const change of publication.changes) {
    if (change.type !== "conversation" || byId.has(change.value.id)) continue;
    byId.set(change.value.id, freshRow(change.value, now));
    added = true;
  }
  for (const change of publication.changes) {
    if (change.type === "entry") {
      const row = byId.get(change.value.conversationId);
      if (row !== undefined) byId.set(row.id, withEntry(row, change.value, seen, now));
      continue;
    }
    if (change.type !== "document" || change.conversationId === undefined || change.value === null) continue;
    const row = byId.get(change.conversationId);
    if (row === undefined) continue;
    const { kind } = change.record;
    if (kind === LiveDoc.definition.kind) byId.set(row.id, withLive(row, valueOf(change.value, LiveDoc), now));
    else if (kind === InboxDoc.definition.kind) byId.set(row.id, withInbox(row, valueOf(change.value, InboxDoc), now));
    else if (kind === UsageDoc.definition.kind) byId.set(row.id, withUsage(row, valueOf(change.value, UsageDoc), now));
    else if (kind === AgentDoc.definition.kind) byId.set(row.id, withAgent(row, valueOf(change.value, AgentDoc)));
  }
  const next = [...byId.values()];
  if (!added && next.every((row, index) => row === rows[index])) return rows;
  return added ? next.sort((a, b) => a.id - b.id) : next;
}

type Seed = { readonly record: ConversationRecord; readonly title: string; readonly notices: readonly EntryId[] };

async function scanRecords(tx: Tx): Promise<readonly ConversationRecord[]> {
  const records: ConversationRecord[] = [];
  let cursor: Cursor | undefined;
  do {
    const page = await tx.scanConversations({ order: "ascending" }, SCAN_PAGE, cursor);
    records.push(...page.items);
    cursor = page.next;
  } while (cursor !== undefined);
  return records;
}

/** This conversation's own entries from `from` on, oldest first, until `take` says to stop. */
async function ownEntries(
  tx: Tx,
  record: ConversationRecord,
  from: EntryId | undefined,
  take: (entry: EntryRecord) => boolean,
): Promise<void> {
  let cursor: Cursor | undefined;
  do {
    const page = await tx.scanEntries(
      { conversationId: record.id, order: "ascending", ...(from === undefined ? {} : { minEntryId: from }) },
      SCAN_PAGE,
      cursor,
    );
    for (const entry of page.items) if (entry.conversationId === record.id && !take(entry)) return;
    cursor = page.next;
  } while (cursor !== undefined);
}

async function seedOf(tx: Tx, record: ConversationRecord, seen: Seen): Promise<Seed> {
  const own = record.parent === undefined ? undefined : record.parent.at;
  let title = EMPTY_TITLE;
  await ownEntries(tx, record, own, (entry) => {
    if (!DimosInputEntry.is(entry) || entry.data.source.kind !== "user") return true;
    title = titleOf(entry.data.text);
    return false;
  });
  const cursor = seen.get(record.id);
  const notices: EntryId[] = [];
  const after = cursor === undefined ? own : ((cursor + 1) as EntryId);
  await ownEntries(tx, record, after, (entry) => {
    const notice = noticeOf(entry);
    if (notice !== null) notices.push(notice);
    return true;
  });
  return { record, title, notices };
}

async function rowOf(harness: Harness, seed: Seed, now: EpochMs, context: Context): Promise<ConversationRow> {
  const id = seed.record.id;
  const [live, inbox, usage, agent] = await Promise.all([
    harness.snapshot(LiveDoc, id, context),
    harness.snapshot(InboxDoc, id, context),
    harness.snapshot(UsageDoc, id, context),
    harness.snapshot(AgentDoc, id, context),
  ]);
  let row: ConversationRow = { ...freshRow(seed.record, now), title: seed.title, unread: seed.notices.length };
  if (live !== undefined) row = withLive(row, live, now);
  if (inbox !== undefined) row = withInbox(row, inbox, now);
  if (usage !== undefined) row = withUsage(row, usage, now);
  if (agent !== undefined) row = withAgent(row, agent);
  return row;
}

type ConversationHistory = {
  readonly events: ReturnType<typeof commitEvents>;
  seen: Seen;
  readonly notices: Map<ConversationId, EntryId[]>;
  refreshing: boolean;
  /** A seen change arrived since the running refresh read the cursors: it reads them once more. */
  again: boolean;
};

export function conversationRows(
  self: Self<Principal>,
  harness: Harness,
  conversations: Conversations,
): Cell<readonly ConversationRow[]> {
  const history = self.resource("conversationHistory", {
    version: 1,
    open: (): ConversationHistory => ({
      events: commitEvents(harness),
      seen: new Map<ConversationId, EntryId>(),
      notices: new Map<ConversationId, EntryId[]>(),
      refreshing: false,
      again: false,
    }),
    close: (history) => history.events.close(),
  });
  const rows = self.state("conversations", GatewayState.conversations, {
    init: async (ctx) => {
      const context = withAbortSignal(ctx.signal, BACKGROUND_CONTEXT);
      const state = history.value;
      state.seen = await conversations.seen(context);
      let boundary = 0;
      const seeds = await harness.commit(async (tx) => {
        boundary = state.events.pending.length;
        const records = await scanRecords(tx);
        const found: Seed[] = [];
        for (const record of records) found.push(await seedOf(tx, record, state.seen));
        return found;
      }, context);
      const now = epochMs(Date.now());
      const initial: ConversationRow[] = [];
      for (const seed of seeds) {
        initial.push(await rowOf(harness, seed, now, context));
        state.notices.set(seed.record.id, [...seed.notices]);
      }
      // The scan already includes publications delivered before its transaction began.
      state.events.pending.splice(0, boundary);
      return initial;
    },
  });

  const refreshSeen = async (ctx: Ctx<Principal>): Promise<void> => {
    const state = history.value;
    state.again = true;
    if (state.refreshing) return;
    state.refreshing = true;
    try {
      do {
        state.again = false;
        const seen = await conversations.seen(withAbortSignal(ctx.signal, BACKGROUND_CONTEXT));
        if (ctx.signal.aborted) return;
        state.seen = seen;
        rows.replace(
          ctx,
          rows.value.map((row) => {
            const kept = (state.notices.get(row.id) ?? []).filter((entry) => unseen(seen.get(row.id), entry));
            state.notices.set(row.id, kept);
            return kept.length === row.unread ? row : { ...row, unread: kept.length };
          }),
        );
      } while (state.again);
    } finally {
      state.refreshing = false;
    }
  };
  self.on(
    "conversationCommits",
    history,
    (history) => history.events,
    (publication, ctx) => {
      const state = history.value;
      let seenChanged = false;
      for (const change of publication.changes) {
        if (change.type === "entry") {
          const notice = noticeOf(change.value);
          const conversation = change.value.conversationId;
          if (notice !== null && unseen(state.seen.get(conversation), notice)) {
            const notices = state.notices.get(conversation) ?? [];
            notices.push(notice);
            state.notices.set(conversation, notices);
          }
        } else if (change.type === "document" && change.record.kind === SEEN_DOC_KIND) seenChanged = true;
      }
      rows.replace(ctx, foldPublication(rows.value, publication, state.seen, epochMs(Date.now())));
      // Commit observers cannot reenter the Session. Chord keeps this listener's root alive for the refresh.
      if (seenChanged) return Promise.resolve().then(() => refreshSeen(ctx));
    },
  );
  self.task(refreshSeen);
  return rows;
}

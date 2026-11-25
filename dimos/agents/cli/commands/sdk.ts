import type { Command } from "#dimos/agents/cli/commands/base.ts";
import type { ConversationId, EntryId, LiveState } from "@earendil-works/pi-durable";
import type { AssistantMessage } from "@earendil-works/pi-ai";
import type { ConversationRow } from "#dimos/agents/shared/protocol/conversation/summary.ts";
import type { Transcript } from "#dimos/agents/shared/protocol/conversation/state.ts";
import type { Ctx, Ref } from "@earendil-works/chord";
import { isChordError } from "@earendil-works/chord";
import type { Static } from "typebox";
import type { ActionGroup } from "#dimos/agents/shared/protocol/actions/base.ts";
import { Value } from "typebox/value";
import { GATEWAY_ACTIONS } from "#dimos/agents/shared/protocol/actions/gateway.ts";
import { CONVERSATION_ACTIONS } from "#dimos/agents/shared/protocol/actions/conversation.ts";
import { json } from "node:stream/consumers";
import { isDeepStrictEqual } from "node:util";
import { connect, type Client } from "#dimos/agents/sdk/main.ts";
import { owner, RefusalError, refusalOfFailure } from "#dimos/agents/sdk/failures.ts";
import { until } from "#dimos/agents/sdk/until.ts";
import { unixSocket } from "#dimos/agents/sdk/node.ts";
import { GatewayContract } from "#dimos/agents/shared/protocol/gateway/contract.ts";
import { ConversationContract } from "#dimos/agents/shared/protocol/conversation/contract.ts";
import { done, type Outcome } from "#dimos/agents/shared/protocol/outcome.ts";
import { epochMs } from "#dimos/agents/shared/protocol/time.ts";
import { reachable } from "#dimos/agents/shared/socket.ts";
import { refusalText } from "#dimos/agents/shared/view-models/outcome.ts";

function write(value: unknown): void {
  process.stdout.write(JSON.stringify(value) + "\n");
}

/** The SDK supplies whole lists. Publish each changed row under its stable id or name. */
function publishRowChanges<K extends "id" | "name">(type: string, key: K) {
  type Row = Record<K, string | number> & Record<string, unknown>;
  let previous: Map<Row[K], Row> | undefined;
  return (rows: readonly Row[]) => {
    const current = new Map(rows.map((row) => [row[key], row]));
    if (previous === undefined) {
      write({ type, change: "snapshot", rows });
    } else {
      for (const [id, row] of current) {
        const before = previous.get(id);
        if (before === undefined) {
          write({ type, change: "added", [key]: id, row });
        } else if (before !== row) {
          const fields = Object.keys(row).filter((field) => !isDeepStrictEqual(before[field], row[field]));
          if (fields.length) write({ type, change: "updated", [key]: id, fields, row });
        }
      }
      for (const id of previous.keys()) {
        if (!current.has(id)) write({ type, change: "removed", [key]: id });
      }
    }
    previous = current;
  };
}

function assistantText(message: Pick<AssistantMessage, "content">): string {
  return message.content.flatMap((block) => (block.type === "text" ? [block.text] : [])).join("");
}

function turn(live: LiveState) {
  if (!live.run) return null;
  const message = live.generation?.message;
  return { run: live.run, messageTimestamp: message?.timestamp ?? null, text: message ? assistantText(message) : "" };
}

type WatchedChat = { id: ConversationId; lastEntry?: EntryId; following: boolean };

/** Stream list changes and attach one passive viewer to each running chat. */
async function watch(client: Client): Promise<never> {
  const gateway = await client.host.get(GatewayContract, {});
  const chats = new Map<ConversationId, WatchedChat>();
  const publishConversations = publishRowChanges("agent/conversations", "id");
  try {
    gateway.jobs.subscribe(publishRowChanges("agent/jobs", "name"));
    gateway.conversations.subscribe((rows) => {
      publishConversations(rows);
      for (const { id, running } of rows) {
        if (!running) continue;
        const chat = chats.get(id) ?? { id, following: false };
        if (chat.following) continue;
        chats.set(id, chat);
        void followChat(client, gateway, chat).catch(console.error);
      }
    });
    return await client.ended;
  } finally {
    gateway.release();
  }
}

async function followChat(client: Client, gateway: Ref<typeof GatewayContract>, chat: WatchedChat): Promise<void> {
  const address = { conversation: chat.id, role: "viewer", markSeen: false } as const;
  const running = (rows: readonly ConversationRow[]) => rows.some((row) => row.id === chat.id && row.running);
  chat.following = true;
  try {
    do {
      const publisher = chatPublisher(chat);
      const room = await client.host.get(ConversationContract, address);
      try {
        room.transcript.subscribe(publisher.publishTranscript);
        room.live.subscribe(publisher.publishLiveTurn);
        // The list can announce a run before the room sees it. Wait for both to reach idle.
        await until(gateway.conversations, (rows) => (running(rows) ? undefined : true));
        await until(room.live, (live) => (live.run ? undefined : true));
      } finally {
        room.release();
      }
      // The idle signal can arrive before the final saved reply. Fetch it before announcing the end.
      await client.host.root(async (ctx) => {
        const final = await ctx.get(ConversationContract, address);
        publisher.publishTranscript(final.transcript.value);
        if (!final.live.value.run && !running(gateway.conversations.value)) publisher.endTurn();
      });
    } while (running(gateway.conversations.value));
  } finally {
    chat.following = false;
  }
}

/** One attachment starts with transcript history, then streams live text and newly saved entries. */
function chatPublisher(chat: WatchedChat) {
  const send = (event: object) => write({ type: "agent/chat", conversation: chat.id, ...event });
  const sentText = new Map<number, string>();
  let previousTurn: { run: NonNullable<LiveState["run"]>; messageTimestamp: number | null } | undefined;
  let newestAssistantEntry: EntryId | undefined;
  let firstTranscript = true;

  function publishTextDelta(message: Pick<AssistantMessage, "timestamp" | "content">): void {
    const messageTimestamp = message.timestamp;
    const text = assistantText(message);
    const previous = sentText.get(messageTimestamp) ?? "";
    if (text === previous) return;
    if (text.startsWith(previous)) {
      send({ messageTimestamp, offset: previous.length, delta: text.slice(previous.length) });
    } else {
      send({ messageTimestamp, reset: true, offset: 0, delta: text });
    }
    sentText.set(messageTimestamp, text);
  }

  function publishLiveTurn({ run, generation }: LiveState): void {
    if (!run) return;
    const message = generation?.message;
    const turn = { run, messageTimestamp: message?.timestamp ?? null };
    if (!isDeepStrictEqual(turn, previousTurn)) {
      send({ turn });
      previousTurn = turn;
    }
    if (message) publishTextDelta(message);
  }

  function publishTranscript({ items }: Transcript): void {
    // A missing cursor replays the whole available window, including after an edit or truncation.
    const cursor = items.findLastIndex((item) => item.entry === chat.lastEntry);
    const entries = items.slice(cursor + 1);
    for (const item of entries) {
      if (item.kind !== "assistant") continue;
      if (newestAssistantEntry !== undefined && item.entry <= newestAssistantEntry) continue;
      // The saved reply can include text that never appeared in the live draft.
      if (!firstTranscript) publishTextDelta(item.message);
      sentText.delete(item.message.timestamp);
      newestAssistantEntry = item.entry;
    }
    firstTranscript = false;
    const newest = entries.at(-1);
    if (newest) {
      send({ entries });
      chat.lastEntry = newest.entry;
    }
  }

  return { publishLiveTurn, publishTranscript, endTurn: () => send({ turn: null }) };
}

const gatewayActions = ["createConversation", "createJob", "updateJob", "deleteJob", "runJob"] as const;

async function invoke(ctx: Ctx, operation: string, id: string | undefined): Promise<Outcome<unknown>> {
  const args = await json(process.stdin);

  function call<G extends ActionGroup, K extends keyof G & string>(
    catalog: G,
    ref: { [N in K]: (input: Static<G[N]["args"]>, ctx: Ctx) => Promise<Outcome<unknown>> },
    name: K,
  ): Promise<Outcome<unknown>> {
    if (!Value.Check<G[K]["args"]>(catalog[name].args, args))
      throw new RefusalError({ code: "invalid", field: name, problem: "invalid arguments" });
    return ref[name](args, ctx);
  }

  if (id !== undefined) {
    const needsOwner = operation === "send" || operation === "abort";
    const room = await ctx.get(
      ConversationContract,
      {
        conversation: Number(id) as ConversationId,
        role: needsOwner ? "owner" : "viewer",
        markSeen: operation === "conversation",
      },
      { state: operation === "conversation" },
    );
    if (operation === "conversation") return done({ transcript: room.transcript.value, turn: turn(room.live.value) });
    if (needsOwner) return call(CONVERSATION_ACTIONS, owner(room), operation);
    if (operation === "image") return call(CONVERSATION_ACTIONS, room, operation);
  } else {
    const list = operation === "conversations" || operation === "jobs";
    const gateway = await ctx.get(GatewayContract, {}, { state: list });
    if (list) return done(gateway[operation].value);
    const action = gatewayActions.find((name) => name === operation);
    if (action !== undefined) return call(GATEWAY_ACTIONS, owner(gateway), action);
  }
  throw new Error("Unknown DimOS gateway operation");
}

/** The Desktop gateway's stdin/stdout interface. A call prints one reply, watch streams JSON lines. */
export const runSdk: Command = async ({ args: [operation, id], paths }) => {
  if (operation === undefined || (operation === "watch" && id !== undefined))
    throw new Error("Usage: dimos agent sdk OPERATION [CONVERSATION_ID]");
  // Closing the Desktop gateway's pipe also stops a watcher waiting for the agent to come online.
  if (operation === "watch") process.stdin.once("end", () => process.exit(0)).resume();
  try {
    if (operation !== "watch" && !(await reachable(paths.socket, 1000))) {
      write({ status: 503, error: "DimOS Agent is offline" });
      return 0;
    }
    const client = await connect(unixSocket({ path: paths.socket, label: "dimos-gateway" }));
    try {
      const request = operation === "watch" ? watch(client) : client.host.root((ctx) => invoke(ctx, operation, id));
      const result = await Promise.race([request, client.ended]);
      if (result.ok) write({ value: result.value });
      else write({ refusal: result.refusal.code, error: refusalText(result.refusal) });
    } finally {
      await client.close();
    }
  } catch (error) {
    if (operation === "watch") throw error;
    if (isChordError(error) && error.reason === "conversation") write({ status: 404, error: "Unknown conversation" });
    else {
      const refusal = refusalOfFailure(error, epochMs(Date.now()));
      write({ refusal: refusal.code, error: refusalText(refusal) });
    }
  }
  return 0;
};

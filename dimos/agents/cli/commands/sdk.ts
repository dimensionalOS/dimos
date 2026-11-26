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
import { createConnection } from "node:net";
import { existsSync, watch as watchDirectory } from "node:fs";
import { mkdir } from "node:fs/promises";
import { basename, dirname } from "node:path";
import { createInterface } from "node:readline";
import { isDeepStrictEqual } from "node:util";
import { connect, type Client } from "#dimos/agents/sdk/main.ts";
import { owner, RefusalError, refusalOfFailure } from "#dimos/agents/sdk/failures.ts";
import { until } from "#dimos/agents/sdk/until.ts";
import { unixSocket } from "#dimos/agents/sdk/node.ts";
import { GatewayContract } from "#dimos/agents/shared/protocol/gateway/contract.ts";
import { ConversationContract } from "#dimos/agents/shared/protocol/conversation/contract.ts";
import { done, type Outcome, type Refusal } from "#dimos/agents/shared/protocol/outcome.ts";
import { epochMs } from "#dimos/agents/shared/protocol/time.ts";
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
async function relayEvents(client: Client): Promise<never> {
  const gateway = await client.host.get(GatewayContract, {});
  const chats = new Map<ConversationId, WatchedChat>();
  const publishConversations = publishRowChanges("agent/conversations", "id");
  try {
    gateway.jobs.subscribe(publishRowChanges("agent/jobs", "name"));
    gateway.conversations.subscribe((rows) => {
      publishConversations(rows);
      for (const chat of chats.values()) {
        if (!chat.following && !rows.some((row) => row.id === chat.id)) chats.delete(chat.id);
      }
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

async function invoke(ctx: Ctx, operation: string, id: string | undefined, args: unknown): Promise<Outcome<unknown>> {
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

const httpStatuses: Partial<Record<Refusal["code"], number>> = {
  invalid: 400,
  viewing: 403,
  not_found: 404,
  running: 409,
  stale: 409,
};

type Failure = { status: number; refusal?: Refusal["code"]; error: string };

function refused(refusal: Refusal): Failure {
  return { status: httpStatuses[refusal.code] ?? 503, refusal: refusal.code, error: refusalText(refusal) };
}

function failureOf(error: unknown): Failure {
  if (isChordError(error) && error.reason === "conversation") return { status: 404, error: "Unknown conversation" };
  return refused(refusalOfFailure(error, epochMs(Date.now())));
}

async function call(client: Client, operation: string, id: string | undefined, args: unknown) {
  try {
    const result = await Promise.race([client.host.root((ctx) => invoke(ctx, operation, id, args)), client.ended]);
    return result.ok ? { value: result.value } : refused(result.refusal);
  } catch (error) {
    return failureOf(error);
  }
}

/** Whether the agent accepts a bare connect on its socket. */
function listening(path: string): Promise<boolean> {
  return new Promise((resolve) => {
    const probe = createConnection(path);
    probe.once("connect", () => {
      probe.destroy();
      resolve(true);
    });
    probe.once("error", () => resolve(false));
  });
}

/** Resolves once the agent's socket file exists; with `replaced`, once a restarted agent creates it again. */
async function socketCreated(path: string, replaced: boolean): Promise<void> {
  if (!replaced && existsSync(path)) return;
  await mkdir(dirname(path), { recursive: true });
  await new Promise<void>((resolve, reject) => {
    const watcher = watchDirectory(dirname(path), (_, name) => {
      if ((name === null || name === basename(path)) && existsSync(path)) finish();
    });
    const finish = () => {
      watcher.close();
      resolve();
    };
    watcher.once("error", reject);
    if (!replaced && existsSync(path)) finish();
  });
}

class NotListening extends Error {}

type Request = { id: number; operation: string; conversation?: number; args?: unknown };

/** The Desktop gateway's one client: JSON-line requests on stdin; replies and agent events as JSON lines on stdout. */
async function serve(socket: string): Promise<never> {
  const offline: Failure = { status: 503, error: "DimOS Agent is offline" };
  let connection: Promise<Client> | undefined;
  let down = offline;
  const lines = createInterface({ input: process.stdin });
  // Closing the Desktop gateway's pipe stops the client, including while it waits for the agent.
  lines.once("close", () => process.exit(0));
  lines.on("line", (line) => {
    const { id, operation, conversation, args = {} } = JSON.parse(line) as Request;
    const reply = connection?.then(
      (client) => call(client, operation, conversation?.toString(), args),
      () => down,
    );
    void Promise.resolve(reply ?? down).then((body) => write({ reply: id, ...body }));
  });
  for (let replaced = false; ; ) {
    await socketCreated(socket, replaced);
    connection = listening(socket).then((up) => {
      if (!up) throw new NotListening();
      return connect(unixSocket({ path: socket, label: "dimos-gateway" }));
    });
    try {
      await relayEvents(await connection);
    } catch (error) {
      // A file nobody listens on is left by a crashed agent; its restart replaces it.
      if (error instanceof NotListening) {
        replaced = true;
        down = offline;
        continue;
      }
      const refusal = refusalOfFailure(error, epochMs(Date.now()));
      // Another build refuses every attempt, so wait for the agent to restart before dialing it again.
      replaced = refusal.code === "other_build";
      down = replaced ? refused(refusal) : offline;
      if (refusal.code !== "stopping") console.error(error);
    } finally {
      const ended = connection;
      connection = undefined;
      await ended.then(
        (client) => client.close(),
        () => {},
      );
    }
  }
}

/** The command-line interface: `serve` runs the Desktop gateway's client; any other operation prints one reply. */
export const runSdk: Command = async ({ args: [operation, id], paths }) => {
  if (operation === undefined || (operation === "serve" && id !== undefined))
    throw new Error("Usage: dimos agent sdk OPERATION [CONVERSATION_ID]");
  if (operation === "serve") return serve(paths.socket);
  try {
    if (!(await listening(paths.socket))) {
      write({ status: 503, error: "DimOS Agent is offline" });
      return 0;
    }
    const args = await json(process.stdin);
    const client = await connect(unixSocket({ path: paths.socket, label: "dimos-gateway" }));
    try {
      write(await call(client, operation, id, args));
    } finally {
      await client.close();
    }
  } catch (error) {
    write(failureOf(error));
  }
  return 0;
};

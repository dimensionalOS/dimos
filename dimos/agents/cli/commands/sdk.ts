import type { Command } from "#dimos/agents/cli/commands/base.ts";
import type { ConversationId, EntryId, LiveState } from "@earendil-works/pi-durable";
import type { Ctx } from "@earendil-works/chord";
import { isChordError } from "@earendil-works/chord";
import type { Static } from "typebox";
import type { ActionGroup } from "#dimos/agents/shared/protocol/actions/base.ts";
import { Value } from "typebox/value";
import { GATEWAY_ACTIONS } from "#dimos/agents/shared/protocol/actions/gateway.ts";
import { CONVERSATION_ACTIONS } from "#dimos/agents/shared/protocol/actions/conversation.ts";
import { json } from "node:stream/consumers";
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

function turn(live: LiveState) {
  if (!live.run) return null;
  return {
    run: live.run,
    text:
      live.generation?.message?.content.flatMap((block) => (block.type === "text" ? [block.text] : [])).join("") ?? "",
  };
}

async function watch(client: Client): Promise<never> {
  const gateway = await client.host.get(GatewayContract, {});
  const active = new Set<ConversationId>();
  const follow = async (conversation: ConversationId) => {
    const room = await client.host.get(ConversationContract, { conversation, role: "viewer", markSeen: false });
    const chat = (update: object) => write({ type: "agent-chat", conversation, ...update });
    let last: EntryId | undefined;
    const history = room.transcript.subscribe(({ items }) => {
      // The HTTP snapshot supplies earlier history. Forward commits after the last item we sent.
      const index = last === undefined ? items.length - 1 : items.findIndex((item) => item.entry === last) + 1;
      const entries = items.slice(Math.max(0, index));
      last = items.at(-1)?.entry;
      if (entries.length) chat({ entries });
    });
    let input = room.live.value.run?.inputs[0];
    const live = room.live.subscribe((state) => {
      if (state.run) {
        input = state.run.inputs[0];
        chat({ turn: turn(state) });
      }
    });
    try {
      // The global list and room cells arrive independently. Finish from the durable answer
      // even when the list goes idle before the room delivers the final transcript entry.
      await until(gateway.conversations, (rows) =>
        rows.find((row) => row.id === conversation)?.running ? undefined : true,
      );
      if (input !== undefined) {
        const outcome = await room.submission({ id: input });
        if (outcome.ok && outcome.value.answer && outcome.value.answer.entry !== last)
          chat({ entries: [outcome.value.answer] });
      }
      if (!gateway.conversations.value.find((row) => row.id === conversation)?.running) chat({ turn: null });
    } finally {
      live();
      history();
      room.release();
    }
  };
  const start = (id: ConversationId) => {
    active.add(id);
    void follow(id).then(
      () => {
        active.delete(id);
        if (gateway.conversations.value.find((row) => row.id === id)?.running) start(id);
      },
      (error: unknown) => {
        active.delete(id);
        console.error(error);
      },
    );
  };
  const jobs = gateway.jobs.subscribe(() => write({ type: "agent-jobs" }));
  const conversations = gateway.conversations.subscribe((rows) => {
    write({ type: "agent-conversations" });
    for (const row of rows) {
      if (row.running && !active.has(row.id)) start(row.id);
    }
  });
  try {
    return await client.ended;
  } finally {
    jobs();
    conversations();
    gateway.release();
  }
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
    if (operation === "image" || operation === "submission") return call(CONVERSATION_ACTIONS, room, operation);
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

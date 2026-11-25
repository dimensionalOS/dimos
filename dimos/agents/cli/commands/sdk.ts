import type { Command } from "#dimos/agents/cli/commands/base.ts";
import type { ConversationId, LiveState } from "@earendil-works/pi-durable";
import type { AssistantMessage } from "@earendil-works/pi-ai";
import type { Ctx } from "@earendil-works/chord";
import { isChordError } from "@earendil-works/chord";
import type { Static } from "typebox";
import type { ActionGroup } from "#dimos/agents/shared/protocol/actions/base.ts";
import { Value } from "typebox/value";
import { GATEWAY_ACTIONS } from "#dimos/agents/shared/protocol/actions/gateway.ts";
import { CONVERSATION_ACTIONS } from "#dimos/agents/shared/protocol/actions/conversation.ts";
import { json } from "node:stream/consumers";
import { connect } from "#dimos/agents/sdk/main.ts";
import { owner, RefusalError, refusalOfFailure } from "#dimos/agents/sdk/failures.ts";
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

function assistantText(message: Pick<AssistantMessage, "content">): string {
  return message.content.flatMap((block) => (block.type === "text" ? [block.text] : [])).join("");
}

function turn(live: LiveState) {
  if (!live.run) return null;
  const message = live.generation?.message;
  return { run: live.run, messageTimestamp: message?.timestamp ?? null, text: message ? assistantText(message) : "" };
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

/** Read one JSON request from stdin and write the agent gateway's reply to stdout. */
export const runSdk: Command = async ({ args: [operation, id], paths }) => {
  if (operation === undefined) throw new Error("Usage: dimos agent sdk OPERATION [CONVERSATION_ID]");
  try {
    if (!(await reachable(paths.socket, 1000))) {
      write({ status: 503, error: "DimOS Agent is offline" });
      return 0;
    }
    const client = await connect(unixSocket({ path: paths.socket, label: "dimos-gateway" }));
    try {
      const request = client.host.root((ctx) => invoke(ctx, operation, id));
      const result = await Promise.race([request, client.ended]);
      if (result.ok) write({ value: result.value });
      else write({ refusal: result.refusal.code, error: refusalText(result.refusal) });
    } finally {
      await client.close();
    }
  } catch (error) {
    if (isChordError(error) && error.reason === "conversation") write({ status: 404, error: "Unknown conversation" });
    else {
      const refusal = refusalOfFailure(error, epochMs(Date.now()));
      write({ refusal: refusal.code, error: refusalText(refusal) });
    }
  }
  return 0;
};

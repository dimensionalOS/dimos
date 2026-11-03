import assert from "node:assert/strict";
import type { ConversationId, SubmissionId, TaskId } from "@earendil-works/pi-durable";
import type { Handlers } from "#dimos/agents/gateway/server/objects/lib/bind.ts";
import type { Principal } from "#dimos/agents/gateway/auth/principal.ts";
import type { Standing } from "#dimos/agents/shared/protocol/actions/standing.ts";
import { BACKGROUND_CONTEXT } from "@earendil-works/chord/context";
import { CONVERSATION_ACTIONS } from "#dimos/agents/shared/protocol/actions/conversation.ts";
import { DimosUnavailable } from "#dimos/agents/harness/dimos/connection.ts";
import { Replies } from "#dimos/agents/gateway/server/replies.ts";
import { bind } from "#dimos/agents/gateway/server/objects/lib/bind.ts";
import { createHost } from "@earendil-works/chord";
import { done } from "#dimos/agents/shared/protocol/outcome.ts";
import { epochMs } from "#dimos/agents/shared/protocol/time.ts";
import { newActionRequest, newUserRequest } from "#dimos/agents/shared/protocol/actions/request.ts";
import { principalKey } from "#dimos/agents/gateway/auth/principal.ts";
import { test } from "node:test";

const CONVERSATION = 1 as ConversationId;

function standing(busy: boolean): Standing {
  return {
    role: "owner",
    link: { state: "up", since: epochMs(0), server: "McpServer" },
    conversation: {
      id: CONVERSATION,
      busy,
      userQueued: 0,
      detachable: [],
      moved: [],
      messageKind: () => "user",
    },
  };
}

const idle = standing(false);

const busy = standing(true);

function recording(calls: string[]): Handlers<typeof CONVERSATION_ACTIONS> {
  return {
    send: async ({ request }) => {
      calls.push(`send:${request}`);
      return done({ submission: 9 as SubmissionId });
    },
    withdraw: async () => done([]),
    abort: async () => {
      calls.push("abort");
      throw new DimosUnavailable({ state: "down", error: "fetch failed", retryAt: epochMs(0) });
    },
    configure: async () => {
      calls.push("configure");
      return done(null);
    },
    edit: async () => done({ conversation: 2 as ConversationId, kind: "fork" }),
    summarizeUpTo: async () => done({ conversation: 2 as ConversationId }),
    summarizeFrom: async () => done({ conversation: 2 as ConversationId }),
    background: async () => done({ task: 1 as TaskId }),
    stopBackground: async () => done(null),
    image: async () => done({ type: "image", data: "", mimeType: "image/png" }),
  };
}

test("bind derives the methods from the catalog, runs the rule first, joins retries by request and words a dead link", async () => {
  const host = createHost<Principal>({ name: "test", self: { kind: "gateway" }, principalKey });
  const calls: string[] = [];
  const replies = new Replies(60_000, BACKGROUND_CONTEXT);
  const viewer = bind("conversation", CONVERSATION_ACTIONS, recording(calls), () => idle, replies, "viewer");
  assert.deepEqual(Object.keys(viewer), ["image"]);

  const whenBusy = bind("conversation", CONVERSATION_ACTIONS, recording(calls), () => busy, replies, "owner");
  const refusedConfigure = await host.root((ctx) => whenBusy.configure({ model: null, thinkingLevel: null }, ctx));
  assert.deepEqual(refusedConfigure, { ok: false, refusal: { code: "running", conversation: CONVERSATION } });
  assert.deepEqual(calls, [], "a refused action never reaches its executor");

  const owner = bind("conversation", CONVERSATION_ACTIONS, recording(calls), () => idle, replies, "owner");
  const request = newUserRequest();
  const args = { request, text: "hello", images: [], whenBusy: "followUp" as const };
  const first = await host.root((ctx) => owner.send(args, ctx));
  const again = await host.root((ctx) => owner.send(args, ctx));
  assert.deepEqual(first, { ok: true, value: { submission: 9 as SubmissionId } });
  assert.equal(again, first, "a repeated request gets the first call's outcome");
  assert.deepEqual(calls, [`send:${request}`], "the executor ran once");

  const aborted = await host.root((ctx) => owner.abort({ request: newActionRequest() }, ctx));
  assert.deepEqual(aborted, {
    ok: false,
    refusal: { code: "dimos_unavailable", detail: "the link is down (fetch failed; next try in 0 s)" },
  });
  await host.close();
});

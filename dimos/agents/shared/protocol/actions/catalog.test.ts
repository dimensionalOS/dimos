import assert from "node:assert/strict";
import type { ActionRequestId } from "./request.ts";
import type { AgentModuleName } from "../runtime/event.ts";
import type { CallId } from "../conversation/calls.ts";
import type { ConversationId, EntryId } from "@earendil-works/pi-durable";
import type { ImageContent } from "@earendil-works/pi-ai";
import type { Standing } from "./standing.ts";
import type { UserRequestId } from "../conversation/input.ts";
import { CONVERSATION_ACTIONS } from "./conversation.ts";
import { GATEWAY_ACTIONS } from "./gateway.ts";
import { epochMs } from "../time.ts";
import { methodsOf } from "./methods.ts";
import { test } from "node:test";

const conversation = 7 as ConversationId;

const request = "6f1d2c3b-4a5e-4f60-8a71-9b8c7d6e5f40" as ActionRequestId;

/** A busy conversation whose user has nothing queued: entry 12 is the user's, 13 the assistant's, 99 left the range. */
const BUSY: Standing = {
  role: "owner",
  link: { state: "up", since: epochMs(1_760_000_000_000), server: "McpServer 127.0.0.1:9990" },
  conversation: {
    id: conversation,
    busy: true,
    userQueued: 0,
    detachable: ["call_1" as CallId],
    moved: ["call_2" as CallId],
    messageKind: (entry) => (entry === 12 ? "user" : entry === 13 ? "other" : null),
  },
};

test("configure on a busy conversation is refused running", () => {
  assert.deepEqual(CONVERSATION_ACTIONS.configure.rule({ model: null, thinkingLevel: null }, BUSY), {
    code: "running",
    conversation: 7,
  });
});

test("createModule while the link is down is refused with the link's clause", () => {
  const retryAt = epochMs(Date.now() + 3_600_000);
  const refusal = GATEWAY_ACTIONS.createModule.rule(
    { name: "door_watch" as AgentModuleName, path: "/tmp/door_watch.py", owner: null },
    { role: "owner", link: { state: "down", error: "boom", retryAt }, conversation: null },
  );
  assert.deepEqual(refusal, { code: "dimos_unavailable", detail: "the link is down (boom; next try in 3600 s)" });
});

test("withdraw with nothing of the user's queued is refused nothing_queued", () => {
  assert.deepEqual(CONVERSATION_ACTIONS.withdraw.rule({ request }, BUSY), { code: "nothing_queued", conversation: 7 });
});

test("edit on an assistant entry is not_a_user_message, on an entry outside the transcript stale", () => {
  const images: ImageContent[] = [];
  const args = { request: "a1b2c3d4-e5f6-4a7b-8c9d-0e1f2a3b4c5d" as UserRequestId, text: "again", images };
  assert.deepEqual(CONVERSATION_ACTIONS.edit.rule({ ...args, message: 13 as EntryId }, BUSY), {
    code: "not_a_user_message",
    conversation: 7,
    entry: 13,
  });
  assert.deepEqual(CONVERSATION_ACTIONS.edit.rule({ ...args, message: 99 as EntryId }, BUSY), {
    code: "stale",
    conversation: 7,
  });
  assert.equal(CONVERSATION_ACTIONS.edit.rule({ ...args, message: 12 as EntryId }, BUSY), null);
});

test("background of a call that is not detachable is refused not_running", () => {
  assert.deepEqual(CONVERSATION_ACTIONS.background.rule({ call: "call_9" as CallId }, BUSY), {
    code: "not_running",
    call: "call_9",
  });
  assert.equal(CONVERSATION_ACTIONS.background.rule({ call: "call_1" as CallId }, BUSY), null);
});

test("stopBackground of a call that is not moved is refused not_running", () => {
  assert.deepEqual(CONVERSATION_ACTIONS.stopBackground.rule({ call: "call_1" as CallId }, BUSY), {
    code: "not_running",
    call: "call_1",
  });
  assert.equal(CONVERSATION_ACTIONS.stopBackground.rule({ call: "call_2" as CallId }, BUSY), null);
});

test("the viewer variant carries only the actions open to any client; the owner carries all ten", () => {
  assert.deepEqual(Object.keys(methodsOf(CONVERSATION_ACTIONS, "viewer")), ["image"]);
  assert.deepEqual(Object.keys(methodsOf(CONVERSATION_ACTIONS, "owner")), [
    "send",
    "withdraw",
    "abort",
    "configure",
    "edit",
    "summarizeUpTo",
    "summarizeFrom",
    "background",
    "stopBackground",
    "image",
  ]);
});

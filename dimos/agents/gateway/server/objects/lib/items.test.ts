import assert from "node:assert/strict";
import type { ConversationId, ConversationView, EntryId, EntryRecord } from "@earendil-works/pi-durable";
import type { DimosInput, UserRequestId } from "#dimos/agents/shared/protocol/conversation/input.ts";
import type { TranscriptMemo } from "#dimos/agents/gateway/server/objects/lib/items.ts";
import { TRANSCRIPT_BUDGET, imageAt, projectTranscript } from "#dimos/agents/gateway/server/objects/lib/items.ts";
import { test } from "node:test";

const CONVERSATION = 1 as ConversationId;

function userInput(text: string): DimosInput {
  return {
    source: { kind: "user", request: "8b1c0b7e-0000-4000-8000-000000000001" as UserRequestId },
    text,
    change: null,
    render: null,
    limit: null,
  };
}

const input: EntryRecord = {
  id: 2 as EntryId,
  conversationId: CONVERSATION,
  kind: "dimos.input",
  model: [
    {
      role: "user",
      content: [
        { type: "text", text: "what is in this picture?" },
        { type: "image", data: "iVBORw0KGgo=", mimeType: "image/png" },
      ],
      timestamp: 0,
    },
  ],
  data: userInput("what is in this picture?"),
};

const assistant: EntryRecord = {
  id: 3 as EntryId,
  conversationId: CONVERSATION,
  kind: "pi.assistant",
  model: [
    {
      role: "assistant",
      content: [{ type: "text", text: "A door." }],
      api: "faux",
      provider: "faux",
      model: "scripted",
      usage: {
        input: 1,
        output: 1,
        cacheRead: 0,
        cacheWrite: 0,
        totalTokens: 2,
        cost: { input: 0, output: 0, cacheRead: 0, cacheWrite: 0, total: 0 },
      },
      stopReason: "stop",
      timestamp: 0,
    },
  ],
};

const toolResult: EntryRecord = {
  id: 4 as EntryId,
  conversationId: CONVERSATION,
  kind: "pi.tool-result",
  model: [
    {
      role: "toolResult",
      toolCallId: "call_1",
      toolName: "bash",
      content: [
        { type: "text", text: "frame 41" },
        { type: "image", data: "/9j/4AAQ", mimeType: "image/jpeg" },
      ],
      isError: false,
      timestamp: 0,
    },
  ],
  data: { diagnostics: [] },
};

const summary: EntryRecord = {
  id: 5 as EntryId,
  conversationId: CONVERSATION,
  kind: "dimos.summary",
  head: 2 as EntryId,
  model: [{ role: "user", content: "Summary of everything before: the user asked about a picture.", timestamp: 0 }],
  data: { from: { conversation: CONVERSATION, entry: 2 as EntryId }, text: "the user asked about a picture" },
};

const view: ConversationView = {
  conversation: { id: CONVERSATION },
  entries: [input, assistant, toolResult, summary],
  docs: {},
};

test("projectTranscript strips image bytes, refs an input's images, keeps summaries and memoizes per entry", () => {
  const memo: TranscriptMemo = new WeakMap();
  const first = projectTranscript(view, memo, TRANSCRIPT_BUDGET);
  assert.equal(first.omitted, 0);
  assert.deepEqual(
    first.items.map((item) => item.entry),
    [2, 3, 4, 5],
  );
  assert.deepEqual(first.items[0], {
    kind: "input",
    entry: 2 as EntryId,
    input: userInput("what is in this picture?"),
    images: [{ at: { entry: 2 as EntryId, message: 0, block: 1 }, mimeType: "image/png" }],
  });
  const result = first.items[2];
  assert.equal(result?.kind, "tool_result");
  if (result?.kind !== "tool_result") return;
  assert.deepEqual(result.result.content, [
    { type: "text", text: "frame 41" },
    { type: "image", data: "", mimeType: "image/jpeg" },
  ]);
  assert.deepEqual(result.result.details, { tool: null });
  assert.deepEqual(first.items[3], {
    kind: "summary",
    entry: 5 as EntryId,
    of: "up_to",
    from: { conversation: CONVERSATION, entry: 2 as EntryId },
    text: "the user asked about a picture",
  });
  assert.deepEqual(
    toolResult.model?.[0]?.role === "toolResult" ? toolResult.model[0].content[1] : null,
    { type: "image", data: "/9j/4AAQ", mimeType: "image/jpeg" },
    "the stored record is not mutated",
  );

  const second = projectTranscript(view, memo, TRANSCRIPT_BUDGET);
  for (let index = 0; index < first.items.length; index += 1)
    assert.equal(second.items[index], first.items[index], `item ${index} is the memoized object`);

  const limited = projectTranscript(view, memo, { items: 2, bytes: TRANSCRIPT_BUDGET.bytes });
  assert.deepEqual(
    limited.items.map((item) => item.entry),
    [4, 5],
    "the budget keeps the newest items",
  );
  assert.equal(limited.omitted, 2);
});

test("imageAt answers the stored block at an address and not_found elsewhere", () => {
  assert.deepEqual(imageAt(input, { entry: 2 as EntryId, message: 0, block: 1 }), {
    ok: true,
    value: { type: "image", data: "iVBORw0KGgo=", mimeType: "image/png" },
  });
  assert.deepEqual(imageAt(input, { entry: 2 as EntryId, message: 0, block: 0 }), {
    ok: false,
    refusal: { code: "not_found", what: "message", name: "2/0/0" },
  });
  assert.deepEqual(imageAt(undefined, { entry: 99 as EntryId, message: 0, block: 0 }), {
    ok: false,
    refusal: { code: "not_found", what: "message", name: "99/0/0" },
  });
});

import assert from "node:assert/strict";
import type { AgentModuleName } from "../../protocol/runtime/event.ts";
import type { AssistantMessage } from "@earendil-works/pi-ai";
import type { CallId } from "../../protocol/conversation/calls.ts";
import type { DimosInput, UserRequestId } from "../../protocol/conversation/input.ts";
import type { EntryId, InboxItem, SubmissionId, TaskId } from "@earendil-works/pi-durable";
import type { TranscriptItem } from "../../protocol/conversation/state.ts";
import { noticesOf } from "./notices.ts";
import { epochMs } from "../../protocol/time.ts";
import { test } from "node:test";
import { timelineOf } from "./timeline.ts";

const R1 = "11111111-1111-4111-8111-111111111111" as UserRequestId;

const R2 = "22222222-2222-4222-8222-222222222222" as UserRequestId;

const R3 = "33333333-3333-4333-8333-333333333333" as UserRequestId;

const R4 = "44444444-4444-4444-8444-444444444444" as UserRequestId;

function userInput(request: UserRequestId, text: string): DimosInput {
  return { source: { kind: "user", request }, text, change: null, render: null, limit: null };
}

const ANSWER: AssistantMessage = {
  role: "assistant",
  content: [
    { type: "text", text: "Checking the door now." },
    { type: "toolCall", id: "call_1", name: "bash", arguments: { command: "python check_door.py" } },
  ],
  api: "anthropic-messages",
  provider: "anthropic",
  model: "claude-opus-4",
  usage: {
    input: 1200,
    output: 40,
    cacheRead: 0,
    cacheWrite: 0,
    totalTokens: 1240,
    cost: { input: 0.01, output: 0.002, cacheRead: 0, cacheWrite: 0, total: 0.012 },
  },
  stopReason: "toolUse",
  timestamp: 1_760_700_000_000,
};

const ITEMS: readonly TranscriptItem[] = [
  { kind: "input", entry: 1 as EntryId, input: userInput(R1, "walk to the kitchen door"), images: [] },
  { kind: "assistant", entry: 2 as EntryId, message: ANSWER },
];

function queued(id: number, mode: "steer" | "followUp", input: DimosInput): InboxItem {
  return { id: id as SubmissionId, mode, content: input.text, entry: { kind: "dimos.input", data: input } };
}

const QUEUE: readonly InboxItem[] = [
  queued(31, "steer", userInput(R2, "also check the fridge")),
  queued(32, "followUp", userInput(R3, "then come back to the dock")),
  queued(33, "steer", {
    source: {
      kind: "event",
      event: {
        id: "0b6648ef-2515-49fb-b333-0a9ec0677a1d",
        module: "door_watch" as AgentModuleName,
        conversation: null,
        at: 1_760_700_000,
        text: "door state changed",
        data: {},
        exports: [],
        suppressed: 0,
      },
    },
    text: "door_watch: door state changed",
    change: null,
    render: null,
    limit: null,
  }),
];

test("rows in order: items, the partial, the running call, the queue, then sends not yet queued", () => {
  const rows = timelineOf(
    {
      transcript: { items: ITEMS, omitted: 0 },
      live: {
        run: { taskId: 9 as TaskId, inputs: [30 as SubmissionId] },
        generation: {
          attempt: 1,
          message: {
            role: "assistant",
            content: [{ type: "text", text: "The door is" }],
            api: "anthropic-messages",
            provider: "anthropic",
            model: "claude-opus-4",
            usage: ANSWER.usage,
            stopReason: "stop",
            timestamp: 1_760_700_000_500,
          },
        },
        tools: [{ callId: "call_1", name: "bash", status: "running", output: "frame 41 door=closed\n" }],
      },
      queue: { items: [...QUEUE] },
      outcomes: [{ submission: 30 as SubmissionId, request: R1, status: "done", answer: 2 as EntryId }],
      detachable: [{ call: "call_1" as CallId, tool: "bash", since: epochMs(1_760_700_000_000) }],
    },
    [
      { request: R3, text: "then come back to the dock" },
      { request: R4, text: "and the window" },
    ],
  );
  assert.deepEqual(
    rows.map((row) => [row.kind, row.key]),
    [
      ["item", "e1"],
      ["item", "e2"],
      ["streaming", "streaming"],
      ["tool", "tcall_1"],
      ["queued", "q31"],
      ["queued", "q32"],
      ["queued", "q33"],
      ["sending", `s${R4}`],
    ],
  );
  const [first, , , tool, steer, , notice, sending] = rows;
  assert.deepEqual(first.kind === "item" ? first.outcome : null, {
    submission: 30,
    request: R1,
    status: "done",
    answer: 2,
  });
  assert.deepEqual(tool.kind === "tool" ? [tool.call.name, tool.backgroundable, tool.since] : null, [
    "bash",
    true,
    1_760_700_000_000,
  ]);
  assert.deepEqual(steer.kind === "queued" ? [steer.row.mode, steer.row.from, steer.row.text, steer.position] : null, [
    "steer",
    "you",
    "also check the fridge",
    0,
  ]);
  assert.deepEqual(notice.kind === "queued" ? [notice.row.from, notice.row.text] : null, [
    "module",
    "door_watch: door state changed",
  ]);
  assert.deepEqual(sending.kind === "sending" ? sending.text : null, "and the window");
});

test("admitted notices remain visible in the queue without a runtime backlog", () => {
  const notices = noticesOf({ items: ITEMS, omitted: 0 }, { items: [...QUEUE] });
  assert.equal(notices.queued.length, 1);
  assert.equal(notices.queued[0]?.from, "module");
  assert.deepEqual(notices.delivered, []);
});

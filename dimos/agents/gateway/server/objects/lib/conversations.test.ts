import assert from "node:assert/strict";
import type { AgentModuleName } from "#dimos/agents/shared/protocol/runtime/event.ts";
import type {
  CommitPublication,
  ConversationId,
  ConversationRecord,
  DocumentId,
  DocumentRecord,
  EntryId,
  EntryRecord,
  Seq,
  SubmissionId,
  TaskId,
} from "@earendil-works/pi-durable";
import type { ConversationRow } from "#dimos/agents/shared/protocol/conversation/summary.ts";
import type { DimosInput, UserRequestId } from "#dimos/agents/shared/protocol/conversation/input.ts";
import { epochMs } from "#dimos/agents/shared/protocol/time.ts";
import { foldPublication } from "#dimos/agents/gateway/server/objects/lib/conversations.ts";
import { test } from "node:test";

const NOW = epochMs(1_760_000_000_000);

const ROOT = 1 as ConversationId;

const FORK = 3 as ConversationId;

const SEEN = new Map<ConversationId, EntryId>();

function userInput(request: string, text: string): DimosInput {
  return { source: { kind: "user", request: request as UserRequestId }, text, change: null, render: null, limit: null };
}

function noticeInput(text: string): DimosInput {
  return {
    source: {
      kind: "event",
      event: {
        id: "0b6648ef-2515-49fb-b333-0a9ec0677a1d",
        module: "door_watch" as AgentModuleName,
        conversation: ROOT,
        at: 1_760_000_000,
        text,
        data: {},
        exports: [],
        suppressed: 0,
      },
    },
    text,
    change: null,
    render: null,
    limit: null,
  };
}

function document(id: number, kind: string, conversation: ConversationId): DocumentRecord {
  return {
    id: id as DocumentId,
    kind,
    createdAt: 1 as Seq,
    scope: { kind: "conversation", conversationId: conversation },
    history: "latest",
    fork: "current",
  };
}

function publication(seq: number, changes: CommitPublication["changes"]): CommitPublication {
  return { seq: seq as Seq, changes };
}

test("foldPublication folds records, the first user input, forks and the live documents into rows", () => {
  const root: ConversationRecord = { id: ROOT };
  const rows1 = foldPublication([], publication(1, [{ type: "conversation", value: root }]), SEEN, NOW);
  const fresh: ConversationRow = {
    id: ROOT,
    parent: null,
    title: "(empty)",
    cwd: null,
    running: null,
    queued: 0,
    unread: 0,
    costUsd: 0,
    updatedAt: NOW,
  };
  assert.deepEqual(rows1, [fresh]);

  const input: EntryRecord = {
    id: 2 as EntryId,
    conversationId: ROOT,
    kind: "dimos.input",
    model: [{ role: "user", content: "walk to the kitchen door\nand wait there", timestamp: 0 }],
    data: userInput("8b1c0b7e-0000-4000-8000-000000000001", "walk to the kitchen door\nand wait there"),
  };
  const rows2 = foldPublication(rows1, publication(2, [{ type: "entry", value: input }]), SEEN, NOW);
  assert.deepEqual(rows2, [{ ...fresh, title: "walk to the kitchen door" }]);

  const fork: ConversationRecord = { id: FORK, parent: { conversationId: ROOT, at: 2 as EntryId } };
  const rows3 = foldPublication(rows2, publication(3, [{ type: "conversation", value: fork }]), SEEN, NOW);
  assert.deepEqual(rows3, [rows2[0], { ...fresh, id: FORK, parent: { conversation: ROOT, at: 2 as EntryId } }]);
  assert.equal(rows3[0], rows2[0], "an untouched row keeps its object");

  const rows4 = foldPublication(
    rows3,
    publication(4, [
      {
        type: "document",
        record: document(10, "pi.live", ROOT),
        conversationId: ROOT,
        version: 1,
        value: {
          run: { taskId: 5 as TaskId, inputs: [4 as SubmissionId] },
          tools: [{ callId: "call_1", name: "bash", status: "running" }],
        },
        ops: [],
      },
    ]),
    SEEN,
    NOW,
  );
  assert.deepEqual(rows4[0]?.running, { kind: "tool", name: "bash", since: NOW });
  assert.equal(rows4[1], rows3[1], "the fork's row is untouched by the root's document");

  const rows5 = foldPublication(
    rows4,
    publication(5, [
      {
        type: "document",
        record: document(11, "pi.inbox", ROOT),
        conversationId: ROOT,
        version: 1,
        value: {
          items: [
            {
              id: 6 as SubmissionId,
              mode: "steer",
              content: "also check the fridge",
              entry: {
                kind: "dimos.input",
                data: userInput("8b1c0b7e-0000-4000-8000-000000000002", "also check the fridge"),
              },
            },
            {
              id: 7 as SubmissionId,
              mode: "followUp",
              content: "then come back",
              entry: { kind: "dimos.input", data: userInput("8b1c0b7e-0000-4000-8000-000000000003", "then come back") },
            },
            {
              id: 8 as SubmissionId,
              mode: "followUp",
              content: "door_watch: door state changed",
              entry: { kind: "dimos.input", data: noticeInput("door state changed") },
            },
          ],
        },
        ops: [],
      },
    ]),
    SEEN,
    NOW,
  );
  assert.equal(rows5[0]?.queued, 2, "only the user's own items count as queued");
  assert.equal(rows5[1], rows3[1]);

  const notice: EntryRecord = {
    id: 9 as EntryId,
    conversationId: ROOT,
    kind: "dimos.input",
    model: [{ role: "user", content: "door state changed", timestamp: 0 }],
    data: noticeInput("door state changed"),
  };
  const rows6 = foldPublication(rows5, publication(6, [{ type: "entry", value: notice }]), SEEN, NOW);
  assert.equal(rows6[0]?.unread, 1, "a notice past the seen cursor is unread");
  const rows7 = foldPublication(
    rows5,
    publication(6, [{ type: "entry", value: notice }]),
    new Map([[ROOT, 9 as EntryId]]),
    NOW,
  );
  assert.equal(rows7[0]?.unread, 0, "a notice at or before the cursor is not");

  const unchanged = foldPublication(rows6, publication(7, []), SEEN, NOW);
  assert.equal(unchanged, rows6, "a publication that changes nothing keeps the rows array");
});

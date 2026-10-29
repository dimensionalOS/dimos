import assert from "node:assert/strict";
import type { ConversationId, EntryId } from "@earendil-works/pi-durable";
import type { ConversationRow } from "../../protocol/conversation/summary.ts";
import { epochMs } from "../../protocol/time.ts";
import { switcherRows } from "./switcher.ts";
import { test } from "node:test";

const NOW = epochMs(1_760_700_000_000);

function row(id: number, fields: Partial<ConversationRow>): ConversationRow {
  return {
    id: id as ConversationId,
    parent: null,
    title: `conversation ${id}`,
    cwd: "/home/dimos/dimos",
    running: null,
    queued: 0,
    unread: 0,
    costUsd: 0,
    updatedAt: epochMs(NOW - 100_000),
    ...fields,
  };
}

const CONVERSATIONS: readonly ConversationRow[] = [
  row(3, { updatedAt: epochMs(NOW - 30_000), queued: 2 }),
  row(7, {
    updatedAt: epochMs(NOW - 20_000),
    running: { kind: "tool", name: "bash", since: epochMs(NOW - 12_000) },
  }),
  row(9, { updatedAt: epochMs(NOW - 15_000), parent: { conversation: 7 as ConversationId, at: 11 as EntryId } }),
  row(12, { updatedAt: epochMs(NOW - 5_000), parent: { conversation: 3 as ConversationId, at: 1 as EntryId } }),
  row(5, { updatedAt: epochMs(NOW - 10_000), running: { kind: "answering", since: epochMs(NOW - 3_000) }, unread: 3 }),
];

test("roots newest first, each followed by its forks one level deeper, with state and unread notices", () => {
  const rows = switcherRows(CONVERSATIONS, 7 as ConversationId, NOW);
  assert.deepEqual(
    rows.map((entry) => entry.id),
    [5, 7, 9, 3, 12],
  );
  assert.deepEqual(
    rows.map((entry) => entry.depth),
    [0, 0, 1, 0, 1],
  );
  assert.deepEqual(
    rows.map((entry) => entry.state),
    ["answering", "running bash 12s", "idle", "queue 2", "idle"],
  );
  assert.deepEqual(
    rows.map((entry) => [entry.unread, entry.level, entry.current]),
    [
      [3, "warn", false],
      [0, "ok", true],
      [0, "ok", false],
      [0, "warn", false],
      [0, "ok", false],
    ],
  );
});

test("a fork whose source is not listed is a root", () => {
  const orphan = row(21, { parent: { conversation: 99 as ConversationId, at: 4 as EntryId } });
  const rows = switcherRows([orphan], null, NOW);
  assert.deepEqual(
    rows.map((entry) => [entry.id, entry.depth, entry.state]),
    [[21, 0, "idle"]],
  );
});

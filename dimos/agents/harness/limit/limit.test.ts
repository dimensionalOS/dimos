import assert from "node:assert/strict";
import type { ConversationId } from "@earendil-works/pi-durable";
import { ContextBudget } from "#dimos/agents/harness/limit/budget.ts";
import { OutputLimiter } from "#dimos/agents/harness/limit/limit.ts";
import { createModels } from "@earendil-works/pi-ai";
import { join } from "node:path";
import { mkdtemp, readFile, rm } from "node:fs/promises";
import { test } from "node:test";
import { tmpdir } from "node:os";

const CALL = "call_184e2dbc83fa4f77a936be41acfd372c|fc_0cb03aa147304768016ad2ab3c42348193a68ff0fc1a289639";

async function setUp(): Promise<{ dir: string; limiter: OutputLimiter; conversation: ConversationId }> {
  const dir = await mkdtemp(join(tmpdir(), "dimos-limit-"));
  return {
    dir,
    limiter: new OutputLimiter({ dir, budget: new ContextBudget(createModels()), defaultMaxChars: 2_000 }),
    conversation: 1071 as ConversationId,
  };
}

test("a cut keeps both ends around one marker naming a short spill file under the conversation", async () => {
  const { dir, limiter, conversation } = await setUp();
  try {
    const text = `${"a".repeat(20_000)}${"z".repeat(20_518)}`;
    const limited = await limiter.limit(text, {
      conversation,
      requested: null,
      spillName: CALL,
      fullTextAt: null,
      argument: "max_chars",
    });
    const spill = join(dir, "1071", "055ad61a98f0.txt");
    assert.equal(
      limited.text,
      `${"a".repeat(1_000)}\n[… 38,518 of 40,518 chars cut. Full: ${spill}. Pass max_chars for more. …]\n${"z".repeat(1_000)}`,
    );
    assert.deepEqual(limited.record, {
      shown: 2_000,
      total: 40_518,
      fullText: spill,
      maxChars: 2_000,
      clampedFrom: null,
    });
    assert.equal(await readFile(spill, "utf8"), text);
  } finally {
    await rm(dir, { recursive: true, force: true });
  }
});

test("a large max_chars is honoured, and an input's marker names its file and offers no argument", async () => {
  const { dir, limiter, conversation } = await setUp();
  try {
    const read = "s".repeat(12_541);
    assert.deepEqual(
      await limiter.limit(read, {
        conversation,
        requested: 23_000,
        spillName: CALL,
        fullTextAt: "/home/dimos/dimos/dimos/agents/skills/person_follow.py",
        argument: "max_chars",
      }),
      { text: read, record: null },
    );
    const input = await limiter.limit("e".repeat(2_500), {
      conversation,
      requested: null,
      spillName: "user:req-1",
      fullTextAt: null,
      argument: null,
    });
    assert.equal(
      input.text,
      `${"e".repeat(1_000)}\n[… 500 of 2,500 chars cut. Full: ${join(dir, "1071", "c678975381bc.txt")}. …]\n${"e".repeat(1_000)}`,
    );
  } finally {
    await rm(dir, { recursive: true, force: true });
  }
});

import assert from "node:assert/strict";
import type { Agent } from "#dimos/agents/harness/main.ts";
import type { ConversationId } from "@earendil-works/pi-durable";
import type { UserRequestId } from "#dimos/agents/shared/protocol/conversation/input.ts";
import { BACKGROUND_CONTEXT } from "@earendil-works/chord/context";
import { DimosConnection } from "#dimos/agents/harness/dimos/connection.ts";
import { after, before, test } from "node:test";
import { dimosFixture } from "#dimos/agents/test/environment.ts";
import { fauxAssistantMessage } from "@earendil-works/pi-ai";
import { join } from "node:path";
import { mkdtempSync, rmSync } from "node:fs";
import { openAgent } from "#dimos/agents/harness/main.ts";
import { scriptedModel } from "#dimos/agents/test/scripted-model.ts";
import { tmpdir } from "node:os";

const dimos = dimosFixture();

const context = BACKGROUND_CONTEXT;

let scratch = "";

function sleep(ms: number): Promise<void> {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

type RecordedSteps = { steps: { conversation: number; entry: string; kind: string; ts: number; text: string }[] };

before(() => {
  scratch = mkdtempSync(join(tmpdir(), "dimos-agent-trajectory-"));
});

after(() => {
  rmSync(scratch, { recursive: true, force: true });
});

test(
  "entries past the cursor reach ordinary recording through HarnessRuntime, and the cursor survives a reopen",
  { timeout: 300_000 },
  async (t) => {
    const system = dimos();
    if (system === null) {
      t.skip("no .venv in the checkout");
      return;
    }
    const model = scriptedModel();
    const open = (): Promise<Agent> =>
      openAgent(
        {
          install: system.install,
          models: model.models,
          storePath: join(scratch, "agent.sqlite"),
          toolResultsDir: join(scratch, "tool-results"),
          renderCacheDir: join(scratch, "render"),
          skillRoots: [],
          defaults: () => ({ model: model.model, thinkingLevel: null, workspace: scratch }),
          gatewayEnv: system.env,
          report: (error) => console.error("[trajectory test report]", error),
        },
        context,
      );
    /** The probe reads AgentStep messages from the ordinary recorder's SqliteStore. */
    const observer = new DimosConnection({
      url: system.install.mcpUrl,
      connectTimeoutMs: 10_000,
      maxBackoffMs: 5_000,
      report: console.error,
    });
    observer.start();
    const read = async (): Promise<RecordedSteps> => {
      const outcome = await observer.callTool("recorded_steps", {}, { caller: null, timeoutMs: 15_000 });
      assert.equal(outcome.isError, false, JSON.stringify(outcome.content));
      return (outcome.structured as { metadata: RecordedSteps }).metadata;
    };
    const turn = async (agent: Agent, conversation: ConversationId, text: string): Promise<void> => {
      const submitted = await agent.inputs.submit(
        conversation,
        {
          source: { kind: "user", request: `req-${text}` as UserRequestId },
          text,
          images: [],
          render: null,
          whenBusy: "reject",
        },
        context,
      );
      assert.equal(submitted.kind, "submitted");
      if (submitted.kind !== "submitted") return;
      assert.equal(
        (await (await agent.harness.submission(submitted.submission, context))?.wait(context))?.status,
        "done",
      );
    };

    /** Waits until the ordinary recorder has stored at least `count` steps. */
    const received = async (count: number): Promise<RecordedSteps> => {
      const deadline = Date.now() + 15_000;
      for (;;) {
        const latest = await read();
        if (latest.steps.length >= count) return latest;
        if (Date.now() > deadline) throw new Error(`only ${latest.steps.length} steps recorded, expected ${count}`);
        await sleep(200);
      }
    };

    let agent: Agent | null = null;
    try {
      assert.equal((await read()).steps.length, 0);
      model.script([fauxAssistantMessage("hello back")]);
      agent = await open();
      const conversation = await agent.conversations.start(null, "trajectory-test", context);
      await turn(agent, conversation, "hi");
      const newest = await agent.harness.commit(
        async (tx) => (await tx.scanEntries({ conversationId: conversation, order: "descending" }, 1)).items[0],
        context,
      );
      assert.ok(newest !== undefined);
      const arrived = await received(2);
      assert.equal(
        arrived.steps.length,
        2,
        "the user input and the answer each became one step; the prompt entry none",
      );
      const answeredAt = newest.model?.[0]?.timestamp;
      assert.ok(answeredAt !== undefined, "the newest entry is the answer");
      assert.deepEqual(
        arrived.steps.at(-1),
        { conversation, entry: String(newest.id), kind: "assistant", text: "hello back", ts: answeredAt / 1000 },
        "the newest step is the answer entry, stamped with its message time in seconds",
      );

      await agent.close(context);
      agent = null;
      agent = await open();
      await sleep(3_000);
      assert.equal((await read()).steps.length, 2, "a reopen sends nothing again: the cursor was committed");

      model.script([fauxAssistantMessage("hello again")]);
      await turn(agent, conversation, "again");
      const later = await received(4);
      assert.equal(later.steps.length, 4);
      const latest = await agent.harness.commit(
        async (tx) => (await tx.scanEntries({ conversationId: conversation, order: "descending" }, 1)).items[0],
        context,
      );
      assert.ok(latest !== undefined);
      const latestAt = latest.model?.[0]?.timestamp;
      assert.ok(latestAt !== undefined, "the newest entry is the answer");
      assert.deepEqual(later.steps.at(-1), {
        conversation,
        entry: String(latest.id),
        kind: "assistant",
        text: "hello again",
        ts: latestAt / 1000,
      });
    } finally {
      try {
        await agent?.close(context);
      } finally {
        await observer.close();
      }
    }
  },
);

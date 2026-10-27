import assert from "node:assert/strict";
import type { CommitPublication } from "@earendil-works/pi-durable";
import type { InputDraft } from "./gate.ts";
import type { JobName } from "../../shared/protocol/jobs/state.ts";
import type { Message } from "@earendil-works/pi-ai";
import type { UserRequestId } from "../../shared/protocol/conversation/input.ts";
import { BACKGROUND_CONTEXT } from "@earendil-works/chord/context";
import { ContextBudget } from "../limit/budget.ts";
import { DimosConnection } from "../dimos/connection.ts";
import { Harness, InboxDoc, LiveDoc, createRegistry } from "@earendil-works/pi-durable";
import { InputGate } from "./gate.ts";
import { OutputLimiter } from "../limit/limit.ts";
import { RuntimeClient } from "../dimos/client.ts";
import { RuntimeObserver } from "../context/observer.ts";
import { SqliteStorage } from "@earendil-works/pi-durable/storage/sqlite";
import { after, before, test } from "node:test";
import { fauxAssistantMessage } from "@earendil-works/pi-ai";
import { join } from "node:path";
import { mkdtempSync, rmSync } from "node:fs";
import { openNodeSqliteDatabase } from "@earendil-works/pi-durable/storage/sqlite/node";
import { scriptedModel } from "../../test/scripted-model.ts";
import { tmpdir } from "node:os";

const context = BACKGROUND_CONTEXT;

let scratch = "";

before(() => {
  scratch = mkdtempSync(join(tmpdir(), "dimos-gate-"));
});

after(() => {
  rmSync(scratch, { recursive: true, force: true });
});

function line(message: Message): string {
  if (message.role === "assistant")
    return `assistant: ${message.content.map((block) => (block.type === "text" ? block.text : "")).join("")}`;
  const content =
    typeof message.content === "string" ? [{ type: "text" as const, text: message.content }] : message.content;
  return `${message.role}: ${content.map((block) => (block.type === "text" ? block.text : "<image>")).join(" | ")}`;
}

function typed(text: string, whenBusy: InputDraft["whenBusy"]): InputDraft {
  return { source: { kind: "user", request: text as UserRequestId }, text, images: [], render: null, whenBusy };
}

test("withdrawing the queue hands back every queued user text in one commit and leaves notices queued", async () => {
  const model = scriptedModel();
  const requests: string[][] = [];
  const reports: unknown[] = [];
  let answerFirst = (): void => undefined;
  const firstHeld = new Promise<void>((resolve) => {
    answerFirst = resolve;
  });
  model.script([
    async (request) => {
      requests.push(request.messages.map(line));
      await firstHeld;
      return fauxAssistantMessage("started");
    },
    (request) => {
      requests.push(request.messages.map(line));
      return fauxAssistantMessage("walking the route");
    },
  ]);
  const storage = await SqliteStorage.open(await openNodeSqliteDatabase(join(scratch, "agent.sqlite")));
  const harness = await Harness.open(
    storage,
    { models: model.models, registry: createRegistry(), onReport: (error) => reports.push(error) },
    context,
  );
  const connection = new DimosConnection({
    url: new URL("http://127.0.0.1:9/mcp"),
    connectTimeoutMs: 1_000,
    maxBackoffMs: 1_000,
    report: (error) => reports.push(error),
  });
  await connection.close();
  const inputs = new InputGate({
    harness,
    observer: new RuntimeObserver({
      harness,
      runtime: new RuntimeClient(connection),
      jobFacts: async () => ({}),
      runsDir: join(scratch, "runs"),
      reuseForMs: 0,
      timeoutMs: 1_000,
      report: (error) => reports.push(error),
    }),
    limiter: new OutputLimiter({
      dir: join(scratch, "results"),
      budget: new ContextBudget(model.models),
      defaultMaxChars: 2_000,
    }),
  });
  const { id } = await harness.createConversation(
    { ownership: { kind: "ownerless" }, agent: { model: model.model } },
    context,
  );

  assert.equal((await inputs.submit(id, typed("start the patrol", "reject"), context)).kind, "submitted");
  const run = (await harness.snapshot(LiveDoc, id, context))?.run;
  assert.ok(run !== undefined, "the first answer holds the run");
  const long = `${"the long one ".repeat(200)}end`;
  for (const draft of [
    typed("first queued", "followUp"),
    typed("a steer", "steer"),
    {
      source: { kind: "job" as const, job: "patrol" as JobName, run: { kind: "manual" as const, toolCall: "call_j" } },
      text: "[Job patrol] Walk the route",
      images: [],
      render: null,
      whenBusy: "followUp" as const,
    },
    typed(long, "followUp"),
  ])
    assert.equal((await inputs.submit(id, draft, context)).kind, "submitted");

  const publications: CommitPublication[] = [];
  const unsubscribe = harness.subscribeCommits((publication) => publications.push(publication));
  const withdrawn = await inputs.withdrawUser(id, "withdraw", "keep", context);
  unsubscribe();

  assert.equal(long.length, 2_603, "longer than the 2,000 characters an input keeps");
  assert.deepEqual(withdrawn, {
    texts: [
      { mode: "followUp", text: "first queued" },
      { mode: "steer", text: "a steer" },
      { mode: "followUp", text: long },
    ],
    aborts: null,
  });
  assert.deepEqual(
    publications.map((publication) =>
      publication.changes.flatMap((change) =>
        change.type === "submission" ? [`${change.value.requestId} ${change.value.status} ${change.value.reason}`] : [],
      ),
    ),
    [
      [
        `user:${long} unanswered withdrawn`,
        "user:a steer unanswered withdrawn",
        "user:first queued unanswered withdrawn",
      ],
    ],
  );
  assert.deepEqual(
    (await harness.snapshot(InboxDoc, id, context))?.items.map((item) => item.mode),
    ["followUp"],
    "the job notice stays queued",
  );
  assert.equal((await inputs.submit(id, typed("queued since", "followUp"), context)).kind, "submitted");
  assert.deepEqual(
    await inputs.withdrawUser(id, "withdraw", "abort", context),
    withdrawn,
    "a repeated request answers with its first withdrawal",
  );
  assert.deepEqual(await inputs.withdrawUser(id, "abort", "abort", context), {
    texts: [{ mode: "followUp", text: "queued since" }],
    aborts: run.taskId,
  });
  assert.deepEqual(
    await inputs.withdrawUser(id, "withdraw last", "keep", context),
    { texts: [], aborts: null },
    "nothing of the user's is left",
  );

  answerFirst();
  await (await harness.conversation(id, context))?.waitForIdle(context);
  assert.deepEqual(requests.at(-1), [
    "user: <dimos_runtime_change>\nDimOS: not reachable, link connecting; any runtime stated earlier is as last seen.\n</dimos_runtime_change> | start the patrol",
    "assistant: started",
    "user: [Job patrol] Walk the route",
  ]);
  assert.deepEqual(reports, []);
  await harness.close(context);
});

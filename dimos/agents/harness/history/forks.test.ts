import assert from "node:assert/strict";
import type { ConversationId, EntryId, EntryRecord } from "@earendil-works/pi-durable";
import type { FauxResponseStep, Message } from "@earendil-works/pi-ai";
import type { UserRequestId } from "#dimos/agents/shared/protocol/conversation/input.ts";
import { BACKGROUND_CONTEXT } from "@earendil-works/chord/context";
import { ContextBudget } from "#dimos/agents/harness/limit/budget.ts";
import { ConversationHistory } from "#dimos/agents/harness/history/forks.ts";
import { DimosConnection } from "#dimos/agents/harness/dimos/connection.ts";
import { DimosInputEntry } from "#dimos/agents/harness/inputs/source.ts";
import { Harness, createRegistry } from "@earendil-works/pi-durable";
import { InputGate } from "#dimos/agents/harness/inputs/gate.ts";
import { OutputLimiter } from "#dimos/agents/harness/limit/limit.ts";
import { RuntimeClient } from "#dimos/agents/harness/dimos/client.ts";
import { RuntimeObserver } from "#dimos/agents/harness/context/observer.ts";
import { SqliteStorage } from "@earendil-works/pi-durable/storage/sqlite";
import { after, before, test } from "node:test";
import { fauxAssistantMessage } from "@earendil-works/pi-ai";
import { join } from "node:path";
import { mkdtempSync, rmSync } from "node:fs";
import { openNodeSqliteDatabase } from "@earendil-works/pi-durable/storage/sqlite/node";
import { scriptedModel } from "#dimos/agents/test/scripted-model.ts";
import { tmpdir } from "node:os";

const context = BACKGROUND_CONTEXT;

let scratch = "";

before(() => {
  scratch = mkdtempSync(join(tmpdir(), "dimos-history-"));
});

after(() => {
  rmSync(scratch, { recursive: true, force: true });
});

/** A message as one line: its role and text, with a tool call as `name(arguments)`. */
function line(message: Message): string {
  if (message.role === "assistant") {
    const blocks = message.content.map((block) => {
      if (block.type === "text") return block.text;
      return block.type === "toolCall" ? `${block.name}(${JSON.stringify(block.arguments)})` : "";
    });
    return `assistant: ${blocks.join("")}`;
  }
  const content =
    typeof message.content === "string" ? [{ type: "text" as const, text: message.content }] : message.content;
  return `${message.role}: ${content.map((block) => (block.type === "text" ? block.text : "<image>")).join(" | ")}`;
}

/** A real store, gate and history on pi-ai's faux provider; every model request is kept as lines. DimOS is down. */
async function open(name: string) {
  const model = scriptedModel();
  const requests: string[][] = [];
  const reports: unknown[] = [];
  const answer =
    (text: string): FauxResponseStep =>
    (request) => {
      requests.push(request.messages.map(line));
      return fauxAssistantMessage(text);
    };
  const storage = await SqliteStorage.open(await openNodeSqliteDatabase(join(scratch, `${name}.sqlite`)));
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
  const observer = new RuntimeObserver({
    harness,
    runtime: new RuntimeClient(connection),
    jobFacts: async () => ({}),
    runsDir: join(scratch, "runs"),
    reuseForMs: 0,
    timeoutMs: 1_000,
    report: (error) => reports.push(error),
  });
  const limiter = new OutputLimiter({
    dir: join(scratch, `${name}-results`),
    budget: new ContextBudget(model.models),
    defaultMaxChars: 2_000,
  });
  const inputs = new InputGate({ harness, observer, limiter });
  const history = new ConversationHistory(harness, inputs, model.models);
  const source = await harness.createConversation(
    { ownership: { kind: "ownerless" }, agent: { model: model.model } },
    context,
  );
  /** Sends `text` as the user and waits until the model answered `reply`. */
  const say = async (conversation: ConversationId, text: string, reply: string): Promise<void> => {
    model.script([answer(reply)]);
    const sent = await inputs.submit(
      conversation,
      {
        source: { kind: "user", request: `${conversation}:${text}` as UserRequestId },
        text,
        images: [],
        render: null,
        whenBusy: "reject",
      },
      context,
    );
    assert.equal(sent.kind, "submitted");
    await (await harness.conversation(conversation, context))?.waitForIdle(context);
  };
  const entries = async (conversation: ConversationId): Promise<readonly EntryRecord[]> => {
    const handle = await harness.conversation(conversation, context);
    assert.ok(handle, `no conversation ${conversation}`);
    return (await handle.entries({ order: "ascending" }, 100, undefined, context)).items;
  };
  /** The entry of the user input whose text is `text`. */
  const input = async (conversation: ConversationId, text: string): Promise<EntryId> => {
    const found = (await entries(conversation)).find((entry) => DimosInputEntry.is(entry) && entry.data.text === text);
    assert.ok(found, `no input "${text}"`);
    return found.id;
  };
  return { model, harness, inputs, history, source: source.id, requests, reports, answer, say, entries, input };
}

const DOWN =
  "<dimos_runtime_change>\nDimOS: not reachable, link connecting; any runtime stated earlier is as last seen.\n</dimos_runtime_change>";

test("an edit forks before the message, so the model gets the history before it and the edited text", async () => {
  const h = await open("edit");
  await h.say(h.source, "count the boxes", "4 boxes");
  await h.say(h.source, "and the crates", "2 crates");
  await h.say(h.source, "and the barrels", "1 barrel");
  const sourceEntries = (await h.entries(h.source)).map((entry) => entry.id);
  const crates = await h.input(h.source, "and the crates");
  h.model.script([h.answer("3 pallets")]);

  const edited = await h.history.edit(h.source, crates, "and the pallets", [], "edit-1" as UserRequestId, context);
  assert.equal(edited.kind, "fork");
  assert.ok(edited.kind === "fork");
  await (await h.harness.conversation(edited.conversation, context))?.waitForIdle(context);

  assert.deepEqual(h.requests.at(-1), [
    `user: ${DOWN} | count the boxes`,
    "assistant: 4 boxes",
    "user: and the pallets",
  ]);
  const again = await h.history.edit(h.source, crates, "and the pallets", [], "edit-1" as UserRequestId, context);
  assert.deepEqual(again, edited, "a retried edit returns the first fork");
  assert.equal(h.requests.length, 4, "a retried edit sends nothing new");
  assert.deepEqual(
    (await h.entries(edited.conversation)).map(
      (entry) => `${entry.conversationId === h.source ? "source" : "fork"} ${entry.kind}`,
    ),
    ["source dimos.input", "source pi.assistant", "fork dimos.input", "fork pi.assistant"],
  );
  assert.deepEqual(
    (await h.entries(h.source)).map((entry) => entry.id),
    sourceEntries,
    "the source is untouched",
  );
  assert.deepEqual(h.reports, []);
  await h.harness.close(context);
});

test("editing the first message starts the fork's context over at the edited text", async () => {
  const h = await open("first");
  await h.say(h.source, "count the boxes", "4 boxes");
  const first = await h.input(h.source, "count the boxes");
  h.model.script([h.answer("5 crates")]);

  const edited = await h.history.edit(h.source, first, "count the crates", [], "edit-first" as UserRequestId, context);
  assert.equal(edited.kind, "new");
  assert.ok(edited.kind === "new");
  await (await h.harness.conversation(edited.conversation, context))?.waitForIdle(context);

  assert.deepEqual(h.requests.at(-1), ["system: ", `user: ${DOWN} | count the crates`]);
  assert.deepEqual(h.reports, []);
  await h.harness.close(context);
});

test("summarize up to a message: the fork's model gets the summary, then the message onward", async () => {
  const h = await open("up-to");
  await h.say(h.source, "count the boxes", "4 boxes");
  await h.say(h.source, "and the crates", "2 crates");
  await h.say(h.source, "and the barrels", "1 barrel");
  const crates = await h.input(h.source, "and the crates");
  h.model.script([h.answer("Boxes: 4.")]);

  const summarized = await h.history.summarizeUpTo(h.source, crates, "up-to-1", context);
  assert.equal(summarized.kind, "fork");
  assert.ok(summarized.kind === "fork");
  await h.say(summarized.conversation, "and the pallets", "3 pallets");

  assert.deepEqual(h.requests.at(-1), [
    "system: ",
    "user: The conversation history before this point was compacted into the following summary:\n\n<summary>\nBoxes: 4.\n</summary>",
    "user: and the crates",
    "assistant: 2 crates",
    "user: and the barrels",
    "assistant: 1 barrel",
    `user: ${DOWN} | and the pallets`,
  ]);
  const summary = (await h.entries(summarized.conversation)).at(-4);
  assert.deepEqual(
    { kind: summary?.kind, head: summary?.head, data: summary?.data },
    {
      kind: "dimos.summary",
      head: crates,
      data: { from: { conversation: h.source, entry: crates }, text: "Boxes: 4." },
    },
  );
  assert.deepEqual(h.reports, []);
  await h.harness.close(context);
});

test("summarize from a message writes pi's branch summary into a fork before it", async () => {
  const h = await open("from");
  await h.say(h.source, "count the boxes", "4 boxes");
  await h.say(h.source, "and the crates", "2 crates");
  await h.say(h.source, "and the barrels", "1 barrel");
  const crates = await h.input(h.source, "and the crates");
  h.model.script([h.answer("## Goal\nCount crates and barrels")]);

  const summarized = await h.history.summarizeFrom(h.source, crates, "from-1", context);
  assert.equal(summarized.kind, "fork");
  assert.ok(summarized.kind === "fork");
  const text =
    "The user explored a different conversation branch before returning here.\nSummary of that exploration:\n\n## Goal\nCount crates and barrels";
  const written = (await h.entries(summarized.conversation)).at(-1);
  assert.deepEqual(
    { kind: written?.kind, head: written?.head, data: written?.data, model: written?.model?.map(line) },
    {
      kind: "dimos.branch_summary",
      head: undefined,
      data: { from: { conversation: h.source, entry: crates }, text },
      model: [
        `user: The following is a summary of a branch that this conversation came back from:\n\n<summary>\n${text}</summary>`,
      ],
    },
  );

  await h.say(summarized.conversation, "and the pallets", "3 pallets");
  assert.deepEqual(h.requests.at(-1), [
    `user: ${DOWN} | count the boxes`,
    "assistant: 4 boxes",
    `user: The following is a summary of a branch that this conversation came back from:\n\n<summary>\n${text}</summary>`,
    "user: and the pallets",
  ]);
  assert.deepEqual(h.reports, []);
  await h.harness.close(context);
});

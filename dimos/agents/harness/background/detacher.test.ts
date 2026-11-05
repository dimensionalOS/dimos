import assert from "node:assert/strict";
import type { CallId, Detachable } from "#dimos/agents/shared/protocol/conversation/calls.ts";
import type { ConversationId, EntryRecord } from "@earendil-works/pi-durable";
import type { DimosInstall } from "#dimos/agents/harness/dimos/install.ts";
import type { FauxResponseStep, Message } from "@earendil-works/pi-ai";
import type { UserRequestId } from "#dimos/agents/shared/protocol/conversation/input.ts";
import { BACKGROUND_CONTEXT } from "@earendil-works/chord/context";
import { ContextBudget } from "#dimos/agents/harness/limit/budget.ts";
import { Detacher } from "#dimos/agents/harness/background/detacher.ts";
import { DimosConnection } from "#dimos/agents/harness/dimos/connection.ts";
import { DimosInputEntry } from "#dimos/agents/harness/inputs/source.ts";
import { FileRenderer } from "#dimos/agents/shared/render/render.ts";
import { Harness, ToolResultEntry, createRegistry } from "@earendil-works/pi-durable";
import { InputGate } from "#dimos/agents/harness/inputs/gate.ts";
import { OutputLimiter } from "#dimos/agents/harness/limit/limit.ts";
import { RuntimeClient } from "#dimos/agents/harness/dimos/client.ts";
import { RuntimeObserver } from "#dimos/agents/harness/context/observer.ts";
import { SqliteStorage } from "@earendil-works/pi-durable/storage/sqlite";
import { after, before, test } from "node:test";
import { bashTool } from "#dimos/agents/harness/tools/bash.ts";
import { executionEnv } from "#dimos/agents/harness/env.ts";
import { fauxAssistantMessage, fauxToolCall } from "@earendil-works/pi-ai";
import { join } from "node:path";
import { mkdirSync, mkdtempSync, rmSync } from "node:fs";
import { openNodeSqliteDatabase } from "@earendil-works/pi-durable/storage/sqlite/node";
import { scriptedModel } from "#dimos/agents/test/scripted-model.ts";
import { tmpdir } from "node:os";
import { toolsExtension } from "#dimos/agents/harness/tools/extension.ts";

const context = BACKGROUND_CONTEXT;

let scratch = "";

before(() => {
  scratch = mkdtempSync(join(tmpdir(), "dimos-background-"));
  mkdirSync(join(scratch, "runs"));
});

after(() => {
  rmSync(scratch, { recursive: true, force: true });
});

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

async function until(check: () => boolean, label: string): Promise<void> {
  const deadline = Date.now() + 10_000;
  while (!check()) {
    if (Date.now() > deadline) throw new Error(`timed out waiting for ${label}`);
    await new Promise((resolve) => setTimeout(resolve, 10));
  }
}

/** pi-ai's faux model that first calls bash with `command` as `call_bg`, then answers each later request in turn. */
function bashModel(command: string, answers: readonly string[]) {
  const model = scriptedModel();
  const requests: string[][] = [];
  const recorded =
    (reply: () => ReturnType<typeof fauxAssistantMessage>): FauxResponseStep =>
    (request) => {
      requests.push(request.messages.map(line));
      return reply();
    };
  model.script([
    recorded(() =>
      fauxAssistantMessage([fauxToolCall("bash", { command }, { id: "call_bg" })], { stopReason: "toolUse" }),
    ),
    ...answers.map((text) => recorded(() => fauxAssistantMessage(text))),
  ]);
  return { model, requests };
}

/** A gateway's harness over the store at `name`: the real bash tool, wrapped by the Detacher. DimOS is down. */
async function open(name: string, model: ReturnType<typeof scriptedModel>, reports: unknown[]) {
  const install: DimosInstall = {
    root: scratch,
    python: "/usr/bin/python3",
    cli: "/usr/bin/false",
    mcpUrl: new URL("http://127.0.0.1:9/mcp"),
    runsDir: join(scratch, "runs"),
    stateDir: join(scratch, "state"),
  };
  const registry = createRegistry();
  const storage = await SqliteStorage.open(await openNodeSqliteDatabase(join(scratch, `${name}.sqlite`)));
  const harness = await Harness.open(
    storage,
    { models: model.models, registry, env: executionEnv(() => scratch), onReport: (error) => reports.push(error) },
    context,
  );
  const connection = new DimosConnection({
    url: install.mcpUrl,
    connectTimeoutMs: 1_000,
    maxBackoffMs: 1_000,
    report: (error) => reports.push(error),
  });
  await connection.close();
  const limiter = new OutputLimiter({
    dir: join(scratch, `${name}-results`),
    budget: new ContextBudget(model.models),
    defaultMaxChars: 2_000,
  });
  const inputs = new InputGate({
    harness,
    observer: new RuntimeObserver({
      harness,
      runtime: new RuntimeClient(connection),
      jobFacts: async () => ({}),
      runsDir: install.runsDir,
      reuseForMs: 0,
      timeoutMs: 1_000,
      report: (error) => reports.push(error),
    }),
    limiter,
  });
  const detacher = new Detacher(inputs, (error) => reports.push(error));
  const renderer = new FileRenderer({
    cacheDir: join(scratch, "render"),
    maxCacheBytes: 2 ** 20,
    maxSourceBytes: 2 ** 20,
  });
  registry.install(
    toolsExtension(
      {
        read: null,
        bash: bashTool({ install, gatewayEnv: process.env, renderer }),
        edit: null,
        write: null,
        inventory: null,
        stream_get: null,
        agent_module: null,
        job: null,
        background: null,
      },
      limiter,
      detacher,
    ),
  );
  registry.install(detacher.extension());
  harness.resume();
  const entries = async (conversation: ConversationId): Promise<readonly EntryRecord[]> => {
    const handle = await harness.conversation(conversation, context);
    assert.ok(handle, `no conversation ${conversation}`);
    return (await handle.entries({ order: "ascending" }, 100, undefined, context)).items;
  };
  const ask = (conversation: ConversationId, text: string) =>
    inputs.submit(
      conversation,
      { source: { kind: "user", request: text as UserRequestId }, text, images: [], render: null, whenBusy: "reject" },
      context,
    );
  return { harness, inputs, detacher, entries, ask };
}

function toolDetails(entries: readonly EntryRecord[]): unknown {
  const result = entries.find((entry) => ToolResultEntry.is(entry))?.model?.[0];
  return result?.role === "toolResult" ? result.details : null;
}

test("a bash call sent to the background returns at once, and its output comes back as a notice", async () => {
  const { model, requests } = bashModel("echo scanning; sleep 1.5; echo survey done", [
    "I will report when the survey finishes.",
    "The survey is done.",
  ]);
  const reports: unknown[] = [];
  const gateway = await open("moved", model, reports);
  const { id } = await gateway.harness.createConversation(
    { ownership: { kind: "ownerless" }, agent: { model: model.model } },
    context,
  );
  const detachable = gateway.detacher.of(id);
  const seen: (readonly Detachable[])[] = [];
  detachable.subscribe((status) => seen.push(status));

  const started = Date.now();
  await gateway.ask(id, "scan the room");
  await until(() => detachable.value.length === 1, "the bash call to be detachable");
  assert.deepEqual(
    detachable.value.map((call) => ({ call: call.call, tool: call.tool })),
    [{ call: "call_bg", tool: "bash" }],
  );

  const detached = await gateway.detacher.detach(id, "call_bg" as CallId, context);
  assert.equal(detached.kind, "detached");
  assert.ok(detached.kind === "detached");
  await (await gateway.harness.conversation(id, context))?.waitForIdle(context);
  const idleAfterMs = Date.now() - started;
  assert.ok(idleAfterMs < 1_500, `the run ended after ${idleAfterMs} ms, before the 1.5 s command did`);
  assert.deepEqual(requests.at(-1)?.slice(-2), [
    'assistant: bash({"command":"echo scanning; sleep 1.5; echo survey done"})',
    "toolResult: Moved to the background as call call_bg. Its result arrives later as a message; the background tool lists it, reads its output so far, or stops it.",
  ]);
  assert.deepEqual(detachable.value, []);
  assert.equal(
    (await gateway.detacher.detach(id, "call_bg" as CallId, context)).kind,
    "not_running",
    "a moved call cannot move twice",
  );

  await until(() => requests.length === 3, "the background notice's turn");
  await (await gateway.harness.conversation(id, context))?.waitForIdle(context);
  assert.deepEqual(requests.at(-1)?.slice(-1), [
    "user: [Background] bash call call_bg finished:\nscanning\nsurvey done\n",
  ]);
  const entries = await gateway.entries(id);
  assert.deepEqual(toolDetails(entries), {
    truncation: null,
    fullOutputPath: null,
    exitCode: null,
    render: null,
    background: { task: detached.task },
    limit: null,
  });
  const notice = entries.findLast((entry) => DimosInputEntry.is(entry));
  assert.deepEqual(DimosInputEntry.is(notice) ? notice.data.source : null, {
    kind: "background",
    task: detached.task,
    call: "call_bg",
    tool: "bash",
    end: "finished",
  });
  assert.deepEqual(
    seen.map((status) => status.map((call) => call.call)),
    [[], ["call_bg"], []],
  );
  assert.deepEqual(reports, []);
  await gateway.harness.close(context);
});

test("a wrapped bash call that is not moved settles as bash always did", async () => {
  const { model, requests } = bashModel("echo quick", ["It printed quick."]);
  const reports: unknown[] = [];
  const gateway = await open("kept", model, reports);
  const { id } = await gateway.harness.createConversation(
    { ownership: { kind: "ownerless" }, agent: { model: model.model } },
    context,
  );

  await gateway.ask(id, "say quick");
  await (await gateway.harness.conversation(id, context))?.waitForIdle(context);

  assert.deepEqual(requests.at(-1)?.slice(-1), ["toolResult: quick\n"]);
  assert.deepEqual(toolDetails(await gateway.entries(id)), {
    truncation: null,
    fullOutputPath: null,
    exitCode: 0,
    render: null,
    limit: null,
  });
  assert.equal((await gateway.detacher.detach(id, "call_bg" as CallId, context)).kind, "not_running");
  assert.deepEqual(reports, []);
  await gateway.harness.close(context);
});

test("work in the background when the gateway restarts comes back as an interrupted notice", async () => {
  const { model, requests } = bashModel("sleep 31.5", ["Waiting for it.", "It was lost; I will run it again."]);
  const reports: unknown[] = [];
  const first = await open("restart", model, reports);
  const { id } = await first.harness.createConversation(
    { ownership: { kind: "ownerless" }, agent: { model: model.model } },
    context,
  );
  await first.ask(id, "wait half a minute");
  await until(() => first.detacher.of(id).value.length === 1, "the bash call to be detachable");
  const detached = await first.detacher.detach(id, "call_bg" as CallId, context);
  assert.ok(detached.kind === "detached");
  await (await first.harness.conversation(id, context))?.waitForIdle(context);
  const closing = Date.now();
  await first.harness.close(context);
  assert.ok(Date.now() - closing < 5_000, "closing stopped the 31.5 s command instead of waiting for it");

  const second = await open("restart", model, reports);
  await until(() => requests.length === 3, "the interrupted notice's turn");
  await (await second.harness.conversation(id, context))?.waitForIdle(context);
  assert.deepEqual(requests.at(-1)?.slice(-1), [
    "user: [Background] bash call call_bg was lost: the gateway restarted while it ran.",
  ]);
  const notice = (await second.entries(id)).findLast((entry) => DimosInputEntry.is(entry));
  assert.deepEqual(DimosInputEntry.is(notice) ? notice.data.source : null, {
    kind: "background",
    task: detached.task,
    call: "call_bg",
    tool: "bash",
    end: "interrupted",
  });
  assert.deepEqual(reports, []);
  await second.harness.close(context);
});

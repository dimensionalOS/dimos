import assert from "node:assert/strict";
import type { ChildProcess } from "node:child_process";
import type { TestContext } from "node:test";
import type { LinkStatus } from "#dimos/agents/shared/protocol/gateway/link.ts";
import { BACKGROUND_CONTEXT } from "@earendil-works/chord/context";
import { ChildAgent } from "#dimos/agents/test/child-driver.ts";
import { after, before, test } from "node:test";
import { createServer } from "node:net";
import { existsSync, mkdirSync, mkdtempSync, readFileSync, rmSync, writeFileSync } from "node:fs";
import { fauxAssistantMessage, fauxToolCall } from "@earendil-works/pi-ai";
import { join } from "node:path";
import { openAgent } from "#dimos/agents/harness/main.ts";
import { parseDimosInstall } from "#dimos/agents/gateway/config/environment.ts";
import { scriptedModel } from "#dimos/agents/test/scripted-model.ts";
import { spawn } from "node:child_process";
import { tmpdir } from "node:os";

const CHILD = join(import.meta.dirname, "..", "test", "agent-child.ts");

let scratch = "";

let env: NodeJS.ProcessEnv = {};

function sleep(ms: number): Promise<void> {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

async function until(check: () => boolean, label: string, timeoutMs = 30_000): Promise<void> {
  const deadline = Date.now() + timeoutMs;
  while (!check()) {
    if (Date.now() > deadline) throw new Error(`timed out waiting for ${label}`);
    await sleep(50);
  }
}

function freePort(): Promise<number> {
  return new Promise((resolve, reject) => {
    const server = createServer();
    server.listen(0, "127.0.0.1", () => {
      const address = server.address();
      server.close(() =>
        typeof address === "object" && address !== null ? resolve(address.port) : reject(new Error("no port")),
      );
    });
  });
}

function child(phase: string, test: TestContext): ChildProcess {
  const child = spawn(process.execPath, [CHILD], {
    env: { ...env, AGENT_CHILD_PHASE: phase },
    stdio: ["ignore", "inherit", "inherit"],
  });
  test.after(() => {
    child.kill("SIGKILL");
  });
  return child;
}

function exited(process: ChildProcess): Promise<number | null> {
  return new Promise((resolve) => process.once("exit", (code) => resolve(code)));
}

function marker(name: string): unknown {
  return JSON.parse(readFileSync(join(scratch, name), "utf8"));
}

before(async () => {
  scratch = mkdtempSync(join(tmpdir(), "dimos-agent-test-"));
  mkdirSync(join(scratch, "root"));
  mkdirSync(join(scratch, "runs"));
  mkdirSync(join(scratch, "bin"));
  writeFileSync(join(scratch, "root", ".env"), "FROM_DOTENV=leaked\n");
  writeFileSync(join(scratch, "bin", "python"), "#!/bin/sh\necho fake python\n", { mode: 0o755 });
  writeFileSync(join(scratch, "long-file"), "x".repeat(200));
  const port = await freePort();
  env = {
    ...process.env,
    AGENT_CHILD_SCRATCH: scratch,
    DIMOS_ROOT: join(scratch, "root"),
    DIMOS_PYTHON: join(scratch, "bin", "python"),
    DIMOS_CLI: join(scratch, "bin", "dimos"),
    DIMOS_MCP_URL: `http://127.0.0.1:${port}/mcp`,
    DIMOS_RUNS_DIR: join(scratch, "runs"),
    DIMOS_STATE_DIR: join(scratch, "state", "dimos"),
    FAKE_API_KEY: "secret",
  };
});

after(() => {
  rmSync(scratch, { recursive: true, force: true });
});

test("a turn survives SIGKILL during a bash call and during a model request", async (test) => {
  const first = child("turn-1", test);
  await until(() => existsSync(join(scratch, "bash-started")), "bash to start");
  first.kill("SIGKILL");
  await exited(first);
  assert.ok(!existsSync(join(scratch, "seen-1.json")), "the model was not asked again before the kill");

  const second = child("turn-2", test);
  await until(() => existsSync(join(scratch, "seen-1.json")), "the resumed turn to ask the model");
  second.kill("SIGKILL");
  await exited(second);

  const third = child("turn-3", test);
  assert.equal(await exited(third), 0, "third run exits cleanly");

  const seen1 = marker("seen-1.json") as { role: string; text: string }[];
  const seen2 = marker("seen-2.json") as { role: string; text: string }[];
  const bashResult = seen1.find((message) => message.role === "toolResult:bash");
  assert.ok(
    bashResult !== undefined && /interrupted/.test(bashResult.text),
    `the interrupted bash reached the model as interrupted: ${JSON.stringify(bashResult)}`,
  );
  assert.deepEqual(seen2, seen1, "the interrupted model request was sent again with the same context");

  const result = marker("result.json") as {
    status: string;
    reason: string | null;
    answer: string | null;
    toolResults: {
      toolCallId: string | null;
      isError: boolean | null;
      details: { limit?: { maxChars: number; total: number } | null; exitCode?: number | null } | null;
      summary: { text: string } | null;
    }[];
  };
  assert.equal(result.status, "done", `submission settled: ${result.reason}`);
  assert.equal(result.answer, "all done");

  const cut = result.toolResults.find((item) => item.toolCallId === "call_read_cut");
  assert.ok(cut !== undefined, "the cut read's result is stored");
  assert.ok(
    cut.summary?.text.includes("[… 150 of 200 chars cut. Full: "),
    `afterTool saw max_chars: ${cut.summary?.text}`,
  );
  assert.equal(cut.details?.limit?.maxChars, 50);
  assert.equal(cut.details?.limit?.total, 200);

  const probe = result.toolResults.find((item) => item.toolCallId === "call_env");
  assert.ok(probe !== undefined, "the environment probe's result is stored");
  assert.equal(probe.isError, false);
  assert.equal(probe.details?.exitCode, 0);
  assert.equal(probe.summary?.text.split("\n")[0], `${join(scratch, "root")}||${join(scratch, "bin", "python")}|`);
});

test("the agent runs with no DimOS at all: the model answers and runtime tool calls are error results", async () => {
  const dir = mkdtempSync(join(tmpdir(), "dimos-agent-nodimos-"));
  mkdirSync(join(dir, "root"));
  mkdirSync(join(dir, "runs"));
  const child = ChildAgent.start(dir, {
    ...env,
    DIMOS_ROOT: join(dir, "root"),
    DIMOS_RUNS_DIR: join(dir, "runs"),
    DIMOS_STATE_DIR: join(dir, "state", "dimos"),
  });
  try {
    await child.call({
      op: "script",
      messages: [
        fauxAssistantMessage(
          [
            fauxToolCall("inventory", {}, { id: "call_inventory" }),
            fauxToolCall("stream_get", { op: "read", channel: "HarnessProbe.position" }, { id: "call_stream" }),
            fauxToolCall("agent_module", { op: "list" }, { id: "call_modules" }),
          ],
          { stopReason: "toolUse" },
        ),
        fauxAssistantMessage("no robot here"),
      ],
    });
    const { conversation } = await child.call<{ conversation: number }>({ op: "start" });
    const turn = await child.call<{ status: string; answer: string | null }>(
      { op: "submit", conversation, text: "what is running?", request: "req-nodimos" },
      120_000,
    );
    assert.deepEqual(turn, { status: "done", reason: null, answer: "no robot here" });
    const results = await child.call<({ toolCallId: string; isError: boolean; text: string } | null)[]>({
      op: "toolResults",
      conversation,
    });
    assert.deepEqual(
      results.map((result) => [result?.toolCallId, result?.isError]).sort(),
      [
        ["call_inventory", true],
        ["call_modules", true],
        ["call_stream", true],
      ],
      `each runtime tool call is an error result: ${JSON.stringify(results)}`,
    );
    const streamResult = results.find((result) => result?.toolCallId === "call_stream");
    assert.ok(streamResult);
    assert.match(streamResult.text, /DimOS is not reachable:/);
    const status = await child.call<{ link: { state: string }; load: { state: string } }>({ op: "status" });
    assert.notEqual(status.link.state, "up");
    assert.equal(status.load.state, "no_system");
    assert.ok(child.alive, "the agent process is still running");
    assert.equal(child.crash, null);
    assert.equal(await child.close(), 0, "the agent closes cleanly");
  } finally {
    if (child.alive) child.kill();
    rmSync(dir, { recursive: true, force: true });
  }
});

test("runtime loading wakes the MCP connection from backoff", { timeout: 30_000 }, async () => {
  const dir = mkdtempSync(join(tmpdir(), "dimos-agent-retry-"));
  const cli = join(dir, "dimos");
  mkdirSync(join(dir, "runs"));
  writeFileSync(cli, `#!/bin/sh\nprintf '%s\\n' '{"loaded":["HarnessRuntime","McpServer"],"already":[]}'\n`, {
    mode: 0o755,
  });
  const gatewayEnv = { ...env, DIMOS_CLI: cli, DIMOS_RUNS_DIR: join(dir, "runs") };
  const model = scriptedModel();
  const agent = await openAgent(
    {
      install: parseDimosInstall(gatewayEnv),
      models: model.models,
      storePath: join(dir, "agent.sqlite"),
      toolResultsDir: join(dir, "tool-results"),
      renderCacheDir: join(dir, "render"),
      skillRoots: [],
      defaults: () => ({ model: model.model, thinkingLevel: null, workspace: join(scratch, "root") }),
      gatewayEnv,
      report: () => {},
    },
    BACKGROUND_CONTEXT,
  );
  try {
    await until(() => {
      const status = agent.connection.status.value;
      return status.state === "down" && status.retryAt - Date.now() > 1_500;
    }, "the MCP connection to enter backoff");
    const before = agent.loader.status.value;
    const loaded = new Promise<LinkStatus>((resolve) => {
      const stop = agent.loader.status.subscribe((status) => {
        if (status === before || status.state !== "loaded") return;
        stop();
        resolve(agent.connection.status.value);
      });
    });
    agent.loader.trigger("tools_missing");
    const status = await loaded;
    assert.equal(status.state, "connecting", "runtime readiness starts a connection without waiting for backoff");
    assert.ok(status.state === "connecting");
    assert.equal(status.attempt, 0, "runtime readiness resets the connection backoff");
    const connecting = agent.connection.status.value;
    agent.connection.retryNow();
    assert.equal(agent.connection.status.value, connecting, "a readiness signal leaves an in-flight connection alone");
  } finally {
    await agent.close(BACKGROUND_CONTEXT);
    rmSync(dir, { recursive: true, force: true });
  }
});

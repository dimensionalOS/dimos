import assert from "node:assert/strict";
import type { Agent } from "../main.ts";
import type { ChildProcess } from "node:child_process";
import type { Socket } from "node:net";
import type { UserRequestId } from "../../shared/protocol/conversation/input.ts";
import { BACKGROUND_CONTEXT } from "@earendil-works/chord/context";
import { after, before, test } from "node:test";
import { createServer } from "node:net";
import { existsSync, mkdirSync, mkdtempSync, readFileSync, rmSync } from "node:fs";
import { fauxAssistantMessage, fauxToolCall } from "@earendil-works/pi-ai";
import { join } from "node:path";
import { listJobs } from "./registry.ts";
import { openAgent } from "../main.ts";
import { parseDimosInstall } from "../../gateway/config/environment.ts";
import { scriptedModel } from "../../test/scripted-model.ts";
import { spawn } from "node:child_process";
import { tmpdir } from "node:os";

const CHILD = join(import.meta.dirname, "..", "..", "test", "agent-child.ts");

const HOUR = 3_600_000;

const DAY = 24 * HOUR;

const context = BACKGROUND_CONTEXT;

let scratch = "";

let closedPort = 0;

function sleep(ms: number): Promise<void> {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

async function until(check: () => Promise<boolean> | boolean, label: string, timeoutMs = 30_000): Promise<void> {
  const deadline = Date.now() + timeoutMs;
  while (!(await check())) {
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

/** The next midnight UTC strictly after `at`, which is what cron `0 0 * * *` in UTC means. */
function nextMidnightUtc(at: number): number {
  const date = new Date(at);
  return Date.UTC(date.getUTCFullYear(), date.getUTCMonth(), date.getUTCDate() + 1);
}

function dimosEnv(dir: string, mcpPort: number): NodeJS.ProcessEnv {
  return {
    ...process.env,
    DIMOS_ROOT: join(dir, "root"),
    DIMOS_PYTHON: join(dir, "bin", "python"),
    DIMOS_CLI: join(dir, "bin", "dimos"),
    DIMOS_MCP_URL: `http://127.0.0.1:${mcpPort}/mcp`,
    DIMOS_RUNS_DIR: join(dir, "runs"),
    DIMOS_STATE_DIR: join(dir, "state", "dimos"),
  };
}

before(async () => {
  scratch = mkdtempSync(join(tmpdir(), "dimos-agent-jobs-"));
  closedPort = await freePort();
});

after(() => {
  rmSync(scratch, { recursive: true, force: true });
});

test("a notify job due while the agent was closed runs once, late, after reopening", async () => {
  const dir = join(scratch, "notify");
  mkdirSync(join(dir, "root"), { recursive: true });
  mkdirSync(join(dir, "runs"));
  const env = dimosEnv(dir, closedPort);
  const clock = { now: Date.now() };
  const model = scriptedModel();
  const open = (): Promise<Agent> =>
    openAgent(
      {
        install: parseDimosInstall(env),
        models: model.models,
        storePath: join(dir, "agent.sqlite"),
        toolResultsDir: join(dir, "tool-results"),
        renderCacheDir: join(dir, "render"),
        skillRoots: [],
        defaults: () => ({ model: model.model, thinkingLevel: null, workspace: join(dir, "root") }),
        gatewayEnv: env,
        report: (error) => console.error("[notify job report]", error),
        now: () => clock.now,
      },
      context,
    );

  model.script([
    fauxAssistantMessage(
      [
        fauxToolCall(
          "job",
          { op: "create", name: "nightly", cron: "0 0 * * *", timezone: "UTC", run: { notify: "time to patrol" } },
          { id: "call_job" },
        ),
      ],
      { stopReason: "toolUse" },
    ),
    fauxAssistantMessage("created"),
  ]);
  let agent = await open();
  const conversation = await agent.conversations.start(null, "start-notify", context);
  const submitted = await agent.inputs.submit(
    conversation,
    {
      source: { kind: "user", request: "req-create" as UserRequestId },
      text: "set up",
      images: [],
      render: null,
      whenBusy: "reject",
    },
    context,
  );
  assert.equal(submitted.kind, "submitted");
  if (submitted.kind !== "submitted") return;
  const creation = await agent.harness.submission(submitted.submission, context);
  assert.equal((await creation?.wait(context))?.status, "done");
  const [job] = await listJobs(agent.harness, context);
  assert.ok(job !== undefined, "the job tool stored the job");
  assert.equal(job.name, "nightly");
  assert.deepEqual(job.runs, []);
  const due = nextMidnightUtc(clock.now);
  assert.deepEqual((await agent.harness.getTask(job.task, context))?.state.checkpoint, { phase: "sleeping", due });
  await agent.close(context);

  clock.now = due + 2 * HOUR;
  model.script([fauxAssistantMessage("noted")]);
  agent = await open();
  await until(async () => ((await listJobs(agent.harness, context))[0]?.runs.length ?? 0) > 0, "the late run");
  const [fired] = await listJobs(agent.harness, context);
  assert.deepEqual(fired?.runs, [
    { run: { kind: "scheduled", due }, firedAt: due + 2 * HOUR, outcome: { kind: "submitted" } },
  ]);
  const notify = await agent.harness.commit(
    (tx) => tx.submissionByRequest(conversation, `job:nightly:${due}`),
    context,
  );
  assert.ok(notify !== undefined, "the notify run was submitted under its request id");
  const settled = await (await agent.harness.submission(notify.id, context))?.wait(context);
  assert.equal(settled?.status, "done");
  await until(async () => {
    const checkpoint = (await agent.harness.getTask(job.task, context))?.state.checkpoint;
    return (
      typeof checkpoint === "object" &&
      checkpoint !== null &&
      !Array.isArray(checkpoint) &&
      checkpoint["phase"] === "sleeping"
    );
  }, "the task to sleep again");
  assert.deepEqual((await agent.harness.getTask(job.task, context))?.state.checkpoint, {
    phase: "sleeping",
    due: due + DAY,
  });
  await agent.close(context);

  agent = await open();
  await sleep(500);
  assert.equal(
    (await listJobs(agent.harness, context))[0]?.runs.length,
    1,
    "a reopen with the clock unmoved fires nothing",
  );
  await agent.close(context);
});

test("a skill job interrupted after attempted is recorded is not called again", async () => {
  const dir = join(scratch, "skill");
  mkdirSync(join(dir, "root"), { recursive: true });
  mkdirSync(join(dir, "runs"));
  const sockets: Socket[] = [];
  const silent = createServer((socket) => sockets.push(socket));
  await new Promise<void>((resolve) => silent.listen(0, "127.0.0.1", resolve));
  const address = silent.address();
  const silentPort = typeof address === "object" && address !== null ? address.port : 0;
  const child = (phase: string, mcpPort: number, clock: number): ChildProcess =>
    spawn(process.execPath, [CHILD], {
      env: {
        ...dimosEnv(dir, mcpPort),
        AGENT_CHILD_SCRATCH: dir,
        AGENT_CHILD_PHASE: phase,
        AGENT_CHILD_CLOCK: String(clock),
      },
      stdio: ["ignore", "inherit", "inherit"],
    });
  const exited = (process: ChildProcess): Promise<number | null> =>
    new Promise((resolve) => process.once("exit", (code) => resolve(code)));
  const marker = (name: string): unknown => JSON.parse(readFileSync(join(dir, name), "utf8"));
  try {
    const start = Date.now();
    assert.equal(await exited(child("job-create", closedPort, start)), 0, "the job was created");
    const job = marker("job.json") as { due: number };
    assert.equal(job.due, nextMidnightUtc(start));

    const late = job.due + 2 * HOUR;
    const attempting = child("job-attempted", silentPort, late);
    await until(() => existsSync(join(dir, "attempted.json")), "the attempted checkpoint");
    assert.deepEqual(marker("attempted.json"), { phase: "firing", due: job.due, attempted: true });
    assert.ok(sockets.length > 0, "the skill call reached the silent listener and was waiting on it");
    attempting.kill("SIGKILL");
    await exited(attempting);

    assert.equal(await exited(child("job-after", closedPort, late)), 0, "the reopened agent exits cleanly");
    const after = marker("runs.json") as { runs: unknown; checkpoint: unknown };
    assert.deepEqual(after.runs, [
      { run: { kind: "scheduled", due: job.due }, firedAt: late, outcome: { kind: "interrupted" } },
    ]);
    assert.deepEqual(after.checkpoint, { phase: "sleeping", due: job.due + DAY });
  } finally {
    for (const socket of sockets) socket.destroy();
    await new Promise<void>((resolve) => silent.close(() => resolve()));
  }
});

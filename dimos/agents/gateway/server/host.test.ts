import assert from "node:assert/strict";
import type { AgentModuleName } from "../../shared/protocol/runtime/event.ts";
import type { AgentPaths } from "../config/paths.ts";
import type { ChildProcess } from "node:child_process";
import type { Host, Readable, Ref, WebSocketLike } from "@earendil-works/chord";
import type { ModelRef } from "@earendil-works/pi-durable";
import type { Provider } from "@earendil-works/pi-ai";
import type { TerminalId } from "../../shared/protocol/auth.ts";
import { ConfigStore } from "../config/store.ts";
import { ConversationContract } from "../../shared/protocol/conversation/contract.ts";
import { GatewayContract } from "../../shared/protocol/gateway/contract.ts";
import { GatewayRunning, GatewayServer } from "./host.ts";
import { LifecycleContract } from "../../shared/protocol/lifecycle/contract.ts";
import { agentPaths } from "../config/paths.ts";
import { createHost, webSocketLink } from "@earendil-works/chord";
import { createServer } from "node:net";
import { unixSocket } from "../../sdk/node.ts";
import { dirname, join } from "node:path";
import { fauxAssistantMessage } from "@earendil-works/pi-ai";
import { gatewayState, stopGateway } from "../../cli/process.ts";
import { mkdtemp, rm, stat } from "node:fs/promises";
import { newActionRequest, newUserRequest } from "../../shared/protocol/actions/request.ts";
import { once } from "node:events";
import { parseJobName } from "../../harness/jobs/registry.ts";
import { parseTimeZone } from "../../harness/jobs/cron.ts";
import { randomUUID } from "node:crypto";
import { reachable } from "../../shared/socket.ts";
import { runGateway } from "../main.ts";
import { scriptedModel } from "../../test/scripted-model.ts";
import { setTimeout as delay } from "node:timers/promises";
import { spawn } from "node:child_process";
import { test } from "node:test";
import { tmpdir } from "node:os";

const PACKAGE = dirname(dirname(import.meta.dirname));

const CHECKOUT = dirname(dirname(PACKAGE));

type ClientPrincipal = { readonly kind: "gateway" };

type Client = { readonly host: Host<ClientPrincipal>; readonly socket: WebSocketLike };

type OwnerConversation = Extract<Ref<typeof ConversationContract>, { readonly variant: "owner" }>;

/** A free TCP port nothing listens on, so the gateway's DimOS link stays down. */
async function freePort(): Promise<number> {
  const server = createServer();
  server.listen(0, "127.0.0.1");
  await once(server, "listening");
  const address = server.address();
  server.close();
  if (address === null || typeof address === "string") throw new Error("no port");
  return address.port;
}

/** A scratch home for every file the gateway owns; the caller removes it after whatever runs in it stopped. */
async function scratch(): Promise<{ readonly root: string; readonly paths: AgentPaths; remove(): Promise<void> }> {
  const root = await mkdtemp(join(tmpdir(), "dimos-agent-gateway-"));
  const paths = agentPaths({
    HOME: root,
    XDG_CONFIG_HOME: join(root, "config"),
    XDG_STATE_HOME: join(root, "state"),
    XDG_CACHE_HOME: join(root, "cache"),
    XDG_RUNTIME_DIR: join(root, "run"),
  });
  return { root, paths, remove: () => rm(root, { recursive: true, force: true }) };
}

function opened(socket: WebSocketLike): Promise<void> {
  return new Promise((resolve, reject) => {
    socket.addEventListener("open", () => resolve());
    socket.addEventListener("error", () => reject(new Error("the socket failed to open")));
  });
}

/**
 * A Chord client host over the real socket through `ws`, as every Node frontend connects. The link wraps the socket
 * before its `open` event: the gateway's `hello` can arrive in the same read as the upgrade response, and the stream
 * flows on nextTick, before a promise continuation would attach the link's listener.
 */
async function connect(paths: AgentPaths, terminal: TerminalId, label: string): Promise<Client> {
  const socket = unixSocket({ path: paths.socket, terminal, label })();
  const open = opened(socket);
  const host = createHost<ClientPrincipal>({ name: `client-${label}`, self: { kind: "gateway" }, kind: "client" });
  host.connect(webSocketLink(socket), { principal: { kind: "gateway" } });
  try {
    await open;
  } catch (error) {
    await host.close();
    throw error;
  }
  return { host, socket };
}

/** Resolves with the first non-undefined pick of the readable's values, now or later. */
function until<T, R>(readable: Readable<T>, pick: (value: T) => R | undefined): Promise<R> {
  return new Promise((resolve) => {
    let settled = false;
    // The listener runs once synchronously inside `subscribe`, before `stop` exists; the unsubscribe waits a microtask.
    const stop = readable.subscribe((value) => {
      if (settled) return;
      const picked = pick(value);
      if (picked === undefined) return;
      settled = true;
      resolve(picked);
      queueMicrotask(() => stop());
    });
  });
}

/**
 * The real gateway composition root in this process, with the faux provider beside the built-in ones; it answers once
 * this resolves, and the test's `after` hook stops it through the lifecycle object if the test did not.
 */
async function startGateway(
  t: { after(fn: () => Promise<void>): void },
  root: string,
  paths: AgentPaths,
  faux: Provider,
  model: ModelRef,
): Promise<{ readonly stopped: Promise<void> }> {
  await (await ConfigStore.open(paths.configFile)).update({ model, workspace: root });
  const running = runGateway({
    paths,
    install: {
      root: CHECKOUT,
      python: join(CHECKOUT, ".venv", "bin", "python"),
      cli: join(CHECKOUT, ".venv", "bin", "dimos"),
      mcpUrl: new URL(`http://127.0.0.1:${await freePort()}/mcp`),
      runsDir: join(root, "runs"),
      stateDir: join(root, "state", "dimos"),
    },
    bundledSkillsDir: join(PACKAGE, "harness", "skills"),
    env: { PATH: process.env.PATH, HOME: root },
    providers: [faux],
  });
  t.after(async () => {
    if (!(await reachable(paths.socket, 500))) return;
    try {
      const stopper = await connect(paths, randomUUID() as TerminalId, "stop");
      const lifecycle = await stopper.host.get(LifecycleContract, {});
      if (lifecycle.variant === "owner") await lifecycle.stop({});
      await stopper.host.close();
    } catch {
      // The lifecycle object did not answer; the gateway's SIGTERM handler is the other shutdown path.
      process.kill(process.pid, "SIGTERM");
    }
    await running;
  });
  // The socket listens before the objects are served, so wait until a client is accepted, not just connected.
  const deadline = Date.now() + 60_000;
  for (;;) {
    try {
      const probe = await connect(paths, randomUUID() as TerminalId, "probe");
      await probe.host.close();
      return { stopped: running };
    } catch {
      if (Date.now() > deadline) throw new Error("the gateway did not start serving within 60 s");
      await delay(50);
    }
  }
}

test(
  "multiple owners and an explicit viewer, idempotent sends, outcomes, a dead link and a reconnect",
  { timeout: 120_000 },
  async (t) => {
    const { root, paths, remove } = await scratch();
    const scripted = scriptedModel();
    const faux = scripted.models.getProvider("faux");
    assert(faux !== undefined);
    scripted.script([fauxAssistantMessage("The first answer."), fauxAssistantMessage("The second answer.")]);
    const gateway = await startGateway(t, root, paths, faux, scripted.model);
    t.after(remove);
    const A_ID = randomUUID() as TerminalId;
    const B_ID = randomUUID() as TerminalId;
    const a = await connect(paths, A_ID, "tui-a");
    const b = await connect(paths, B_ID, "tui-b");
    t.after(async () => {
      await a.host.close();
      await b.host.close();
    });

    const gw = await a.host.get(GatewayContract, {});
    assert.equal(gw.variant, "owner", "every terminal on the socket operates the system");
    if (gw.variant !== "owner") return;
    const created = await gw.createConversation({ request: newActionRequest(), cwd: root });
    assert.equal(created.ok, true);
    if (!created.ok) return;
    const { conversation } = created.value;
    const row = await until(gw.conversations, (rows) => rows.find((candidate) => candidate.id === conversation));
    assert.deepEqual(
      { parent: row.parent, title: row.title, cwd: row.cwd },
      { parent: null, title: "(empty)", cwd: root },
    );

    const owner = await a.host.get(ConversationContract, { conversation, role: "owner" });
    assert.equal(owner.variant, "owner");
    if (owner.variant !== "owner") return;
    const otherOwner = await b.host.get(ConversationContract, { conversation, role: "owner" });
    assert.equal(otherOwner.variant, "owner", "both clients can control the same conversation");
    if (otherOwner.variant !== "owner") return;
    const viewer = await b.host.get(ConversationContract, { conversation, role: "viewer" });
    assert.equal(viewer.variant, "viewer", "an explicit viewer request remains read-only");
    // A member outside the held variant, called on purpose: Chord refuses it on the wire.
    await assert.rejects(
      (viewer as unknown as OwnerConversation).send({
        request: newUserRequest(),
        text: "x",
        images: [],
        whenBusy: "steer",
      }),
      (error: unknown) => error instanceof Error && "code" in error && error.code === "not_in_variant",
    );

    const request = newUserRequest();
    const args = { request, text: "hello", images: [], whenBusy: "followUp" as const };
    const first = await owner.send(args);
    const again = await owner.send(args);
    assert.equal(first.ok, true);
    if (!first.ok) return;
    assert.deepEqual(again, first, "the same request answers with the first submission");
    const outcome = await until(owner.outcomes, (rows) =>
      rows.find((candidate) => candidate.submission === first.value.submission),
    );
    assert.deepEqual({ status: outcome.status, request: outcome.request }, { status: "done", request });
    const answer = await until(owner.transcript, (transcript) =>
      transcript.items.find((item) => item.kind === "assistant"),
    );
    assert.deepEqual(answer.kind === "assistant" ? answer.message.content : null, [
      { type: "text", text: "The first answer." },
    ]);
    const input = await until(owner.transcript, (transcript) => transcript.items.find((item) => item.kind === "input"));
    assert.deepEqual(input.kind === "input" ? input.input.source : null, { kind: "user", request });

    assert.deepEqual(await owner.configure({ model: scripted.model, thinkingLevel: null }), { ok: true, value: null });
    const moduleRefused = await gw.createModule({
      name: "door_watch" as AgentModuleName,
      path: join(root, "door_watch.py"),
      owner: conversation,
    });
    assert.equal(moduleRefused.ok, false);
    if (moduleRefused.ok) return;
    assert.equal(moduleRefused.refusal.code, "dimos_unavailable");

    const secondRequest = newUserRequest();
    const secondArgs = {
      request: secondRequest,
      text: "from the second client",
      images: [],
      whenBusy: "followUp" as const,
    };
    const second = await otherOwner.send(secondArgs);
    assert.equal(second.ok, true);
    if (!second.ok) return;
    assert.notEqual(second.value.submission, first.value.submission);
    const secondOutcome = await until(owner.outcomes, (rows) =>
      rows.find((candidate) => candidate.submission === second.value.submission),
    );
    assert.deepEqual(
      { status: secondOutcome.status, request: secondOutcome.request },
      { status: "done", request: secondRequest },
    );
    await until(viewer.transcript, (transcript) =>
      transcript.items.find(
        (item) =>
          item.kind === "assistant" &&
          item.message.content.some((part) => part.type === "text" && part.text === "The second answer."),
      ),
    );

    a.socket.close();
    assert.deepEqual(
      await otherOwner.send(secondArgs),
      second,
      "another client stays usable after the first disconnects",
    );
    const a2 = await connect(paths, A_ID, "tui-a");
    t.after(() => a2.host.close());
    const back = await a2.host.get(ConversationContract, { conversation, role: "owner" });
    assert.equal(back.variant, "owner", "reconnection does not depend on another client's attachment");
    if (back.variant !== "owner") return;
    assert.deepEqual(await back.send(args), first, "a retry after reconnect returns the original submission");
    const userInputs = back.transcript.value.items.filter(
      (item) => item.kind === "input" && item.input.source.kind === "user",
    );
    assert.equal(userInputs.length, 2, "each client's request appears once in the shared transcript");
    viewer.release();
    otherOwner.release();
    back.release();

    const jobGateway = await b.host.get(GatewayContract, {});
    assert(jobGateway.variant === "owner");
    const job = {
      name: parseJobName("shared_job"),
      cron: "0 0 1 1 *",
      timeZone: parseTimeZone("UTC"),
      action: { kind: "notify" as const, text: "scheduled input" },
      conversation,
    };
    assert.deepEqual(jobGateway.jobs.value, [], "the jobs document need not exist at startup");
    const createdJob = await jobGateway.createJob(job);
    assert(createdJob.ok);
    const jobRow = await until(jobGateway.jobs, (rows) =>
      rows.find((row) => row.name === job.name && row.nextDue !== null),
    );
    assert.equal(jobRow.upcoming.length, 3);
    assert.equal(jobRow.nextDue, createdJob.value.nextDue);
    const changedJob = await jobGateway.updateJob({ ...job, cron: "0 12 1 1 *" });
    assert(changedJob.ok);
    const savedJob = await until(jobGateway.jobs, (rows) =>
      rows.find(
        (row) => row.name === job.name && row.cron === "0 12 1 1 *" && row.nextDue === changedJob.value.nextDue,
      ),
    );

    const lifecycle = await b.host.get(LifecycleContract, {});
    assert.equal(lifecycle.variant, "owner");
    if (lifecycle.variant !== "owner") return;
    assert.equal(lifecycle.process.value.pid, process.pid);
    assert.deepEqual(await lifecycle.stop({}), { ok: true, value: null });
    await gateway.stopped;
    assert.deepEqual(await gatewayState(paths, 500), { kind: "stopped" });

    const restarted = await startGateway(t, root, paths, faux, scripted.model);
    const c = await connect(paths, randomUUID() as TerminalId, "after-restart");
    t.after(() => c.host.close());
    const recovered = await c.host.get(GatewayContract, {});
    assert(recovered.variant === "owner");
    assert.deepEqual(recovered.jobs.value, [savedJob], "the store initializes its cells from durable state");
    assert.equal(recovered.conversations.value.find((row) => row.id === conversation)?.title, "hello");
    assert.deepEqual(await recovered.deleteJob({ name: job.name }), { ok: true, value: null });
    await until(recovered.jobs, (rows) => (rows.length === 0 ? true : undefined));
    const stop = await c.host.get(LifecycleContract, {});
    assert(stop.variant === "owner");
    await stop.stop({});
    await restarted.stopped;
  },
);

function holdSocket(path: string): Promise<ChildProcess> {
  const child = spawn(process.execPath, [
    "--input-type=module",
    "-e",
    'import {createServer} from "node:net";createServer(() => {}).listen(process.argv[1],()=>process.stdout.write("ready"));',
    path,
  ]);
  return once(child.stdout, "data").then(() => child);
}

test("status answers within its limit for a stale socket file and for a socket nobody answers on", async (t) => {
  const { paths, remove } = await scratch();
  t.after(remove);
  await (await GatewayServer.bind(paths.socket)).close();

  const holder = await holdSocket(paths.socket);
  t.after(async () => {
    holder.kill("SIGKILL");
  });
  let started = performance.now();
  assert.deepEqual(await gatewayState(paths, 500), { kind: "not_answering" });
  assert(performance.now() - started < 1_500, "a socket that accepts and never answers does not hang status");
  await assert.rejects(GatewayServer.bind(paths.socket), GatewayRunning);

  holder.kill("SIGKILL");
  await once(holder, "exit");
  started = performance.now();
  assert.deepEqual(await gatewayState(paths, 500), { kind: "stale_socket" });
  assert(performance.now() - started < 1_000, "a stale socket file does not hang status");

  const replacement = await GatewayServer.bind(paths.socket);
  assert.deepEqual(await gatewayState(paths, 500), { kind: "not_answering" }, "bound, not yet serving");
  await replacement.close();

  const stale = await holdSocket(paths.socket);
  stale.kill("SIGKILL");
  await once(stale, "exit");
  assert.equal(await stopGateway(paths), "not_running");
  await assert.rejects(stat(paths.socket), { code: "ENOENT" }, "stop removes the stale socket file");
});

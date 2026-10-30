import assert from "node:assert/strict";
import type { AddressInfo } from "node:net";
import type { ConversationId, SubmissionId, TaskId } from "@earendil-works/pi-durable";
import type { GatewayProcess } from "../shared/protocol/lifecycle/contract.ts";
import type { TerminalId } from "../shared/protocol/auth.ts";
import type { IncomingMessage } from "node:http";
import type { Link, Self } from "@earendil-works/chord";
import type { LinkStatus } from "../shared/protocol/gateway/link.ts";
import type { LoadStatus } from "../shared/protocol/runtime/load.ts";
import type { LoginState } from "../shared/protocol/auth.ts";
import type { Outcome } from "../shared/protocol/outcome.ts";
import type { Transcript } from "../shared/protocol/conversation/state.ts";
import type { UserRequestId } from "../shared/protocol/conversation/input.ts";
import { BACKGROUND_CONTEXT } from "@earendil-works/chord/context";
import { ConversationContract } from "../shared/protocol/conversation/contract.ts";
import { GatewayContract } from "../shared/protocol/gateway/contract.ts";
import { LifecycleContract } from "../shared/protocol/lifecycle/contract.ts";
import { WebSocket, WebSocketServer } from "ws";
import { acceptWebSockets, createHost, webSocketLink } from "@earendil-works/chord";
import { carried } from "../shared/protocol/schema.ts";
import { connect } from "./main.ts";
import { done, refused } from "../shared/protocol/outcome.ts";
import { epochMs } from "../shared/protocol/time.ts";
import { once } from "node:events";
import { owner, RefusalError } from "./failures.ts";
import { retryByRequestId } from "./retry.ts";
import { Replies } from "../gateway/server/replies.ts";
import { test } from "node:test";
import { until } from "./until.ts";

type Principal =
  | { readonly kind: "terminal"; readonly terminal: TerminalId; readonly label: string }
  | { readonly kind: "gateway" };

const SINCE = epochMs(1_760_700_000_000);

const PROCESS: GatewayProcess = {
  build: { version: "0.1.0", build: "3f9c2a1b7d0e", executable: "/tmp/cli.js", node: "v26.10.0" },
  pid: 51234,
  startedAt: SINCE,
};

const LINK: LinkStatus = { state: "up", since: SINCE, server: "McpServer 127.0.0.1:9990" };

const TRANSCRIPT: Transcript = {
  items: [
    {
      kind: "input",
      entry: 1 as import("@earendil-works/pi-durable").EntryId,
      input: {
        source: { kind: "user", request: "00000000-0000-4000-8000-000000000001" as UserRequestId },
        text: "walk to the kitchen door",
        change: null,
        render: null,
        limit: null,
      },
      images: [],
    },
  ],
  omitted: 0,
};

const R1 = "aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaa" as UserRequestId;

const R2 = "bbbbbbbb-bbbb-4bbb-8bbb-bbbbbbbbbbbb" as UserRequestId;

const R3 = "cccccccc-cccc-4ccc-8ccc-cccccccccccc" as UserRequestId;

const unused = async (): Promise<Outcome<never>> => refused({ code: "invalid", field: "test", problem: "not served" });

/** The served objects with literal state over a real Chord host, counting what reached each executor. */
function serveGateway(afterSend: (request: UserRequestId | null) => void = () => {}) {
  const replies = new Replies(60_000, BACKGROUND_CONTEXT);
  let viewer = false;
  const host = createHost<Principal>({
    name: "gateway",
    self: { kind: "gateway" },
    principalKey: (principal) => (principal.kind === "gateway" ? "gateway" : `terminal:${principal.terminal}`),
  });
  const calls = { send: new Map<UserRequestId, number>(), configure: 0, effects: 0 };
  const cell = <T>(self: Self<Principal>, name: string, value: T) => self.state(name, carried<T>(), value);

  host.serve(LifecycleContract, (_input, _principal, self) => ({
    variant: "owner",
    process: cell(self, "process", PROCESS),
    stop: async () => done(null),
  }));

  host.serve(GatewayContract, (_input, _principal, self) => ({
    variant: "viewer",
    link: cell(self, "link", LINK),
    load: cell<LoadStatus>(self, "load", { state: "idle" }),
    runs: cell(self, "runs", []),
    conversations: cell(self, "conversations", []),
    modules: cell(self, "modules", []),
    streams: cell(self, "streams", []),
    jobs: cell(self, "jobs", []),
    settings: cell(self, "settings", { workspace: null, model: null, thinkingLevel: null, theme: null }),
    accounts: cell(self, "accounts", { providers: [], models: [] }),
    loginFlow: cell<LoginState>(self, "loginFlow", { phase: "idle" }),
  }));

  host.serve(ConversationContract, ({ conversation, role }, principal, self) => {
    if (principal.kind !== "terminal") throw new Error("only terminals attach");
    const state = {
      record: cell(self, "record", { id: conversation }),
      transcript: cell(self, "transcript", TRANSCRIPT),
      live: cell(self, "live", { run: { taskId: 9 as TaskId, inputs: [] } }),
      queue: cell(self, "queue", { items: [] }),
      usage: cell(self, "usage", { models: {}, tools: {} }),
      agent: cell(self, "agent", {}),
      outcomes: cell(self, "outcomes", []),
      detachable: cell(self, "detachable", []),
      moved: cell(self, "moved", []),
      modules: cell(self, "modules", []),
      jobs: cell(self, "jobs", []),
      streams: cell(self, "streams", []),
    };
    if (role === "viewer" || viewer) return { variant: "viewer", ...state, image: unused };
    return {
      variant: "owner",
      ...state,
      image: unused,
      send: async ({ request }: { readonly request: UserRequestId }) => {
        const before = calls.send.get(request);
        calls.send.set(request, before === undefined ? 1 : before + 1);
        const reply = await replies.once(request, async () => {
          calls.effects++;
          return done({ submission: 41 as SubmissionId });
        });
        afterSend(request);
        return reply;
      },
      withdraw: unused,
      abort: unused,
      configure: async () => {
        calls.configure += 1;
        afterSend(null);
        return refused({ code: "running", conversation });
      },
      edit: unused,
      summarizeUpTo: unused,
      summarizeFrom: unused,
      background: unused,
      stopBackground: unused,
    };
  });

  return {
    host,
    calls,
    downgrade: () => {
      viewer = true;
    },
  };
}

function principalOf(request: IncomingMessage): Principal | undefined {
  const url = new URL(request.url === undefined ? "/" : request.url, "http://127.0.0.1");
  const terminal = url.searchParams.get("terminal");
  const label = url.searchParams.get("label");
  if (terminal === null || label === null) return undefined;
  return { kind: "terminal", terminal: terminal as TerminalId, label };
}

test("direct references retain state over reconnect; server validation and same-request retry stay authoritative", async (t) => {
  const accepted: WebSocket[] = [];
  let loseReply = false;
  const { host, calls, downgrade } = serveGateway(() => {
    if (loseReply) {
      loseReply = false;
      accepted.at(-1)?.terminate();
    }
  });
  const server = new WebSocketServer({ host: "127.0.0.1", port: 0 });
  await once(server, "listening");
  const { port } = server.address() as AddressInfo;
  server.on("connection", (socket) => accepted.push(socket));
  acceptWebSockets(server, host, { principal: principalOf });
  t.after(async () => {
    await host.close();
    server.close();
  });
  const client = await connect(() => new WebSocket(`ws://127.0.0.1:${port}/?terminal=t1&label=test`), {
    backoffMs: { min: 20, max: 100 },
  });
  t.after(() => client.close());
  const gateway = await client.host.get(GatewayContract, {});
  const lifecycle = await client.host.get(LifecycleContract, {});
  const conversation = await client.host.get(ConversationContract, {
    conversation: 1 as ConversationId,
    role: "owner",
  });
  t.after(() => {
    conversation.release();
    gateway.release();
    lifecycle.release();
  });
  assert.equal(client.connection.value.state, "live");
  assert.deepEqual(gateway.link.value, LINK);
  assert.deepEqual(lifecycle.process.value, PROCESS);
  assert.deepEqual(conversation.transcript.value, TRANSCRIPT);
  const running = await owner(conversation).configure({ model: null, thinkingLevel: null });
  assert.deepEqual(running, refused({ code: "running", conversation: 1 as ConversationId }));
  assert.equal(calls.configure, 1, "the server validates the direct call");
  assert.deepEqual(
    await owner(conversation).send({ request: R1, text: "hello", images: [], whenBusy: "steer" }),
    done({ submission: 41 }),
  );
  const viewer = await client.host.get(ConversationContract, { conversation: 1 as ConversationId, role: "viewer" });
  assert.equal(viewer.variant, "viewer");
  assert.throws(() => owner(viewer), RefusalError);
  viewer.release();
  assert.equal(conversation.variant, "owner", "releasing another hold leaves the owner usable");

  const states: string[] = [];
  client.connection.subscribe(() => states.push(client.connection.value.state));
  accepted[0].close();
  await until(client.connection, (value) => (value.state === "reconnecting" ? true : undefined));
  assert.deepEqual(conversation.transcript.value, TRANSCRIPT);
  const pending = owner(conversation).send({ request: R2, text: "during the gap", images: [], whenBusy: "steer" });
  assert.deepEqual(await pending, done({ submission: 41 }));
  assert.deepEqual(states, ["live", "reconnecting", "live"]);

  loseReply = true;
  await assert.rejects(owner(conversation).configure({ model: null, thinkingLevel: null }), { code: "unavailable" });
  assert.equal(calls.configure, 2, "non-idempotent methods are never replayed automatically");
  await client.whenLive();
  loseReply = true;
  const args = { request: R3, text: "lost reply", images: [], whenBusy: "followUp" as const };
  assert.deepEqual(
    await retryByRequestId(client, args, (same) => owner(conversation).send(same)),
    done({ submission: 41 }),
  );
  assert.equal(calls.send.get(R3), 2, "one retry after an ambiguous reply");
  assert.equal(calls.effects, 3, "the retry reuses its request ID, so each input executes once");
  downgrade();
  const changed = new Promise<void>((resolve) =>
    conversation.onShape((shape) => {
      if (shape.variant === "viewer") resolve();
    }),
  );
  accepted.at(-1)?.close();
  await changed;
  assert.equal(conversation.variant, "viewer");
  assert.throws(() => owner(conversation), RefusalError);
});

test("lifecycle-only connections and terminal refusal settle pending waits without acquiring Gateway", async (t) => {
  const host = createHost<Principal>({ name: "gateway", self: { kind: "gateway" } });
  host.serve(LifecycleContract, (_args, _principal, self) => ({
    variant: "owner",
    process: self.state("process", carried<GatewayProcess>(), PROCESS),
    stop: async () => done(null),
  }));
  const server = new WebSocketServer({ host: "127.0.0.1", port: 0 });
  await once(server, "listening");
  const { port } = server.address() as AddressInfo;
  const links: Link[] = [];
  server.on("connection", (socket) => {
    const link = webSocketLink(socket, { compress: true, acceptPacked: false });
    links.push(link);
    host.accept(link, { principal: { kind: "terminal", terminal: "t3" as TerminalId, label: "cli" }, kind: "client" });
  });
  t.after(async () => {
    await host.close();
    server.close();
  });
  const client = await connect(() => new WebSocket(`ws://127.0.0.1:${port}/`), { backoffMs: { min: 20, max: 100 } });
  t.after(() => client.close());
  const lifecycle = await client.host.get(LifecycleContract, {});
  t.after(() => lifecycle.release());
  assert.deepEqual(lifecycle.process.value, PROCESS);
  assert.deepEqual(await owner(lifecycle).stop({}), done(null));
  links[0].close("stopping");
  await until(client.connection, (value) => (value.state === "refused" ? true : undefined));
  assert.deepEqual(client.connection.value, { state: "refused", refusal: { code: "stopping" } });
  await assert.rejects(Promise.race([owner(lifecycle).stop({}), client.ended]), { refusal: { code: "stopping" } });
  await assert.rejects(client.whenLive(), { refusal: { code: "stopping" } });
});

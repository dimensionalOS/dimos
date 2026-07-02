// gatewayWs(): connect settle/timeout, reconnect backoff with sub+QoS replay, in-flight
// RPC fast-fail on a dropped socket, and the ping heartbeat closing a half-open link.
import assert from "node:assert/strict";

import { createGatewayWsTransport } from "./gatewayWs.ts";
import type { Status } from "../types.ts";

class FakeWebSocket {
  static instances: FakeWebSocket[] = [];
  static readonly CONNECTING = 0;
  static readonly OPEN = 1;
  static readonly CLOSING = 2;
  static readonly CLOSED = 3;
  url: string;
  binaryType = "blob";
  readyState = 0;
  sent: string[] = [];
  onopen: (() => void) | null = null;
  onclose: (() => void) | null = null;
  onmessage: ((e: { data: unknown }) => void) | null = null;
  onerror: (() => void) | null = null;
  constructor(url: string) {
    this.url = url;
    FakeWebSocket.instances.push(this);
  }
  send(data: string) {
    this.sent.push(data);
  }
  close() {
    if (this.readyState === FakeWebSocket.CLOSED) return;
    this.readyState = FakeWebSocket.CLOSED;
    this.onclose?.();
  }
  // test helpers — the "server" side
  serverOpen() {
    this.readyState = FakeWebSocket.OPEN;
    this.onopen?.();
  }
  serverSend(obj: unknown) {
    this.onmessage?.({ data: JSON.stringify(obj) });
  }
  msgs(): Array<Record<string, unknown>> {
    return this.sent.map((s) => JSON.parse(s));
  }
}

const realWs = globalThis.WebSocket;
function withFakeWs(fn: () => Promise<void>): () => Promise<void> {
  return async () => {
    FakeWebSocket.instances = [];
    (globalThis as { WebSocket: unknown }).WebSocket = FakeWebSocket;
    try {
      await fn();
    } finally {
      (globalThis as { WebSocket: unknown }).WebSocket = realWs;
    }
  };
}

const tick = (ms: number) => new Promise((r) => setTimeout(r, ms));

Deno.test(
  "connect resolves on open; replays subs + declared QoS + list",
  withFakeWs(async () => {
    const t = createGatewayWsTransport({ url: "ws://gw/ws", heartbeatMs: 0, _retryBaseMs: 5 });
    t.subscribe("/odom", { maxHz: 5, priority: "high" }); // declared before the socket exists
    const p = t.connect();
    const sock = FakeWebSocket.instances[0];
    sock.serverOpen();
    await p;
    const subs = sock.msgs().filter((m) => m.op === "subscribe");
    assert.equal(subs.length, 1);
    assert.equal(subs[0].topic, "/odom");
    assert.equal(subs[0].maxHz, 5);
    assert.equal(subs[0].priority, "high");
    assert.ok(sock.msgs().some((m) => m.op === "list"));
    t.close();
  }),
);

Deno.test(
  "connect rejects on timeout; background retry still opens later",
  withFakeWs(async () => {
    const t = createGatewayWsTransport({
      url: "ws://gw/ws",
      connectTimeoutMs: 20,
      heartbeatMs: 0,
      _retryBaseMs: 5,
    });
    const statuses: Status[] = [];
    t.onStatus((s) => statuses.push(s));
    await assert.rejects(t.connect(), /timed out/);
    FakeWebSocket.instances[0].close(); // the refused first attempt reports closed
    await tick(15); // > backoff
    assert.ok(FakeWebSocket.instances.length >= 2);
    FakeWebSocket.instances.at(-1)!.serverOpen();
    assert.ok(statuses.includes("open"));
    t.close();
  }),
);

Deno.test(
  "server drop rejects in-flight rpc; reconnect replays subs with QoS",
  withFakeWs(async () => {
    const t = createGatewayWsTransport({ url: "ws://gw/ws", heartbeatMs: 0, _retryBaseMs: 5 });
    const p = t.connect();
    const s1 = FakeWebSocket.instances[0];
    s1.serverOpen();
    await p;
    t.subscribe("/odom", { maxHz: 10 });
    const rpcP = t.rpc("GO2Load", "start_all");
    s1.close(); // server drop
    await assert.rejects(rpcP, /connection closed/);
    await tick(15);
    const s2 = FakeWebSocket.instances.at(-1)!;
    assert.notEqual(s2, s1);
    s2.serverOpen();
    const subs = s2.msgs().filter((m) => m.op === "subscribe");
    assert.equal(subs[0]?.topic, "/odom");
    assert.equal(subs[0]?.maxHz, 10);
    t.close();
  }),
);

Deno.test(
  "heartbeat: an unanswered ping closes the socket and reconnect kicks in",
  withFakeWs(async () => {
    const t = createGatewayWsTransport({ url: "ws://gw/ws", heartbeatMs: 10, _retryBaseMs: 5 });
    const p = t.connect();
    const s1 = FakeWebSocket.instances[0];
    s1.serverOpen();
    await p;
    await tick(25); // beat 1 sends the ping; beat 2 finds it unanswered → close
    assert.ok(s1.msgs().some((m) => m.op === "ping"));
    assert.equal(s1.readyState, FakeWebSocket.CLOSED);
    await tick(15);
    assert.ok(FakeWebSocket.instances.length >= 2); // reconnect attempt spawned
    t.close();
  }),
);

Deno.test(
  "heartbeat: answered pings keep the socket open",
  withFakeWs(async () => {
    const t = createGatewayWsTransport({ url: "ws://gw/ws", heartbeatMs: 10, _retryBaseMs: 5 });
    const p = t.connect();
    const s1 = FakeWebSocket.instances[0];
    s1.send = (data: string) => { // server auto-answers pings
      s1.sent.push(data);
      const m = JSON.parse(data);
      if (m.op === "ping") {
        queueMicrotask(() => s1.serverSend({ op: "pong", id: m.id, serverTs: Date.now() }));
      }
    };
    s1.serverOpen();
    await p;
    await tick(35); // several beats
    assert.equal(s1.readyState, FakeWebSocket.OPEN);
    assert.equal(FakeWebSocket.instances.length, 1);
    t.close();
  }),
);

Deno.test(
  "close(): no retry, pending connect rejected",
  withFakeWs(async () => {
    const t = createGatewayWsTransport({ url: "ws://gw/ws", heartbeatMs: 0, _retryBaseMs: 5 });
    const p = t.connect();
    t.close();
    await assert.rejects(p, /transport closed/);
    await tick(15);
    assert.equal(FakeWebSocket.instances.length, 1); // no reconnect after close()
  }),
);

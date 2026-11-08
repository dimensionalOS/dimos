import { assert, assertEquals, assertNotEquals } from "@std/assert";
import { FakeTime } from "jsr:@std/testing@1/time";
import {
  DataFrameStreamReader,
  decodeDataFrame,
  type FrameHeader,
  MAX_TOPIC_SOURCES,
  type Msg,
  type TopicMsg,
  topicMsgFromUnknown,
  type TopicOpenMsg,
  type TopicReadyMsg,
  type TopicSubMsg,
} from "@dimos/shared";
import type { FrameSend, ViewerSink } from "./forward.ts";
import { TopicBudget, TopicEgress } from "./topic_forward.ts";
import { type TopicProvider, TopicRegistry, type TopicViewer } from "./topics.ts";

// A transport boundary: writes complete without browser receipts, exactly
// the case that would conceal unbounded QUIC buffering behind a small FIFO.
class Sink implements ViewerSink {
  frames: Uint8Array[] = [];
  decoded() {
    const reader = new DataFrameStreamReader();
    return this.frames.flatMap((bytes) => reader.push(bytes));
  }
  kicked: string[] = [];
  aborted = 0;
  openStream() {
    return Promise.resolve({
      write: (bytes: Uint8Array) => {
        this.frames.push(bytes);
        return Promise.resolve();
      },
      abort: () => {
        this.aborted++;
        return Promise.resolve();
      },
    });
  }
  sendFrame(bytes: Uint8Array): FrameSend {
    this.frames.push(bytes);
    let aborted = false;
    return {
      done: Promise.resolve(),
      get aborted() {
        return aborted;
      },
      abort: () => {
        if (!aborted) this.aborted++;
        aborted = true;
      },
      supersede: () => false,
    };
  }
  kick(reason: string) {
    this.kicked.push(reason);
  }
}
class Provider implements TopicProvider {
  messages: Msg[] = [];
  publications: Uint8Array[] = [];
  guards: Uint8Array[] = [];
  failures: string[] = [];
  sendTopicPublication(bytes: Uint8Array) {
    this.publications.push(bytes);
  }
  sendTopicGuard(bytes: Uint8Array) {
    this.guards.push(bytes);
  }
  failTopics(reason: string) {
    this.failures.push(reason);
  }
  sendControl(msg: Msg) {
    this.messages.push(msg);
  }
  opens(): TopicOpenMsg[] {
    return this.messages.filter((m) => m.t === "topic_open");
  }
}
class Viewer implements TopicViewer {
  messages: Msg[] = [];
  sink = new Sink();
  sendMsg = (msg: Msg) => {
    this.messages.push(msg);
  };
  ready(): TopicReadyMsg[] {
    return this.messages.filter((m) => m.t === "topic_ready");
  }
}
const spec: TopicSubMsg = {
  t: "topic_sub",
  id: "one",
  source: "workspace",
  keyExpr: "demo/**",
  delivery: "reliable",
  profile: "raw",
  maxHz: 1000,
};
function setup() {
  const registry = new TopicRegistry();
  const provider = new Provider();
  registry.registerSource(provider, "workspace", {
    read: true,
    write: true,
    profiles: ["raw"],
  });
  provider.messages = [];
  const viewer = new Viewer();
  registry.addViewer(viewer, viewer.sendMsg);
  const send = (msg: TopicMsg, target = viewer) =>
    registry.onViewerMsg(target, msg, target.sendMsg);
  const opened = (open = provider.opens().at(-1)!) =>
    registry.onProviderMsg(provider, {
      t: "topic_opened",
      route: open.route,
      gen: open.gen,
    });
  const activate = (ready = viewer.ready().at(-1)!, target = viewer) =>
    send({
      t: "topic_activate",
      route: ready.route,
      gen: ready.gen,
    }, target);
  const sample = (
    seq: number,
    key = "demo/a",
    open = provider.opens().at(-1)!,
    peer = provider,
    stream = orderedStream,
  ) => {
    const header: FrameHeader = {
      ch: open.route,
      seq,
      ts: 5,
      delivery: open.delivery,
      meta: { key, encoding: "application/octet-stream", gen: open.gen, ingressTs: 4 },
    };
    registry.onFrame(peer, header, new Uint8Array([seq % 256]), stream);
  };
  const orderedStream = Symbol("ordered");
  return { registry, provider, viewer, send, opened, activate, sample };
}
async function flush() {
  for (let i = 0; i < 20; i++) await Promise.resolve();
}

Deno.test("different latest rates share one provider route and retain each viewer's last pending sample", async () => {
  using time = new FakeTime(0);
  const { registry, provider, viewer, send, opened, activate, sample } = setup();
  try {
    send({ ...spec, id: "slow", delivery: "latest", maxHz: 5 });
    opened();
    activate();
    const slow = viewer.ready()[0].route;
    sample(1);
    await flush();
    send({ ...spec, id: "fast", delivery: "latest", maxHz: 30 });
    activate();
    const fast = viewer.ready()[1].route;
    assertEquals(provider.opens().length, 1);
    assertEquals(provider.messages.filter((m) => m.t === "topic_activate").map((m) => m.maxHz), [
      5,
      30,
    ]);
    time.tick(34);
    sample(2);
    await flush();
    time.tick(34);
    sample(3);
    await flush();
    const sequences = (route: string) =>
      viewer.sink.decoded().filter((f) => f.header.ch === route).map((f) => f.header.seq);
    assertEquals(sequences(slow), [1]);
    assertEquals(sequences(fast), [2, 3]);
    time.tick(132);
    await flush();
    assertEquals(sequences(slow), [1, 3]); // The final sample drains even after the publisher goes quiet.
    send({ t: "topic_unsub", id: "fast" });
    assertEquals(provider.messages.filter((m) => m.t === "topic_activate").at(-1)?.maxHz, 5);
    assertEquals(provider.opens().length, 1);
  } finally {
    registry.viewerClosed(viewer);
  }
});

const GUARD_OPEN: FrameHeader = {
  ch: "@guard",
  seq: 0,
  ts: 0,
  delivery: "reliable",
  meta: {
    action: "open",
    id: "drive",
    source: "workspace",
    gen: 1,
    key: "robot/cmd",
    encoding: "application/octet-stream",
    group: "drive",
    timeoutMs: 400,
    takeover: false,
    owner: "forged",
    handle: "@p:forged",
  },
};
function guardSetup() {
  const registry = new TopicRegistry(() => 0);
  const provider = new Provider();
  registry.registerSource(provider, "workspace", {
    read: true,
    write: true,
    profiles: ["raw"],
    guardedPublishers: true,
  });
  const viewer = new Viewer();
  registry.addViewer(viewer, viewer.sendMsg);
  const open = GUARD_OPEN;
  const installed = () => {
    const meta = decodeDataFrame(provider.guards[0]).header.meta!;
    const handle = meta.handle as string;
    registry.onProviderMsg(provider, {
      t: "topic_guard_ready",
      id: handle,
      handle,
      gen: 1,
      ticket: 1,
      timeoutMs: 400,
    });
    return handle;
  };
  const command = (handle: string, seq: number): FrameHeader => ({
    ch: "@guard",
    seq,
    ts: 0,
    delivery: "reliable",
    meta: {
      action: "command",
      source: "workspace",
      gen: 1,
      handle,
      ticket: 1,
    },
  });
  return { registry, provider, viewer, open, installed, command };
}

Deno.test("guard creation replaces the browser-supplied owner and handle", () => {
  const h = guardSetup();
  h.registry.publish(h.viewer, h.open, new Uint8Array([0, 255]));
  const upstream = decodeDataFrame(h.provider.guards[0]);
  assertNotEquals(upstream.header.meta!.owner, "forged");
  assertNotEquals(upstream.header.meta!.handle, "@p:forged");
  assertEquals(upstream.payload, new Uint8Array([0, 255]));
  const handle = h.installed();
  assertEquals(h.viewer.messages.at(-1), {
    t: "topic_guard_ready",
    id: "drive",
    handle,
    gen: 1,
    ticket: 1,
    timeoutMs: 400,
  });
});

Deno.test("guard handles and provider replies remain bound to connection and generation", () => {
  const h = guardSetup();
  h.registry.publish(h.viewer, h.open, new Uint8Array([0]));
  const handle = h.installed();
  const stranger = new Viewer();
  h.registry.addViewer(stranger, stranger.sendMsg);
  const controls = h.provider.messages.length;
  h.registry.onViewerMsg(stranger, { t: "topic_guard_stop", handle, gen: 1 }, stranger.sendMsg);
  h.registry.onViewerMsg(
    stranger,
    { t: "topic_guard_heartbeat", handle, gen: 1, ticket: 1 },
    stranger.sendMsg,
  );
  h.registry.publish(stranger, h.command(handle, 1), new Uint8Array([9]));
  h.registry.onViewerMsg(h.viewer, { t: "topic_guard_stop", handle, gen: 0 }, h.viewer.sendMsg);
  assertEquals(h.provider.messages.length, controls);
  assertEquals(h.provider.guards.length, 1);
  const received = h.viewer.messages.length;
  h.registry.onProviderMsg(new Provider(), {
    t: "topic_guard_closed",
    handle,
    gen: 1,
    code: "forged",
  });
  h.registry.onProviderMsg(h.provider, { t: "topic_guard_closed", handle, gen: 0, code: "stale" });
  assertEquals(h.viewer.messages.length, received);
  h.registry.publish(h.viewer, h.command(handle, 1), new Uint8Array([1]));
  assertEquals(h.provider.guards.length, 2);
});

Deno.test("page loss during guard creation stops only after installation acknowledgement", () => {
  const h = guardSetup();
  h.registry.publish(h.viewer, h.open, new Uint8Array([0]));
  h.registry.viewerClosed(h.viewer);
  assertEquals(h.provider.messages.filter((msg) => msg.t === "topic_guard_stop"), []);
  const handle = h.installed();
  assertEquals(h.provider.messages.at(-1), { t: "topic_guard_stop", handle, gen: 1 });
  assertEquals(h.viewer.messages.filter((msg) => msg.t === "topic_guard_ready"), []);
});

Deno.test("guard command budgets include in-flight data until the provider replies", () => {
  const h = guardSetup();
  h.registry.publish(h.viewer, h.open, new Uint8Array([0]));
  const handle = h.installed();
  for (let seq = 1; seq <= 9; seq++) {
    h.registry.publish(h.viewer, h.command(handle, seq), new Uint8Array([seq]));
  }
  assertEquals(h.provider.guards.length, 9); // one open plus eight commands
  assertEquals(h.viewer.messages.at(-1), {
    t: "topic_guard_result",
    handle,
    gen: 1,
    seq: 9,
    outcome: "rejected",
    code: "guard_busy",
    providerTs: 0,
  });
  h.registry.onProviderMsg(h.provider, {
    t: "topic_guard_result",
    handle,
    gen: 1,
    seq: 1,
    outcome: "accepted",
    code: "",
    providerTs: 123,
  });
  h.registry.publish(h.viewer, h.command(handle, 9), new Uint8Array([9]));
  assertEquals(h.provider.guards.length, 10);
  h.registry.sourceClosed(h.provider);
  assertEquals(h.viewer.messages.at(-1)?.t, "topic_sources");
  assert(
    h.viewer.messages.some((msg) =>
      msg.t === "topic_guard_closed" && msg.code === "source_offline"
    ),
  );
});

Deno.test("a provider that never acknowledges guard installation is disconnected", () => {
  const h = guardSetup();
  h.registry.publish(h.viewer, h.open, new Uint8Array([0]));
  h.registry.reap(10_001);
  h.registry.reap(20_000);
  assertEquals(h.provider.failures, ["guard_provider_timeout"]);
});

Deno.test("a closed registry lists no sources and denies subscriptions, lists, writes and guards", () => {
  const registry = new TopicRegistry(undefined, false);
  const provider = new Provider();
  registry.registerSource(provider, "workspace", {
    read: true,
    write: true,
    profiles: ["raw"],
    guardedPublishers: true,
  });
  provider.messages = [];
  const viewer = new Viewer();
  registry.addViewer(viewer, viewer.sendMsg);
  assertEquals(viewer.messages, [{ t: "topic_sources", v: 1, sources: [] }]);
  registry.onViewerMsg(viewer, spec, viewer.sendMsg);
  registry.onViewerMsg(
    viewer,
    { t: "topic_list", id: "list", source: "workspace", prefix: "", cursor: "", limit: 1 },
    viewer.sendMsg,
  );
  registry.onViewerMsg(viewer, { t: "topic_opened", route: "@t:forged", gen: 1 }, viewer.sendMsg);
  registry.publish(viewer, publication(), new Uint8Array([1]));
  registry.publish(viewer, GUARD_OPEN, new Uint8Array([0]));
  assertEquals(
    viewer.messages.slice(1).map((m) =>
      (m.t === "topic_error" || m.t === "topic_pub_result") && m.code
    ),
    [
      "topic_read_denied",
      "topic_read_denied",
      "invalid_topic_operation",
      "topic_write_denied",
      "topic_write_denied",
    ],
  );
  assertEquals(provider.messages, []);
  assertEquals(provider.publications, []);
  assertEquals(provider.guards, []);
});

Deno.test("the source count stops where a topic_sources announcement would become invalid", () => {
  const h = setup();
  const providers = Array.from({ length: MAX_TOPIC_SOURCES - 1 }, () => new Provider());
  providers.forEach((provider, i) => {
    assertEquals(h.registry.registerSource(provider, `source-${i}`), null);
  });
  assertEquals(h.registry.registerSource(new Provider(), "one-more"), "topic_source_limit");
  const announced = h.viewer.messages.at(-1) as unknown as Record<string, unknown>;
  assertEquals(announced.t, "topic_sources");
  assertEquals((announced.sources as unknown[]).length, MAX_TOPIC_SOURCES);
  assertNotEquals(topicMsgFromUnknown(announced), null);
  h.registry.sourceClosed(providers[0]);
  assertEquals(h.registry.registerSource(new Provider(), "one-more"), null);
  h.registry.viewerClosed(h.viewer);
});

Deno.test("an open registry announces every connected source to every viewer", () => {
  const h = setup();
  const other = new Viewer();
  h.registry.addViewer(other, other.sendMsg);
  const expected: Msg = {
    t: "topic_sources",
    v: 1,
    sources: [{ source: "workspace", gen: 1, read: true, write: true, profiles: ["raw"] }],
  };
  assertEquals(other.messages, [expected]);
  assertEquals(h.viewer.messages, [expected]);
  h.registry.sourceClosed(h.provider);
  assertEquals(other.messages.at(-1), { t: "topic_sources", v: 1, sources: [] });
});

Deno.test("readiness requires provider installation; data cannot overtake client activation", async () => {
  const h = setup();
  h.send(spec);
  assertEquals(h.viewer.ready(), []);
  h.sample(1);
  assertEquals(h.viewer.sink.frames, []);
  h.opened();
  assertEquals(h.viewer.ready().length, 1);
  assertEquals(h.provider.messages.map((m) => m.t), ["topic_open"]);
  h.activate();
  h.sample(1, "demo/a");
  h.sample(2, "demo/b");
  h.sample(3, "demo/a");
  await flush();
  assertEquals(h.viewer.sink.decoded().map((f) => f.header.meta?.key), [
    "demo/a",
    "demo/b",
    "demo/a",
  ]);
  assertEquals(h.viewer.sink.decoded().map((f) => [...f.payload]), [[1], [2], [3]]);
  h.registry.viewerClosed(h.viewer);
  assertEquals(h.viewer.sink.aborted, 1);
});

Deno.test("shared subscriptions have independent activation and last-user disposal", async () => {
  const h = setup();
  h.send(spec);
  h.opened();
  h.activate();
  h.send({ ...spec, id: "two" });
  assertEquals(h.provider.opens().length, 1);
  const second = h.viewer.ready()[1];
  h.sample(1);
  await flush();
  assertEquals(h.viewer.sink.frames.length, 1);
  h.activate(second);
  await flush();
  assertEquals(
    h.viewer.sink.decoded().map((f) => f.header.ch),
    h.viewer.ready().map((r) => r.route),
  );
  h.send({ t: "topic_unsub", id: "one" });
  assertEquals(h.provider.messages.some((m) => m.t === "topic_close"), false);
  h.send({ t: "topic_unsub", id: "two" });
  assertEquals(h.provider.messages.at(-1)?.t, "topic_close");
});

Deno.test("handles are bound to the viewer and generation", () => {
  const h = setup();
  h.send(spec);
  h.opened();
  const other = new Viewer();
  h.registry.addViewer(other, other.sendMsg);
  h.activate(h.viewer.ready()[0], other);
  h.send({ t: "topic_activate", route: h.viewer.ready()[0].route, gen: 100 });
  h.send(
    { t: "topic_credit", route: h.viewer.ready()[0].route, gen: 1, frames: 0, bytes: 0 },
    other,
  );
  assertEquals(h.provider.messages.map((m) => m.t), ["topic_open"]);
  assertEquals(other.messages.slice(1).map((m) => m.t === "topic_error" && m.code), [
    "unknown_topic_handle",
    "unknown_topic_handle",
  ]);
  h.registry.viewerClosed(h.viewer);
  h.registry.viewerClosed(other);
});

Deno.test("source restart reopens desired subscriptions with fresh routes and ignores old peers", async () => {
  const h = setup();
  h.send(spec);
  h.opened();
  h.activate();
  const old = h.provider.opens()[0];
  const oldReady = h.viewer.ready()[0];
  h.registry.sourceClosed(h.provider);
  const replacement = new Provider();
  assertEquals(h.registry.registerSource(replacement, "workspace"), null);
  assertEquals(h.registry.registerSource(new Provider(), "workspace"), "topic_source_conflict");
  const open = replacement.opens()[0];
  assertNotEquals(open.gen, old.gen);
  assertNotEquals(open.route, old.route);
  h.registry.onProviderMsg(replacement, { t: "topic_opened", route: open.route, gen: open.gen });
  assertNotEquals(h.viewer.ready().at(-1)?.route, oldReady.route);
  h.activate();
  h.sample(1, "demo/stale", old);
  h.sample(1, "demo/forged", open);
  h.sample(1, "demo/new", open, replacement);
  await flush();
  assertEquals(h.viewer.sink.decoded().map((f) => f.header.meta?.key), ["demo/new"]);
  h.registry.viewerClosed(h.viewer);
});

Deno.test("reliable upstream routes reject independent sample streams instead of reordering", async () => {
  const h = setup();
  h.send(spec);
  h.opened();
  h.activate();
  h.sample(1);
  await flush();
  h.sample(2, "demo/b", h.provider.opens()[0], h.provider, Symbol("other stream"));
  assertEquals(h.viewer.sink.kicked, []);
  assertEquals(h.viewer.messages.at(-1), {
    t: "topic_error",
    id: spec.id,
    code: "topic_order_lost",
    message: "topic order lost",
    outcome: "interrupted",
  });
  assertEquals(h.viewer.sink.frames.length, 1);
  h.registry.viewerClosed(h.viewer);
});

Deno.test("latest wildcard slots preserve quiet keys and reject stale sequence numbers per key", async () => {
  const h = setup();
  h.send({ ...spec, delivery: "latest" });
  h.opened();
  h.activate();
  h.sample(5, "demo/busy");
  h.sample(1, "demo/quiet");
  h.sample(4, "demo/busy");
  await flush();
  assertEquals(h.viewer.sink.decoded().map((f) => f.header.meta?.key), [
    "demo/busy",
    "demo/quiet",
  ]);
  for (let i = 0; i < 127; i++) {
    h.sample(i + 6, `demo/new${i}`);
    await flush();
  }
  assertEquals(h.viewer.sink.kicked, []);
  assertEquals(h.viewer.messages.at(-1), {
    t: "topic_error",
    id: spec.id,
    code: "topic_key_limit",
    message: "topic key limit",
    outcome: "interrupted",
  });
  h.registry.viewerClosed(h.viewer);
});

for (
  const code of ["topic_overflow", "topic_key_limit", "topic_send_failed", "topic_forward_failed"]
) {
  Deno.test(`provider ${code} reports the route error while other routes continue`, async () => {
    const h = setup();
    h.send(spec);
    h.opened();
    h.activate();
    const broken = h.provider.opens()[0];
    h.send({ ...spec, id: "healthy", keyExpr: "healthy/**" });
    h.opened();
    h.activate();
    h.registry.onProviderMsg(h.provider, {
      t: "topic_error",
      id: broken.route,
      code,
      message: code,
      outcome: "interrupted",
    });
    assertEquals(h.viewer.messages.at(-1), {
      t: "topic_error",
      id: spec.id,
      code,
      message: code.replaceAll("_", " "),
      outcome: "interrupted",
    });
    h.sample(1, "healthy/after-fault");
    await flush();
    assertEquals(h.viewer.sink.decoded().map((frame) => frame.header.meta?.key), [
      "healthy/after-fault",
    ]);
    assertEquals(h.viewer.sink.kicked, []);
    h.registry.viewerClosed(h.viewer);
  });
}

Deno.test("listing uses private receipt ids and rejects a different provider or oversized page", () => {
  const h = setup();
  h.send({
    t: "topic_list",
    id: "list",
    source: "workspace",
    prefix: "demo/",
    cursor: "",
    limit: 1,
  });
  const request = h.provider.messages[0];
  assert(request.t === "topic_list");
  assertNotEquals(request.id, "list");
  const page: TopicMsg = {
    t: "topic_page",
    id: request.id,
    source: "workspace",
    gen: 1,
    entries: [{ key: "demo/a", encoding: "", type: "", availability: "observed" }],
    cursor: "next",
  };
  h.registry.onProviderMsg(new Provider(), page);
  h.registry.onProviderMsg(h.provider, { ...page, entries: [...page.entries, ...page.entries] });
  assertEquals(h.viewer.messages.length, 1);
  h.registry.onProviderMsg(h.provider, page);
  assertEquals(h.viewer.messages.at(-1), { ...page, id: "list" });
  h.registry.viewerClosed(h.viewer);
});

Deno.test("reliable byte budget includes transport-accepted bytes until browser credit", async () => {
  const sink = new Sink();
  const budget = new TopicBudget(6);
  const failures: string[] = [];
  const egress = new TopicEgress("reliable", sink, [budget], (code) => failures.push(code));
  try {
    egress.activate();
    egress.offer("a", new Uint8Array([1, 2, 3]));
    egress.offer("b", new Uint8Array([4, 5, 6]));
    await flush();
    // Both logical frames arrive before any receipt, whether written together
    // or separately. Credits still release individual frame boundaries.
    assertEquals(sink.frames.flatMap((bytes) => [...bytes]), [1, 2, 3, 4, 5, 6]);
    assertEquals(budget.bytes, 6);
    egress.credit(1, 3);
    assertEquals(budget.bytes, 3);
    egress.offer("a", new Uint8Array([7, 8, 9]));
    await flush();
    egress.offer("a", new Uint8Array([10]));
    assertEquals(failures, ["topic_overflow"]);
    assertEquals(budget.bytes, 0);
  } finally {
    egress.dispose();
  }
});

Deno.test("fabricated reliable credit cannot release queued bytes", async () => {
  const budget = new TopicBudget(100);
  const failures: string[] = [];
  const egress = new TopicEgress("reliable", new Sink(), [budget], (code) => failures.push(code));
  try {
    egress.offer("a", new Uint8Array(20));
    egress.credit(1, 20); // not active, nothing reached the transport
    await flush();
    assertEquals(failures, ["invalid_credit"]);
    assertEquals(budget.bytes, 0);
  } finally {
    egress.dispose();
  }
});

Deno.test("latest pending bytes share the aggregate budget and a quiet key survives replacement", async () => {
  const sink = new Sink();
  const budget = new TopicBudget(10);
  const egress = new TopicEgress("latest", sink, [budget], () => {
    throw new Error("unexpected overflow");
  });
  try {
    egress.offer("busy", new Uint8Array([1, 1]));
    egress.offer("quiet", new Uint8Array([2, 2]));
    egress.offer("busy", new Uint8Array([3, 3, 3]));
    assertEquals(budget.bytes, 5);
    egress.activate();
    await flush();
    assertEquals(sink.frames.map((b) => [...b]), [[2, 2], [3, 3, 3]]);
  } finally {
    egress.dispose();
  }
  assertEquals(budget.bytes, 0);
});

function publication(id = "write", gen = 1, source = "workspace"): FrameHeader {
  return {
    ch: "@put",
    seq: 1,
    ts: 0,
    delivery: "reliable",
    meta: {
      id,
      source,
      gen,
      key: "runtime/command",
      encoding: "application/octet-stream",
    },
  };
}

Deno.test("binary publication requires provider write capability and a current source generation", () => {
  const h = setup();
  h.registry.registerSource(new Provider(), "sensors", {
    read: true,
    write: false,
    profiles: ["raw"],
  });
  h.registry.publish(h.viewer, publication("denied", 2, "sensors"), new Uint8Array([0, 255]));
  assertEquals(h.viewer.messages.at(-1), {
    t: "topic_pub_result",
    id: "denied",
    outcome: "rejected",
    code: "topic_write_denied",
    message: "topic write denied",
    providerTs: 0,
  });
  h.registry.publish(h.viewer, publication("old", 0), new Uint8Array([1]));
  assertEquals(h.provider.publications, []);
  const stale = h.viewer.messages.at(-1);
  assertEquals(stale?.t === "topic_pub_result" && stale.code, "stale_generation");
  h.registry.publish(h.viewer, publication(), new Uint8Array([0, 255, 1]));
  const forwarded = decodeDataFrame(h.provider.publications[0]);
  assertEquals([...forwarded.payload], [0, 255, 1]);
  assertNotEquals(forwarded.header.meta?.id, "write");
  const response: TopicMsg = {
    t: "topic_pub_result",
    id: forwarded.header.meta!.id as string,
    outcome: "accepted",
    code: "",
    message: "",
    providerTs: 5,
  };
  const received = h.viewer.messages.length;
  h.registry.onProviderMsg(new Provider(), response);
  assertEquals(h.viewer.messages.length, received);
  h.registry.onProviderMsg(h.provider, response);
  assertEquals(h.viewer.messages.at(-1), { ...response, id: "write" });
  h.registry.viewerClosed(h.viewer);
});

Deno.test("pending publications stay bounded until provider acknowledgement, then loss is unknown", () => {
  const h = setup();
  const writer = new Viewer();
  h.registry.addViewer(writer, writer.sendMsg);
  for (let i = 0; i < 17; i++) {
    h.registry.publish(writer, publication(`write${i}`), new Uint8Array([i]));
  }
  assertEquals(h.provider.publications.length, 16);
  assertEquals(writer.messages.at(-1), {
    t: "topic_pub_result",
    id: "write16",
    outcome: "rejected",
    code: "topic_publish_limit",
    message: "topic publish limit",
    providerTs: 0,
  });
  h.registry.reap(Date.now() + 20_000);
  assertEquals(h.provider.failures, ["topic_publish_timeout"]);
  h.registry.sourceClosed(h.provider);
  const results = writer.messages.filter((m) => m.t === "topic_pub_result");
  assertEquals(results.slice(1).map((m) => m.outcome), Array(16).fill("unknown"));
  h.registry.viewerClosed(writer);
  h.registry.viewerClosed(h.viewer);
});

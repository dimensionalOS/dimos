import { assert, assertEquals, assertNotEquals } from "@std/assert";
import { FakeTime } from "jsr:@std/testing@1/time";
import {
  ControlFrameReader,
  DataFrameStreamReader,
  decodeDataFrame,
  encodeControlFrame,
  type FrameHeader,
  MAX_CONTROL_PAYLOAD_BYTES,
  MAX_TOPIC_SOURCES,
  type Msg,
  type OptionRanges,
  topicCapabilitiesOk,
  type TopicMsg,
  topicMsgFromUnknown,
  type TopicOpenMsg,
  type TopicReadyMsg,
  type TopicRtcOfferMsg,
  type TopicSettings,
  type TopicSettingsUpdate,
  type TopicSubMsg,
  type TopicVariantMsg,
} from "@dimos/shared";
import type { FrameSend, ViewerSink } from "./forward.ts";
import { TopicBudget, TopicEgress } from "./topic_forward.ts";
import { type TopicProvider, TopicRegistry, type TopicViewer, VARIANT_GRACE_MS } from "./topics.ts";

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

Deno.test("a source at the viewer's own rate loses no frames to late deliveries", async () => {
  using time = new FakeTime(0);
  const { registry, viewer, send, opened, activate, sample } = setup();
  try {
    send({ ...spec, delivery: "latest", maxHz: 30 });
    opened();
    activate();
    const period = 1000 / 30;
    let at = 0;
    for (let seq = 1; seq <= 300; seq++) {
      // Every third frame reaches the relay 1.5 ms late, as a busy event loop delivers it.
      const arrival = (seq - 1) * period + (seq % 3 === 0 ? 1.5 : 0);
      time.tick(arrival - at);
      at = arrival;
      sample(seq);
      await flush();
    }
    time.tick(2 * period);
    await flush();
    assertEquals(viewer.sink.decoded().length, 300);
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

Deno.test("a source whose capabilities would push the announcement past one control frame is refused", () => {
  const h = setup();
  const profileOptions: Record<string, OptionRanges> = {};
  for (let p = 0; p < 16; p++) {
    profileOptions[`profile-${p}`] = {
      choice: { kind: "enum", values: Array.from({ length: 16 }, (_, i) => `value-${i}`) },
    };
  }
  const capabilities = {
    read: true,
    write: false,
    profiles: Object.keys(profileOptions),
    profileOptions,
  };
  assert(topicCapabilitiesOk(capabilities));
  let admitted = 0;
  while (h.registry.registerSource(new Provider(), `big-${admitted}`, capabilities) === null) {
    admitted++;
  }
  assert(admitted > 0 && admitted < MAX_TOPIC_SOURCES - 1);
  const announced = h.viewer.messages.at(-1)!;
  assertEquals(announced.t, "topic_sources");
  assert(
    new TextEncoder().encode(JSON.stringify(announced)).byteLength <= MAX_CONTROL_PAYLOAD_BYTES,
  );
  assertEquals(new ControlFrameReader().push(encodeControlFrame(announced)).length, 1);
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

// Encode variants (PLAN_2a): a media route serves up to four settings sets,
// each viewer points at one, and the relay keeps every viewer's own rate gate.
const OPTIONS: Record<string, OptionRanges> = {
  raw: { maxHz: { kind: "number", max: 1000, default: 1000 } },
  "image.jpeg.low.v1": {
    maxHz: { kind: "number", max: 1000, default: 30 },
    maxResolution: { kind: "size", min: 16, max: 4096, default: [640, 360] },
    quality: { kind: "int", min: 1, max: 100, default: 60 },
  },
};
const LOW: TopicSettings = { maxHz: 30, maxResolution: [640, 360], quality: 60 };
const camera: TopicSubMsg = {
  t: "topic_sub",
  id: "cam",
  source: "workspace",
  keyExpr: "sensor/cam",
  delivery: "latest",
  profile: "image.jpeg.low.v1",
  maxHz: 30,
};
const rejection = (revision: number, code: string, id = "cam"): Msg => ({
  t: "topic_error",
  id,
  revision,
  code,
  message: code.replaceAll("_", " "),
  outcome: "rejected",
});
function mediaSetup(
  profileOptions: Record<string, OptionRanges> | null = OPTIONS,
  profiles = ["raw", "image.jpeg.low.v1"],
) {
  let clock = 1000;
  const registry = new TopicRegistry(() => clock);
  const provider = new Provider();
  registry.registerSource(provider, "workspace", {
    read: true,
    write: false,
    profiles,
    ...(profileOptions === null ? {} : { profileOptions }),
  });
  provider.messages = [];
  const viewers = new Map<string, Viewer>();
  const viewer = (name: string) => {
    let v = viewers.get(name);
    if (v === undefined) {
      v = new Viewer();
      registry.addViewer(v, v.sendMsg);
      viewers.set(name, v);
    }
    return v;
  };
  const send = (msg: TopicMsg, target: Viewer) => registry.onViewerMsg(target, msg, target.sendMsg);
  const subscribe = (
    name: string,
    settings?: Partial<TopicSettings>,
    over: Partial<TopicSubMsg> = {},
  ) => {
    const v = viewer(name);
    send({ ...camera, ...over, ...(settings === undefined ? {} : { settings }) }, v);
    return v;
  };
  const opened = (open = provider.opens().at(-1)!) =>
    registry.onProviderMsg(provider, { t: "topic_opened", route: open.route, gen: open.gen });
  const variants = () =>
    provider.messages.filter((m): m is TopicVariantMsg => m.t === "topic_variant");
  const variantReady = (msg = variants().at(-1)!) =>
    registry.onProviderMsg(provider, { ...msg, t: "topic_variant_ready" });
  const activate = (v: Viewer, ready = v.ready().at(-1)!) =>
    send({ t: "topic_activate", route: ready.route, gen: ready.gen }, v);
  const frame = (
    seq: number,
    variant: string | undefined,
    key = "sensor/cam",
    open = provider.opens().at(-1)!,
  ) =>
    registry.onFrame(
      provider,
      {
        ch: open.route,
        seq,
        ts: 5,
        delivery: "latest",
        meta: {
          key,
          encoding: "image/jpeg",
          gen: open.gen,
          ingressTs: 4,
          ...(variant === undefined ? {} : { variant }),
        },
      },
      new Uint8Array([seq]),
      Symbol("frame"),
    );
  const received = (v: Viewer) => v.sink.decoded().map((f) => f.header.seq);
  const tick = (ms: number) => {
    clock += ms;
    registry.reap(clock);
  };
  const update = (v: Viewer, revision: number, options: TopicSettingsUpdate) =>
    send({ t: "topic_update", id: "cam", revision, options }, v);
  const last = (v: Viewer) => v.messages.at(-1);
  const close = () => {
    for (const v of viewers.values()) registry.viewerClosed(v);
  };
  return {
    registry,
    provider,
    subscribe,
    opened,
    variants,
    variantReady,
    activate,
    frame,
    received,
    tick,
    update,
    last,
    close,
  };
}

Deno.test("media viewers share one route, distinct settings get their own variant, and frames dispatch on meta.variant", async () => {
  const h = mediaSetup();
  const a = h.subscribe("a", { quality: 40 });
  const open = h.provider.opens()[0];
  assertEquals(open.settings, { maxHz: 30, maxResolution: [640, 360], quality: 40 });
  assertEquals(a.ready(), []);
  h.opened();
  assertEquals(a.ready()[0].settings, { maxHz: 30, maxResolution: [640, 360], quality: 40 });
  // Identical settings join the variant that already encodes them.
  const b = h.subscribe("b", { quality: 40 });
  assertEquals(b.ready()[0].settings, { maxHz: 30, maxResolution: [640, 360], quality: 40 });
  assertEquals(h.provider.opens().length, 1);
  assertEquals(h.variants(), []);
  const c = h.subscribe("c", { maxResolution: [320, 240] });
  assertEquals(h.variants().map((m) => [m.variant, m.settings]), [
    ["v2", { maxHz: 30, maxResolution: [320, 240], quality: 60 }],
  ]);
  assertEquals(c.ready(), []);
  h.variantReady();
  assertEquals(c.ready()[0].settings, { maxHz: 30, maxResolution: [320, 240], quality: 60 });
  for (const v of [a, b, c]) h.activate(v);
  assertEquals(h.provider.messages.filter((m) => m.t === "topic_activate"), [
    { t: "topic_activate", route: open.route, gen: open.gen },
  ]);
  h.frame(1, "v1");
  h.frame(2, "v2");
  h.frame(3, undefined);
  h.frame(4, "v3");
  await flush();
  assertEquals([h.received(a), h.received(b), h.received(c)], [[1], [1], [2]]);
  assertEquals(a.sink.decoded()[0].header.meta, {
    key: "sensor/cam",
    encoding: "image/jpeg",
    gen: open.gen,
    ingressTs: 4,
  });
  h.close();
});

Deno.test("an update moves the viewer once the new variant encodes, and the idle variant closes after its grace", async () => {
  const h = mediaSetup();
  const a = h.subscribe("a");
  h.opened();
  h.activate(a);
  const open = h.provider.opens()[0];
  h.update(a, 1, { quality: 40 });
  assertEquals(h.variants().at(-1), {
    t: "topic_variant",
    route: open.route,
    gen: open.gen,
    variant: "v2",
    settings: { maxHz: 30, maxResolution: [640, 360], quality: 40 },
  });
  // Until the provider encodes the new variant, the old one keeps flowing.
  h.frame(1, "v1");
  await flush();
  assertEquals(h.received(a), [1]);
  assertEquals(a.messages.some((m) => m.t === "topic_applied"), false);
  h.variantReady();
  assertEquals(h.last(a), {
    t: "topic_applied",
    id: "cam",
    revision: 1,
    settings: { maxHz: 30, maxResolution: [640, 360], quality: 40 },
  });
  h.tick(100);
  h.frame(2, "v1");
  h.frame(3, "v2");
  await flush();
  assertEquals(h.received(a), [1, 3]);
  // v1 has no viewer: it closes after the grace, not at once.
  h.tick(VARIANT_GRACE_MS - 101);
  assertEquals(h.provider.messages.some((m) => m.t === "topic_variant_close"), false);
  h.tick(1);
  assertEquals(h.provider.messages.at(-1), {
    t: "topic_variant_close",
    route: open.route,
    gen: open.gen,
    variant: "v1",
  });
  // A reset returns to the profile settings, in a fresh variant: ids are never reused.
  h.update(a, 2, { quality: null });
  assertEquals(h.variants().at(-1), {
    t: "topic_variant",
    route: open.route,
    gen: open.gen,
    variant: "v3",
    settings: LOW,
  });
  h.variantReady();
  assertEquals(h.last(a), { t: "topic_applied", id: "cam", revision: 2, settings: LOW });
  h.close();
});

Deno.test("a variant in its grace period serves a new viewer and stays open", async () => {
  const h = mediaSetup();
  const a = h.subscribe("a");
  h.opened();
  h.update(a, 1, { quality: 40 });
  h.variantReady();
  h.tick(VARIANT_GRACE_MS / 2);
  // v1 still encodes the profile settings: b takes it instead of a fresh variant.
  const b = h.subscribe("b");
  assertEquals(b.ready()[0].settings, LOW);
  assertEquals(h.variants().length, 1);
  h.tick(VARIANT_GRACE_MS);
  assertEquals(h.provider.messages.some((m) => m.t === "topic_variant_close"), false);
  h.activate(b);
  h.frame(1, "v1");
  await flush();
  assertEquals(h.received(b), [1]);
  h.close();
});

Deno.test("a change sent before the previous one is acknowledged keeps that one's settings", () => {
  const h = mediaSetup();
  const a = h.subscribe("a");
  h.opened();
  h.update(a, 1, { quality: 40 });
  h.update(a, 2, { maxResolution: [320, 240] });
  assertEquals(h.variants().map((m) => m.settings), [
    { maxHz: 30, maxResolution: [640, 360], quality: 40 },
    { maxHz: 30, maxResolution: [320, 240], quality: 40 },
  ]);
  h.variantReady();
  assertEquals(h.last(a), {
    t: "topic_applied",
    id: "cam",
    revision: 2,
    settings: { maxHz: 30, maxResolution: [320, 240], quality: 40 },
  });
  h.close();
});

Deno.test("a variant acknowledged before the route opens serves its viewer once the route is open", () => {
  const h = mediaSetup();
  h.subscribe("a");
  const b = h.subscribe("b", { quality: 40 });
  assertEquals(h.variants().map((m) => m.variant), ["v2"]);
  // A provider may answer the variant before the route itself.
  h.variantReady();
  assertEquals(b.ready(), []);
  h.opened();
  assertEquals(b.ready().map((m) => m.settings), [{ ...LOW, quality: 40 }]);
  h.close();
});

Deno.test("a viewer moving off a variant lowers that variant's rate to whoever remains", () => {
  const h = mediaSetup();
  const a = h.subscribe("a");
  h.subscribe("b", { maxHz: 5 });
  h.opened();
  const open = h.provider.opens()[0];
  assertEquals(h.variants(), []);
  h.update(a, 1, { quality: 40 });
  h.variantReady();
  // Only b is left on v1: it samples at b's rate now.
  assertEquals(h.variants().at(-1), {
    t: "topic_variant",
    route: open.route,
    gen: open.gen,
    variant: "v1",
    settings: { ...LOW, maxHz: 5 },
  });
  h.close();
});

Deno.test("frames queued under the old variant are dropped when the viewer moves", async () => {
  const h = mediaSetup();
  const a = h.subscribe("a", { maxHz: 1 });
  h.opened();
  h.activate(a);
  h.frame(1, "v1");
  h.frame(2, "v1");
  await flush();
  assertEquals(h.received(a), [1]);
  h.update(a, 1, { quality: 40 });
  h.variantReady();
  // The second frame was due now, but it carries the old settings.
  h.tick(1000);
  await flush();
  assertEquals(h.received(a), [1]);
  h.frame(3, "v2");
  await flush();
  assertEquals(h.received(a), [1, 3]);
  h.close();
});

Deno.test("a fifth settings set rejects with variant_limit until a variant in its grace period can be retired", () => {
  const h = mediaSetup();
  const viewers = [10, 20, 30, 40].map((quality, i) => h.subscribe(`viewer${i}`, { quality }));
  h.opened();
  for (const announced of h.variants()) h.variantReady(announced);
  assertEquals(viewers.map((v) => v.ready().length), [1, 1, 1, 1]);
  h.update(viewers[0], 1, { quality: 50 });
  assertEquals(h.last(viewers[0]), rejection(1, "variant_limit"));
  assertEquals(h.variants().length, 3);
  assertEquals(viewers[0].ready()[0].settings?.quality, 10);
  // An unused variant gives way at once; its grace only guards against churn.
  h.registry.viewerClosed(viewers[3]);
  h.update(viewers[0], 2, { quality: 50 });
  assertEquals(h.provider.messages.slice(-2).map((m) => m.t), [
    "topic_variant_close",
    "topic_variant",
  ]);
  assertEquals(h.variants().at(-1)?.variant, "v5");
  h.close();
});

Deno.test("a frame delayed past its variant's retirement never reaches a viewer of the replacement", async () => {
  const h = mediaSetup();
  const viewers = [10, 20, 30, 40].map((quality, i) => h.subscribe(`viewer${i}`, { quality }));
  h.opened();
  for (const announced of h.variants()) h.variantReady(announced);
  h.activate(viewers[0]);
  // v4 at quality 40 retires at once to make room, while its frames may still fly.
  h.registry.viewerClosed(viewers[3]);
  h.update(viewers[0], 1, { quality: 50 });
  assertEquals(h.provider.messages.at(-2), {
    t: "topic_variant_close",
    route: h.provider.opens()[0].route,
    gen: h.provider.opens()[0].gen,
    variant: "v4",
  });
  assertEquals(h.variants().at(-1)?.variant, "v5");
  h.variantReady();
  assertEquals(h.last(viewers[0]), {
    t: "topic_applied",
    id: "cam",
    revision: 1,
    settings: { ...LOW, quality: 50 },
  });
  h.frame(1, "v4");
  await flush();
  assertEquals(h.received(viewers[0]), []);
  h.tick(100);
  h.frame(2, "v5");
  await flush();
  assertEquals(h.received(viewers[0]), [2]);
  h.close();
});

Deno.test("viewer rates stay separate and raise only their variant; settings validate against the advertised ranges", () => {
  const h = mediaSetup();
  const slow = h.subscribe("slow", undefined, { maxHz: 5 });
  h.opened();
  const open = h.provider.opens()[0];
  assertEquals(open.settings?.maxHz, 5);
  const fast = h.subscribe("fast", undefined, { maxHz: 30 });
  assertEquals(h.variants().at(-1), {
    t: "topic_variant",
    route: open.route,
    gen: open.gen,
    variant: "v1",
    settings: LOW,
  });
  assertEquals(fast.ready()[0].settings, LOW);
  assertEquals(slow.ready()[0].settings, { ...LOW, maxHz: 5 });
  h.activate(slow);
  assertEquals(h.provider.messages.at(-1), {
    t: "topic_activate",
    route: open.route,
    gen: open.gen,
  });
  h.registry.viewerClosed(fast);
  assertEquals(h.variants().at(-1)?.settings.maxHz, 5);
  // Beyond the profile ceiling comes back clamped; null resets to it.
  h.update(slow, 1, { maxHz: 60 });
  assertEquals(h.last(slow), { t: "topic_applied", id: "cam", revision: 1, settings: LOW });
  h.update(slow, 2, { maxHz: null, maxResolution: [3000, 3000] });
  assertEquals(h.last(slow), { t: "topic_applied", id: "cam", revision: 2, settings: LOW });
  h.update(slow, 3, { quality: 0 });
  assertEquals(h.last(slow), rejection(3, "option_out_of_range"));
  h.update(slow, 4, { keepEvery: 2 });
  assertEquals(h.last(slow), rejection(4, "unsupported_option"));
  assertEquals(h.variants().length, 3); // rejected and same-variant updates open nothing
  h.close();
});

Deno.test("an older provider without profile options takes rate changes only, gated by the relay", async () => {
  const h = mediaSetup(null);
  const refused = h.subscribe("refused", { quality: 40 });
  assertEquals(h.last(refused), {
    t: "topic_error",
    id: "cam",
    code: "unsupported_option",
    message: "unsupported option",
    outcome: "rejected",
  });
  assertEquals(h.provider.messages, []);
  const a = h.subscribe("a");
  const open = h.provider.opens()[0];
  assertEquals(open.settings, undefined);
  h.opened();
  assertEquals(a.ready()[0].settings, { maxHz: 30 });
  h.activate(a);
  assertEquals(h.provider.messages.at(-1), {
    t: "topic_activate",
    route: open.route,
    gen: open.gen,
    maxHz: 30,
  });
  h.update(a, 1, { maxHz: 10 });
  assertEquals(h.last(a), { t: "topic_applied", id: "cam", revision: 1, settings: { maxHz: 10 } });
  assertEquals(h.provider.messages.at(-1), {
    t: "topic_activate",
    route: open.route,
    gen: open.gen,
    maxHz: 10,
  });
  h.update(a, 2, { quality: 40 });
  assertEquals(h.last(a), rejection(2, "unsupported_option"));
  h.frame(1, undefined);
  await flush();
  assertEquals(h.received(a), [1]);
  h.close();
});

Deno.test("a late variant acknowledgement is ignored and a source restart reopens the settings in force", () => {
  const h = mediaSetup();
  const a = h.subscribe("a");
  h.opened();
  const open = h.provider.opens()[0];
  h.update(a, 1, { quality: 40 });
  h.variantReady();
  h.tick(VARIANT_GRACE_MS);
  assertEquals(h.provider.messages.at(-1)?.t, "topic_variant_close");
  h.update(a, 2, { quality: 50 });
  assertEquals(h.variants().at(-1)?.variant, "v3");
  // The provider answers for the retired v1 first; that id is gone for good.
  h.registry.onProviderMsg(h.provider, {
    t: "topic_variant_ready",
    route: open.route,
    gen: open.gen,
    variant: "v1",
    settings: LOW,
  });
  assertEquals(a.messages.filter((m) => m.t === "topic_applied").length, 1);
  h.variantReady();
  assertEquals(h.last(a), {
    t: "topic_applied",
    id: "cam",
    revision: 2,
    settings: { ...LOW, quality: 50 },
  });
  // Restart: the desired subscription reopens its current settings on a fresh route.
  h.registry.sourceClosed(h.provider);
  const replacement = new Provider();
  h.registry.registerSource(replacement, "workspace", {
    read: true,
    write: false,
    profiles: ["raw", "image.jpeg.low.v1"],
    profileOptions: OPTIONS,
  });
  const reopened = replacement.opens()[0];
  assertNotEquals(reopened.route, open.route);
  assertEquals(reopened.settings, { ...LOW, quality: 50 });
  h.close();
});

Deno.test("a raw route takes rate updates in the relay and a reliable route still keeps every event", async () => {
  const h = setup();
  h.send(spec);
  h.opened();
  h.activate();
  const open = h.provider.opens()[0];
  h.send({ t: "topic_update", id: spec.id, revision: 1, options: { maxHz: 1 } });
  assertEquals(h.viewer.messages.at(-1), {
    t: "topic_applied",
    id: spec.id,
    revision: 1,
    settings: { maxHz: 1 },
  });
  assertEquals(h.provider.messages.at(-1), {
    t: "topic_activate",
    route: open.route,
    gen: open.gen,
    maxHz: 1,
  });
  h.sample(1);
  h.sample(2);
  h.sample(3);
  await flush();
  assertEquals(h.viewer.sink.decoded().map((f) => f.header.seq), [1, 2, 3]);
  h.send({ t: "topic_update", id: spec.id, revision: 2, options: { quality: 40 } });
  assertEquals(h.viewer.messages.at(-1), rejection(2, "unsupported_option", spec.id));
  h.send({ t: "topic_update", id: "nope", revision: 3, options: {} });
  assertEquals(h.viewer.messages.at(-1), rejection(3, "unknown_topic_handle", "nope"));
  h.registry.viewerClosed(h.viewer);
});

// Direct video (PLAN_2b): the relay signals one peer per subscription and
// never carries the media.
const VIDEO_OPTIONS: Record<string, OptionRanges> = {
  ...OPTIONS,
  "video.h264.v1": {
    maxHz: { kind: "number", max: 1000, default: 30 },
    maxResolution: { kind: "size", min: 16, max: 4096, default: [1920, 1080] },
  },
};
function videoSetup() {
  const h = mediaSetup(VIDEO_OPTIONS, ["raw", "image.jpeg.low.v1", "video.h264.v1"]);
  const v = h.subscribe("cam", undefined, { profile: "video.h264.v1" });
  h.opened();
  const ready = v.ready()[0];
  const open = h.provider.opens()[0];
  const offer = (over: Partial<TopicRtcOfferMsg> = {}, target: Viewer = v) =>
    h.registry.onViewerMsg(
      target,
      {
        t: "topic_rtc_offer",
        id: "cam",
        route: ready.route,
        gen: ready.gen,
        sdp: "v=0 β",
        ...over,
      },
      target.sendMsg,
    );
  const forwarded = () =>
    h.provider.messages.filter((m): m is TopicRtcOfferMsg => m.t === "topic_rtc_offer");
  const rtc = (target: Viewer = v) => target.messages.filter((m) => m.t.startsWith("topic_rtc_"));
  const fromProvider = (msg: TopicMsg) => h.registry.onProviderMsg(h.provider, msg);
  return { ...h, v, ready, open, offer, forwarded, rtc, fromProvider };
}

Deno.test("a video offer reaches the provider with a private id, its route and variant, and replies map back", () => {
  const h = videoSetup();
  assertEquals(h.ready.settings, { maxHz: 30, maxResolution: [1920, 1080] });
  h.offer();
  const [upstream] = h.forwarded();
  assertNotEquals(upstream.id, "cam");
  assertEquals(upstream, {
    t: "topic_rtc_offer",
    id: upstream.id,
    route: h.open.route,
    gen: h.open.gen,
    sdp: "v=0 β",
    variant: "v1",
  });
  const peer = { id: upstream.id, route: h.open.route, gen: h.open.gen };
  h.fromProvider({ t: "topic_rtc_answer", ...peer, sdp: "answer" });
  h.fromProvider({ t: "topic_rtc_candidate", ...peer, candidate: "candidate:1 host", mid: "0" });
  h.registry.onViewerMsg(h.v, {
    t: "topic_rtc_candidate",
    id: "cam",
    route: h.ready.route,
    gen: h.ready.gen,
    candidate: "",
    mid: "0",
  }, h.v.sendMsg);
  const viewer = { id: "cam", route: h.ready.route, gen: h.ready.gen };
  assertEquals(h.rtc(), [
    { t: "topic_rtc_answer", ...viewer, sdp: "answer" },
    { t: "topic_rtc_candidate", ...viewer, candidate: "candidate:1 host", mid: "0" },
  ]);
  assertEquals(h.provider.messages.at(-1), {
    t: "topic_rtc_candidate",
    ...peer,
    candidate: "",
    mid: "0",
  });
  h.fromProvider({ t: "topic_rtc_close", ...peer, reason: "ice_failed" });
  assertEquals(h.rtc().at(-1), { t: "topic_rtc_close", ...viewer, reason: "ice_failed" });
  // The peer is gone: a late candidate is dropped, a provider reply too.
  const sent = h.provider.messages.length;
  const received = h.v.messages.length;
  h.registry.onViewerMsg(
    h.v,
    { t: "topic_rtc_candidate", ...viewer, candidate: "x", mid: "0" },
    h.v.sendMsg,
  );
  h.fromProvider({ t: "topic_rtc_answer", ...peer, sdp: "late" });
  assertEquals([h.provider.messages.length, h.v.messages.length], [sent, received]);
  h.close();
});

Deno.test("a video offer binds to this viewer's route, id and generation, and one offer is in flight per route", () => {
  const h = videoSetup();
  const stranger = new Viewer();
  h.registry.addViewer(stranger, stranger.sendMsg);
  h.offer({}, stranger);
  assertEquals(stranger.messages.at(-1), {
    t: "topic_error",
    id: "cam",
    code: "unknown_topic_handle",
    message: "unknown topic handle",
    outcome: "rejected",
  });
  h.offer({ gen: h.ready.gen + 1 });
  h.offer({ id: "other" });
  h.offer({ route: "@t:v999" });
  assertEquals(h.v.messages.filter((m) => m.t === "topic_error").length, 3);
  assertEquals(h.forwarded(), []);
  h.offer();
  h.offer();
  assertEquals(h.v.messages.at(-1), {
    t: "topic_error",
    id: "cam",
    code: "rtc_offer_pending",
    message: "rtc offer pending",
    outcome: "rejected",
  });
  assertEquals(h.forwarded().length, 1);
  // Answered, a new offer replaces the peer: the provider sees the old one close.
  const [first] = h.forwarded();
  h.fromProvider({
    t: "topic_rtc_answer",
    id: first.id,
    route: h.open.route,
    gen: h.open.gen,
    sdp: "a",
  });
  h.offer();
  assertEquals(h.provider.messages.slice(-2).map((m) => m.t), [
    "topic_rtc_close",
    "topic_rtc_offer",
  ]);
  assertEquals(h.provider.messages.at(-2), {
    t: "topic_rtc_close",
    id: first.id,
    route: h.open.route,
    gen: h.open.gen,
    reason: "replaced",
  });
  assertNotEquals(h.forwarded()[1].id, first.id);
  h.registry.viewerClosed(stranger);
  h.close();
});

Deno.test("dropping the viewer or its subscription closes the peer at the provider", () => {
  const h = videoSetup();
  h.offer();
  const [upstream] = h.forwarded();
  h.registry.onViewerMsg(h.v, { t: "topic_unsub", id: "cam" }, h.v.sendMsg);
  assertEquals(h.provider.messages.slice(-2), [
    {
      t: "topic_rtc_close",
      id: upstream.id,
      route: h.open.route,
      gen: h.open.gen,
      reason: "route_closed",
    },
    { t: "topic_close", route: h.open.route, gen: h.open.gen },
  ]);
  const received = h.v.messages.length;
  h.fromProvider({
    t: "topic_rtc_answer",
    id: upstream.id,
    route: h.open.route,
    gen: h.open.gen,
    sdp: "late",
  });
  assertEquals(h.v.messages.length, received);
  const again = h.subscribe("cam", undefined, { profile: "video.h264.v1" });
  h.opened();
  const ready = again.ready().at(-1)!;
  h.offer({ route: ready.route, gen: ready.gen }, again);
  const second = h.forwarded()[1];
  h.registry.viewerClosed(again);
  assertEquals(h.provider.messages.at(-2), {
    t: "topic_rtc_close",
    id: second.id,
    route: h.provider.opens()[1].route,
    gen: h.open.gen,
    reason: "route_closed",
  });
  h.close();
});

Deno.test("rtc_limit from the provider ends the peer at the viewer, who may offer again", () => {
  const h = videoSetup();
  h.offer();
  const [first] = h.forwarded();
  h.fromProvider({
    t: "topic_rtc_close",
    id: first.id,
    route: h.open.route,
    gen: h.open.gen,
    reason: "rtc_limit",
  });
  assertEquals(h.rtc().at(-1), {
    t: "topic_rtc_close",
    id: "cam",
    route: h.ready.route,
    gen: h.ready.gen,
    reason: "rtc_limit",
  });
  assertEquals(h.provider.messages.filter((m) => m.t === "topic_rtc_close"), []);
  h.offer();
  assertEquals(h.forwarded().length, 2);
  assertNotEquals(h.forwarded()[1].id, first.id);
  h.close();
});

Deno.test("an unanswered video offer times out on both sides and the 65th candidate closes the peer", () => {
  const h = videoSetup();
  h.activate(h.v);
  h.offer();
  const [first] = h.forwarded();
  h.tick(9_999);
  assertEquals(h.rtc(), []);
  h.tick(1);
  assertEquals(h.rtc(), [{
    t: "topic_rtc_close",
    id: "cam",
    route: h.ready.route,
    gen: h.ready.gen,
    reason: "rtc_answer_timeout",
  }]);
  assertEquals(h.provider.messages.at(-1), {
    t: "topic_rtc_close",
    id: first.id,
    route: h.open.route,
    gen: h.open.gen,
    reason: "rtc_answer_timeout",
  });
  h.offer();
  const second = h.forwarded()[1];
  h.fromProvider({
    t: "topic_rtc_answer",
    id: second.id,
    route: h.open.route,
    gen: h.open.gen,
    sdp: "a",
  });
  h.tick(60_000);
  assertEquals(h.rtc().length, 2);
  for (let n = 0; n < 64; n++) {
    h.registry.onViewerMsg(h.v, {
      t: "topic_rtc_candidate",
      id: "cam",
      route: h.ready.route,
      gen: h.ready.gen,
      candidate: `candidate:${n}`,
      mid: "0",
    }, h.v.sendMsg);
  }
  assertEquals(h.provider.messages.filter((m) => m.t === "topic_rtc_candidate").length, 64);
  h.registry.onViewerMsg(h.v, {
    t: "topic_rtc_candidate",
    id: "cam",
    route: h.ready.route,
    gen: h.ready.gen,
    candidate: "candidate:64",
    mid: "0",
  }, h.v.sendMsg);
  assertEquals(h.provider.messages.at(-1), {
    t: "topic_rtc_close",
    id: second.id,
    route: h.open.route,
    gen: h.open.gen,
    reason: "rtc_candidate_limit",
  });
  assertEquals(h.rtc().at(-1), {
    t: "topic_rtc_close",
    id: "cam",
    route: h.ready.route,
    gen: h.ready.gen,
    reason: "rtc_candidate_limit",
  });
  h.close();
});

Deno.test("a settings change to another encoder closes the peer with settings_changed and the new offer names the new variant", async () => {
  const h = videoSetup();
  h.activate(h.v);
  h.offer();
  const [first] = h.forwarded();
  h.fromProvider({
    t: "topic_rtc_answer",
    id: first.id,
    route: h.open.route,
    gen: h.open.gen,
    sdp: "a",
  });
  h.update(h.v, 1, { maxResolution: [640, 360] });
  assertEquals(h.variants().at(-1)?.variant, "v2");
  h.variantReady();
  assertEquals(h.provider.messages.at(-1), {
    t: "topic_rtc_close",
    id: first.id,
    route: h.open.route,
    gen: h.open.gen,
    reason: "settings_changed",
  });
  assertEquals(h.v.messages.slice(-2), [
    {
      t: "topic_rtc_close",
      id: "cam",
      route: h.ready.route,
      gen: h.ready.gen,
      reason: "settings_changed",
    },
    {
      t: "topic_applied",
      id: "cam",
      revision: 1,
      settings: { maxHz: 30, maxResolution: [640, 360] },
    },
  ]);
  h.offer();
  assertEquals(h.forwarded()[1].variant, "v2");
  // No data frames ever flow on a video route.
  h.frame(1, "v2");
  await flush();
  assertEquals(h.received(h.v), []);
  h.close();
});

Deno.test("a video rate is part of the encoder: viewers at one rate share it, another rate gets its own", () => {
  const h = videoSetup();
  const profile = "video.h264.v1";
  const full: TopicSettings = { maxHz: 30, maxResolution: [1920, 1080] };
  const slow = h.subscribe("slow", undefined, { profile, maxHz: 5 });
  assertEquals(h.variants().map((m) => [m.variant, m.settings]), [["v2", { ...full, maxHz: 5 }]]);
  h.variantReady();
  assertEquals(slow.ready()[0].settings, { ...full, maxHz: 5 });
  assertEquals(h.v.ready()[0].settings, full);
  const slower = h.subscribe("slower", undefined, { profile, maxHz: 5 });
  assertEquals(slower.ready()[0].settings, { ...full, maxHz: 5 });
  assertEquals(h.variants().length, 1);
  // v1 stays at 30 and v2 at 5: neither viewer raises the other's encoder.
  h.registry.viewerClosed(slower);
  assertEquals(h.variants().length, 1);
  // A rate change moves the viewer to another encoder, like any other setting.
  h.offer();
  const [first] = h.forwarded();
  h.fromProvider({
    t: "topic_rtc_answer",
    id: first.id,
    route: h.open.route,
    gen: h.open.gen,
    sdp: "a",
  });
  h.update(h.v, 1, { maxHz: 5 });
  assertEquals(h.v.messages.slice(-2), [
    {
      t: "topic_rtc_close",
      id: "cam",
      route: h.ready.route,
      gen: h.ready.gen,
      reason: "settings_changed",
    },
    { t: "topic_applied", id: "cam", revision: 1, settings: { ...full, maxHz: 5 } },
  ]);
  h.offer();
  assertEquals(h.forwarded()[1].variant, "v2");
  h.close();
});

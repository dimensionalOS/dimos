import { afterEach, describe, expect, it, vi } from "vitest";
import {
  type DataFrame,
  decodeDataFrame,
  type Msg,
  PROTOCOL_VERSION,
  type TopicSettings,
  type TopicSubMsg,
  type TopicUpdateMsg,
} from "@dimos/shared";
import { connect, type Session } from "./session.ts";
import { TopicClient, type TopicOptions, type TopicSample } from "./topics.ts";
import { FakeRelayEnd, INFO, until } from "./testing/fakeRelay.ts";

describe("raw topic subscriptions", () => {
  const owned: { close(): void }[] = [];
  afterEach(() => {
    for (const item of owned.splice(0)) item.close();
    vi.useRealTimers();
  });
  function setup() {
    const client = new TopicClient("workspace");
    owned.push(client);
    const sent: Msg[] = [];
    const disconnect = vi.fn();
    const binary: Uint8Array[] = [];
    client.connected((m) => sent.push(m), disconnect, (bytes) => {
      binary.push(bytes);
      return Promise.resolve();
    });
    const sources = (gen = 1, write = false) =>
      client.onMessage({
        t: "topic_sources",
        v: 1,
        sources: [{ source: "workspace", gen, read: true, write, profiles: ["raw"] }],
      });
    const subs = () => sent.filter((m): m is TopicSubMsg => m.t === "topic_sub");
    const ready = (route = "@t:v1", gen = 1, sub = subs().at(-1)!) =>
      client.onMessage({ t: "topic_ready", id: sub.id, source: "workspace", route, gen });
    const sample = (
      seq: number,
      key = "demo/a",
      route = "@t:v1",
      gen = 1,
      delivery: "reliable" | "latest" = "reliable",
    ): DataFrame => ({
      header: {
        ch: route,
        seq,
        ts: 30,
        delivery,
        meta: { key, gen, encoding: "application/octet-stream", ingressTs: 20, sourceTs: 10 },
      },
      payload: new Uint8Array([seq, 0, 255]),
    });
    return { client, sent, disconnect, sources, subs, ready, sample, binary };
  }

  it("returns a handle immediately, acknowledges installation, and calls every event outside UI ticks", async () => {
    const h = setup();
    const seen: TopicSample[] = [];
    const sub = h.client.subscribe(
      "demo/**",
      { delivery: "reliable" },
      (sample) => seen.push(sample),
    );
    const installed = vi.fn();
    sub.ready().then(installed);
    expect(h.subs()).toEqual([]);
    h.sources();
    await Promise.resolve();
    expect(installed).not.toHaveBeenCalled();
    h.ready();
    await sub.ready();
    expect(h.sent.at(-1)).toEqual({ t: "topic_activate", route: "@t:v1", gen: 1 });
    for (let i = 1; i <= 100; i++) h.client.ingest(h.sample(i, `demo/key${i % 3}`));
    expect(seen.map((s) => s.sequence)).toEqual(Array.from({ length: 100 }, (_, i) => i + 1));
    expect(seen[0]).toMatchObject({
      key: "demo/key1",
      payload: new Uint8Array([1, 0, 255]),
      sourceTs: 10,
      ingressTs: 20,
      sentTs: 30,
    });
    await until(() => h.sent.some((m) => m.t === "topic_credit"));
    expect(h.sent.at(-1)).toMatchObject({ t: "topic_credit", frames: 100 });
  });

  it("bounds a page at 64 handles and admits a new subscription after disposal", async () => {
    const h = setup();
    h.sources();
    const handles = Array.from(
      { length: 64 },
      (_, index) => h.client.subscribe(`screen/${index}`, {}, vi.fn()),
    );
    for (const [index, request] of h.subs().entries()) h.ready(`@t:screen${index}`, 1, request);
    await Promise.all(handles.map((handle) => handle.ready()));
    const refused = h.client.subscribe("screen/extra", {}, vi.fn());
    await expect(refused.ready()).rejects.toMatchObject({ code: "topic_limit" });
    expect(h.subs()).toHaveLength(64);
    expect(handles.every((handle) => handle.status().phase === "ready")).toBe(true);
    handles[0].close();
    refused.close();
    const replacement = h.client.subscribe("screen/extra", {}, vi.fn());
    h.ready("@t:replacement");
    await expect(replacement.ready()).resolves.toBeUndefined();
    expect(h.subs()).toHaveLength(65);
    expect(h.disconnect).not.toHaveBeenCalled();
  });

  it("does not announce recovery during a source's interruption burst", async () => {
    const h = setup();
    h.sources();
    const handles = ["camera", "events"].map((key) => h.client.subscribe(key, {}, vi.fn()));
    const requests = h.subs();
    requests.forEach((request, index) => h.ready(`@t:before${index}`, 1, request));
    await Promise.all(handles.map((handle) => handle.ready()));
    const reconnected = vi.fn();
    h.client.onReconnect(reconnected);
    for (const request of requests) {
      h.client.onMessage({
        t: "topic_error",
        id: request.id,
        code: "source_offline",
        outcome: "interrupted",
        message: "source disconnected",
      });
      expect(reconnected).not.toHaveBeenCalled();
    }
    h.client.onMessage({ t: "topic_sources", v: 1, sources: [] });
    h.sources(2);
    h.ready("@t:after0", 2, requests[0]);
    expect(reconnected).not.toHaveBeenCalled();
    h.ready("@t:after1", 2, requests[1]);
    expect(reconnected).toHaveBeenCalledOnce();
  });

  it("credits original wire bytes when valid JSON has noncanonical number spelling", async () => {
    const h = setup();
    h.sources();
    const received = vi.fn();
    const sub = h.client.subscribe("demo/**", { delivery: "reliable" }, received);
    h.ready();
    await sub.ready();
    const sample = h.sample(1);
    const json = "\n " + JSON.stringify(sample.header).replace('"ts":30', '"ts":30.0');
    const header = new TextEncoder().encode(json);
    const wire = new Uint8Array(8 + header.length + sample.payload.length);
    new DataView(wire.buffer).setUint32(0, header.length, true);
    new DataView(wire.buffer).setUint32(4, sample.payload.length, true);
    wire.set(header, 8);
    wire.set(sample.payload, 8 + header.length);

    h.client.ingest(decodeDataFrame(wire));

    expect(received).toHaveBeenCalledOnce();
    await until(() => h.sent.some((m) => m.t === "topic_credit"));
    expect(h.sent.at(-1)).toMatchObject({
      t: "topic_credit",
      frames: 1,
      bytes: wire.byteLength,
    });
    expect(h.disconnect).not.toHaveBeenCalled();
  });

  it("shares identical wire subscriptions while updates reopen only the requesting consumer", async () => {
    const h = setup();
    h.sources();
    const first = h.client.subscribe("demo/**", { delivery: "latest", maxHz: 10 }, vi.fn());
    const second = h.client.subscribe("demo/**", { delivery: "latest", maxHz: 10 }, vi.fn());
    expect(h.subs()).toHaveLength(1);
    h.ready();
    await Promise.all([first.ready(), second.ready()]);
    const update = second.update({ maxHz: 30 });
    expect(h.subs().map((s) => s.maxHz)).toEqual([10, 30]);
    expect(first.status().phase).toBe("ready");
    expect(h.sent.filter((m) => m.t === "topic_unsub")).toHaveLength(0);
    h.ready("@t:v2");
    await update;
    first.close();
    second.close();
    expect(h.sent.filter((m) => m.t === "topic_unsub")).toHaveLength(2);
  });

  it("preserves quiet latest keys and drops stale samples independently", () => {
    const h = setup();
    h.sources();
    const seen: string[] = [];
    h.client.subscribe("demo/**", {}, (s) => seen.push(`${s.key}:${s.sequence}`));
    h.ready();
    h.client.ingest(h.sample(100, "demo/busy", "@t:v1", 1, "latest"));
    h.client.ingest(h.sample(1, "demo/quiet", "@t:v1", 1, "latest"));
    h.client.ingest(h.sample(99, "demo/busy", "@t:v1", 1, "latest"));
    expect(seen).toEqual(["demo/busy:100", "demo/quiet:1"]);
  });

  it("keeps desired subscriptions across source restart and notifies reconnect only after readiness", async () => {
    const h = setup();
    h.sources();
    const seen = vi.fn();
    const reconnect = vi.fn();
    h.client.onReconnect(reconnect);
    const sub = h.client.subscribe("demo/**", { delivery: "reliable" }, seen);
    h.ready();
    await sub.ready();
    h.client.onMessage({
      t: "topic_error",
      id: h.subs()[0].id,
      code: "source_offline",
      message: "offline",
      outcome: "interrupted",
    });
    const resumed = vi.fn();
    const waiting = sub.ready().then(resumed);
    h.client.ingest(h.sample(1));
    h.sources(2);
    await Promise.resolve();
    expect(reconnect).not.toHaveBeenCalled();
    expect(resumed).not.toHaveBeenCalled();
    expect(h.subs()).toHaveLength(1); // relay already owns the desired subscription
    h.ready("@t:new", 2);
    await waiting;
    h.client.ingest(h.sample(1));
    h.client.ingest(h.sample(1, "demo/new", "@t:new", 2));
    expect(seen).toHaveBeenCalledTimes(1);
    expect(reconnect).toHaveBeenCalledTimes(1);
  });

  it("reissues desired subscriptions after connection loss and rejects pending lists", async () => {
    const h = setup();
    h.sources();
    h.client.subscribe("demo/**", { delivery: "reliable" }, vi.fn());
    h.ready();
    const page = h.client.list();
    h.client.disconnected();
    await expect(page).rejects.toMatchObject({ code: "connection_lost" });
    h.client.connected((m) => h.sent.push(m), h.disconnect);
    h.sources(3);
    expect(h.subs()).toHaveLength(2);
    h.ready("@t:new", 3);
  });

  it("lets the relay answer for an unannounced source and waits while no source is connected", async () => {
    const h = setup();
    h.client.onMessage({ t: "topic_sources", v: 1, sources: [] });
    const named = h.client.subscribe("**", {}, vi.fn());
    const invalid = h.client.subscribe("demo/a", { maxHz: -1 }, vi.fn());
    await expect(invalid.ready()).rejects.toMatchObject({ code: "invalid_topic_options" });
    expect(h.subs().map((sub) => sub.source)).toEqual(["workspace"]);
    expect(named.status().phase).toBe("connecting");

    const anonymous = new TopicClient();
    owned.push(anonymous);
    const sent: Msg[] = [];
    anonymous.connected((m) => sent.push(m), vi.fn());
    anonymous.onMessage({ t: "topic_sources", v: 1, sources: [] });
    const waiting = anonymous.subscribe("**", {}, vi.fn());
    expect(waiting.status()).toMatchObject({
      phase: "interrupted",
      error: { code: "source_offline" },
    });
    expect(sent).toEqual([]);
    anonymous.onMessage({
      t: "topic_sources",
      v: 1,
      sources: [{ source: "workspace", gen: 1, read: true, write: false, profiles: ["raw"] }],
    });
    expect(sent.map((m) => m.t)).toEqual(["topic_sub"]);
    const request = sent[0] as TopicSubMsg;
    anonymous.onMessage({
      t: "topic_ready",
      id: request.id,
      source: "workspace",
      route: "@t:v9",
      gen: 1,
    });
    await expect(waiting.ready()).resolves.toBeUndefined();
  });

  it("rejects reads a provider does not offer without a wire request", async () => {
    const h = setup();
    h.client.onMessage({
      t: "topic_sources",
      v: 1,
      sources: [{ source: "workspace", gen: 1, read: false, write: true, profiles: ["raw"] }],
    });
    const denied = h.client.subscribe("**", {}, vi.fn());
    await expect(denied.ready()).rejects.toMatchObject({ code: "topic_read_denied" });
    await expect(h.client.list()).rejects.toMatchObject({ code: "topic_read_denied" });
    expect(h.subs()).toEqual([]);
  });

  it("reliable gaps interrupt instead of hiding lost events behind the latest value", () => {
    const h = setup();
    h.sources();
    const received = vi.fn();
    const sub = h.client.subscribe("demo/**", { delivery: "reliable" }, received);
    h.ready();
    h.client.ingest(h.sample(1));
    h.client.ingest(h.sample(3));
    expect(received).toHaveBeenCalledTimes(1);
    expect(sub.status()).toMatchObject({
      phase: "interrupted",
      error: { code: "topic_order_lost" },
    });
    expect(h.disconnect).not.toHaveBeenCalled();
  });

  it.each([
    "topic_overflow",
    "topic_order_lost",
    "topic_key_limit",
    "topic_stream_closed",
    "topic_send_failed",
  ])(
    "recovers %s on just the failed route, preserving its error and unrelated traffic",
    async (code) => {
      vi.useFakeTimers();
      const h = setup();
      h.sources();
      const failed = h.client.subscribe("broken/**", { delivery: "reliable" }, vi.fn());
      const old = h.subs()[0];
      h.ready("@t:broken");
      const received = vi.fn();
      const healthy = h.client.subscribe("healthy/**", { delivery: "reliable" }, received);
      h.ready("@t:healthy");
      const reconnected = vi.fn();
      h.client.onReconnect(reconnected);

      // Stream reset can arrive before the control error carrying the real cause.
      h.client.streamClosed("@t:broken");
      h.client.onMessage({
        t: "topic_error",
        id: old.id,
        code,
        outcome: "interrupted",
        message: code,
      });
      expect(failed.status()).toMatchObject({ phase: "interrupted", error: { code } });
      h.client.ingest(h.sample(1, "healthy/key", "@t:healthy"));
      expect(received).toHaveBeenCalledOnce();
      expect(healthy.status().phase).toBe("ready");
      expect(h.disconnect).not.toHaveBeenCalled();

      await vi.advanceTimersByTimeAsync(1000);
      expect(h.subs()).toHaveLength(3);
      expect(h.subs().at(-1)?.keyExpr).toBe("broken/**");
      expect(h.subs().at(-1)?.id).not.toBe(old.id);
      h.ready("@t:recovered");
      await failed.ready();
      expect(reconnected).toHaveBeenCalledOnce();
      h.client.onMessage({
        t: "topic_error",
        id: old.id,
        code,
        outcome: "interrupted",
        message: code,
      });
      expect(failed.status().phase).toBe("ready");
    },
  );

  it.each(["subscribe", "update", "reconnect", "backoff"])(
    "retries a rejected shared group on %s",
    async (action) => {
      vi.useFakeTimers();
      const h = setup();
      h.sources();
      const first = h.client.subscribe("events/**", { delivery: "reliable" }, vi.fn());
      const second = h.client.subscribe("events/**", { delivery: "reliable" }, vi.fn());
      h.client.onMessage({
        t: "topic_error",
        id: h.subs()[0].id,
        code: "topic_ready_timeout",
        message: "slow provider",
        outcome: "rejected",
      });
      await expect(first.ready()).rejects.toMatchObject({ code: "topic_ready_timeout" });
      let third;
      let updated;
      if (action === "subscribe") {
        third = h.client.subscribe("events/**", { delivery: "reliable" }, vi.fn());
      }
      if (action === "update") updated = second.update({});
      if (action === "reconnect") {
        h.client.disconnected();
        h.client.connected((msg) => h.sent.push(msg), h.disconnect);
        h.sources();
      }
      if (action === "backoff") await vi.advanceTimersByTimeAsync(1000);
      expect(h.subs()).toHaveLength(2);
      h.ready();
      await Promise.all([first.ready(), second.ready(), third?.ready(), updated]);
      expect(first.status().phase).toBe("ready");
      expect(second.status().phase).toBe("ready");
    },
  );

  it("resyncs recovered subscriptions even while another desired source is offline", () => {
    const h = setup();
    const sources = () => h.sources();
    sources();
    h.client.subscribe("events/**", { delivery: "reliable" }, vi.fn());
    h.ready();
    const resync = vi.fn();
    h.client.onReconnect(resync);
    const remote = h.client.subscribe("remote/**", { source: "remote" }, vi.fn());
    const offline = () =>
      h.client.onMessage({
        t: "topic_error",
        id: h.subs().filter((sub) => sub.source === "remote").at(-1)!.id,
        code: "source_offline",
        outcome: "interrupted",
        message: "offline",
      });
    offline();
    resync.mockClear();
    h.client.disconnected();
    h.client.connected((msg) => h.sent.push(msg), h.disconnect);
    sources();
    offline();
    h.ready("@t:restored", 1, h.subs().filter((sub) => sub.source === "workspace").at(-1)!);
    expect(resync).toHaveBeenCalledOnce();
    expect(remote.status().phase).toBe("interrupted");
  });

  it("rejects unknown subscription options without sending a request", async () => {
    const h = setup();
    h.sources();
    const sub = h.client.subscribe("demo/a", { maxHz: 30, rate: 2 } as never, vi.fn());
    await expect(sub.ready()).rejects.toMatchObject({ code: "invalid_topic_options" });
    expect(h.subs()).toEqual([]);
  });

  it("publishes immutable binary bytes only with write capability and a provider receipt", async () => {
    const h = setup();
    h.sources();
    await expect(h.client.publish("runtime/new", new Uint8Array([1]))).rejects.toMatchObject({
      code: "topic_write_denied",
      outcome: "rejected",
    });
    h.sources(1, true);
    const payload = new Uint8Array([0, 255, 128]);
    const accepted = vi.fn();
    const receipt = h.client.publish("runtime/new", payload).then(accepted);
    payload.fill(7);
    const frame = decodeDataFrame(h.binary[0]);
    expect([...frame.payload]).toEqual([0, 255, 128]);
    await Promise.resolve();
    expect(accepted).not.toHaveBeenCalled();
    h.client.onMessage({
      t: "topic_pub_result",
      id: frame.header.meta!.id as string,
      outcome: "accepted",
      code: "",
      message: "",
      providerTs: 20,
    });
    await receipt;
    expect(accepted).toHaveBeenCalledWith({ id: frame.header.meta!.id, providerTs: 20 });
  });

  it("bounds pending writes, reports connection loss as unknown, and never replays them", async () => {
    const h = setup();
    h.sources(1, true);
    const writes = Array.from(
      { length: 16 },
      () => h.client.publish("runtime/new", new Uint8Array([1])).catch((error) => error),
    );
    await expect(h.client.publish("runtime/new", new Uint8Array([1]))).rejects.toMatchObject({
      code: "topic_publish_limit",
      outcome: "rejected",
    });
    h.client.disconnected();
    expect((await Promise.all(writes)).map((error) => error.outcome)).toEqual(
      Array(16).fill("unknown"),
    );
    h.client.connected((m) => h.sent.push(m), h.disconnect, (bytes) => {
      h.binary.push(bytes);
      return Promise.resolve();
    });
    h.sources(2, true);
    expect(h.binary).toHaveLength(16);
    await expect(h.client.publish("runtime/**", new Uint8Array([1]))).rejects.toMatchObject({
      outcome: "rejected",
    });
  });

  it("list receipts are source/generation bound and return bounded descriptor pages", async () => {
    const h = setup();
    h.sources();
    const result = h.client.list({ prefix: "demo/", limit: 1 });
    const request = h.sent.at(-1)!;
    expect(request.t).toBe("topic_list");
    if (request.t !== "topic_list") throw new Error("missing list request");
    h.client.onMessage({
      t: "topic_page",
      id: request.id,
      source: "workspace",
      gen: 1,
      entries: [{
        key: "demo/a",
        encoding: "application/json",
        type: "",
        availability: "observed",
      }],
      cursor: "next",
    });
    await expect(result).resolves.toMatchObject({
      source: "workspace",
      generation: 1,
      cursor: "next",
    });
  });

  it("settles list and publication deadlines without replaying an unknown put", async () => {
    vi.useFakeTimers();
    const h = setup();
    h.sources(1, true);
    const listing = h.client.list().catch((error) => error);
    const publication = h.client.publish("put/key", new Uint8Array([42])).catch((error) => error);
    await vi.advanceTimersByTimeAsync(15_000);
    expect(await listing).toMatchObject({ code: "topic_list_timeout", outcome: "unknown" });
    expect(await publication).toMatchObject({ code: "connection_lost", outcome: "unknown" });
    expect(h.disconnect).toHaveBeenCalledOnce();
    expect(h.binary).toHaveLength(1);
  });

  it("sends binary puts and guarded commands on independent session streams with checked receipts", async () => {
    const relay = new FakeRelayEnd();
    const session = connect({ topicSource: "workspace" }, {
      fetchInfo: () => Promise.resolve(INFO),
      createWebTransport: () => relay.wt,
    });
    owned.push(session);
    await until(() => relay.sent.some((message) => message.t === "hello"));
    relay.push({ t: "welcome", v: PROTOCOL_VERSION });
    relay.push({
      t: "topic_sources",
      v: 1,
      sources: [{
        source: "workspace",
        gen: 1,
        read: true,
        write: true,
        profiles: ["raw"],
        guardedPublishers: true,
      }],
    });
    await until(() => session.topics.sources().length === 1);
    const put = session.topics.publish("put/key", new Uint8Array([0, 255, 128]));
    const publisher = session.topics.publisher("robot/cmd", new Uint8Array([0]), {
      group: "drive",
      timeoutMs: 400,
    });
    await until(() => relay.publications.length === 1 && relay.guards.length === 1);
    expect(relay.publications[0].payload).toEqual(new Uint8Array([0, 255, 128]));
    expect(relay.guards[0].payload).toEqual(new Uint8Array([0]));
    relay.push({
      t: "topic_pub_result",
      id: relay.publications[0].header.meta!.id as string,
      outcome: "accepted",
      code: "",
      message: "",
      providerTs: 123,
    });
    relay.push({
      t: "topic_guard_ready",
      id: relay.guards[0].header.meta!.id as string,
      handle: "@p:session",
      gen: 1,
      ticket: 1,
      timeoutMs: 400,
    });
    await expect(put).resolves.toMatchObject({ providerTs: 123 });
    await publisher.ready();
    const command = publisher.publish(new Uint8Array([1]));
    await until(() => relay.guards.length === 2);
    expect(relay.guards[1].header).toMatchObject({ ch: "@guard", seq: 1, meta: { ticket: 1 } });
    expect(relay.publications).toHaveLength(1);
    relay.push({
      t: "topic_guard_result",
      handle: "@p:session",
      gen: 1,
      seq: 1,
      outcome: "accepted",
      code: "",
      providerTs: 124,
    });
    await expect(command).resolves.toMatchObject({ providerTs: 124 });
    publisher.close();
    await until(() => relay.sent.some((message) => message.t === "topic_guard_stop"));
  });

  it("a refused guard stream retires its publisher and the next one opens a fresh stream", async () => {
    const relay = new FakeRelayEnd();
    const session = connect({ topicSource: "workspace" }, {
      fetchInfo: () => Promise.resolve(INFO),
      createWebTransport: () => relay.wt,
    });
    owned.push(session);
    await until(() => relay.sent.some((message) => message.t === "hello"));
    relay.push({ t: "welcome", v: PROTOCOL_VERSION });
    relay.push({
      t: "topic_sources",
      v: 1,
      sources: [{
        source: "workspace",
        gen: 1,
        read: true,
        write: true,
        profiles: ["raw"],
        guardedPublishers: true,
      }],
    });
    await until(() => session.topics.sources().length === 1);
    relay.failBinaryStreams = 1;
    const options = { group: "drive", timeoutMs: 400 };
    const refused = session.topics.publisher("robot/cmd", new Uint8Array([0]), options);
    await expect(refused.ready()).rejects.toMatchObject({ code: "guard_transport_failed" });
    const publisher = session.topics.publisher("robot/cmd", new Uint8Array([7]), options);
    await until(() => relay.guards.length === 1);
    expect(relay.guards[0].payload).toEqual(new Uint8Array([7]));
    publisher.close();
  });

  it("notifies HTTP resync through the session only after topic readiness on a replacement transport", async () => {
    const first = new FakeRelayEnd(), second = new FakeRelayEnd();
    let connections = 0;
    const session = connect({ topicSource: "workspace" }, {
      fetchInfo: () => Promise.resolve(INFO),
      createWebTransport: () => (++connections === 1 ? first.wt : second.wt),
    });
    owned.push(session);
    const sub = session.topics.subscribe("events/**", { delivery: "reliable" }, vi.fn());
    const resync = vi.fn();
    session.topics.onReconnect(resync);
    for (const relay of [first, second]) {
      await until(() => relay.sent.some((message) => message.t === "hello"));
      relay.push({ t: "welcome", v: PROTOCOL_VERSION });
      relay.push({
        t: "topic_sources",
        v: 1,
        sources: [{ source: "workspace", gen: 1, read: true, write: false, profiles: ["raw"] }],
      });
      await until(() => relay.sent.some((message) => message.t === "topic_sub"));
      expect(resync).not.toHaveBeenCalled();
      const request = relay.sent.find((message) => message.t === "topic_sub")!;
      relay.push({
        t: "topic_ready",
        id: request.id,
        source: "workspace",
        route: "@t:events",
        gen: 1,
      });
      await sub.ready();
      if (relay === first) {
        relay.endControl();
        await until(() => sub.status().phase === "interrupted");
      }
    }
    expect(resync).toHaveBeenCalledOnce();
  });

  it("session ingests raw topics without a watched robot, manifest, or UI tick", async () => {
    const relay = new FakeRelayEnd();
    const session: Session = connect({ topicSource: "workspace", uiTickMs: 60_000 }, {
      fetchInfo: () => Promise.resolve(INFO),
      createWebTransport: () => relay.wt,
    });
    owned.push(session);
    const values: number[] = [];
    const sub = session.topics.subscribe(
      "events/**",
      { delivery: "reliable" },
      (s) => values.push(s.payload[0]),
    );
    await until(() => relay.sent.some((m) => m.t === "hello"));
    relay.push({ t: "welcome", v: PROTOCOL_VERSION });
    relay.push({
      t: "topic_sources",
      v: 1,
      sources: [{ source: "workspace", gen: 1, read: true, write: false, profiles: ["raw"] }],
    });
    await until(() => relay.sent.some((m) => m.t === "topic_sub"));
    const request = relay.sent.find((m): m is TopicSubMsg => m.t === "topic_sub")!;
    relay.push({
      t: "topic_ready",
      id: request.id,
      source: "workspace",
      route: "@t:session",
      gen: 1,
    });
    await sub.ready();
    relay.pushRaw(1, new Uint8Array([42]), "@t:session", {
      key: "events/a",
      encoding: "zenoh/bytes",
      gen: 1,
      ingressTs: 1,
    });
    await until(() => values.length === 1);
    expect(values).toEqual([42]);
    expect(session.status.get().manifest).toBeNull();
    relay.endTopic("@t:session");
    await until(() => sub.status().phase === "interrupted");
    expect(sub.status().error?.code).toBe("topic_stream_closed");
  });
});

describe("topic settings updates", () => {
  const owned: { close(): void }[] = [];
  afterEach(() => {
    for (const item of owned.splice(0)) item.close();
  });
  const OPTIONS = {
    raw: { maxHz: { kind: "number", max: 1000, default: 1000 } },
    "image.jpeg.low.v1": {
      maxHz: { kind: "number", max: 1000, default: 30 },
      maxResolution: { kind: "size", min: 16, max: 4096, default: [640, 360] },
      quality: { kind: "int", min: 1, max: 100, default: 60 },
    },
  } as const;
  const LOW: TopicSettings = { maxHz: 30, maxResolution: [640, 360], quality: 60 };
  function setup() {
    const client = new TopicClient("workspace");
    owned.push(client);
    const sent: Msg[] = [];
    client.connected((m) => sent.push(m), vi.fn());
    const sources = () =>
      client.onMessage({
        t: "topic_sources",
        v: 1,
        sources: [{
          source: "workspace",
          gen: 1,
          read: true,
          write: false,
          profiles: ["raw", "image.jpeg.low.v1"],
          profileOptions: OPTIONS,
        }],
      });
    sources();
    const subs = () => sent.filter((m): m is TopicSubMsg => m.t === "topic_sub");
    const updates = () => sent.filter((m): m is TopicUpdateMsg => m.t === "topic_update");
    const ready = (route: string, sub = subs().at(-1)!, settings: TopicSettings = LOW) =>
      client.onMessage({
        t: "topic_ready",
        id: sub.id,
        source: "workspace",
        route,
        gen: 1,
        settings,
      });
    const applied = (revision: number, settings: TopicSettings, update = updates().at(-1)!) =>
      client.onMessage({ t: "topic_applied", id: update.id, revision, settings });
    const rejected = (revision: number, code: string, update = updates().at(-1)!) =>
      client.onMessage({
        t: "topic_error",
        id: update.id,
        revision,
        code,
        message: code,
        outcome: "rejected",
      });
    const camera = (options: TopicOptions = {}, callback = vi.fn()) =>
      client.subscribe("sensor/cam", { profile: "image.jpeg.low.v1", ...options }, callback);
    const frame = (seq: number, route: string) =>
      client.ingest({
        header: {
          ch: route,
          seq,
          ts: 1,
          delivery: "latest",
          meta: { key: "sensor/cam", gen: 1, encoding: "image/jpeg", ingressTs: 0 },
        },
        payload: new Uint8Array([seq]),
      });
    return { client, sent, sources, subs, updates, ready, applied, rejected, camera, frame };
  }

  it("reports the normalized request before acknowledgement and the relay's settings after", async () => {
    const h = setup();
    const sub = h.camera({ quality: 90, maxHz: 60 });
    expect(h.subs().at(-1)).toMatchObject({
      profile: "image.jpeg.low.v1",
      maxHz: 60,
      settings: { quality: 90 },
    });
    expect(sub.settings()).toEqual(LOW);
    h.ready("@t:v1", undefined, { maxHz: 30, maxResolution: [640, 360], quality: 55 });
    await sub.ready();
    expect(sub.settings()).toEqual({ maxHz: 30, maxResolution: [640, 360], quality: 55 });
    // A request the advertised ranges rule out never reaches the wire.
    const refused = h.camera({ keepEvery: 2 });
    await expect(refused.ready()).rejects.toMatchObject({ code: "unsupported_option" });
    expect(h.subs()).toHaveLength(1);
  });

  it("updates a handle alone in its group in place: no reopen, no status change, no missed frame", async () => {
    const h = setup();
    const seen = vi.fn();
    const sub = h.camera({}, seen);
    h.ready("@t:v1");
    await sub.ready();
    const phases: string[] = [];
    sub.onStatus((status) => phases.push(status.phase));
    const settings = vi.fn();
    const unsubscribeSettings = sub.onSettings(settings);
    const update = sub.update({ quality: 40, maxResolution: [320, 240] });
    expect(h.updates()).toEqual([{
      t: "topic_update",
      id: h.subs()[0].id,
      revision: 1,
      options: { maxResolution: [320, 240], quality: 40 },
    }]);
    expect(h.subs()).toHaveLength(1);
    h.frame(1, "@t:v1");
    expect(seen).toHaveBeenCalledTimes(1);
    expect(sub.settings()).toEqual(LOW);
    expect(settings).not.toHaveBeenCalled();
    h.applied(1, { maxHz: 30, maxResolution: [320, 240], quality: 40 });
    await expect(update).resolves.toEqual({ maxHz: 30, maxResolution: [320, 240], quality: 40 });
    expect(sub.settings()).toEqual({ maxHz: 30, maxResolution: [320, 240], quality: 40 });
    expect(phases).toEqual(["ready"]);
    expect(settings).toHaveBeenCalledExactlyOnceWith(sub.settings());
    unsubscribeSettings();
    // null resets one key to the profile value; the relay says which.
    const reset = sub.update({ maxResolution: null });
    expect(h.updates().at(-1)).toMatchObject({ revision: 2, options: { maxResolution: null } });
    h.applied(2, { maxHz: 30, maxResolution: [640, 360], quality: 40 });
    await expect(reset).resolves.toEqual({ maxHz: 30, maxResolution: [640, 360], quality: 40 });
    expect(settings).toHaveBeenCalledOnce();
    // The group now answers to its new settings: a twin joins it, not the wire.
    const twin = h.camera({ quality: 40 });
    expect(h.subs()).toHaveLength(1);
    expect(twin.status().phase).toBe("ready");
    expect(twin.settings()).toEqual({ maxHz: 30, maxResolution: [640, 360], quality: 40 });
  });

  it("a rejected update keeps the settings and status; local checks never reach the wire", async () => {
    const h = setup();
    const sub = h.camera();
    h.ready("@t:v1");
    await sub.ready();
    await expect(sub.update({ maxBitrate: 1 } as TopicOptions)).rejects.toMatchObject({
      code: "unsupported_option",
    });
    await expect(sub.update({ keepEvery: 2 })).rejects.toMatchObject({
      code: "unsupported_option",
    });
    await expect(sub.update({ quality: 0 })).rejects.toMatchObject({
      code: "option_out_of_range",
    });
    await expect(sub.update({ quality: 1.5 })).rejects.toMatchObject({
      code: "invalid_topic_options",
    });
    expect(h.updates()).toEqual([]);
    const update = sub.update({ quality: 40 });
    h.rejected(1, "variant_limit");
    await expect(update).rejects.toMatchObject({ code: "variant_limit", outcome: "rejected" });
    expect(sub.settings()).toEqual(LOW);
    expect(sub.status().phase).toBe("ready");
    // Quality 40 was never granted: a twin asking for it opens its own
    // subscription, while the acknowledged settings stay shared.
    const refused = h.camera({ quality: 40 });
    expect(h.subs()).toHaveLength(2);
    expect(refused.status().phase).toBe("connecting");
    const twin = h.camera();
    expect(h.subs()).toHaveLength(2);
    expect(twin.status().phase).toBe("ready");
    expect(twin.settings()).toEqual(LOW);
  });

  it("sends changes one at a time, each merged into the acknowledged request", async () => {
    const h = setup();
    const sub = h.camera();
    h.ready("@t:v1");
    await sub.ready();
    const first = sub.update({ quality: 40 });
    const second = sub.update({ maxResolution: [320, 240] });
    expect(h.updates().map((m) => m.revision)).toEqual([1]);
    // Until the relay answers, a twin cannot tell which settings it would
    // get: it opens its own subscription.
    const twin = h.camera();
    expect(h.subs()).toHaveLength(2);
    expect(twin.status().phase).toBe("connecting");
    h.applied(2, { ...LOW, quality: 40 }); // not the revision in flight
    expect(h.updates()).toHaveLength(1);
    h.applied(1, { ...LOW, quality: 40 });
    await expect(first).resolves.toEqual({ ...LOW, quality: 40 });
    expect(h.updates().map((m) => [m.revision, m.options])).toEqual([
      [1, { quality: 40 }],
      [2, { maxResolution: [320, 240] }],
    ]);
    h.applied(2, { maxHz: 30, maxResolution: [320, 240], quality: 40 });
    await expect(second).resolves.toEqual({ maxHz: 30, maxResolution: [320, 240], quality: 40 });
    // The request carries both changes: a twin with them joins the group.
    const both = h.camera({ quality: 40, maxResolution: [320, 240] });
    expect(h.subs()).toHaveLength(2);
    expect(both.status().phase).toBe("ready");
    both.close();
    // Connection loss settles pending changes as interrupted, never silently.
    const lost = sub.update({ quality: 10 });
    const queued = sub.update({ quality: 20 });
    h.client.disconnected();
    await expect(lost).rejects.toMatchObject({ code: "connection_lost", outcome: "interrupted" });
    await expect(queued).rejects.toMatchObject({ code: "connection_lost", outcome: "interrupted" });
  });

  it("resets maxHz in place to the profile default and preserves it on reconnect", async () => {
    const h = setup();
    const sub = h.camera({ maxHz: 5 });
    h.ready("@t:v1", undefined, { ...LOW, maxHz: 5 });
    const reset = sub.update({ maxHz: null });
    expect(h.updates()[0].options).toEqual({ maxHz: null });
    h.applied(1, LOW);
    await expect(reset).resolves.toEqual(LOW);
    h.client.disconnected();
    h.client.connected((m) => h.sent.push(m), vi.fn());
    h.sources();
    expect(h.subs().at(-1)?.maxHz).toBe(30);
  });

  it("resets maxHz on a shared handle to the profile default", async () => {
    const h = setup();
    const a = h.camera({ maxHz: 5 });
    const b = h.camera({ maxHz: 5 });
    h.ready("@t:v1", undefined, { ...LOW, maxHz: 5 });
    const reset = b.update({ maxHz: null });
    expect(h.subs()[1].maxHz).toBe(30);
    h.ready("@t:v2");
    await expect(reset).resolves.toEqual(LOW);
    expect(a.settings()).toEqual({ ...LOW, maxHz: 5 });
  });

  it("keeps an omitted maxHz and resets raw maxHz to the protocol default", async () => {
    const h = setup();
    const sub = h.camera({ profile: "raw", maxHz: 5 });
    h.ready("@t:v1", undefined, { maxHz: 5 });
    const unchanged = sub.update({ maxHz: undefined });
    expect(h.updates()[0].options).toEqual({});
    h.applied(1, { maxHz: 5 });
    await expect(unchanged).resolves.toEqual({ maxHz: 5 });
    h.camera({ profile: "raw", maxHz: 5 });
    expect(h.subs()).toHaveLength(1);
    const reset = sub.update({ maxHz: null });
    expect(h.subs()[1].maxHz).toBe(1000);
    h.ready("@t:v2", undefined, { maxHz: 1000 });
    await expect(reset).resolves.toEqual({ maxHz: 1000 });
  });

  it("queues successive changes while a shared handle waits for its new group", async () => {
    const h = setup();
    const a = h.camera();
    const b = h.camera();
    h.ready("@t:v1");
    const first = b.update({ quality: 40 });
    const firstResult = expect(first).resolves.toEqual({ ...LOW, quality: 40 });
    const second = b.update({ maxResolution: [320, 240] });
    const secondResult = expect(second).resolves.toEqual({
      ...LOW,
      quality: 40,
      maxResolution: [320, 240],
    });
    expect(h.subs()).toHaveLength(2);
    expect(h.updates()).toEqual([]);
    h.ready("@t:v2", h.subs()[1], { ...LOW, quality: 40 });
    await firstResult;
    expect(h.updates()).toEqual([{
      t: "topic_update",
      id: h.subs()[1].id,
      revision: 1,
      options: { maxResolution: [320, 240] },
    }]);
    h.applied(1, { ...LOW, quality: 40, maxResolution: [320, 240] });
    await secondResult;
    expect(a.settings()).toEqual(LOW);
    h.camera({ quality: 40, maxResolution: [320, 240] });
    expect(h.subs()).toHaveLength(2);
  });

  it("rejects a waiting move and queued changes on disconnect and recovers the original group", async () => {
    const h = setup();
    const a = h.camera();
    const b = h.camera();
    h.ready("@t:v1");
    const first = b.update({ quality: 40 });
    const firstResult = expect(first).rejects.toMatchObject({
      code: "connection_lost",
      outcome: "interrupted",
    });
    const second = b.update({ maxResolution: [320, 240] });
    const secondResult = expect(second).rejects.toMatchObject({
      code: "connection_lost",
      outcome: "interrupted",
    });
    h.client.disconnected();
    await Promise.all([firstResult, secondResult]);
    expect(b.settings()).toEqual(LOW);
    expect(b.status().phase).toBe("interrupted");
    h.client.connected((m) => h.sent.push(m), vi.fn());
    h.sources();
    expect(h.subs()).toHaveLength(3);
    expect(h.subs()[2]).toMatchObject({ id: h.subs()[0].id });
    h.ready("@t:recovered");
    await Promise.all([a.ready(), b.ready()]);
    const retried = b.update({ quality: 40 });
    expect(h.subs()).toHaveLength(4);
    h.ready("@t:retry", undefined, { ...LOW, quality: 40 });
    await expect(retried).resolves.toEqual({ ...LOW, quality: 40 });
  });

  it("keeps queued changes when an acknowledged update moves immediately to a ready group", async () => {
    const h = setup();
    const a = h.camera();
    const b = h.camera({ maxHz: 5, quality: 40 });
    h.ready("@t:v1", h.subs()[0]);
    h.ready("@t:v2", h.subs()[1], { ...LOW, maxHz: 5, quality: 40 });
    const first = a.update({ maxHz: 5 });
    const second = a.update({ quality: 40 });
    const third = a.update({ maxResolution: [320, 240] });
    h.applied(1, { ...LOW, maxHz: 5 });
    await expect(first).resolves.toEqual({ ...LOW, maxHz: 5 });
    await expect(second).resolves.toEqual({ ...LOW, maxHz: 5, quality: 40 });
    expect(h.subs()).toHaveLength(3);
    expect(h.subs()[2]).toMatchObject({
      maxHz: 5,
      settings: { quality: 40, maxResolution: [320, 240] },
    });
    const settings = { maxHz: 5, quality: 40, maxResolution: [320, 240] as [number, number] };
    h.ready("@t:v3", h.subs()[2], settings);
    await expect(third).resolves.toEqual(settings);
    expect(b.settings()).toEqual({ ...LOW, maxHz: 5, quality: 40 });
  });

  it("merges a queued change into the original settings after a move is rejected", async () => {
    const h = setup();
    h.camera();
    const sub = h.camera();
    h.ready("@t:v1");
    const first = sub.update({ quality: 40 });
    const second = sub.update({ maxResolution: [320, 240] });
    h.client.onMessage({
      t: "topic_error",
      id: h.subs()[1].id,
      code: "variant_limit",
      message: "variant limit",
      outcome: "rejected",
    });
    await expect(first).rejects.toMatchObject({ code: "variant_limit" });
    expect(h.subs()).toHaveLength(3);
    expect(h.subs()[2].settings).toEqual({ maxResolution: [320, 240] });
    const settings: TopicSettings = { ...LOW, maxResolution: [320, 240] };
    h.ready("@t:v3", undefined, settings);
    await expect(second).resolves.toEqual(settings);
  });

  it("rejects a move and its queued changes when the handle closes", async () => {
    const h = setup();
    const a = h.camera();
    const b = h.camera();
    h.ready("@t:v1");
    const first = b.update({ quality: 40 });
    const second = b.update({ maxResolution: [320, 240] });
    b.close();
    await expect(first).rejects.toMatchObject({ code: "closed" });
    await expect(second).rejects.toMatchObject({ code: "closed" });
    expect(a.status().phase).toBe("ready");
    expect(a.settings()).toEqual(LOW);
    expect(h.sent.filter((m) => m.t === "topic_unsub")).toEqual([
      { t: "topic_unsub", id: h.subs()[1].id },
    ]);
  });

  it("an update onto settings another group serves joins that group instead of displacing it", async () => {
    const h = setup();
    const a = h.camera();
    const b = h.camera({ quality: 40 });
    expect(h.subs()).toHaveLength(2);
    h.ready("@t:v1", h.subs()[0]);
    h.ready("@t:v2", h.subs()[1], { ...LOW, quality: 40 });
    await Promise.all([a.ready(), b.ready()]);
    const update = a.update({ quality: 40 });
    expect(h.updates()).toEqual([]);
    await expect(update).resolves.toEqual({ ...LOW, quality: 40 });
    expect(h.sent.filter((m) => m.t === "topic_unsub")).toEqual([
      { t: "topic_unsub", id: h.subs()[0].id },
    ]);
    // b's group is still the client's to manage.
    h.client.disconnected();
    expect(a.status().phase).toBe("interrupted");
    expect(b.status().phase).toBe("interrupted");
    h.client.close();
    expect(b.status().phase).toBe("closed");
  });

  it("moves a handle sharing its group to a new one only once that is ready, without a gap", async () => {
    const h = setup();
    const seenA = vi.fn();
    const seenB = vi.fn();
    const a = h.camera({}, seenA);
    const b = h.camera({}, seenB);
    expect(h.subs()).toHaveLength(1);
    h.ready("@t:v1");
    await Promise.all([a.ready(), b.ready()]);
    const phases: string[] = [];
    b.onStatus((status) => phases.push(status.phase));
    const update = b.update({ quality: 40 });
    expect(h.updates()).toEqual([]);
    expect(h.subs()).toHaveLength(2);
    expect(h.subs()[1]).toMatchObject({ settings: { quality: 40 } });
    h.frame(1, "@t:v1");
    expect(seenB).toHaveBeenCalledTimes(1);
    expect(b.settings()).toEqual(LOW);
    h.ready("@t:v2", h.subs()[1], { ...LOW, quality: 40 });
    await expect(update).resolves.toEqual({ ...LOW, quality: 40 });
    expect(phases).toEqual(["ready"]);
    h.frame(2, "@t:v1");
    h.frame(3, "@t:v2");
    expect(seenA.mock.calls.map((call) => call[0].sequence)).toEqual([1, 2]);
    expect(seenB.mock.calls.map((call) => call[0].sequence)).toEqual([1, 3]);
    expect(h.sent.filter((m) => m.t === "topic_unsub")).toHaveLength(0);
    a.close();
    expect(h.sent.filter((m) => m.t === "topic_unsub")).toHaveLength(1);
  });

  it("a handle changing settings before its first subscription is ready becomes ready with the new one", async () => {
    const h = setup();
    const phases: string[] = [];
    const sub = h.camera();
    sub.onStatus((status) => phases.push(status.phase));
    const update = sub.update({ quality: 40 });
    expect(h.subs()).toHaveLength(2);
    h.ready("@t:v2", h.subs()[1], { ...LOW, quality: 40 });
    await expect(update).resolves.toEqual({ ...LOW, quality: 40 });
    expect(sub.status().phase).toBe("ready");
    await sub.ready();
    expect(phases).toEqual(["connecting", "ready"]);
    expect(h.sent.filter((m) => m.t === "topic_unsub")).toHaveLength(1);
  });

  it("a move the relay rejects leaves the handle on its group with its settings", async () => {
    const h = setup();
    const a = h.camera();
    const b = h.camera();
    h.ready("@t:v1");
    await Promise.all([a.ready(), b.ready()]);
    const update = b.update({ quality: 40 });
    h.client.onMessage({
      t: "topic_error",
      id: h.subs()[1].id,
      code: "variant_limit",
      message: "variant limit",
      outcome: "rejected",
    });
    await expect(update).rejects.toMatchObject({ code: "variant_limit" });
    expect(b.status().phase).toBe("ready");
    expect(b.settings()).toEqual(LOW);
    h.frame(1, "@t:v1");
    expect(a.status().phase).toBe("ready");
    // The abandoned group is gone: the next such request opens a fresh one.
    const again = b.update({ quality: 40 });
    expect(h.subs()).toHaveLength(3);
    h.ready("@t:v3", h.subs()[2], { ...LOW, quality: 40 });
    await expect(again).resolves.toEqual({ ...LOW, quality: 40 });
  });

  it("a changed identity reopens the handle and leaves its old group to the others", async () => {
    const h = setup();
    const a = h.camera();
    const b = h.camera();
    h.ready("@t:v1");
    await Promise.all([a.ready(), b.ready()]);
    const update = b.update({ profile: "raw" });
    expect(b.status().phase).toBe("connecting");
    expect(h.subs()).toHaveLength(2);
    expect(h.subs()[1]).toMatchObject({ profile: "raw", maxHz: 30 });
    expect(h.sent.filter((m) => m.t === "topic_unsub")).toHaveLength(0);
    h.ready("@t:v2", h.subs()[1], { maxHz: 30 });
    await expect(update).resolves.toEqual({ maxHz: 30 });
    expect(a.status().phase).toBe("ready");
    expect(b.status().phase).toBe("ready");
  });
});

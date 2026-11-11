import { spawn } from "node:child_process";
import { once } from "node:events";
import { createInterface } from "node:readline";
import { fileURLToPath } from "node:url";
import assert from "node:assert/strict";
import { test, type TestContext } from "node:test";
import { setTimeout as delay } from "node:timers/promises";
import { deflateSync } from "node:zlib";
import sharp from "sharp";
import { Type } from "typebox";
import { Value } from "typebox/value";
import type { TopicSample, TopicDescriptor } from "@dimos/sdk";
import type { FrameRecord } from "#dimos/agents/sdk/streams/base.ts";
import { InventorySchema } from "#dimos/agents/shared/protocol/runtime/inventory.ts";
import { StreamRefSchema } from "#dimos/agents/shared/protocol/streams.ts";
import { openRelay, readTopic, topicStreams, toFrame } from "#dimos/agents/sdk/relay.ts";
import { RemoteFrames } from "#dimos/agents/sdk/streams/remote.ts";

const stream = Value.Parse(StreamRefSchema, {
  channel: "custom/remapped/key",
  encoding: "application/cbor",
  source: { module: "Sensor", name: "value", direction: "out", type: "int" },
});
function sample(payload: Uint8Array, encoding = "application/cbor", metadata = {}): TopicSample {
  return {
    key: stream.channel,
    payload,
    encoding,
    source: "adapter-test",
    generation: 1,
    sequence: 1,
    ingressTs: 1,
    sentTs: 1,
    metadata,
  };
}
async function until(check: () => boolean | Promise<boolean>): Promise<void> {
  const deadline = Date.now() + 8_000;
  while (!(await check())) {
    assert.ok(Date.now() < deadline, "topic adapter did not reach expected state");
    await delay(10);
  }
}

test("topic samples decode into images, clouds and typed bounded values", async () => {
  const jpeg = await sharp({ create: { width: 12, height: 8, channels: 3, background: "red" } })
    .jpeg()
    .toBuffer();
  const header = { frameId: "camera", acquisitionTs: 1, headerSeq: 1 };
  const image = await toFrame(
    { ...stream, profile: "image.jpeg.balanced.v1" },
    sample(jpeg, "image/jpeg", { ...header, w: 12, h: 8 }),
  );
  assert.deepEqual(image, { kind: "image", channel: stream.channel, at: 1000, width: 12, height: 8, jpeg });
  const cloud = await toFrame(
    { ...stream, profile: "cloud.q16.balanced.v1" },
    sample(deflateSync(Uint8Array.of(1, 0, 2, 0, 3, 0)), "application/vnd.dimos.cloud.q16.zlib.v1", {
      ...header,
      count: 1,
      sourceCount: 1,
      intensity: false,
      color: false,
      origin: [0, 0, 0],
      scale: 0.5,
    }),
  );
  assert.deepEqual(cloud, { kind: "cloud", channel: stream.channel, at: 1000, xyz: Float32Array.of(0.5, 1, 1.5) });
  const bytes = Buffer.alloc(12);
  bytes.writeInt32BE(42, 8);
  const lcm = {
    type: "example.Count",
    fp: "0000000000000000",
    structs: { "example.Count": [["count", "int32_t", null]] },
  };
  const value = await toFrame({ ...stream, encoding: "application/x-lcm", lcm }, sample(bytes, "application/x-lcm"));
  assert.ok(value.kind === "value" && value.text.includes("count: 42"));
  const gridStream = Value.Parse(StreamRefSchema, { ...stream, source: { ...stream.source, type: "OccupancyGrid" } });
  const grid = await toFrame(
    gridStream,
    sample(
      Buffer.from(
        "omRpbmZvpGV3aWR0aAJmaGVpZ2h0AmpyZXNvbHV0aW9u+z+5mZmZmZmaZm9yaWdpbqJocG9zaXRpb26iYXgBYXkCa29yaWVudGF0aW9upGF4AGF5AGF6AGF3AWRkYXRhhCAAGGQYMg==",
        "base64",
      ),
    ),
  );
  assert.deepEqual(grid, {
    kind: "grid",
    channel: stream.channel,
    at: 1000,
    width: 2,
    height: 2,
    resolution: 0.1,
    origin: [1, 2, 0],
    cells: Int8Array.of(-1, 0, 100, 50),
  });
  const text = Buffer.from("x".repeat(300));
  const long = await toFrame(stream, sample(Buffer.concat([Buffer.from([0x79, 1, 44]), text])));
  assert.ok(long.kind === "value" && long.text.length === 200);
  await assert.rejects(toFrame(stream, sample(Uint8Array.of(0xff))), /CBOR/);
  await assert.rejects(
    toFrame({ ...stream, encoding: "future/encoding" }, sample(bytes, "future/encoding")),
    /Unsupported/,
  );
});

test("real topic provider discovery, reads, cancellation and preview lifecycle", { timeout: 90_000 }, async (t) => {
  const relay = await topicFixture(t);
  const session = await openRelay(relay.endpoint);
  const frames = new RemoteFrames();
  t.after(() => {
    frames.close();
    session.close();
  });
  const entries = Array.from({ length: 45 }, (_, i) => ({
    key: i === 0 ? stream.channel : `topic/${String(i).padStart(2, "0")}`,
    encoding: "application/cbor",
    type: "int",
    availability: "known" as const,
  }));
  await relay.descriptors(entries);
  const inventory = Value.Parse(InventorySchema, {
    environment: {
      python: "",
      dimos_root: "",
      dimos_version: null,
      dimos_commit: null,
      transport: "zenoh",
      robot_ip: null,
      robot_ips: null,
      zenoh_connect: null,
      zenoh_scouting: false,
      python_api_guide: null,
      relay: relay.endpoint,
    },
    runtime: {
      modules: [],
      streams: entries.map((entry) => ({
        module: "Sensor",
        name: entry.key,
        direction: "out",
        type: "int",
        channel: entry.key,
      })),
    },
    agent_modules: [],
  });
  let streams = await topicStreams(session, inventory, AbortSignal.timeout(10_000));
  assert.equal(streams.length, 45, "all catalog pages are read");
  assert.ok(streams.some((s) => s.channel === stream.channel && s.source?.module === "Sensor"));
  const selected = streams.find((s) => s.channel === stream.channel);
  assert.ok(selected);
  async function receive(mode: "next" | "latest", n: number) {
    const ready = session.topics.subscribe(stream.channel, { profile: selected!.profile }, () => {});
    try {
      await ready.ready();
      const [result] = await Promise.all([
        readTopic(session, selected!, mode, AbortSignal.timeout(5_000)),
        relay.frame(stream.channel, Uint8Array.of(n)),
      ]);
      return result;
    } finally {
      ready.close();
    }
  }
  assert.equal((await receive("latest", 1)).sample.payload[0], 1);
  assert.equal((await receive("next", 2)).sample.payload[0], 2, "next ignores earlier cached data");
  const cached = await readTopic(session, selected, "latest", AbortSignal.timeout(5_000));
  assert.equal(cached.stale, true);
  await assert.rejects(readTopic(session, selected, "latest", AbortSignal.timeout(30)), { name: "TimeoutError" });
  const abort = new AbortController();
  const pending = readTopic(session, selected, "latest", abort.signal);
  abort.abort();
  await assert.rejects(pending, { name: "AbortError" });
  await assert.rejects(
    readTopic(session, { ...selected, profile: "invalid.profile" }, "next", AbortSignal.timeout(5_000)),
  );

  const records: FrameRecord[] = [];
  frames.onRecord((record) => records.push(record));
  frames.want(new Set([stream.channel]));
  frames.configure(relay.endpoint, streams);
  async function preview(n: number) {
    await until(async () => {
      if (records.some((r) => r.kind === "value" && r.text === String(n))) return true;
      await relay.frame(stream.channel, Uint8Array.of(n));
      return false;
    });
  }
  await preview(3);
  await until(() =>
    records.some((r) => r.kind === "rates" && r.sample.counts.some((c) => c.hz > 0 && c.ageS !== null)),
  );
  frames.want(new Set());
  const before = records.filter((r) => r.kind === "value").length;
  await relay.frame(stream.channel, Uint8Array.of(4));
  await delay(100);
  assert.equal(records.filter((r) => r.kind === "value").length, before);
  inventory.runtime.streams = inventory.runtime.streams.filter((s) => s.channel !== stream.channel);
  streams = await topicStreams(session, inventory, AbortSignal.timeout(5_000));
  assert.equal(streams.length, 44, "historical observed descriptors do not outlive live wiring");
  frames.configure(relay.endpoint, streams);
  frames.want(new Set([stream.channel]));
  await relay.frame(stream.channel, Uint8Array.of(5));
  await delay(100);
  assert.equal(records.filter((r) => r.kind === "value").length, before);
  frames.configure(relay.endpoint, [selected]);
  await preview(6);
  const generation = session.topics.sources()[0].gen;
  await relay.disconnect();
  await until(() => session.topics.sources().some((s) => s.gen !== generation));
  await assert.rejects(readTopic(session, selected, "latest", AbortSignal.timeout(2_300)), { name: "TimeoutError" });
  await preview(7);
  await relay.restart();
  await preview(8);
  frames.configure(null);
  frames.configure(relay.endpoint, [selected]);
  frames.close();
  frames.close();
  const closed = records.length;
  await delay(200);
  assert.equal(records.length, closed, "close suppresses pending connection and decoding work");
});

const PYTHON =
  process.env.DIMOS_RELAY_TEST_PYTHON ?? fileURLToPath(new URL("../../../../.venv/bin/python", import.meta.url));

async function topicFixture(t: TestContext) {
  const child = spawn(PYTHON, [fileURLToPath(new URL("./demo_relay.py", import.meta.url))], {
    env: {
      ...process.env,
      DIMOS_WEB_DIR: fileURLToPath(new URL("../../../../web", import.meta.url)),
      DIMOS_WEB_DIR_BUILD: "0",
    },
    stdio: ["pipe", "pipe", "inherit"],
  });
  const lines = createInterface({ input: child.stdout });
  const replies = lines[Symbol.asyncIterator]();
  const exit = once(child, "exit");
  t.after(async () => {
    child.stdin.end();
    const kill = setTimeout(() => child.kill("SIGKILL"), 5_000);
    try {
      await exit;
    } finally {
      clearTimeout(kill);
      lines.close();
    }
  });
  async function response() {
    for (;;) {
      const { value, done } = await replies.next();
      assert.ok(!done, "topic fixture exited before replying");
      if (value.startsWith("{")) return JSON.parse(value);
    }
  }
  const endpoint = Value.Parse(Type.Object({ url: Type.String(), source: Type.String() }), await response());
  async function command(value: unknown): Promise<void> {
    child.stdin.write(`${JSON.stringify(value)}\n`);
    await response();
  }
  return {
    endpoint,
    descriptors: (entries: TopicDescriptor[]) => command({ kind: "descriptors", entries }),
    restart: () => command({ kind: "restart" }),
    disconnect: () => command({ kind: "disconnect" }),
    frame: (key: string, bytes: Uint8Array, encoding = "application/cbor") =>
      command({ kind: "frame", key, bytes: Buffer.from(bytes).toString("base64"), encoding }),
  };
}

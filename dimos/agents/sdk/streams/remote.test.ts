import assert from "node:assert/strict";
import { spawn } from "node:child_process";
import { existsSync } from "node:fs";
import { once } from "node:events";
import { createInterface } from "node:readline";
import { setTimeout as delay } from "node:timers/promises";
import { fileURLToPath } from "node:url";
import { deflateSync } from "node:zlib";
import { test } from "node:test";
import type { TestContext } from "node:test";
import type { ChannelSpec, Slot } from "@dimos/sdk";
import type { FrameRecord } from "./base.ts";
import type { Channel } from "../../shared/protocol/streams.ts";
import { connect, createDecoderRegistry, TRACK_ENCODING } from "@dimos/sdk";
import { PROTOCOL_VERSION } from "@dimos/shared";
import { Type } from "typebox";
import { Value } from "typebox/value";
import sharp from "sharp";
import { FakeRelayEnd, INFO, manifest, ROBOT_A, spec } from "../../../../web/sdk/src/testing/fakeRelay.ts";
import { epochMs } from "../../shared/protocol/time.ts";
import { channelFor } from "../relay.ts";
import { RemoteFrames, toFrame } from "./remote.ts";

const CHANNEL = Value.Parse(Type.Unsafe<Channel>(Type.String({ minLength: 1 })), "preview");
const AT = epochMs(1234);
const decoders = createDecoderRegistry();
const PYTHON =
  process.env.DIMOS_RELAY_TEST_PYTHON ?? fileURLToPath(new URL("../../../../.venv/bin/python", import.meta.url));

function previewSpec(encoding = "json.v1", params = {}): ChannelSpec {
  return spec({ ch: "preview", encoding, params });
}

function slot(spec: ChannelSpec, bytes: Uint8Array, meta = {}): Slot {
  const decode = decoders.resolve(spec);
  assert.ok(decode);
  return { ...decode(bytes, { ch: spec.ch, seq: 1, ts: 1, delivery: "reliable", meta }), seq: 1, ts: 1, version: 1 };
}

async function until(check: () => boolean): Promise<void> {
  const deadline = Date.now() + 5_000;
  while (!check()) {
    assert.ok(Date.now() < deadline, "adapter did not reach the expected state");
    await delay(5);
  }
}

test("decoded SDK slots become images, signed grids, clouds and bounded values", async () => {
  const jpeg = await sharp({ create: { width: 12, height: 8, channels: 3, background: "red" } })
    .jpeg()
    .toBuffer();
  const imageSpec = previewSpec("jpeg.v1");
  const image = await toFrame(CHANNEL, imageSpec, slot(imageSpec, jpeg), AT);
  assert.deepEqual(image, { kind: "image", channel: CHANNEL, at: AT, width: 12, height: 8, jpeg });
  assert.ok(image.kind === "image" && image.jpeg === jpeg, "JPEG bytes are reused");

  const gridSpec = previewSpec("costmap.zlib.v1");
  assert.deepEqual(
    await toFrame(
      CHANNEL,
      gridSpec,
      slot(gridSpec, deflateSync(Uint8Array.of(255, 0, 100, 50)), {
        w: 2,
        h: 2,
        res: 0.1,
        origin: [1, 2, 0.5],
      }),
      AT,
    ),
    {
      kind: "grid",
      channel: CHANNEL,
      at: AT,
      width: 2,
      height: 2,
      resolution: 0.1,
      origin: [1, 2, 0.5],
      cells: Int8Array.of(-1, 0, 100, 50),
    },
  );

  const chunk = new Uint8Array(524);
  chunk[12] = 1;
  const cloudSpec = previewSpec("voxels.zlib.v1");
  assert.deepEqual(
    await toFrame(CHANNEL, cloudSpec, slot(cloudSpec, deflateSync(chunk), { res: 1, n: 1, chunks: 1 }), AT),
    {
      kind: "cloud",
      channel: CHANNEL,
      at: AT,
      xyz: Float32Array.of(0.5, 0.5, 0.5),
    },
  );

  const lcmSpec = previewSpec("example.Count.lcm.v1", {
    lcm: { type: "example.Count", fp: "0000000000000000", structs: { "example.Count": [["count", "int32_t", null]] } },
  });
  const lcm = Buffer.alloc(12);
  lcm.writeInt32BE(42, 8);
  const value = await toFrame(CHANNEL, lcmSpec, slot(lcmSpec, lcm), AT);
  assert.ok(value.kind === "value" && value.text.includes("count: 42"));
  const long = await toFrame(
    CHANNEL,
    previewSpec(),
    slot(previewSpec(), Buffer.from(JSON.stringify("x".repeat(300)))),
    AT,
  );
  assert.ok(long.kind === "value" && long.text.length === 200);
});

test("conversion errors and unsupported encodings produce error records", async () => {
  const gridSpec = previewSpec("costmap.zlib.v1");
  const bad = await toFrame(
    CHANNEL,
    gridSpec,
    slot(gridSpec, Uint8Array.of(1, 2, 3), {
      w: 1,
      h: 1,
      res: 1,
      origin: [0, 0, 0],
    }),
    AT,
  );
  assert.equal(bad.kind, "error");
  assert.equal(
    (await toFrame(CHANNEL, previewSpec("future.v1"), { value: undefined, seq: 1, ts: 1, version: 1 }, AT)).kind,
    "error",
  );
});

test("channel lookup uses exact manifest IDs without source metadata", async (t) => {
  const relay = new FakeRelayEnd();
  const session = connect({ robot: ROBOT_A.id }, { fetchInfo: async () => INFO, createWebTransport: () => relay.wt });
  t.after(() => {
    relay.endControl();
    session.close();
  });
  assert.equal(channelFor(session, CHANNEL), undefined);
  await until(() => relay.sent.some((message) => message.t === "hello"));
  relay.push({ t: "welcome", v: PROTOCOL_VERSION });
  relay.push({ t: "robots", robots: [ROBOT_A] });
  relay.pushManifest(ROBOT_A.id, manifest([previewSpec()]));
  await until(() => session.status.get().manifest !== null);
  assert.equal(channelFor(session, CHANNEL)?.ch, "preview");
  relay.pushManifest(ROBOT_A.id, manifest([previewSpec(), spec({ ch: "sensor.number", encoding: "json.v1" })]));
  await until(() => channelFor(session, "sensor.number") !== undefined);
  assert.equal(channelFor(session, "dimos/number/int"), undefined);
  relay.pushManifest(ROBOT_A.id, manifest([previewSpec(TRACK_ENCODING)]));
  await until(() => session.status.get().manifest?.channels[0].encoding === TRACK_ENCODING);
  assert.equal(channelFor(session, "sensor.number"), undefined);
  assert.throws(() => channelFor(session, CHANNEL), /rtc=false/);
});

async function realRelay(t: TestContext) {
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
      assert.ok(!done, "relay fixture exited before replying");
      if (value.startsWith("{")) return JSON.parse(value);
    }
  }
  const endpoint = Value.Parse(Type.Object({ url: Type.String(), robot: Type.String() }), await response());
  async function command(value: unknown): Promise<void> {
    child.stdin.write(`${JSON.stringify(value)}\n`);
    await response();
  }
  return {
    endpoint,
    manifest: (channels: ChannelSpec[]) => command({ kind: "manifest", channels }),
    disconnect: () => command({ kind: "disconnect" }),
    restart: () => command({ kind: "restart" }),
    frame: (bytes: Uint8Array, meta = {}) =>
      command({ kind: "frame", channel: "preview", bytes: Buffer.from(bytes).toString("base64"), meta }),
  };
}

test("real relay adapter lifecycle", { skip: !existsSync(PYTHON), timeout: 60_000 }, async (t) => {
  const relay = await realRelay(t);
  await relay.manifest([previewSpec()]);
  const records: FrameRecord[] = [];
  const errors: unknown[] = [];
  const frames = new RemoteFrames((error) => errors.push(error));
  t.after(() => frames.close());
  frames.onRecord((record) => records.push(record));
  frames.want(new Set([CHANNEL]));
  frames.configure(relay.endpoint);

  async function receive(value: number): Promise<void> {
    for (let i = 0; i < 60; i++) {
      await relay.frame(Buffer.from(JSON.stringify({ value })));
      if (records.some((record) => record.kind === "value" && record.text.includes(String(value)))) return;
      await delay(50);
    }
    assert.fail(`no preview for ${value}`);
  }

  await t.test("receives frames and SDK rates, cancels and restores wanted channels", async () => {
    await receive(101);
    await until(() =>
      records.some((r) => r.kind === "rates" && r.sample.counts.some((c) => c.hz > 0 && c.ageS !== null)),
    );
    frames.configure({ ...relay.endpoint });
    frames.want(new Set());
    const before = records.filter((r) => r.kind === "value").length;
    await relay.frame(Buffer.from("102"));
    await delay(600);
    assert.equal(records.filter((r) => r.kind === "value").length, before);
    frames.want(new Set([CHANNEL]));
    await receive(103);
  });

  await t.test("manifest removal and reappearance resume wanted channels on the same producer", async () => {
    await relay.manifest([]);
    await until(
      () => records.at(-1)?.kind === "rates" && records.some((r) => r.kind === "rates" && r.sample.counts.length === 0),
    );
    const before = records.filter((record) => record.kind === "value").length;
    await relay.frame(Buffer.from("106"));
    await delay(100);
    assert.equal(records.filter((record) => record.kind === "value").length, before);
    await relay.manifest([previewSpec()]);
    await receive(107);
  });

  await t.test("producer reconnection restores subscriptions and accepts restarted sequences", async () => {
    await relay.disconnect();
    await delay(600);
    await relay.manifest([previewSpec()]);
    await receive(104);
  });

  await t.test("relay restart reconnects without recreating RemoteFrames", async () => {
    await relay.restart();
    await relay.manifest([previewSpec()]);
    await receive(105);
  });

  for (const action of ["coalesce", "cancel", "disconnect"] as const) {
    await t.test(`async image conversion handles ${action}`, async () => {
      await relay.manifest([previewSpec("jpeg.v1")]);
      frames.want(new Set([CHANNEL]));
      const jpeg = await sharp({ create: { width: 12, height: 8, channels: 3, background: "red" } })
        .jpeg()
        .toBuffer();
      const before = records.filter((record) => record.kind === "image").length;
      let release = () => {};
      let started = false;
      const gate = new Promise<void>((resolve) => {
        release = resolve;
      });
      const metadata = t.mock.method(sharp.prototype, "metadata", async () => {
        started = true;
        await gate;
        return { width: 12, height: 8 };
      });
      try {
        for (let i = 0; i < 40 && !started; i++) {
          await relay.frame(jpeg);
          await delay(50);
        }
        assert.ok(started);
        if (action === "coalesce") {
          for (let i = 0; i < 5; i++) await relay.frame(jpeg);
          await delay(600);
          release();
          await until(() => records.filter((record) => record.kind === "image").length === before + 2);
          assert.equal(metadata.mock.callCount(), 2);
        } else {
          if (action === "cancel") frames.want(new Set());
          else {
            await relay.disconnect();
            await until(() => {
              const last = records.at(-1);
              return last?.kind === "rates" && last.sample.counts.length === 0;
            });
          }
          release();
          await delay(100);
          assert.equal(records.filter((record) => record.kind === "image").length, before);
        }
      } finally {
        release();
        metadata.mock.restore();
      }
    });
  }

  await t.test("endpoint replacement and close suppress pending connection work", async () => {
    frames.configure(null);
    frames.configure(relay.endpoint);
    frames.configure(null);
    frames.configure(relay.endpoint);
    frames.close();
    const before = records.length;
    await delay(600);
    assert.equal(records.length, before);
    assert.deepEqual(errors, []);
  });

  await t.test("raw session receives data and repeated close lets its process exit", async () => {
    await relay.manifest([previewSpec()]);
    const source = `
      import { openRelay } from ${JSON.stringify(new URL("../relay.ts", import.meta.url).href)};
      const session = await openRelay(${JSON.stringify(relay.endpoint)});
      const deadline = setTimeout(() => { session.close(); process.exitCode = 1; }, 10000);
      session.subscribe("preview", ({slot}) => {
        if (slot?.value?.answer !== 42) return;
        clearTimeout(deadline);
        session.close(); session.close();
      });
    `;
    const child = spawn(
      process.execPath,
      [
        "--import",
        fileURLToPath(new URL("../../test/web-sdk.mjs", import.meta.url)),
        "--input-type=module",
        "-e",
        source,
      ],
      { stdio: "inherit" },
    );
    const exit = once(child, "exit");
    const kill = setTimeout(() => child.kill("SIGKILL"), 12_000);
    const send = setInterval(() => {
      void relay.frame(Buffer.from('{"answer":42}'));
    }, 100);
    try {
      const [code, signal] = await exit;
      assert.equal(signal, null, "the Node provider leaked a handle after close");
      assert.equal(code, 0);
    } finally {
      clearTimeout(kill);
      clearInterval(send);
      if (child.exitCode === null) child.kill("SIGKILL");
    }
  });
});

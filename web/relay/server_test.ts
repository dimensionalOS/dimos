// In-process loopback e2e: startRelay + Deno's own WebTransport client as
// both robot and viewer. Deno's client CAN receive relay-initiated uni
// streams (verified; the 2.6.10 incoming-uni bug is server-side receive
// only), so this covers the full forwarding path without a browser.
import { assert, assertEquals, assertRejects } from "@std/assert";
import { fileURLToPath } from "node:url";
import {
  ControlFrameReader,
  DataFrameStreamReader,
  decodeDatagram,
  encodeControlFrame,
  encodeDataFrame,
  encodeDatagram,
  type FrameHeader,
  type Msg,
  PROTOCOL_VERSION,
  type RobotInfo,
  type RobotManifest,
} from "@dimos/shared";
import { startRelay } from "./server.ts";

const ROBOT: RobotInfo = { id: "deno-bot", name: "Deno Bot", model: "test" };
// Raw (un-normalized) on purpose: the relay must forward it verbatim.
const CHANNELS = [
  { ch: "color_image", encoding: "jpeg.v1", delivery: "latest", maxHz: 15.5 },
  { ch: "odom", encoding: "pose.json.v1", delivery: "reliable", maxHz: 20.5 },
];
const MANIFEST: RobotManifest = {
  version: 1,
  channels: CHANNELS,
  panels: [{ id: "color_image", kind: "video", channels: ["color_image"] }],
  layout: "color_image",
};

function certOpts(hashB64: string): WebTransportOptions {
  return {
    serverCertificateHashes: [{
      algorithm: "sha-256",
      value: Uint8Array.from(atob(hashB64), (c) => c.charCodeAt(0)),
    }],
  };
}

function within<T>(promise: Promise<T>, what: string, ms = 8000): Promise<T> {
  let timer: number;
  const timeout = new Promise<T>((_, reject) => {
    timer = setTimeout(() => reject(new Error(`${what} timed out after ${ms} ms`)), ms);
  });
  return Promise.race([promise, timeout]).finally(() => clearTimeout(timer));
}

/** Pull-based message queue over a stream of control frames (BYOB reads). */
function controlQueue(readable: ReadableStream<Uint8Array>): () => Promise<Msg> {
  const queue: Msg[] = [];
  const waiters: ((msg: Msg) => void)[] = [];
  (async () => {
    const frames = new ControlFrameReader();
    const reader = readable.getReader({ mode: "byob" });
    while (true) {
      const { value, done } = await reader.read(new Uint8Array(8 * 1024));
      if (value && value.byteLength) {
        for (const msg of frames.push(value)) {
          const waiter = waiters.shift();
          if (waiter) waiter(msg);
          else queue.push(msg);
        }
      }
      if (done) break;
    }
  })().catch(() => {});
  return () => {
    const msg = queue.shift();
    if (msg) return Promise.resolve(msg);
    return new Promise<Msg>((resolve) => waiters.push(resolve));
  };
}

/** Pull-based message queue over incoming datagrams (junk skipped). */
function datagramQueue(readable: ReadableStream<Uint8Array>): () => Promise<Msg> {
  const queue: Msg[] = [];
  const waiters: ((msg: Msg) => void)[] = [];
  (async () => {
    for await (const dg of readable) {
      const msg = decodeDatagram(dg);
      if (msg === null) continue;
      const waiter = waiters.shift();
      if (waiter) waiter(msg);
      else queue.push(msg);
    }
  })().catch(() => {});
  return () => {
    const msg = queue.shift();
    if (msg) return Promise.resolve(msg);
    return new Promise<Msg>((resolve) => waiters.push(resolve));
  };
}

/**
 * Collect forwarded data frames arriving on relay-initiated uni streams
 * (one frame per latest stream; back-to-back frames on a reliable channel's
 * persistent stream). `onAcceptLoopDeath` fires if the accept loop errors
 * for good (client-side bug 10).
 */
function frameQueue(
  wt: WebTransport,
  onAcceptLoopDeath?: (e: unknown) => void,
): () => Promise<{ header: FrameHeader; payload: Uint8Array }> {
  const queue: { header: FrameHeader; payload: Uint8Array }[] = [];
  const waiters: ((f: { header: FrameHeader; payload: Uint8Array }) => void)[] = [];
  const deliver = (frame: { header: FrameHeader; payload: Uint8Array }) => {
    const waiter = waiters.shift();
    if (waiter) waiter(frame);
    else queue.push(frame);
  };
  (async () => {
    for await (const stream of wt.incomingUnidirectionalStreams) {
      (async () => {
        const frames = new DataFrameStreamReader();
        const reader = (stream as ReadableStream<Uint8Array>).getReader();
        while (true) {
          const { value, done } = await reader.read();
          if (value && value.byteLength) frames.push(value).forEach(deliver);
          if (done) break;
        }
      })().catch(() => {});
    }
  })().catch((e) => onAcceptLoopDeath?.(e));
  return () => {
    const frame = queue.shift();
    if (frame) return Promise.resolve(frame);
    return new Promise((resolve) => waiters.push(resolve));
  };
}

async function sendRobotFrame(robot: WebTransport, header: FrameHeader, payload: Uint8Array) {
  const stream = await robot.createBidirectionalStream();
  const writer = stream.writable.getWriter();
  await writer.write(encodeDataFrame(header, payload));
  await writer.close(); // FIN is delayed by Deno (bug 2); the relay reads by byte count
}

// Test that one suspended viewer must cost only itself. Its stale streams are
// reset ("aborted, not queued") while the healthy viewer keeps receiving fresh
// frames at full rate. This is also the permanent proof that reaping keeps
// working against real quinn streams.
//
// MUST RUN FIRST in this file: a preceding test's teardown churn (or plain CPU
// starvation, e.g. a 2-CPU CI runner) lets a relay reset race Deno's preamble
// read of a not-yet-accepted incoming uni stream - the client-side analog of
// README bug 10. The client's accept glue then errors or silently hangs, its
// stream credit freezes at ~100, and the healthy viewer starves. Not fixable
// from test code (no client-side raw-QuicConn API), so wedged rounds are
// retried - but only on the wedge's own signatures. Any other failure, or 4
// wedged rounds, still fails: a relay regression cannot hide behind the retries.
Deno.test({
  name: "a viewer that stops reading is reset, not queued; others keep full rate",
  sanitizeOps: false,
  sanitizeResources: false,
}, async () => {
  for (let attempt = 1;; attempt++) {
    const round: RoundState = { glueDeath: null, lastInflight: NaN };
    try {
      await runBackpressureRound(round);
      return;
    } catch (e) {
      // Wedged = accept loop errored, or inflight froze at exactly 1: the
      // never-reaped unaccepted carrier, only frozen client credit does that.
      const wedged = round.glueDeath !== null || round.lastInflight === 1;
      if (!wedged || attempt >= 4) throw e;
      console.log(
        `[test] round ${attempt} hit the Deno incoming-uni wedge (` +
          (round.glueDeath !== null
            ? `accept loop died: ${round.glueDeath}`
            : "accept loop hung: carrier stuck unaccepted") +
          "); retrying",
      );
      // 5 s matters: the dead round's churn re-arms the race if we go sooner.
      await new Promise((resolve) => setTimeout(resolve, 5000));
    }
  }
});

interface RoundState {
  glueDeath: unknown; // healthy accept-loop error, recorded pre-teardown only
  lastInflight: number; // healthy inflight as last seen by the reap poll
}

/** One backpressure round against a fresh relay; fills `round` so the
 * caller can tell the Deno wedge from a relay regression. */
async function runBackpressureRound(round: RoundState): Promise<void> {
  const relay = await startRelay({ port: 0 });
  const httpBase = `http://127.0.0.1:${relay.httpPort}`;
  // The healthy viewer is the one that accepted the most sends.
  const fetchHealthyInflight = async (): Promise<number> => {
    const stats = await (await fetch(`${httpBase}/api/stats`)).json();
    const channels = (stats.perViewer as {
      channels: Record<string, { sent: number; inflight: number }>;
    }[])
      .map((v) => v.channels.color_image)
      .filter((c) => c !== undefined)
      .sort((a, b) => b.sent - a.sent);
    return channels[0]?.inflight ?? -1;
  };
  const clients: WebTransport[] = [];
  let tearingDown = false;
  try {
    const robot = new WebTransport(`${relay.wtUrl}/robot`, certOpts(relay.certHash));
    clients.push(robot);
    await within(robot.ready, "robot connect");
    const robotDatagrams = datagramQueue(robot.datagrams.readable);
    const robotDgWriter = robot.datagrams.writable.getWriter();
    await robotDgWriter.write(
      encodeDatagram({
        t: "hello",
        v: PROTOCOL_VERSION,
        role: "robot",
        robot: ROBOT,
        manifest: {
          version: 1,
          channels: [{ ch: "color_image", encoding: "jpeg.v1", delivery: "latest", maxHz: 100 }],
        },
      }),
    );
    await within(robotDatagrams(), "robot hello reply");

    const attachViewer = async (name: string): Promise<WebTransport> => {
      const wt = new WebTransport(`${relay.wtUrl}/viewer`, certOpts(relay.certHash));
      clients.push(wt);
      await within(wt.ready, `${name} connect`);
      const control = await wt.createBidirectionalStream();
      const writer = control.writable.getWriter();
      const next = controlQueue(control.readable);
      await writer.write(encodeControlFrame({ t: "hello", v: PROTOCOL_VERSION, role: "viewer" }));
      await writer.write(encodeControlFrame({ t: "watch", robotId: ROBOT.id }));
      await writer.write(encodeControlFrame({ t: "sub", ch: "color_image" }));
      let msg: Msg;
      do {
        msg = await within(next(), `${name} manifest`);
      } while (msg.t !== "manifest");
      return wt;
    };

    const healthy = await attachViewer("healthy");
    const healthyFrames = frameQueue(healthy, (e) => {
      if (!tearingDown) round.glueDeath = e;
    });
    const stalled = await attachViewer("stalled");
    void stalled; // never reads incomingUnidirectionalStreams: a suspended tab

    const payload = new Uint8Array(8 * 1024).fill(9);
    const fetchStalled = async () => {
      const stats = await (await fetch(`${httpBase}/api/stats`)).json();
      const viewers = stats.perViewer as {
        channels: Record<string, { aborted: number; queued: number; sent: number }>;
      }[];
      // The stalled viewer is the one whose acceptance count froze.
      return viewers
        .map((v) => v.channels.color_image)
        .filter((c) => c !== undefined)
        .sort((a, b) => a.sent - b.sent)[0];
    };

    // Pump frames until the stalled viewer's stale streams are being reset
    // under backpressure. Onset needs its uni-stream credit (~100) exhausted
    // plus one LATEST_STALE_MS window, so give it a generous frame budget.
    let lastSeq = 0;
    let stalledStats = await fetchStalled();
    for (let seq = 1; seq <= 1200; seq++) {
      await sendRobotFrame(
        robot,
        { ch: "color_image", seq, ts: seq / 100, delivery: "latest" },
        payload,
      );
      lastSeq = seq;
      if (seq % 50 === 0) {
        stalledStats = await fetchStalled();
        if (stalledStats !== undefined && stalledStats.aborted >= 3) break;
      }
      await new Promise((resolve) => setTimeout(resolve, 10));
    }
    assert(stalledStats !== undefined, "stalled viewer never got a policy");
    assert(stalledStats.aborted >= 3, `expected aborted resets, got ${stalledStats.aborted}`);
    assert(stalledStats.queued <= 1, `latest queue must stay 0|1, got ${stalledStats.queued}`);

    // The healthy viewer kept receiving fresh frames the whole time: drain its
    // queue and require a seq from the era after the stalled viewer wedged.
    let newest = 0;
    const drainUntil = Date.now() + 8000;
    while (newest < lastSeq - 50 && Date.now() < drainUntil) {
      newest = Math.max(newest, (await within(healthyFrames(), "healthy frame")).header.seq);
    }
    assert(newest >= lastSeq - 50, `healthy viewer stalled at seq ${newest} of ${lastSeq}`);

    // The input has quiesced, so no newer offer will reap the healthy viewer's
    // last accepted stream: only the relay's periodic reap timer can return it
    // (against real quinn streams).
    const reapDeadline = Date.now() + 5000;
    let healthyInflight = -1;
    while (Date.now() < reapDeadline) {
      healthyInflight = await fetchHealthyInflight();
      round.lastInflight = healthyInflight;
      if (healthyInflight === 0) break;
      await new Promise((resolve) => setTimeout(resolve, 200));
    }
    assertEquals(healthyInflight, 0, "idle reap never reset the healthy viewer's last stream");
  } catch (e) {
    // A wedge during pump/drain dies before the reap poll but leaves the
    // same stuck-carrier signature; settle one stale window, then probe.
    if (round.glueDeath === null && Number.isNaN(round.lastInflight)) {
      await new Promise((resolve) => setTimeout(resolve, 700));
      round.lastInflight = await fetchHealthyInflight().catch(() => NaN);
    }
    throw e;
  } finally {
    tearingDown = true;
    for (const wt of clients) {
      try {
        wt.close();
      } catch {
        // session already dead
      }
    }
    await relay.shutdown();
  }
}

Deno.test({
  name: "relay loopback e2e",
  // QUIC endpoint + WT sessions keep background ops alive past shutdown();
  // their teardown is asynchronous in Deno 2.6.
  sanitizeOps: false,
  sanitizeResources: false,
}, async (t) => {
  const relay = await startRelay({ port: 0 });
  const httpBase = `http://127.0.0.1:${relay.httpPort}`;

  await t.step("/api/info matches the handle; no cockpit dist -> 404 with a hint", async () => {
    const info = await (await fetch(`${httpBase}/api/info`)).json();
    assertEquals(info, {
      wtUrl: `${relay.wtUrl}/viewer`,
      certHash: relay.certHash,
      v: PROTOCOL_VERSION,
    });
    assert(relay.wtUrl.startsWith("https://127.0.0.1:"), relay.wtUrl);
    // Without a cockpit dist the relay serves no files at all: / explains how
    // to get one (traversal guards are covered by the cockpit-dist test).
    const index = await fetch(`${httpBase}/`);
    assertEquals(index.status, 404);
    assert((await index.text()).includes("cockpit dist not built"));
    const stray = await fetch(`${httpBase}/debug.html`);
    await stray.body?.cancel();
    assertEquals(stray.status, 404);
  });

  const viewer = new WebTransport(`${relay.wtUrl}/viewer`, certOpts(relay.certHash));
  await within(viewer.ready, "viewer connect");
  const viewerFrames = frameQueue(viewer);
  const viewerDatagrams = datagramQueue(viewer.datagrams.readable);
  const control = await within(viewer.createBidirectionalStream(), "control stream");
  const controlWriter = control.writable.getWriter();
  const nextControl = controlQueue(control.readable);

  await t.step("viewer control: hello -> welcome + robots, ping -> pong", async () => {
    await controlWriter.write(
      encodeControlFrame({ t: "hello", v: PROTOCOL_VERSION, role: "viewer" }),
    );
    assertEquals(await within(nextControl(), "welcome"), {
      t: "welcome",
      v: PROTOCOL_VERSION,
    });
    assertEquals(await within(nextControl(), "robots"), { t: "robots", robots: [] });
    await controlWriter.write(encodeControlFrame({ t: "ping", n: 1, ts: 123.5 }));
    assertEquals(await within(nextControl(), "pong"), { t: "pong", n: 1, ts: 123.5 });
  });

  await t.step("viewer datagram ping -> pong (relay answers itself)", async () => {
    const dgWriter = viewer.datagrams.writable.getWriter();
    await dgWriter.write(encodeDatagram({ t: "ping", n: 2, ts: 124.5 }));
    assertEquals(await within(viewerDatagrams(), "datagram pong"), {
      t: "pong",
      n: 2,
      ts: 124.5,
    });
    dgWriter.releaseLock();
  });

  const robot = new WebTransport(`${relay.wtUrl}/robot`, certOpts(relay.certHash));
  await within(robot.ready, "robot connect");
  const robotDatagrams = datagramQueue(robot.datagrams.readable);
  const robotDgWriter = robot.datagrams.writable.getWriter();

  await t.step("robot hello (identity + manifest) -> welcome + baseline subs", async () => {
    await robotDgWriter.write(
      encodeDatagram({
        t: "hello",
        v: PROTOCOL_VERSION,
        role: "robot",
        robot: ROBOT,
        manifest: MANIFEST,
      }),
    );
    // Registration and welcome are separate datagrams, so their relative
    // arrival is not a protocol guarantee.
    const replies = [
      await within(robotDatagrams(), "robot hello reply"),
      await within(robotDatagrams(), "robot hello reply"),
    ];
    assertEquals(replies.find((msg) => msg.t === "welcome"), {
      t: "welcome",
      v: PROTOCOL_VERSION,
    });
    assertEquals(replies.find((msg) => msg.t === "subs"), {
      t: "subs",
      chs: [],
      n: 1,
    });
  });

  await t.step("registration pushes robots to the greeted viewer", async () => {
    assertEquals(await within(nextControl(), "robots push"), {
      t: "robots",
      robots: [ROBOT],
    });
  });

  await t.step("watch -> manifest reply; subs snapshot reaches the robot", async () => {
    await controlWriter.write(encodeControlFrame({ t: "watch", robotId: ROBOT.id }));
    assertEquals(await within(nextControl(), "manifest"), {
      t: "manifest",
      robotId: ROBOT.id,
      manifest: MANIFEST,
    });
    await controlWriter.write(encodeControlFrame({ t: "sub", ch: "odom" }));
    await controlWriter.write(encodeControlFrame({ t: "sub", ch: "color_image" }));
    // One snapshot per sub message; skip ahead to the full set.
    let subs: Msg;
    do {
      subs = await within(robotDatagrams(), "subs snapshot");
    } while (subs.t === "subs" && subs.chs.length < 2);
    assert(subs.t === "subs");
    assertEquals(subs.chs, ["color_image", "odom"]);
  });

  await t.step("robot frames fan out to the viewer on uni streams", async () => {
    const odomPayload = new TextEncoder().encode('{"x":1.5,"yaw":0.25}');
    await sendRobotFrame(
      robot,
      { ch: "odom", seq: 1, ts: 10.5, delivery: "reliable" },
      odomPayload,
    );
    const imagePayload = new Uint8Array(100_000);
    imagePayload.fill(7);
    await sendRobotFrame(
      robot,
      { ch: "color_image", seq: 2, ts: 11.5, delivery: "latest", meta: { w: 320, h: 240 } },
      imagePayload,
    );

    const got = [
      await within(viewerFrames(), "first forwarded frame"),
      await within(viewerFrames(), "second forwarded frame"),
    ];
    // one-stream-per-message may arrive out of order; sort by seq
    got.sort((a, b) => a.header.seq - b.header.seq);
    assertEquals(got[0].header, { ch: "odom", seq: 1, ts: 10.5, delivery: "reliable" });
    assertEquals(got[0].payload, odomPayload);
    assertEquals(got[1].header, {
      ch: "color_image",
      seq: 2,
      ts: 11.5,
      delivery: "latest",
      meta: { w: 320, h: 240 },
    });
    assertEquals(got[1].payload, imagePayload);
  });

  await t.step("a viewer that never subscribed receives nothing", async () => {
    const idle = new WebTransport(`${relay.wtUrl}/viewer`, certOpts(relay.certHash));
    await within(idle.ready, "idle viewer connect");
    const idleStream = await idle.createBidirectionalStream();
    const idleWriter = idleStream.writable.getWriter();
    const idleControl = controlQueue(idleStream.readable);
    await idleWriter.write(encodeControlFrame({ t: "hello", v: PROTOCOL_VERSION, role: "viewer" }));
    await within(idleControl(), "idle welcome");

    await sendRobotFrame(
      robot,
      { ch: "odom", seq: 3, ts: 12.5, delivery: "reliable" },
      new Uint8Array([3]),
    );
    // The subscribed viewer's receipt proves routing ran with both present.
    assertEquals((await within(viewerFrames(), "odom for subscriber")).header.seq, 3);
    const stats = await (await fetch(`${httpBase}/api/stats`)).json();
    const idleStats = stats.perViewer.find((v: { watched: string | null }) => v.watched === null);
    assertEquals(idleStats.channels, {});
    idle.close();
  });

  await t.step("unsub stops forwarding that channel", async () => {
    await controlWriter.write(encodeControlFrame({ t: "unsub", ch: "color_image" }));
    // Ordered control stream: the pong below proves the unsub was processed.
    await controlWriter.write(encodeControlFrame({ t: "ping", n: 9, ts: 99.5 }));
    assertEquals(await within(nextControl(), "pong after unsub"), { t: "pong", n: 9, ts: 99.5 });

    await sendRobotFrame(
      robot,
      { ch: "color_image", seq: 4, ts: 13.5, delivery: "latest" },
      new Uint8Array([4]),
    );
    await sendRobotFrame(
      robot,
      { ch: "odom", seq: 5, ts: 14.5, delivery: "reliable" },
      new Uint8Array([5]),
    );
    // Only odom arrives; the image frame was not forwarded.
    const got = await within(viewerFrames(), "odom after unsub");
    assertEquals(got.header.ch, "odom");
    assertEquals(got.header.seq, 5);
  });

  await t.step("/api/stats reflects sessions and traffic", async () => {
    // The idle viewer's close is asynchronous on the relay side; poll it out.
    let stats = await (await fetch(`${httpBase}/api/stats`)).json();
    for (let i = 0; i < 80 && stats.viewers !== 1; i++) {
      await new Promise((resolve) => setTimeout(resolve, 100));
      stats = await (await fetch(`${httpBase}/api/stats`)).json();
    }
    assertEquals(stats.robots, [ROBOT]);
    assertEquals(stats.viewers, 1);
    assertEquals(stats.perRobot[ROBOT.id].subs, ["odom"]);
    assertEquals(stats.perRobot[ROBOT.id].channels.odom.framesIn, 3);
    assertEquals(stats.perRobot[ROBOT.id].channels.odom.delivery, "reliable");
    const viewerStats = stats.perViewer.find(
      (v: { watched: string | null }) => v.watched === ROBOT.id,
    );
    assertEquals(viewerStats.subs, ["odom"]);
    assertEquals(viewerStats.channels.odom.sent, 3);
    // Reset counters and rates (T5): a healthy reliable channel never resets.
    assertEquals(viewerStats.channels.odom.delivery, "reliable");
    assertEquals(viewerStats.channels.odom.aborted, 0);
    assertEquals(viewerStats.channels.odom.expired, 0);
    assertEquals(viewerStats.channels.odom.inflight, 0);
    assert(viewerStats.channels.odom.bytesOut > 0);
    assertEquals(typeof viewerStats.channels.odom.fps, "number");
    assert(stats.perRobot[ROBOT.id].channels.odom.bytesIn > 0);
    assertEquals(typeof stats.perRobot[ROBOT.id].channels.odom.bps, "number");
  });

  await t.step("robot hello without robot{} -> missing_robot_id + close", async () => {
    const bare = new WebTransport(`${relay.wtUrl}/robot`, certOpts(relay.certHash));
    await within(bare.ready, "bare robot connect");
    const bareDatagrams = datagramQueue(bare.datagrams.readable);
    const bareWriter = bare.datagrams.writable.getWriter();
    await bareWriter.write(encodeDatagram({ t: "hello", v: PROTOCOL_VERSION, role: "robot" }));
    const err = await within(bareDatagrams(), "missing_robot_id error");
    assertEquals(err.t, "error");
    assertEquals((err as { code: string }).code, "missing_robot_id");
    await within(bare.closed.catch(() => {}), "bare robot session close");
  });

  await t.step("robot hello with an invalid manifest -> invalid_manifest + close", async () => {
    const dup = new WebTransport(`${relay.wtUrl}/robot`, certOpts(relay.certHash));
    await within(dup.ready, "dup-manifest robot connect");
    const dupDatagrams = datagramQueue(dup.datagrams.readable);
    const dupWriter = dup.datagrams.writable.getWriter();
    await dupWriter.write(encodeDatagram({
      t: "hello",
      v: PROTOCOL_VERSION,
      role: "robot",
      robot: { id: "dup-bot", name: "Dup Bot", model: "test" },
      manifest: { version: 1, channels: [CHANNELS[0], CHANNELS[0]] },
    }));
    const err = await within(dupDatagrams(), "invalid_manifest error");
    assertEquals(err.t, "error");
    assertEquals((err as { code: string }).code, "invalid_manifest");
    await within(dup.closed.catch(() => {}), "dup-manifest robot close");
  });

  await t.step("robot hello with the previous protocol version -> error + close", async () => {
    // A v1 bridge would misread the v2 persistent reliable stream as one
    // frame; the handshake must fail loudly instead.
    const old = new WebTransport(`${relay.wtUrl}/robot`, certOpts(relay.certHash));
    await within(old.ready, "old-version robot connect");
    const oldDatagrams = datagramQueue(old.datagrams.readable);
    const oldWriter = old.datagrams.writable.getWriter();
    await oldWriter.write(encodeDatagram({
      t: "hello",
      v: 1,
      role: "robot",
      robot: { id: "old-bot", name: "Old Bot", model: "test" },
    }));
    const err = await within(oldDatagrams(), "old-version error");
    assertEquals(err.t, "error");
    assertEquals((err as { code: string }).code, "version_mismatch");
    await within(old.closed.catch(() => {}), "old-version robot close");
  });

  await t.step("hello with a wrong version -> error + close", async () => {
    const bad = new WebTransport(`${relay.wtUrl}/viewer`, certOpts(relay.certHash));
    await within(bad.ready, "bad-version viewer connect");
    const stream = await bad.createBidirectionalStream();
    const writer = stream.writable.getWriter();
    const next = controlQueue(stream.readable);
    await writer.write(encodeControlFrame({ t: "hello", v: 99, role: "viewer" }));
    const err = await within(next(), "version error");
    assertEquals(err.t, "error");
    assertEquals((err as { code: string }).code, "version_mismatch");
    await within(bad.closed.catch(() => {}), "bad-version session close");
  });

  await t.step("viewer commands before hello -> hello_required + close", async () => {
    const ungreeted = new WebTransport(`${relay.wtUrl}/viewer`, certOpts(relay.certHash));
    await within(ungreeted.ready, "ungreeted viewer connect");
    const stream = await ungreeted.createBidirectionalStream();
    const writer = stream.writable.getWriter();
    const next = controlQueue(stream.readable);
    await writer.write(encodeControlFrame({ t: "watch", robotId: ROBOT.id }));
    const err = await within(next(), "hello_required error");
    assertEquals(err.t, "error");
    assertEquals((err as { code: string }).code, "hello_required");
    await within(ungreeted.closed.catch(() => {}), "ungreeted viewer close");
  });

  await t.step("robot role on /viewer -> role_mismatch + close", async () => {
    const wrongRole = new WebTransport(`${relay.wtUrl}/viewer`, certOpts(relay.certHash));
    await within(wrongRole.ready, "wrong-role viewer connect");
    const stream = await wrongRole.createBidirectionalStream();
    const writer = stream.writable.getWriter();
    const next = controlQueue(stream.readable);
    await writer.write(
      encodeControlFrame({
        t: "hello",
        v: PROTOCOL_VERSION,
        role: "robot",
        robot: { id: "wrong-leg", name: "Wrong Leg", model: "test" },
      }),
    );
    const err = await within(next(), "role_mismatch error");
    assertEquals(err.t, "error");
    assertEquals((err as { code: string }).code, "role_mismatch");
    await within(wrongRole.closed.catch(() => {}), "wrong-role viewer close");
  });

  await t.step("unknown WebTransport endpoint is rejected", async () => {
    const unknown = new WebTransport(`${relay.wtUrl}/unknown`, certOpts(relay.certHash));
    await within(unknown.closed.catch(() => {}), "unknown endpoint close");
    const stats = await (await fetch(`${httpBase}/api/stats`)).json();
    assertEquals(stats.robots, [ROBOT]);
    assertEquals(stats.viewers, 1);
  });

  await t.step("a garbage control message is dropped, not fatal to the loop", async () => {
    // A well-framed but invalid body (JSON null) must not kill the viewer's
    // control loop: a following ping still gets a pong.
    const junk = encodeControlFrame(null as unknown as Msg);
    await controlWriter.write(junk);
    await controlWriter.write(encodeControlFrame({ t: "ping", n: 7, ts: 77.5 }));
    assertEquals(await within(nextControl(), "pong after junk"), { t: "pong", n: 7, ts: 77.5 });
  });

  viewer.close();
  robot.close();
  await relay.shutdown();
});

Deno.test({
  name: "relay serves the cockpit dist when configured",
  sanitizeOps: false,
  sanitizeResources: false,
}, async () => {
  const cockpitDir = fileURLToPath(new URL("./testdata/fake_cockpit", import.meta.url));
  const relay = await startRelay({ port: 0, cockpitDir });
  const httpBase = `http://127.0.0.1:${relay.httpPort}`;
  try {
    const index = await (await fetch(`${httpBase}/`)).text();
    assert(index.includes("fake cockpit index"));
    const asset = await fetch(`${httpBase}/assets/app.js`);
    assertEquals(asset.status, 200);
    assertEquals(asset.headers.get("content-type"), "application/javascript");
    await asset.body?.cancel();
    // Traversal probes. The client/URL parser normalizes these two away from
    // the tree before the guard sees them, so they 404 on absence.
    for (const path of ["/../etc/passwd", "/%2e%2e/etc/passwd"]) {
      const probe = await fetch(`${httpBase}${path}`);
      await probe.body?.cancel();
      assertEquals(probe.status, 404, path);
    }
    // These survive normalization and must be rejected by the containment
    // check: a leading "//" makes new URL() jump to the filesystem root, and
    // encoded slashes let a "../" escape reassemble after decoding.
    for (const path of ["//etc/passwd", "/..%2f..%2f..%2f..%2fetc%2fpasswd"]) {
      const probe = await fetch(`${httpBase}${path}`);
      await probe.body?.cancel();
      assertEquals(probe.status, 400, path);
    }
    // A symlink whose target lies outside the root must not be followed
    // (readFile follows symlinks; the containment check compares realpaths).
    const escape = await fetch(`${httpBase}/escape.txt`);
    await escape.body?.cancel();
    assertEquals(escape.status, 400);
    // A symlink staying inside the root still serves.
    const alias = await (await fetch(`${httpBase}/alias.txt`)).text();
    assert(alias.includes("fake cockpit index"));
  } finally {
    await relay.shutdown();
  }
});

Deno.test("startRelay rejects a bad served dir with a labeled error", async () => {
  // Before binding anything, so nothing leaks: a typo'd path names the
  // offending option, and a file (realpath-able, would 404 everything) is
  // rejected too.
  await assertRejects(
    () => startRelay({ cockpitDir: "/no/such/dir" }),
    Error,
    "cockpitDir does not exist: /no/such/dir",
  );
  await assertRejects(
    () => startRelay({ cockpitDir: fileURLToPath(import.meta.url) }),
    Error,
    "cockpitDir is not a directory",
  );
});

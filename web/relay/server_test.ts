// In-process loopback e2e: startRelay + Deno's own WebTransport client as
// both robot and viewer. Deno's client CAN receive relay-initiated uni
// streams (verified; the 2.6.10 incoming-uni bug is server-side receive
// only), so this covers the full forwarding path without a browser.
import { assert, assertEquals, assertRejects } from "@std/assert";
import { fileURLToPath } from "node:url";
import {
  CONTROL_CHANNEL,
  ControlFrameReader,
  DataFrameStreamReader,
  decodeDatagram,
  encodeControlFrame,
  encodeDataFrame,
  encodeDatagram,
  type FrameHeader,
  MAX_CONTROL_PAYLOAD_BYTES,
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

/** Robot control carrier messages: @control frames arriving back-to-back on
 * the one relay-opened uni stream (payloads reuse the datagram encoding). */
function robotControl(wt: WebTransport): () => Promise<Msg> {
  const nextFrame = frameQueue(wt);
  return async () => {
    const { header, payload } = await nextFrame();
    assertEquals(header.ch, CONTROL_CHANNEL);
    const msg = decodeDatagram(payload);
    assert(msg !== null, "undecodable @control payload");
    return msg;
  };
}

async function sendRobotFrame(robot: WebTransport, header: FrameHeader, payload: Uint8Array) {
  const stream = await robot.createBidirectionalStream();
  const writer = stream.writable.getWriter();
  await writer.write(encodeDataFrame(header, payload));
  await writer.close(); // FIN is delayed by Deno (bug 2); the relay reads by byte count
}

/** v5 robot hello: an @control frame (datagram-encoded payload) on a fresh
 * one-shot bidi stream; replies stay datagrams. */
async function sendRobotHello(robot: WebTransport, msg: Msg) {
  await sendRobotFrame(
    robot,
    { ch: CONTROL_CHANNEL, seq: 0, ts: 0.5, delivery: "reliable" },
    encodeDatagram(msg),
  );
}

/** Next message of type `t`, skipping interleaved messages of other types. */
async function nextOfType(next: () => Promise<Msg>, t: Msg["t"], what: string): Promise<Msg> {
  let msg: Msg;
  do {
    msg = await within(next(), what);
  } while (msg.t !== t);
  return msg;
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
    await sendRobotHello(robot, {
      t: "hello",
      v: PROTOCOL_VERSION,
      role: "robot",
      robot: ROBOT,
      manifest: {
        version: 1,
        channels: [{ ch: "color_image", encoding: "jpeg.v1", delivery: "latest", maxHz: 100 }],
      },
    });
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

    // Big frames on purpose. The old stream-per-frame policy wedged the
    // stalled viewer on *stream* credit (~100 uni streams), so 8 KB frames
    // were enough; a persistent stream wedges on the connection's *byte*
    // window instead, which no realistic frame count of 8 KB would fill.
    const payload = new Uint8Array(128 * 1024).fill(9);
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
    /** The other one: highest acceptance count. */
    const fetchHealthy = async () => {
      const stats = await (await fetch(`${httpBase}/api/stats`)).json();
      const viewers = stats.perViewer as {
        channels: Record<string, { aborted: number; queued: number; sent: number }>;
      }[];
      return viewers
        .map((v) => v.channels.color_image)
        .filter((c) => c !== undefined)
        .sort((a, b) => b.sent - a.sent)[0];
    };

    // Pump frames until the stalled viewer's watchdog has reset its stream.
    // Latest now packs frames onto one persistent stream
    // (LatestPersistentChannel), so onset is no longer "~100 uni streams of
    // credit exhausted" but "one write parked past the reset threshold": the
    // stalled viewer's write blocks once its flow-control window fills, and
    // ~2 s later the watchdog resets the stream and counts ONE abort (the
    // counter means streams reset, not ticks waited).
    //
    // The budget is therefore small and deliberate: this harness spends one
    // robot-leg bidi stream per frame and quinn grants ~100, so a run that
    // never trips the watchdog must fail on the assertion below rather than
    // pump itself into a RangeError that says nothing about the relay.
    let lastSeq = 0;
    let stalledStats = await fetchStalled();
    for (let seq = 1; seq <= 95; seq++) {
      await sendRobotFrame(
        robot,
        { ch: "color_image", seq, ts: seq / 100, delivery: "latest" },
        payload,
      );
      lastSeq = seq;
      if (seq % 10 === 0) {
        stalledStats = await fetchStalled();
        if (stalledStats !== undefined && stalledStats.aborted >= 1) break;
      }
      await new Promise((resolve) => setTimeout(resolve, 40));
    }
    assert(stalledStats !== undefined, "stalled viewer never got a policy");
    assert(stalledStats.aborted >= 1, `expected an aborted reset, got ${stalledStats.aborted}`);
    assert(stalledStats.queued <= 1, `latest queue must stay 0|1, got ${stalledStats.queued}`);

    // The healthy viewer kept receiving fresh frames the whole time: drain its
    // queue and require a seq from the era after the stalled viewer wedged.
    let newest = 0;
    const drainUntil = Date.now() + 8000;
    while (newest < lastSeq - 10 && Date.now() < drainUntil) {
      newest = Math.max(newest, (await within(healthyFrames(), "healthy frame")).header.seq);
    }
    assert(newest >= lastSeq - 10, `healthy viewer stalled at seq ${newest} of ${lastSeq}`);

    // And the watchdog told the two viewers apart rather than firing on both:
    // the healthy viewer's writes settled every time, so no reap tick ever
    // found one outstanding. This is the half that makes `aborted` a usable
    // stalled-viewer gauge in /api/stats instead of a general noise counter.
    round.lastInflight = await fetchHealthyInflight();
    const healthyCh = await fetchHealthy();
    assertEquals(healthyCh?.aborted, 0, "the healthy viewer must not look stalled");
    assert((healthyCh?.sent ?? 0) > (stalledStats?.sent ?? 0), "healthy viewer fell behind");
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
  const robotCtrl = robotControl(robot);

  await t.step(
    "robot hello (@control stream frame) -> welcome + carrier baseline subs",
    async () => {
      await sendRobotHello(robot, {
        t: "hello",
        v: PROTOCOL_VERSION,
        role: "robot",
        robot: ROBOT,
        manifest: MANIFEST,
      });
      assertEquals(await within(robotDatagrams(), "robot welcome"), {
        t: "welcome",
        v: PROTOCOL_VERSION,
      });
      // Registration always forces a baseline snapshot (the empty set included)
      // as the first frame of the relay-opened robot control carrier.
      assertEquals(await within(robotCtrl(), "carrier baseline subs"), {
        t: "subs",
        chs: [],
        n: 1,
      });
    },
  );

  await t.step("a repeated identical hello re-sends welcome (lost-welcome healing)", async () => {
    await sendRobotHello(robot, {
      t: "hello",
      v: PROTOCOL_VERSION,
      role: "robot",
      robot: ROBOT,
      manifest: MANIFEST,
    });
    assertEquals(await nextOfType(robotDatagrams, "welcome", "second welcome"), {
      t: "welcome",
      v: PROTOCOL_VERSION,
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
      subs = await within(robotCtrl(), "subs snapshot");
    } while (subs.t === "subs" && subs.chs.length < 2);
    assert(subs.t === "subs");
    assertEquals(subs.chs, ["color_image", "odom"]);
  });

  await t.step("rapid sub/unsub churn converges to the exact final set", async () => {
    // Every message below flips the active set, so each produces exactly one
    // snapshot; the ordered carrier must deliver all 12 with the final set
    // last (nothing lost, nothing reordered).
    for (let i = 0; i < 5; i++) {
      await controlWriter.write(encodeControlFrame({ t: "unsub", ch: "odom" }));
      await controlWriter.write(encodeControlFrame({ t: "sub", ch: "odom" }));
    }
    await controlWriter.write(encodeControlFrame({ t: "unsub", ch: "color_image" }));
    await controlWriter.write(encodeControlFrame({ t: "sub", ch: "color_image" }));
    let subs: Msg | null = null;
    for (let i = 0; i < 12; i++) subs = await within(robotCtrl(), `churn snapshot ${i}`);
    assert(subs !== null && subs.t === "subs");
    assertEquals(subs.chs, ["color_image", "odom"]);
    // The relay's own view agrees (lastChs feeds perRobot.subs).
    const stats = await (await fetch(`${httpBase}/api/stats`)).json();
    assertEquals(stats.perRobot[ROBOT.id].subs, ["color_image", "odom"]);
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

  /** Expect one robot hello attempt to be rejected with `code` + close. */
  async function expectRobotReject(
    name: string,
    code: string,
    send: (wt: WebTransport) => Promise<void>,
  ) {
    const wt = new WebTransport(`${relay.wtUrl}/robot`, certOpts(relay.certHash));
    await within(wt.ready, `${name} connect`);
    const datagrams = datagramQueue(wt.datagrams.readable);
    await send(wt);
    const err = await within(datagrams(), `${name} error`);
    assertEquals(err.t, "error");
    assertEquals((err as { code: string }).code, code);
    await within(wt.closed.catch(() => {}), `${name} close`);
  }

  await t.step("robot hello without robot{} -> missing_robot_id + close", async () => {
    await expectRobotReject(
      "bare",
      "missing_robot_id",
      (wt) => sendRobotHello(wt, { t: "hello", v: PROTOCOL_VERSION, role: "robot" }),
    );
  });

  await t.step("robot hello with an invalid manifest -> invalid_manifest + close", async () => {
    await expectRobotReject("dup-manifest", "invalid_manifest", (wt) =>
      sendRobotHello(wt, {
        t: "hello",
        v: PROTOCOL_VERSION,
        role: "robot",
        robot: { id: "dup-bot", name: "Dup Bot", model: "test" },
        manifest: { version: 1, channels: [CHANNELS[0], CHANNELS[0]] },
      }));
  });

  await t.step("manifest with a reserved @-channel -> invalid_manifest + close", async () => {
    await expectRobotReject("reserved-ch", "invalid_manifest", (wt) =>
      sendRobotHello(wt, {
        t: "hello",
        v: PROTOCOL_VERSION,
        role: "robot",
        robot: { id: "reserved-bot", name: "Reserved Bot", model: "test" },
        manifest: {
          version: 1,
          channels: [{ ch: "@sneaky", encoding: "jpeg.v1", delivery: "latest", maxHz: 15.5 }],
        },
      }));
  });

  await t.step("stream hello with the previous protocol version -> error + close", async () => {
    // A v4 bridge would still hello over datagrams (covered below); this
    // pins the version gate on the new @control path itself.
    await expectRobotReject("old-version", "version_mismatch", (wt) =>
      sendRobotHello(wt, {
        t: "hello",
        v: 1,
        role: "robot",
        robot: { id: "old-bot", name: "Old Bot", model: "test" },
      }));
  });

  await t.step("robot datagram hello (any version) -> version_mismatch + close", async () => {
    // v5 moved the robot hello onto @control stream frames; a datagram hello
    // is how a v4-or-older bridge announces itself and must fail loudly.
    for (const v of [1, PROTOCOL_VERSION]) {
      await expectRobotReject(`datagram-hello-v${v}`, "version_mismatch", async (wt) => {
        const writer = wt.datagrams.writable.getWriter();
        await writer.write(encodeDatagram({
          t: "hello",
          v,
          role: "robot",
          robot: { id: "dg-bot", name: "Datagram Bot", model: "test" },
        }));
        writer.releaseLock();
      });
    }
  });

  await t.step("@control with an invalid header field -> invalid_control + close", async () => {
    // The reserved-channel classification keys off the raw ch, so a control
    // frame with a malformed header (even one carrying a valid hello
    // payload) fails closed instead of passing as ordinary pre-hello data.
    await expectRobotReject("bad-header-control", "invalid_control", (wt) =>
      sendRobotFrame(
        wt,
        { ch: CONTROL_CHANNEL, seq: 0, ts: 0.5, delivery: "bogus" } as unknown as FrameHeader,
        encodeDatagram({
          t: "hello",
          v: PROTOCOL_VERSION,
          role: "robot",
          robot: { id: "bad-header-bot", name: "Bad Header Bot", model: "test" },
        }),
      ));
  });

  await t.step("garbage @control payload before hello -> invalid_control + close", async () => {
    await expectRobotReject("garbage-control", "invalid_control", (wt) =>
      sendRobotFrame(
        wt,
        { ch: CONTROL_CHANNEL, seq: 0, ts: 0.5, delivery: "reliable" },
        new TextEncoder().encode("{not json"),
      ));
  });

  await t.step("unknown reserved channel before hello -> invalid_control + close", async () => {
    await expectRobotReject("future-control", "invalid_control", (wt) =>
      sendRobotFrame(
        wt,
        { ch: "@future", seq: 0, ts: 0.5, delivery: "reliable" },
        encodeDatagram({ t: "ping", n: 1, ts: 1.5 }),
      ));
  });

  await t.step("oversized @control payload -> control_too_large + close", async () => {
    await expectRobotReject("oversized-control", "control_too_large", (wt) =>
      sendRobotFrame(
        wt,
        { ch: CONTROL_CHANNEL, seq: 0, ts: 0.5, delivery: "reliable" },
        new Uint8Array(MAX_CONTROL_PAYLOAD_BYTES + 1),
      ));
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

  // The steps below register extra short-lived robots, whose robots pushes
  // land in the main viewer's control queue; they run last so the queue- and
  // stats-sensitive steps above stay deterministic.

  await t.step("hello mutating identity or manifest -> hello_mismatch + close", async () => {
    const wt = new WebTransport(`${relay.wtUrl}/robot`, certOpts(relay.certHash));
    await within(wt.ready, "mut connect");
    const datagrams = datagramQueue(wt.datagrams.readable);
    const hello: Msg = {
      t: "hello",
      v: PROTOCOL_VERSION,
      role: "robot",
      robot: { id: "mut-bot", name: "Mut Bot", model: "test" },
      manifest: MANIFEST,
    };
    await sendRobotHello(wt, hello);
    await nextOfType(datagrams, "welcome", "mut welcome");
    await sendRobotHello(wt, {
      ...hello,
      robot: { id: "mut-bot", name: "Renamed Bot", model: "test" },
    });
    const err = await nextOfType(datagrams, "error", "mut error");
    assertEquals((err as { code: string }).code, "hello_mismatch");
    await within(wt.closed.catch(() => {}), "mut close");
  });

  await t.step("data frames before hello are dropped and counted", async () => {
    const wt = new WebTransport(`${relay.wtUrl}/robot`, certOpts(relay.certHash));
    await within(wt.ready, "late connect");
    const datagrams = datagramQueue(wt.datagrams.readable);
    const before = (await (await fetch(`${httpBase}/api/stats`)).json()).framesFromUnregistered ??
      0;
    await sendRobotFrame(
      wt,
      { ch: "odom", seq: 1, ts: 1.5, delivery: "reliable" },
      new Uint8Array([1]),
    );
    let after = before;
    for (let i = 0; i < 80 && after <= before; i++) {
      await new Promise((resolve) => setTimeout(resolve, 50));
      after = (await (await fetch(`${httpBase}/api/stats`)).json()).framesFromUnregistered;
    }
    assert(after > before, "pre-hello data frame was not counted");
    // The session survived the stray frame: a hello still registers.
    await sendRobotHello(wt, {
      t: "hello",
      v: PROTOCOL_VERSION,
      role: "robot",
      robot: { id: "late-bot", name: "Late Bot", model: "test" },
    });
    await nextOfType(datagrams, "welcome", "late welcome");
    wt.close();
  });

  await t.step("a manifest far beyond the old datagram budget registers", async () => {
    const wt = new WebTransport(`${relay.wtUrl}/robot`, certOpts(relay.certHash));
    await within(wt.ready, "fat connect");
    const datagrams = datagramQueue(wt.datagrams.readable);
    const fatCtrl = robotControl(wt);
    // Ids long enough that the full 40-channel subs snapshot also exceeds
    // the old datagram budget, not just the hello.
    const fatChs = Array.from({ length: 40 }, (_, i) => `ch_${i}_${"pad".repeat(8)}`);
    const fatManifest: RobotManifest = {
      version: 1,
      channels: fatChs.map((ch, i) => ({
        ch,
        encoding: "pose.json.v1",
        delivery: "reliable",
        maxHz: 10.5,
        params: { note: `padding for channel ${i} so the hello dwarfs a datagram` },
      })),
    };
    const hello: Msg = {
      t: "hello",
      v: PROTOCOL_VERSION,
      role: "robot",
      robot: { id: "fat-bot", name: "Fat Bot", model: "test" },
      manifest: fatManifest,
    };
    assert(encodeDatagram(hello).byteLength > 1200, "fat hello must exceed the datagram budget");
    await sendRobotHello(wt, hello);
    await nextOfType(datagrams, "welcome", "fat welcome");

    // A watching viewer receives the fat manifest verbatim.
    const fatViewer = new WebTransport(`${relay.wtUrl}/viewer`, certOpts(relay.certHash));
    await within(fatViewer.ready, "fat viewer connect");
    const stream = await fatViewer.createBidirectionalStream();
    const writer = stream.writable.getWriter();
    const next = controlQueue(stream.readable);
    await writer.write(encodeControlFrame({ t: "hello", v: PROTOCOL_VERSION, role: "viewer" }));
    await writer.write(encodeControlFrame({ t: "watch", robotId: "fat-bot" }));
    let msg: Msg;
    do {
      msg = await within(next(), "fat manifest reply");
    } while (msg.t !== "manifest");
    assertEquals(msg.manifest, fatManifest);

    // Subscribing to every channel produces a snapshot far beyond the old
    // datagram budget; the carrier must deliver it whole and exact.
    for (const ch of fatChs) await writer.write(encodeControlFrame({ t: "sub", ch }));
    let subs: Msg;
    do {
      subs = await within(fatCtrl(), "fat subs snapshot");
    } while (subs.t === "subs" && subs.chs.length < fatChs.length);
    assert(subs.t === "subs");
    assertEquals(subs.chs, [...fatChs].sort());
    assert(
      encodeDatagram(subs).byteLength > 1200,
      "fat snapshot must exceed the old datagram budget",
    );
    fatViewer.close();
    wt.close();
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

Deno.test({
  name: "relay serves /sdk.js with local CORS and no-cache",
  sanitizeOps: false,
  sanitizeResources: false,
}, async () => {
  const sdkDir = fileURLToPath(new URL("./testdata/fake_sdk_dist", import.meta.url));
  const relay = await startRelay({ port: 0, sdkDir });
  const httpBase = `http://127.0.0.1:${relay.httpPort}`;
  try {
    const sdk = await fetch(`${httpBase}/sdk.js`);
    assertEquals(sdk.status, 200);
    assertEquals(sdk.headers.get("content-type"), "application/javascript");
    assertEquals(sdk.headers.get("access-control-allow-origin"), "*");
    assertEquals(sdk.headers.get("cache-control"), "no-cache");
    assert((await sdk.text()).includes("fake sdk bundle"));
    // The discovery endpoints carry the local wildcard too, so a page on
    // another loopback origin (Vite dev server) can bootstrap.
    for (const path of ["/api/info", "/api/stats"]) {
      const res = await fetch(`${httpBase}${path}`);
      assertEquals(res.headers.get("access-control-allow-origin"), "*", path);
      await res.body?.cancel();
    }
  } finally {
    await relay.shutdown();
  }
});

Deno.test({
  name: "missing sdk bundle yields a build hint, never cockpit HTML",
  sanitizeOps: false,
  sanitizeResources: false,
}, async () => {
  const cockpitDir = fileURLToPath(new URL("./testdata/fake_cockpit", import.meta.url));
  const relay = await startRelay({ port: 0, cockpitDir });
  const httpBase = `http://127.0.0.1:${relay.httpPort}`;
  try {
    const sdk = await fetch(`${httpBase}/sdk.js`);
    assertEquals(sdk.status, 404);
    assertEquals(sdk.headers.get("access-control-allow-origin"), "*");
    const body = await sdk.text();
    assert(body.includes("sdk bundle not built"));
    // An HTML fallback imported as a module would be a baffling syntax error
    // in the consumer page; the hint must win over the static root.
    assert(!body.includes("fake cockpit index"));
  } finally {
    await relay.shutdown();
  }
});

Deno.test({
  name: "serve-dir replaces the cockpit root; api and sdk keep precedence",
  sanitizeOps: false,
  sanitizeResources: false,
}, async () => {
  const cockpitDir = fileURLToPath(new URL("./testdata/fake_cockpit", import.meta.url));
  const sdkDir = fileURLToPath(new URL("./testdata/fake_sdk_dist", import.meta.url));
  const serveDir = fileURLToPath(new URL("./testdata/fake_user_dir", import.meta.url));
  const relay = await startRelay({ port: 0, cockpitDir, sdkDir, serveDir });
  const httpBase = `http://127.0.0.1:${relay.httpPort}`;
  try {
    const index = await (await fetch(`${httpBase}/`)).text();
    assert(index.includes("fake user index"));
    assert(!index.includes("fake cockpit index"));
    // JavaScript from the user root is module-importable: the strict module
    // MIME type (also for .mjs) plus the local CORS wildcard, so a page on
    // another local origin can import it by absolute URL.
    for (const path of ["/page.js", "/module.mjs"]) {
      const mod = await fetch(`${httpBase}${path}`);
      assertEquals(mod.status, 200, path);
      assertEquals(mod.headers.get("content-type"), "application/javascript", path);
      assertEquals(mod.headers.get("access-control-allow-origin"), "*", path);
      await mod.body?.cancel();
    }
    // Decoy files in the user dir must not shadow the relay's own routes.
    const info = await (await fetch(`${httpBase}/api/info`)).json();
    assert(typeof info.wtUrl === "string");
    const sdk = await (await fetch(`${httpBase}/sdk.js`)).text();
    assert(sdk.includes("fake sdk bundle"));
    assert(!sdk.includes("decoy"));
    // The traversal and symlink-escape guards apply to the user root too.
    const traverse = await fetch(`${httpBase}//etc/passwd`);
    await traverse.body?.cancel();
    assertEquals(traverse.status, 400);
    const escape = await fetch(`${httpBase}/escape.txt`);
    await escape.body?.cancel();
    assertEquals(escape.status, 400);
  } finally {
    await relay.shutdown();
  }
});

Deno.test("startRelay refuses a non-loopback host without the unsafe override", async () => {
  // Before binding anything: the local trust model (wildcard CORS, no auth,
  // optional user files) must not silently reach a LAN address.
  await assertRejects(
    () => startRelay({ host: "0.0.0.0" }),
    Error,
    "host 0.0.0.0 is not loopback",
  );
});

Deno.test({
  name: "the unsafe override binds a non-loopback host explicitly",
  sanitizeOps: false,
  sanitizeResources: false,
}, async () => {
  const relay = await startRelay({ host: "0.0.0.0", port: 0, unsafeNonLoopback: true });
  try {
    const res = await fetch(`http://127.0.0.1:${relay.httpPort}/api/info`);
    assertEquals(res.status, 200);
    await res.body?.cancel();
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
  await assertRejects(
    () => startRelay({ serveDir: "/no/such/dir" }),
    Error,
    "serveDir does not exist: /no/such/dir",
  );
  await assertRejects(
    () => startRelay({ serveDir: fileURLToPath(import.meta.url) }),
    Error,
    "serveDir is not a directory",
  );
  await assertRejects(
    () => startRelay({ sdkDir: "/no/such/dir" }),
    Error,
    "sdkDir does not exist: /no/such/dir",
  );
});

// The property a persistent latest stream has to earn: a viewer that reads
// slower than the robot produces must fall only BOUNDEDLY behind. This is the
// measurement the persistent-stream design was originally shipped without.
//
// Why it is not obvious: once writer.write() resolves, the frame is only
// accepted into QUIC's send buffer, not delivered. Frames ride one ordered
// stream, so a backlog inside that buffer cannot be superseded the way
// #pending can - the slow reader has to receive the stale bytes before it
// reaches the fresh ones. If nothing bounded that buffer, the viewer's
// displayed frame would age without limit while the relay reported zero
// drops. What actually bounds it is QUIC flow control: the window fills,
// write() stops resolving, #pending starts superseding (drops climb), and the
// stall watchdog resets the stream and discards the backlog.
Deno.test({
  name: "a slow reader falls only boundedly behind on a latest channel",
  sanitizeOps: false,
  sanitizeResources: false,
}, async () => {
  const relay = await startRelay({ port: 0 });
  const clients: WebTransport[] = [];
  try {
    const robot = new WebTransport(`${relay.wtUrl}/robot`, certOpts(relay.certHash));
    clients.push(robot);
    await within(robot.ready, "robot connect");
    const robotDatagrams = datagramQueue(robot.datagrams.readable);
    await sendRobotHello(robot, {
      t: "hello",
      v: PROTOCOL_VERSION,
      role: "robot",
      robot: ROBOT,
      channels: CHANNELS,
      manifest: MANIFEST,
    });
    await nextOfType(robotDatagrams, "welcome", "robot welcome");

    const viewer = new WebTransport(`${relay.wtUrl}/viewer`, certOpts(relay.certHash));
    clients.push(viewer);
    await within(viewer.ready, "viewer connect");
    const control = await viewer.createBidirectionalStream();
    const cw = control.writable.getWriter();
    const nextControl = controlQueue(control.readable);
    await cw.write(encodeControlFrame({ t: "hello", v: PROTOCOL_VERSION, role: "viewer" }));
    await cw.write(encodeControlFrame({ t: "watch", robotId: ROBOT.id }));
    await cw.write(encodeControlFrame({ t: "sub", ch: "color_image" }));
    await nextOfType(nextControl, "manifest", "viewer manifest");

    // The slow reader: pulls from the relay's stream on a slack timer, far
    // slower than the robot publishes, recording the newest seq it has seen.
    let newestSeen = -1;
    let framesSeen = 0;
    (async () => {
      for await (const stream of viewer.incomingUnidirectionalStreams) {
        (async () => {
          const frames = new DataFrameStreamReader();
          const reader = (stream as ReadableStream<Uint8Array>).getReader();
          while (true) {
            const { value, done } = await reader.read();
            if (value && value.byteLength) {
              for (const f of frames.push(value)) {
                if (f.header.ch !== "color_image") continue;
                framesSeen++;
                if (f.header.seq > newestSeen) newestSeen = f.header.seq;
              }
            }
            if (done) break;
            await new Promise((r) => setTimeout(r, 120)); // deliberately slow
          }
        })().catch(() => {});
      }
    })().catch(() => {});

    // ~32 KB is a realistic 640x360 JPEG frame; 30 fps for ~9 s.
    const payload = new Uint8Array(32 * 1024).fill(7);
    const TOTAL = 270;
    let worstLag = 0;
    for (let seq = 1; seq <= TOTAL; seq++) {
      await sendRobotFrame(
        robot,
        { ch: "color_image", seq, ts: seq / 30, delivery: "latest" },
        payload,
      );
      if (newestSeen >= 0) worstLag = Math.max(worstLag, seq - newestSeen);
      await new Promise((r) => setTimeout(r, 33));
    }
    const deadline = Date.now() + 6000;
    while (newestSeen < TOTAL - 60 && Date.now() < deadline) {
      await new Promise((r) => setTimeout(r, 100));
    }

    console.log(
      `[slow-reader] worst lag ${worstLag} frames, final lag ${TOTAL - newestSeen}, ` +
        `received ${framesSeen} of ${TOTAL}`,
    );
    assert(framesSeen > 0, "the slow reader never received a frame at all");
    // The bound: a reader ~4x too slow must not end up minutes behind. 120
    // frames at 30 fps is 4 s of video - generous next to the unbounded growth
    // this test exists to rule out.
    assert(
      worstLag < 120,
      `slow reader fell ${worstLag} frames behind (unbounded backlog?); saw ${framesSeen}`,
    );
    assert(
      TOTAL - newestSeen < 120,
      `slow reader ended ${TOTAL - newestSeen} frames behind the newest`,
    );
  } finally {
    for (const c of clients) {
      try {
        c.close();
      } catch { /* already gone */ }
    }
    await relay.shutdown();
  }
});

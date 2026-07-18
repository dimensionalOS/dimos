// In-process loopback e2e: startRelay + Deno's own WebTransport client as
// both robot and viewer. Deno's client CAN receive relay-initiated uni
// streams (verified; the 2.6.10 incoming-uni bug is server-side receive
// only), so this covers the full forwarding path without a browser.
import { assert, assertEquals } from "@std/assert";
import {
  type ChannelSpec,
  ControlFrameReader,
  decodeDatagram,
  encodeControlFrame,
  encodeDataFrame,
  encodeDatagram,
  type FrameHeader,
  type Msg,
  PROTOCOL_VERSION,
  type RobotInfo,
} from "@dimos/shared";
import { readDataFrameBytes } from "./forward.ts";
import { startRelay } from "./server.ts";

const ROBOT: RobotInfo = { id: "deno-bot", name: "Deno Bot", model: "test" };
const CHANNELS: ChannelSpec[] = [
  { ch: "color_image", encoding: "jpeg.v1", delivery: "latest", maxHz: 15.5 },
  { ch: "odom", encoding: "pose.json.v1", delivery: "reliable", maxHz: 20.5 },
];

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

/** Collect forwarded data frames arriving on relay-initiated uni streams. */
function frameQueue(
  wt: WebTransport,
): () => Promise<{ header: FrameHeader; payload: Uint8Array }> {
  const queue: { header: FrameHeader; payload: Uint8Array }[] = [];
  const waiters: ((f: { header: FrameHeader; payload: Uint8Array }) => void)[] = [];
  (async () => {
    for await (const stream of wt.incomingUnidirectionalStreams) {
      readDataFrameBytes(stream)
        .then((bytes) => {
          const headerLen = new DataView(bytes.buffer, bytes.byteOffset).getUint32(0, true);
          const header = JSON.parse(
            new TextDecoder().decode(bytes.subarray(8, 8 + headerLen)),
          ) as FrameHeader;
          const payload = bytes.subarray(8 + headerLen);
          const waiter = waiters.shift();
          if (waiter) waiter({ header, payload });
          else queue.push({ header, payload });
        })
        .catch(() => {});
    }
  })().catch(() => {});
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

Deno.test({
  name: "relay loopback e2e",
  // QUIC endpoint + WT sessions keep background ops alive past shutdown();
  // their teardown is asynchronous in Deno 2.6.
  sanitizeOps: false,
  sanitizeResources: false,
}, async (t) => {
  const relay = await startRelay({ port: 0 });
  const httpBase = `http://127.0.0.1:${relay.httpPort}`;

  await t.step("/api/info matches the handle and the debug page serves", async () => {
    const info = await (await fetch(`${httpBase}/api/info`)).json();
    assertEquals(info, {
      wtUrl: `${relay.wtUrl}/viewer`,
      certHash: relay.certHash,
      v: PROTOCOL_VERSION,
    });
    assert(relay.wtUrl.startsWith("https://127.0.0.1:"), relay.wtUrl);
    const page = await (await fetch(`${httpBase}/debug.html`)).text();
    assert(page.includes("DimOS relay debug"));
    const index = await (await fetch(`${httpBase}/`)).text();
    assert(index.includes("DimOS relay debug"));
    // Traversal probes. The client/URL parser normalizes these two away from
    // the tree before the guard sees them, so they 404 on absence.
    for (const path of ["/../etc/passwd", "/%2e%2e/etc/passwd"]) {
      const res = await fetch(`${httpBase}${path}`);
      await res.body?.cancel();
      assertEquals(res.status, 404, path);
    }
    // These survive normalization and must be rejected by the containment
    // check: a leading "//" makes new URL() jump to the filesystem root, and
    // encoded slashes let a "../" escape reassemble after decoding.
    for (const path of ["//etc/passwd", "/..%2f..%2f..%2f..%2fetc%2fpasswd"]) {
      const res = await fetch(`${httpBase}${path}`);
      await res.body?.cancel();
      assertEquals(res.status, 400, path);
    }
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
        manifest: { channels: CHANNELS },
      }),
    );
    assertEquals(await within(robotDatagrams(), "robot welcome"), {
      t: "welcome",
      v: PROTOCOL_VERSION,
    });
    // Registration pushes the forced baseline snapshot (no viewers yet).
    assertEquals(await within(robotDatagrams(), "baseline subs"), {
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
      channels: CHANNELS,
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

// ViewerSession unit tests over a fake WebTransport: the send-priority
// scheme (control > reliable telemetry > latest video) must hold for every
// stream the relay creates or replies on. Wire-level scheduling itself is
// quinn's job; here we pin the orders the relay assigns.
import { assert, assertEquals } from "@std/assert";
import {
  CONTROL_CHANNEL,
  ControlFrameReader,
  decodeDatagram,
  encodeControlFrame,
  encodeDataFrame,
  encodeDatagram,
  type FrameHeader,
  type Msg,
  PROTOCOL_VERSION,
} from "@dimos/shared";
import { Auth } from "./auth.ts";
import { Registry } from "./registry.ts";
import {
  CONTROL_SEND_ORDER,
  LATEST_SEND_ORDER,
  RELIABLE_SEND_ORDER,
  RobotSession,
  ViewerSession,
} from "./session.ts";

function tick(): Promise<void> {
  return new Promise((resolve) => setTimeout(resolve, 0));
}

Deno.test("send-order bands: control above reliable above latest", () => {
  assert(CONTROL_SEND_ORDER > RELIABLE_SEND_ORDER);
  assert(RELIABLE_SEND_ORDER > LATEST_SEND_ORDER);
});

Deno.test("the sink opens one stream per call at the order it is given", async () => {
  const orders: (number | undefined)[] = [];
  const wt = {
    createUnidirectionalStream(opts?: WebTransportSendStreamOptions) {
      orders.push(opts?.sendOrder);
      return Promise.resolve(new WritableStream<Uint8Array>());
    },
  } as unknown as WebTransport;
  const session = new ViewerSession(wt, 1, new Registry());
  await session.sink.openStream(LATEST_SEND_ORDER);
  await session.sink.openStream(RELIABLE_SEND_ORDER);
  assertEquals(orders, [LATEST_SEND_ORDER, RELIABLE_SEND_ORDER]);
});

Deno.test("sink: a stream create stuck on credit is counted, then kicked", async () => {
  // The viewer's incoming-stream credit is exhausted: the create never
  // resolves. The stall is counted (and forces a GC, which returns the
  // credit of disposed streams), and the viewer is kicked at the deadline.
  let closed: string | null = null;
  const wt = {
    createUnidirectionalStream: () => new Promise<WebTransportSendStream>(() => {}),
    close(info?: WebTransportCloseInfo) {
      closed = info?.reason ?? "";
    },
  } as unknown as WebTransport;
  const registry = new Registry();
  const session = new ViewerSession(wt, 1, registry, null, { creditStallMs: 20, creditKickMs: 60 });
  const open = session.sink.openStream(LATEST_SEND_ORDER);
  await new Promise((resolve) => setTimeout(resolve, 40));
  const stats = () => registry.stats() as { streamCreditStalls: number };
  assertEquals(stats().streamCreditStalls, 1);
  assertEquals(closed, null);
  await new Promise((resolve) => setTimeout(resolve, 40));
  assertEquals(closed, "stream credit exhausted");
  assertEquals(stats().streamCreditStalls, 1); // once per create
  void open.catch(() => {});
});

Deno.test("the viewer control stream is raised to CONTROL_SEND_ORDER", async () => {
  const written: Uint8Array[] = [];
  const controlWritable = new WritableStream<Uint8Array>({
    write(chunk) {
      written.push(chunk);
    },
  });
  const wt = {
    closed: new Promise<void>(() => {}),
    datagrams: {
      readable: new ReadableStream<Uint8Array>(),
      writable: new WritableStream<Uint8Array>(),
    },
    incomingBidirectionalStreams: new ReadableStream({
      start(controller) {
        controller.enqueue({
          readable: new ReadableStream<Uint8Array>({
            start(controller) {
              controller.enqueue(
                encodeControlFrame({ t: "hello", v: PROTOCOL_VERSION, role: "viewer" }),
              );
            },
          }),
          writable: controlWritable,
        });
      },
    }),
  } as unknown as WebTransport;
  const session = new ViewerSession(wt, 1, new Registry());
  session.start();
  for (let i = 0; i < 100 && written.length < 2; i++) await tick();
  const frames = new ControlFrameReader();
  const replies = written.flatMap((chunk) => frames.push(chunk));
  assertEquals(replies[0], { t: "welcome", v: PROTOCOL_VERSION } as Msg);
  // The viewer opened this stream, so its send half started at the default
  // order 0; the session must have raised it above reliable telemetry.
  assertEquals(
    (controlWritable as unknown as { sendOrder?: number }).sendOrder,
    CONTROL_SEND_ORDER,
  );
});

// Viewer auth (T12d): with an Auth, a hello must carry a known viewer token.
const VIEWER_TOKEN = "viewer-token-0123456789abcdef";
const AUTH = new Auth(new Map(), new Map([["paul", VIEWER_TOKEN]]));

/** One viewer hello over a fake WebTransport: the control replies, whether
 * the session closed the transport, and the session itself. */
async function viewerHandshake(
  hello: Msg,
  auth: Auth | null,
): Promise<{ replies: Msg[]; closed: boolean; session: ViewerSession }> {
  const written: Uint8Array[] = [];
  let closed = false;
  const wt = {
    closed: new Promise<void>(() => {}),
    close: () => {
      closed = true;
    },
    datagrams: {
      readable: new ReadableStream<Uint8Array>(),
      writable: new WritableStream<Uint8Array>(),
    },
    incomingBidirectionalStreams: new ReadableStream({
      start(controller) {
        controller.enqueue({
          readable: new ReadableStream<Uint8Array>({
            start(controller) {
              controller.enqueue(encodeControlFrame(hello));
            },
          }),
          writable: new WritableStream<Uint8Array>({
            write(chunk) {
              written.push(chunk);
            },
          }),
        });
      },
    }),
  } as unknown as WebTransport;
  const session = new ViewerSession(wt, 1, new Registry(), auth);
  session.start();
  for (let i = 0; i < 100 && written.length < 1; i++) await tick();
  // A rejection closes 250 ms later (closeAfterFlush); wait it out so the
  // timer does not outlive the test.
  await new Promise((resolve) => setTimeout(resolve, 300));
  const frames = new ControlFrameReader();
  return { replies: written.flatMap((chunk) => frames.push(chunk)), closed, session };
}

Deno.test("viewer auth: a hello without or with a wrong token is rejected and closed", async () => {
  const missing = await viewerHandshake({ t: "hello", v: PROTOCOL_VERSION, role: "viewer" }, AUTH);
  assertEquals(missing.replies, [
    { t: "error", code: "auth_failed", message: "missing viewer token" } as Msg,
  ]);
  assert(missing.closed);
  assertEquals(missing.session.greeted, false);
  const wrong = await viewerHandshake(
    { t: "hello", v: PROTOCOL_VERSION, role: "viewer", token: "wrong-token-0123456789abcdef" },
    AUTH,
  );
  assertEquals(wrong.replies, [
    { t: "error", code: "auth_failed", message: "invalid viewer token" } as Msg,
  ]);
  assert(wrong.closed);
});

Deno.test("viewer auth: the right token is greeted and names the session", async () => {
  const ok = await viewerHandshake(
    { t: "hello", v: PROTOCOL_VERSION, role: "viewer", token: VIEWER_TOKEN },
    AUTH,
  );
  assertEquals(ok.replies[0], { t: "welcome", v: PROTOCOL_VERSION } as Msg);
  assertEquals(ok.session.name, "paul");
  assertEquals(ok.closed, false);
});

Deno.test("viewer auth: version and role mismatches keep their own codes; auth off is unchanged", async () => {
  const old = await viewerHandshake({ t: "hello", v: 99, role: "viewer" }, AUTH);
  assertEquals((old.replies[0] as { code: string }).code, "version_mismatch");
  const robot = await viewerHandshake({ t: "hello", v: PROTOCOL_VERSION, role: "robot" }, AUTH);
  assertEquals((robot.replies[0] as { code: string }).code, "role_mismatch");
  const off = await viewerHandshake({ t: "hello", v: PROTOCOL_VERSION, role: "viewer" }, null);
  assertEquals(off.replies[0], { t: "welcome", v: PROTOCOL_VERSION } as Msg);
  assertEquals(off.session.name, null);
});

// ---------------------------------------------------------------------------
// RobotSession frame ordering over fake QUIC streams: reliable frames
// dispatch in accept order up to a bounded hold; latest frames never wait.

const PREAMBLE = new Uint8Array([0x40, 0x41, 0x00]);

/** One fake robot bidi stream whose bytes the test releases in pieces. */
class FakeRobotStream {
  readonly readable: ReadableStream<Uint8Array>;
  readonly writable = new WritableStream<Uint8Array>();
  #controller!: ReadableByteStreamController;
  constructor() {
    this.readable = new ReadableStream({
      type: "bytes",
      start: (c) => {
        this.#controller = c;
      },
    });
  }
  push(bytes: Uint8Array): void {
    this.#controller.enqueue(bytes);
  }
  end(): void {
    this.#controller.close();
  }
  fail(): void {
    this.#controller.error(new Error("reset"));
  }
}

function robotFrame(ch: string, seq: number, delivery: "latest" | "reliable"): Uint8Array {
  const header: FrameHeader = { ch, seq, ts: seq + 0.5, delivery };
  return encodeDataFrame(header, new Uint8Array(32).fill(seq));
}

function robotHarness(timing: { probeMs?: number } = {}) {
  let enqueueBidi!: (s: FakeRobotStream) => void;
  const conn = {
    incomingBidirectionalStreams: new ReadableStream<FakeRobotStream>({
      start(c) {
        enqueueBidi = (s) => c.enqueue(s);
      },
    }),
  } as unknown as Deno.QuicConn;
  /** Datagrams the session sent, decoded. */
  const sentDatagrams: Msg[] = [];
  let pushDatagram!: (bytes: Uint8Array) => void;
  let closed: string | null = null;
  const wt = {
    closed: new Promise<void>(() => {}),
    datagrams: {
      readable: new ReadableStream<Uint8Array>({
        start(c) {
          pushDatagram = (bytes) => c.enqueue(bytes);
        },
      }),
      writable: new WritableStream<Uint8Array>({
        write(chunk) {
          const msg = decodeDatagram(chunk);
          if (msg !== null) sentDatagrams.push(msg);
        },
      }),
    },
    createUnidirectionalStream: () => Promise.resolve(new WritableStream<Uint8Array>()),
    close: (info?: WebTransportCloseInfo) => {
      closed = info?.reason ?? "";
    },
  } as unknown as WebTransport;
  const registry = new Registry();
  const dispatched: number[] = [];
  registry.onRobotFrame = (_peer, _bytes, header) => {
    dispatched.push(header!.seq);
  };
  const session = new RobotSession(wt, conn, registry, null, timing);
  session.start();
  /** Accept a stream carrying `bytes` whole, or just its first `split` bytes. */
  const accept = (bytes: Uint8Array, split?: number): FakeRobotStream => {
    const stream = new FakeRobotStream();
    enqueueBidi(stream);
    const whole = new Uint8Array([...PREAMBLE, ...bytes]);
    if (split === undefined) {
      stream.push(whole);
      stream.end();
    } else {
      stream.push(whole.slice(0, split));
    }
    return stream;
  };
  return {
    session,
    registry,
    dispatched,
    accept,
    sentDatagrams,
    pushDatagram: (msg: Msg) => pushDatagram(encodeDatagram(msg)),
    closedReason: () => closed,
  };
}

async function settle(): Promise<void> {
  for (let i = 0; i < 5; i++) await tick();
}

Deno.test("robot frames: a stream's frames route in order as they complete; streams are independent", async () => {
  const { session, registry, dispatched, accept } = robotHarness();
  accept(encodeDataFrame(
    { ch: CONTROL_CHANNEL, seq: 0, ts: 0.5, delivery: "reliable" },
    encodeDatagram({
      t: "hello",
      v: PROTOCOL_VERSION,
      role: "robot",
      robot: { id: "r1", name: "r1", model: "t" },
    }),
  ));
  await settle();
  assert(session.info !== null, "registered");

  // A persistent reliable stream: frame 1 arrives whole, frame 2 in pieces.
  const frame1 = robotFrame("odom", 1, "reliable");
  const frame2 = robotFrame("odom", 2, "reliable");
  const both = new Uint8Array([...frame1, ...frame2]);
  const cut = PREAMBLE.length + frame1.byteLength + 5;
  const odom = accept(both, cut);
  await settle();
  assertEquals(dispatched, [1]);
  // A one-shot latest stream completes meanwhile and does not wait.
  accept(robotFrame("cam", 3, "latest"));
  await settle();
  assertEquals(dispatched, [1, 3]);
  odom.push(new Uint8Array([...PREAMBLE, ...both]).slice(cut));
  await settle();
  assertEquals(dispatched, [1, 3, 2]);
  odom.push(robotFrame("odom", 4, "reliable"));
  await settle();
  assertEquals(dispatched, [1, 3, 2, 4]);

  // A stream reset mid-frame loses the partial and is counted.
  const dead = accept(robotFrame("odom", 5, "reliable"), 2);
  dead.fail();
  await settle();
  assertEquals(dispatched, [1, 3, 2, 4]);
  assertEquals((registry.stats() as { framesIncomplete: number }).framesIncomplete, 1);
});

Deno.test("robot probe: a pong answers it, silence times it out, supersede closes", async () => {
  const h = robotHarness({ probeMs: 40 });
  const answered = h.session.probe();
  await settle();
  const ping = h.sentDatagrams.find((m) => m.t === "ping");
  assert(ping !== undefined && ping.t === "ping", "the probe is a ping datagram");
  h.pushDatagram({ t: "pong", n: ping.n, ts: ping.ts });
  assertEquals(await answered, true);

  const silent = h.session.probe();
  await settle();
  h.pushDatagram({ t: "pong", n: 999, ts: 0.5 }); // some other ping's pong: not ours
  assertEquals(await silent, false);

  h.session.supersede();
  assertEquals(h.session.closed, "superseded");
  const error = h.sentDatagrams.find((m) => m.t === "error");
  assertEquals(error?.t === "error" ? error.code : null, "robot_superseded");
  assertEquals(await h.session.probe(), false); // a closing session is not alive
  await new Promise((resolve) => setTimeout(resolve, 300)); // closeAfterFlush
  assertEquals(h.closedReason(), "superseded");
});

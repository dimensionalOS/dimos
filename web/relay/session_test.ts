// ViewerSession unit tests over a fake WebTransport: the send-priority
// scheme (control > reliable telemetry > latest video) must hold for every
// stream the relay creates or replies on. Wire-level scheduling itself is
// quinn's job; here we pin the orders the relay assigns.
import { assert, assertEquals } from "@std/assert";
import {
  CONTROL_CHANNEL,
  ControlFrameReader,
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
  ORDER_HOLD_MAX,
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

function robotHarness() {
  let enqueueBidi!: (s: FakeRobotStream) => void;
  const conn = {
    incomingBidirectionalStreams: new ReadableStream<FakeRobotStream>({
      start(c) {
        enqueueBidi = (s) => c.enqueue(s);
      },
    }),
  } as unknown as Deno.QuicConn;
  const wt = {
    closed: new Promise<void>(() => {}),
    datagrams: {
      readable: new ReadableStream<Uint8Array>(),
      writable: new WritableStream<Uint8Array>(),
    },
    createUnidirectionalStream: () => Promise.resolve(new WritableStream<Uint8Array>()),
    close: () => {},
  } as unknown as WebTransport;
  const registry = new Registry();
  const dispatched: number[] = [];
  registry.onRobotFrame = (_peer, _bytes, header) => {
    dispatched.push(header!.seq);
  };
  const session = new RobotSession(wt, conn, registry, null);
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
  return { session, registry, dispatched, accept };
}

async function settle(): Promise<void> {
  for (let i = 0; i < 5; i++) await tick();
}

Deno.test("robot frames: reliable frames wait for an incomplete earlier one, latest do not", async () => {
  const { session, dispatched, accept } = robotHarness();
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

  // Frame 1's payload is held back after its header; 2 and 3 complete.
  const frame1 = robotFrame("odom", 1, "reliable");
  const headerEnd = PREAMBLE.length + 8 +
    JSON.stringify({ ch: "odom", seq: 1, ts: 1.5, delivery: "reliable" }).length;
  const straggler = accept(frame1, headerEnd);
  accept(robotFrame("odom", 2, "reliable"));
  accept(robotFrame("cam", 3, "latest"));
  await settle();
  assertEquals(dispatched, [3]); // the latest frame passed; 2 waits for 1

  straggler.push(new Uint8Array([...PREAMBLE, ...frame1]).slice(headerEnd));
  straggler.end();
  await settle();
  assertEquals(dispatched, [3, 1, 2]);

  // A stream that fails before its header leaves the queue.
  const dead = accept(robotFrame("odom", 4, "reliable"), 2);
  accept(robotFrame("odom", 5, "reliable"));
  await settle();
  assertEquals(dispatched, [3, 1, 2]);
  dead.fail();
  await settle();
  assertEquals(dispatched, [3, 1, 2, 5]);
});

Deno.test("robot frames: the hold behind a straggler is bounded", async () => {
  const { dispatched, accept } = robotHarness();
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
  const frame1 = robotFrame("odom", 1, "reliable");
  const headerEnd = PREAMBLE.length + 8 +
    JSON.stringify({ ch: "odom", seq: 1, ts: 1.5, delivery: "reliable" }).length;
  const straggler = accept(frame1, headerEnd);
  for (let seq = 2; seq <= ORDER_HOLD_MAX; seq++) accept(robotFrame("odom", seq, "reliable"));
  await settle();
  assertEquals(dispatched, []); // ORDER_HOLD_MAX - 1 held
  accept(robotFrame("odom", ORDER_HOLD_MAX + 1, "reliable"));
  await settle();
  // The hold overflowed: every completed frame went out in order, the
  // straggler still pending.
  assertEquals(dispatched, Array.from({ length: ORDER_HOLD_MAX }, (_, i) => i + 2));
  straggler.push(new Uint8Array([...PREAMBLE, ...frame1]).slice(headerEnd));
  straggler.end();
  await settle();
  assertEquals(dispatched.at(-1), 1); // late, as before ordering
});

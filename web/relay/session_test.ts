// ViewerSession unit tests over a fake WebTransport: the send-priority
// scheme (control > reliable telemetry > latest video) must hold for every
// stream the relay creates or replies on. Wire-level scheduling itself is
// quinn's job; here we pin the orders the relay assigns.
import { assert, assertEquals, assertRejects } from "@std/assert";
import { ControlFrameReader, encodeControlFrame, type Msg, PROTOCOL_VERSION } from "@dimos/shared";
import { Registry } from "./registry.ts";
import {
  CONTROL_SEND_ORDER,
  RELIABLE_SEND_ORDER,
  STALE_STREAM_ERROR_CODE,
  ViewerSession,
} from "./session.ts";

function tick(): Promise<void> {
  return new Promise((resolve) => setTimeout(resolve, 0));
}

Deno.test("send-order bands: control above reliable above every latest stream", () => {
  assert(CONTROL_SEND_ORDER > RELIABLE_SEND_ORDER);
  assert(RELIABLE_SEND_ORDER > 0); // latest streams are all negative
});

Deno.test("the sink assigns the documented sendOrder to each stream kind", async () => {
  const orders: (number | undefined)[] = [];
  const wt = {
    createUnidirectionalStream(opts?: WebTransportSendStreamOptions) {
      orders.push(opts?.sendOrder);
      return Promise.resolve(new WritableStream<Uint8Array>());
    },
  } as unknown as WebTransport;
  const session = new ViewerSession(wt, 1, new Registry());
  await session.sink.sendFrame(new Uint8Array([1])).done;
  await session.sink.sendFrame(new Uint8Array([2])).done;
  await session.sink.openStream();
  // Latest streams count down (oldest first, all below reliable).
  assertEquals(orders, [-1, -2, RELIABLE_SEND_ORDER]);
});

// ---------------------------------------------------------------------------
// FrameSend contract of the real sink: latest streams are never closed (a
// closed stream cannot be aborted) and abort() works in every phase - before
// the stream exists, during a stalled write, and after acceptance.

interface SpyStream {
  stream: WritableStream<Uint8Array>;
  writes: Uint8Array[];
  aborts: unknown[];
  closed: boolean;
}

function spyStream(opts: { stallWrite?: boolean } = {}): SpyStream {
  const spy: SpyStream = { writes: [], aborts: [], closed: false, stream: null! };
  spy.stream = new WritableStream<Uint8Array>({
    write(chunk, controller) {
      spy.writes.push(chunk);
      if (!opts.stallWrite) return Promise.resolve();
      // A stalled transport write must still honor an abort: the spec only
      // invokes the underlying abort() after the pending write settles, and
      // controller.signal is how the transport learns to give up.
      return new Promise<void>((_, reject) => {
        controller.signal.addEventListener("abort", () => reject(controller.signal.reason));
      });
    },
    abort(reason) {
      spy.aborts.push(reason);
    },
    close() {
      spy.closed = true;
    },
  });
  return spy;
}

function sinkOver(create: () => Promise<WritableStream<Uint8Array>>) {
  const wt = { createUnidirectionalStream: create } as unknown as WebTransport;
  return new ViewerSession(wt, 1, new Registry()).sink;
}

Deno.test("sink: acceptance never closes the stream; abort after acceptance resets it", async () => {
  const spy = spyStream();
  const sink = sinkOver(() => Promise.resolve(spy.stream));
  const send = sink.sendFrame(new Uint8Array([7]));
  await send.done;
  assertEquals(spy.writes.length, 1);
  assertEquals(spy.closed, false); // never FIN'd: the stream must stay abortable

  send.abort();
  send.abort(); // idempotent
  await tick();
  assertEquals(spy.aborts.length, 1);
  const reason = spy.aborts[0] as WebTransportError;
  assertEquals(reason.streamErrorCode, STALE_STREAM_ERROR_CODE);
  assertEquals(send.aborted, true);
});

Deno.test("sink: abort during a pending create rejects done and resets on arrival", async () => {
  const spy = spyStream();
  let releaseCreate!: () => void;
  const sink = sinkOver(() =>
    new Promise((resolve) => {
      releaseCreate = () => resolve(spy.stream);
    })
  );
  const send = sink.sendFrame(new Uint8Array([7]));
  send.abort();
  await assertRejects(() => send.done); // rejects while the create still hangs

  releaseCreate(); // credit finally granted: the zombie stream must be reset
  await tick();
  assertEquals(spy.aborts.length, 1);
  assertEquals(spy.writes.length, 0); // the stale frame was never written
});

Deno.test("sink: abort during a stalled write rejects done and resets the stream", async () => {
  const spy = spyStream({ stallWrite: true });
  const sink = sinkOver(() => Promise.resolve(spy.stream));
  const send = sink.sendFrame(new Uint8Array([7]));
  await tick();
  assertEquals(spy.writes.length, 1); // write started, wedged in flow control

  send.abort();
  await assertRejects(() => send.done);
  await tick();
  assertEquals(spy.aborts.length, 1);
});

Deno.test("sink: a create failure rejects done without the aborted flag", async () => {
  const sink = sinkOver(() => Promise.reject(new Error("connection lost")));
  const send = sink.sendFrame(new Uint8Array([7]));
  await assertRejects(() => send.done, Error, "connection lost");
  assertEquals(send.aborted, false); // policies must treat this as a real failure
});

Deno.test("sink: supersede before the create resolves late-binds the payload", async () => {
  const spy = spyStream();
  let releaseCreate!: () => void;
  const sink = sinkOver(() =>
    new Promise((resolve) => {
      releaseCreate = () => resolve(spy.stream);
    })
  );
  const send = sink.sendFrame(new Uint8Array([1]));
  assertEquals(send.supersede(new Uint8Array([2])), true);
  assertEquals(send.supersede(new Uint8Array([3])), true); // repeatable while wedged

  releaseCreate(); // credit finally granted: the write picks up the latest binding
  await send.done;
  assertEquals(spy.writes, [new Uint8Array([3])]);
});

Deno.test("sink: supersede is refused once the write starts and after abort", async () => {
  const stalled = spyStream({ stallWrite: true });
  const stalledSink = sinkOver(() => Promise.resolve(stalled.stream));
  const inWrite = stalledSink.sendFrame(new Uint8Array([1]));
  await tick();
  assertEquals(stalled.writes.length, 1); // write started, wedged in flow control
  assertEquals(inWrite.supersede(new Uint8Array([2])), false);
  inWrite.abort(); // release the stalled write
  await assertRejects(() => inWrite.done);

  const wedgedSink = sinkOver(() => new Promise(() => {})); // create never resolves
  const wedged = wedgedSink.sendFrame(new Uint8Array([3]));
  wedged.abort(); // the send's stream is condemned: supersede must refuse too
  assertEquals(wedged.supersede(new Uint8Array([4])), false);
  await assertRejects(() => wedged.done);
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

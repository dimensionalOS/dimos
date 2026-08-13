// ViewerSession unit tests over a fake WebTransport: the send-priority
// scheme (control > reliable telemetry > latest video) must hold for every
// stream the relay creates or replies on. Wire-level scheduling itself is
// quinn's job; here we pin the orders the relay assigns.
import { assert, assertEquals } from "@std/assert";
import { ControlFrameReader, encodeControlFrame, type Msg, PROTOCOL_VERSION } from "@dimos/shared";
import { Registry } from "./registry.ts";
import { CONTROL_SEND_ORDER, RELIABLE_SEND_ORDER, ViewerSession } from "./session.ts";

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
  await session.sink.sendFrame(new Uint8Array([1]));
  await session.sink.sendFrame(new Uint8Array([2]));
  await session.sink.openStream();
  // Latest streams count down (oldest first, all below reliable).
  assertEquals(orders, [-1, -2, RELIABLE_SEND_ORDER]);
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

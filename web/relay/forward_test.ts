import { assertEquals, assertRejects } from "@std/assert";
import { encodeDataFrame, type FrameHeader } from "@dimos/shared";
import {
  type FrameWriter,
  LatestChannel,
  parseRobotFrameHeader,
  readDataFrameBytes,
  readWebTransportPreamble,
  ReliableChannel,
  type ViewerSink,
} from "./forward.ts";

class FakeSink implements ViewerSink {
  sent: Uint8Array[] = [];
  kicked: string | null = null;
  streamsOpened = 0;
  streamsAborted = 0;
  auto: boolean;
  manualOpen = false;
  #waiters: { resolve: () => void; reject: (e: Error) => void }[] = [];
  #openWaiters: (() => void)[] = [];

  constructor(auto = true) {
    this.auto = auto;
  }

  sendFrame(bytes: Uint8Array): Promise<void> {
    return this.#write(bytes);
  }

  openStream(): Promise<FrameWriter> {
    this.streamsOpened++;
    const writer: FrameWriter = {
      write: (bytes: Uint8Array) => this.#write(bytes),
      abort: () => {
        this.streamsAborted++;
        return Promise.resolve();
      },
    };
    if (!this.manualOpen) return Promise.resolve(writer);
    return new Promise((resolve) => this.#openWaiters.push(() => resolve(writer)));
  }

  #write(bytes: Uint8Array): Promise<void> {
    this.sent.push(bytes);
    if (this.auto) return Promise.resolve();
    return new Promise<void>((resolve, reject) => this.#waiters.push({ resolve, reject }));
  }

  release(n = 1): void {
    while (n-- > 0) this.#waiters.shift()?.resolve();
  }

  rejectWrite(): void {
    this.#waiters.shift()?.reject(new Error("stream aborted"));
  }

  releaseOpen(): void {
    this.#openWaiters.shift()?.();
  }

  kick(reason: string): void {
    this.kicked = reason;
  }
}

function tick(): Promise<void> {
  return new Promise((resolve) => setTimeout(resolve, 0));
}

function frame(n: number): Uint8Array {
  return new Uint8Array([n]);
}

function dataFrame(ch: string, seq: number, delivery: "latest" | "reliable"): Uint8Array {
  const header: FrameHeader = { ch, seq, ts: seq + 0.5, delivery };
  return encodeDataFrame(header, new Uint8Array([seq]));
}

Deno.test("latest: newest replaces pending while a write is in flight", async () => {
  const sink = new FakeSink(false);
  const ch = new LatestChannel(sink);
  ch.offer(frame(1)); // begins writing
  ch.offer(frame(2)); // parked in the pending slot
  ch.offer(frame(3)); // replaces frame 2
  await tick();
  assertEquals(sink.sent.length, 1);
  sink.release();
  await tick();
  assertEquals(sink.sent, [frame(1), frame(3)]);
  sink.release();
  await tick();
  assertEquals(ch.sent, 2);
  assertEquals(ch.dropped, 1);
  assertEquals(ch.queued(), 0);
  assertEquals(sink.kicked, null);
});

Deno.test("latest: the final frame is always eventually delivered", async () => {
  const sink = new FakeSink(false);
  const ch = new LatestChannel(sink);
  for (let i = 0; i < 100; i++) ch.offer(frame(i));
  sink.release(100);
  await tick();
  sink.release(100);
  await tick();
  assertEquals(sink.sent.length, 2); // first + newest, everything between dropped
  assertEquals(sink.sent[1], frame(99));
  assertEquals(ch.dropped, 98);
});

Deno.test("latest: fast sink delivers everything", async () => {
  const sink = new FakeSink();
  const ch = new LatestChannel(sink);
  for (let i = 0; i < 5; i++) {
    ch.offer(frame(i));
    await tick();
  }
  assertEquals(sink.sent.length, 5);
  assertEquals(ch.dropped, 0);
});

Deno.test("reliable: FIFO order, zero drops", async () => {
  const sink = new FakeSink(false);
  const ch = new ReliableChannel(sink);
  for (let i = 0; i < 10; i++) ch.offer(frame(i));
  await tick(); // the persistent stream opens before the first write
  for (let i = 0; i < 10; i++) {
    sink.release();
    await tick();
  }
  assertEquals(sink.sent, Array.from({ length: 10 }, (_, i) => frame(i)));
  assertEquals(ch.sent, 10);
  assertEquals(ch.dropped, 0);
  assertEquals(sink.kicked, null);
  // All frames rode one persistent stream (per-frame streams starve Firefox's
  // uni-stream credit; see ReliableChannel docs).
  assertEquals(sink.streamsOpened, 1);
});

Deno.test("reliable: drain pauses reuse the same persistent stream", async () => {
  const sink = new FakeSink();
  const ch = new ReliableChannel(sink);
  ch.offer(frame(1));
  await tick();
  assertEquals(sink.sent, [frame(1)]); // FIFO idle: the writing loop ended
  ch.offer(frame(2));
  await tick();
  assertEquals(sink.sent, [frame(1), frame(2)]);
  assertEquals(sink.streamsOpened, 1);
});

Deno.test("reliable: queue overflow kicks the viewer", async () => {
  const sink = new FakeSink(false);
  const ch = new ReliableChannel(sink);
  // 1 in flight + 64 queued is accepted; the next one overflows.
  for (let i = 0; i < 66 && sink.kicked === null; i++) ch.offer(frame(i));
  await tick();
  assertEquals(sink.kicked, "reliable channel overflow");
});

Deno.test("latest: dispose during an in-flight write discards the pending frame", async () => {
  const sink = new FakeSink(false);
  const ch = new LatestChannel(sink);
  ch.offer(frame(1)); // write in flight
  ch.offer(frame(2)); // parked in the pending slot
  ch.dispose();
  assertEquals(ch.queued(), 0);
  sink.release(); // the in-flight write completes after disposal
  await tick();
  ch.offer(frame(3)); // ignored after dispose
  await tick();
  assertEquals(sink.sent, [frame(1)]);
  assertEquals(sink.kicked, null);
});

Deno.test("latest: an in-flight write failing after dispose does not kick", async () => {
  const sink = new FakeSink(false);
  const ch = new LatestChannel(sink);
  ch.offer(frame(1));
  ch.dispose();
  sink.rejectWrite(); // the stream died with the disposal
  await tick();
  assertEquals(sink.kicked, null);
});

Deno.test("reliable: dispose aborts the persistent stream and discards the queue", async () => {
  const sink = new FakeSink(false);
  const ch = new ReliableChannel(sink);
  ch.offer(frame(1));
  await tick(); // stream opened; first write in flight
  ch.offer(frame(2));
  ch.offer(frame(3));
  ch.dispose();
  assertEquals(ch.queued(), 0);
  assertEquals(sink.streamsAborted, 1);
  sink.rejectWrite(); // the aborted stream fails the in-flight write
  await tick();
  ch.offer(frame(4)); // ignored after dispose
  await tick();
  assertEquals(sink.sent, [frame(1)]);
  assertEquals(sink.streamsOpened, 1);
  assertEquals(sink.kicked, null);
});

Deno.test("reliable: dispose while the persistent stream opens still releases it", async () => {
  const sink = new FakeSink(false);
  sink.manualOpen = true;
  const ch = new ReliableChannel(sink);
  ch.offer(frame(1)); // drain is awaiting openStream
  ch.dispose();
  sink.releaseOpen();
  await tick();
  assertEquals(sink.streamsAborted, 1);
  assertEquals(sink.sent, []);
  assertEquals(sink.kicked, null);
});

// Lost-wakeup regression: the vulnerable window is between the drain loop
// observing an empty queue and #writing clearing in its .finally, a fixed
// number of microtasks later. Sweeping the depth at which the next offer
// lands hits the window whatever the engine's await scheduling (depths 1-2
// on Deno 2.6, where the stranded frame stayed queued forever).
Deno.test("latest: a frame offered in the drain-completion window still sends", async () => {
  for (let depth = 0; depth < 5; depth++) {
    const sink = new FakeSink(false);
    const ch = new LatestChannel(sink);
    ch.offer(frame(1));
    sink.release();
    for (let i = 0; i < depth; i++) await Promise.resolve();
    ch.offer(frame(2));
    await tick();
    sink.release();
    await tick();
    assertEquals([ch.sent, ch.queued()], [2, 0], `depth ${depth}`);
  }
});

Deno.test("reliable: a frame offered in the drain-completion window still sends", async () => {
  for (let depth = 0; depth < 5; depth++) {
    const sink = new FakeSink(false);
    const ch = new ReliableChannel(sink);
    ch.offer(frame(1));
    await tick(); // persistent stream opened; first write in flight
    sink.release();
    for (let i = 0; i < depth; i++) await Promise.resolve();
    ch.offer(frame(2));
    await tick();
    sink.release();
    await tick();
    assertEquals([ch.sent, ch.queued()], [2, 0], `depth ${depth}`);
  }
});

Deno.test("parseRobotFrameHeader accepts valid frames and rejects junk", () => {
  const good = dataFrame("odom", 4, "reliable");
  assertEquals(parseRobotFrameHeader(good), { ch: "odom", seq: 4, ts: 4.5, delivery: "reliable" });
  assertEquals(parseRobotFrameHeader(new Uint8Array([1, 2, 3])), null); // shorter than a header
  assertEquals(parseRobotFrameHeader(new Uint8Array(16)), null); // headerLen=0 -> JSON.parse("")
  const badDelivery = encodeDataFrame(
    { ch: "cam", seq: 1, ts: 1.5, delivery: "bogus" } as unknown as FrameHeader,
    new Uint8Array([7]),
  );
  assertEquals(parseRobotFrameHeader(badDelivery), null);
});

function byteStream(...chunks: Uint8Array[]): ReadableStream<Uint8Array> {
  // type "bytes" so BYOB readers work, like a real QUIC receive stream.
  return new ReadableStream({
    type: "bytes",
    start(controller) {
      for (const chunk of chunks) controller.enqueue(chunk);
      controller.close();
    },
  });
}

// Stream type 0x41 exceeds the 1-byte varint range, so on the wire it is the
// 2-byte varint [0x40, 0x41] (what aioquic sends).

Deno.test("preamble: consumed so the data frame parses from the remainder", async () => {
  const frame = dataFrame("cam", 3, "latest");
  // enqueue a copy: the byte stream detaches the chunk's buffer on read
  const rs = byteStream(new Uint8Array([0x40, 0x41, 0x00]), frame.slice());
  assertEquals(await readWebTransportPreamble(rs), 0);
  assertEquals(await readDataFrameBytes(rs), frame);
});

Deno.test("preamble: multi-byte varint session id", async () => {
  // session id 0x14c as a 2-byte varint (0x40 | 0x01, 0x4c)
  const rs = byteStream(new Uint8Array([0x40, 0x41, 0x41, 0x4c]));
  assertEquals(await readWebTransportPreamble(rs), 0x14c);
});

Deno.test("preamble: non-WebTransport stream type rejects", async () => {
  await assertRejects(
    () => readWebTransportPreamble(byteStream(new Uint8Array([0x17, 0x00]))),
    Error,
    "not a WebTransport data stream",
  );
});

Deno.test("preamble: stream ending mid-preamble rejects", async () => {
  await assertRejects(
    () => readWebTransportPreamble(byteStream(new Uint8Array([0x40, 0x41]))),
    Error,
    "stream ended mid-preamble",
  );
});

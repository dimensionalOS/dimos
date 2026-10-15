import { assert, assertEquals, assertRejects, assertStrictEquals } from "@std/assert";
import {
  CONTROL_CHANNEL,
  encodeDataFrame,
  type FrameHeader,
  MAX_CONTROL_PAYLOAD_BYTES,
} from "@dimos/shared";
import {
  ACK_BUDGET,
  ACK_BUDGET_MAX,
  ACK_TIMEOUT_MS,
  ControlPayloadTooLargeError,
  type FrameWriter,
  IncompleteFrameError,
  LATEST_SEND_ORDER,
  LatestChannel,
  parseRobotFrameHeader,
  Rate,
  readRobotFrames,
  RELIABLE_SEND_ORDER,
  ReliableChannel,
  type RobotFrame,
  TokenBucket,
  type ViewerSink,
} from "./forward.ts";

class FakeSink implements ViewerSink {
  /** Every frame written, in order (both policies). */
  sent: Uint8Array[] = [];
  kicked: string | null = null;
  kicks = 0;
  streamsOpened = 0;
  /** sendOrder of every stream opened, in order. */
  orders: number[] = [];
  streamsAborted = 0;
  auto: boolean;
  manualOpen = false;
  #waiters: { resolve: () => void; reject: (e: Error) => void }[] = [];
  #openWaiters: (() => void)[] = [];

  constructor(auto = true) {
    this.auto = auto;
  }

  #shared: Promise<FrameWriter> | null = null;

  sharedStream(order: number): Promise<FrameWriter> {
    return this.#shared ??= this.openStream(order);
  }

  streamsReleased = 0;
  #idle = new Map<number, FrameWriter[]>();

  releaseStream(order: number, writer: FrameWriter): void {
    this.streamsReleased++;
    this.#idle.set(order, [...(this.#idle.get(order) ?? []), writer]);
  }

  openStream(order: number): Promise<FrameWriter> {
    const idle = this.#idle.get(order)?.pop();
    if (idle !== undefined) return Promise.resolve(idle);
    this.streamsOpened++;
    this.orders.push(order);
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

  /** Accept the oldest unaccepted write. */
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
    this.kicks++;
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
  ch.offer(frame(1), 1);
  await tick(); // the persistent stream opened; frame 1's write is in flight
  ch.offer(frame(2), 2); // parked in the pending slot
  ch.offer(frame(3), 3); // replaces frame 2
  assertEquals(sink.sent, [frame(1)]);
  assertEquals(ch.writing(), true);
  sink.release();
  await tick();
  assertEquals(sink.sent, [frame(1), frame(3)]);
  sink.release();
  await tick();
  assertEquals(ch.sent, 2);
  assertEquals(ch.dropped, 1);
  assertEquals(ch.queued(), 0);
  assertEquals(ch.writing(), false);
  assertEquals(sink.kicked, null);
  assertEquals(sink.streamsOpened, 1); // one stream for the channel's life
});

Deno.test("latest: the final frame is always eventually delivered", async () => {
  const sink = new FakeSink(false);
  const ch = new LatestChannel(sink);
  for (let i = 0; i < 100; i++) ch.offer(frame(i), i);
  await tick(); // the stream opens after the burst: only the newest is left
  sink.release();
  await tick();
  assertEquals(sink.sent, [frame(99)]);
  assertEquals(ch.dropped, 99);
  assertEquals(ch.sent, 1);
});

Deno.test("latest: fast sink with prompt acks delivers everything", async () => {
  const sink = new FakeSink();
  const ch = new LatestChannel(sink);
  for (let i = 0; i < 5; i++) {
    ch.offer(frame(i), i);
    await tick();
    ch.ack(i); // the viewer read it
  }
  assertEquals(sink.sent.length, 5);
  assertEquals(ch.dropped, 0);
  assertEquals(ch.stalled, 0);
  assertEquals(ch.unacked(), 0);
});

Deno.test("reliable: FIFO order, zero drops", async () => {
  const sink = new FakeSink(false);
  const ch = new ReliableChannel(sink);
  for (let i = 0; i < 10; i++) ch.offer(frame(i), i);
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
  ch.offer(frame(1), 1);
  await tick();
  assertEquals(sink.sent, [frame(1)]); // FIFO idle: the writing loop ended
  ch.offer(frame(2), 2);
  await tick();
  assertEquals(sink.sent, [frame(1), frame(2)]);
  assertEquals(sink.streamsOpened, 1);
});

Deno.test("reliable: overflow kicks once and empties the FIFO", async () => {
  const sink = new FakeSink(false);
  const ch = new ReliableChannel(sink);
  // Nothing drains (the persistent stream is still opening), so the 65th
  // queued frame overflows; the channel kicks once, self-disposes, and
  // ignores the rest of the burst instead of re-queueing + re-kicking.
  for (let i = 0; i < 200; i++) ch.offer(frame(i), i);
  await tick();
  assertEquals(sink.kicks, 1);
  assertEquals(sink.kicked, "reliable channel overflow");
  assertEquals(ch.queued(), 0);
  assertEquals([sink.streamsAborted, sink.streamsReleased], [0, 1]); // nothing written before the kick: the stream goes back to the viewer
  ch.offer(frame(200), 200); // still a no-op until transport teardown completes
  assertEquals(ch.queued(), 0);
  assertEquals(sink.kicks, 1);
});

Deno.test("reliable: a lone frame over the byte cap is sent, a backlog behind it kicks", async () => {
  const big = new Uint8Array(17 * 1024 * 1024);
  const alone = new FakeSink();
  new ReliableChannel(alone).offer(big, 1);
  await tick();
  assertEquals(alone.kicked, null);
  assertEquals(alone.sent.length, 1);
  assertStrictEquals(alone.sent[0], big);
  const backlog = new FakeSink(false);
  const ch = new ReliableChannel(backlog);
  ch.offer(big, 1);
  ch.offer(frame(1), 1); // queued behind the unsent big frame: together over the cap
  assertEquals(backlog.kicked, "reliable channel overflow");
  assertEquals(ch.queued(), 0);
});

Deno.test("latest: dispose aborts the stream and discards the pending frame", async () => {
  const sink = new FakeSink(false);
  const ch = new LatestChannel(sink);
  ch.offer(frame(1), 1);
  await tick(); // write in flight
  ch.offer(frame(2), 2); // parked
  ch.dispose();
  assertEquals(ch.queued(), 0);
  assertEquals(sink.streamsAborted, 1);
  sink.rejectWrite(); // the aborted stream fails the in-flight write
  await tick();
  ch.offer(frame(3), 3); // ignored after dispose
  await tick();
  assertEquals(sink.sent, [frame(1)]);
  assertEquals(sink.kicked, null); // disposal failures are not viewer failures
});

Deno.test("latest: dispose while the stream opens still releases it", async () => {
  const sink = new FakeSink(false);
  sink.manualOpen = true;
  const ch = new LatestChannel(sink);
  ch.offer(frame(1), 1); // drain is awaiting openStream
  ch.dispose();
  sink.releaseOpen();
  await tick();
  assertEquals([sink.streamsAborted, sink.streamsReleased], [0, 1]); // nothing written: reusable
  assertEquals(sink.sent, []);
  assertEquals(sink.kicked, null);
});

Deno.test("reliable: dispose aborts the persistent stream and discards the queue", async () => {
  const sink = new FakeSink(false);
  const ch = new ReliableChannel(sink);
  ch.offer(frame(1), 1);
  await tick(); // stream opened; first write in flight
  ch.offer(frame(2), 2);
  ch.offer(frame(3), 3);
  ch.dispose();
  assertEquals(ch.queued(), 0);
  assertEquals(sink.streamsAborted, 1);
  sink.rejectWrite(); // the aborted stream fails the in-flight write
  await tick();
  ch.offer(frame(4), 4); // ignored after dispose
  await tick();
  assertEquals(sink.sent, [frame(1)]);
  assertEquals(sink.streamsOpened, 1);
  assertEquals(sink.kicked, null);
});

Deno.test("reliable: dispose while the persistent stream opens still releases it", async () => {
  const sink = new FakeSink(false);
  sink.manualOpen = true;
  const ch = new ReliableChannel(sink);
  ch.offer(frame(1), 1); // drain is awaiting openStream
  ch.dispose();
  sink.releaseOpen();
  await tick();
  assertEquals([sink.streamsAborted, sink.streamsReleased], [0, 1]); // nothing written: reusable
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
    ch.offer(frame(1), 1);
    await tick(); // stream opened; first write in flight
    sink.release();
    for (let i = 0; i < depth; i++) await Promise.resolve();
    ch.offer(frame(2), 2);
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
    ch.offer(frame(1), 1);
    await tick(); // persistent stream opened; first write in flight
    sink.release();
    for (let i = 0; i < depth; i++) await Promise.resolve();
    ch.offer(frame(2), 2);
    await tick();
    sink.release();
    await tick();
    assertEquals([ch.sent, ch.queued()], [2, 0], `depth ${depth}`);
  }
});

// ---------------------------------------------------------------------------
// The stalled-viewer signal: a write the transport accepts only after
// LATEST_STALL_MS (the viewer's stream window is full) counts as stalled.

function timedChannel(
  sink: FakeSink,
  start = 100_000,
): { ch: LatestChannel; tick(ms: number): void; now(): number } {
  let now = start;
  const ch = new LatestChannel(sink, { now: () => now });
  return { ch, tick: (ms: number) => now += ms, now: () => now };
}

Deno.test("latest: a write accepted late counts as one stall; prompt ones never", async () => {
  const sink = new FakeSink(false);
  const { ch, tick: advance } = timedChannel(sink);
  ch.offer(frame(1), 1);
  await tick(); // write in flight: the viewer is not reading
  ch.offer(frame(2), 2);
  ch.offer(frame(3), 3); // the pending slot sheds while the write waits
  advance(600); // past LATEST_STALL_MS
  assertEquals(ch.stalled, 0); // counted at acceptance, not by polling
  sink.release(); // the viewer resumed
  await tick();
  assertEquals(ch.stalled, 1);
  assertEquals(ch.dropped, 1);
  assertEquals(sink.sent, [frame(1), frame(3)]); // the newest followed at once
  sink.release();
  await tick();
  assertEquals(ch.stalled, 1); // the prompt write is not a stall
  assertEquals(ch.sent, 2);
  assertEquals(sink.kicked, null);
});

Deno.test("latest: the ack budget follows the viewer's delivery rate, between 2 and 10 frames", async () => {
  let now = 1_000_000;
  const sink = new FakeSink();
  const ch = new LatestChannel(sink, { now: () => now });
  assertEquals(ch.budget(), ACK_BUDGET);
  // A viewer acknowledging 25 frames a second over the rate window (the
  // window's edge bucket drops about a tenth, so 20 fps would read as 9).
  for (let seq = 1; seq <= 125; seq++) {
    ch.offer(frame(seq), seq);
    await tick();
    ch.ack(seq);
    now += 40;
  }
  ch.tick(now);
  assertEquals(ch.budget(), ACK_BUDGET_MAX); // ~22 fps x 0.5 s = 11, the cap
  for (let seq = 101; seq <= 112; seq++) {
    ch.offer(frame(seq), seq);
    await tick();
  }
  assertEquals([ch.unacked(), ch.queued()], [ACK_BUDGET_MAX, 1]); // ten in flight, one pending
  now += 6000; // the acks stop: the window empties and the budget falls back
  ch.tick(now);
  assertEquals(ch.budget(), ACK_BUDGET);
});

Deno.test("latest: at most ACK_BUDGET frames await their ack; an ack releases the next", async () => {
  const sink = new FakeSink();
  const ch = new LatestChannel(sink);
  for (let i = 1; i <= 5; i++) {
    ch.offer(frame(i), i);
    await tick();
  }
  // Frames 1 and 2 went out; 3 and 4 were shed by 5, which waits.
  assertEquals(sink.sent, [frame(1), frame(2)]);
  assertEquals([ch.unacked(), ch.queued(), ch.dropped], [ACK_BUDGET, 1, 2]);

  ch.ack(1); // cumulative: frees frame 1's slot
  await tick();
  assertEquals(sink.sent, [frame(1), frame(2), frame(5)]);
  assertEquals(ch.unacked(), 2);
  ch.ack(5); // frees everything at or below 5
  assertEquals(ch.unacked(), 0);
  ch.ack(99); // a stray ack releases nothing and breaks nothing
  assertEquals([ch.sent, ch.ackTimeouts], [3, 0]);
});

Deno.test("latest: the ack timeout releases a frame nobody acknowledged", async () => {
  const sink = new FakeSink();
  const { ch, tick: advance, now } = timedChannel(sink);
  ch.offer(frame(1), 1);
  await tick();
  ch.offer(frame(2), 2);
  await tick();
  ch.offer(frame(3), 3);
  await tick();
  assertEquals(sink.sent, [frame(1), frame(2)]);
  advance(ACK_TIMEOUT_MS - 1);
  ch.tick(now());
  assertEquals(sink.sent.length, 2); // not yet
  advance(1);
  ch.tick(now()); // frames 1 and 2 time out together
  await tick();
  assertEquals(sink.sent, [frame(1), frame(2), frame(3)]);
  assertEquals([ch.ackTimeouts, ch.unacked()], [2, 1]);
});

Deno.test("streams open at their band's sendOrder: reliable above latest", async () => {
  const sink = new FakeSink();
  new LatestChannel(sink).offer(frame(1), 1);
  new ReliableChannel(sink).offer(frame(2), 2);
  await tick();
  assertEquals(sink.orders, [LATEST_SEND_ORDER, RELIABLE_SEND_ORDER]);
  assert(RELIABLE_SEND_ORDER > LATEST_SEND_ORDER);
});

Deno.test("latest: a real transport failure kicks exactly once", async () => {
  const sink = new FakeSink(false);
  const ch = new LatestChannel(sink);
  ch.offer(frame(1), 1);
  await tick();
  ch.offer(frame(2), 2); // parked
  sink.rejectWrite(); // connection died
  await tick();
  assertEquals(sink.kicked, "write failed");
  assertEquals(sink.kicks, 1);
  assertEquals(sink.streamsAborted, 1); // the kick's dispose releases the stream
  ch.offer(frame(3), 3); // ignored after the dispose
  await tick();
  assertEquals(sink.sent, [frame(1)]);
});

Deno.test("rate: bucketed trailing window with idle decay and wraparound", () => {
  const rate = new Rate();
  const t0 = 1_000_000;
  // 10 frames of 1000 B over one second.
  for (let i = 0; i < 10; i++) rate.push(1000, t0 + i * 100);
  const busy = rate.snapshot(t0 + 1000);
  assertEquals(busy.fps, 2); // 10 frames / 5 s window
  assertEquals(busy.bps, 2000);

  // Enough silence slides the burst's first bucket out but keeps the tail.
  const later = rate.snapshot(t0 + 5200);
  assert(later.fps > 0 && later.fps < busy.fps, `partial decay, got ${later.fps}`);

  // Beyond the window everything zeroes.
  assertEquals(rate.snapshot(t0 + 20_000), { fps: 0, bps: 0 });

  // Wraparound: pushes far apart still land in the right buckets.
  rate.push(500, t0 + 30_000);
  assertEquals(rate.snapshot(t0 + 30_000), { fps: 0.2, bps: 100 });
});

Deno.test("token bucket: burst to capacity, continuous refill, no wobble drain", () => {
  const bucket = new TokenBucket(2); // capacity max(1, ceil(2)) = 2
  const t0 = 1_000_000;
  assertEquals(bucket.take(t0), true);
  assertEquals(bucket.take(t0), true);
  assertEquals(bucket.take(t0), false); // burst spent
  // 2/s: half a second buys exactly one token.
  assertEquals(bucket.take(t0 + 499), false);
  assertEquals(bucket.take(t0 + 500), true);
  // A backwards clock must not drain tokens (the failed take at t0+500ms+1
  // refilled nothing; going back 400ms keeps the balance).
  assertEquals(bucket.take(t0 + 100), false);
  // Idle refill clamps at capacity: a long pause buys 2 tokens, not 20.
  assertEquals(bucket.take(t0 + 60_000), true);
  assertEquals(bucket.take(t0 + 60_000), true);
  assertEquals(bucket.take(t0 + 60_000), false);

  // Fractional rates keep at least one token of capacity.
  const slow = new TokenBucket(0.5);
  assertEquals(slow.capacity, 1);
  assertEquals(slow.take(t0), true);
  assertEquals(slow.take(t0 + 1999), false);
  assertEquals(slow.take(t0 + 2000), true);
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
// 2-byte varint [0x40, 0x41] (what aioquic sends), then the session id.
const PREAMBLE = new Uint8Array([0x40, 0x41, 0x00]);

/** Every frame of a stream, and whether the stream ended cleanly. */
async function readAll(rs: ReadableStream<Uint8Array>): Promise<RobotFrame[]> {
  const frames: RobotFrame[] = [];
  await readRobotFrames(rs, (f) => frames.push(f));
  return frames;
}

Deno.test("readRobotFrames: the preamble is consumed and the frame parses", async () => {
  const frame = dataFrame("cam", 3, "latest");
  // enqueue copies: the byte stream detaches the chunk's buffer on read
  const [got] = await readAll(byteStream(PREAMBLE.slice(), frame.slice()));
  assertEquals(got.bytes, frame);
  assertEquals(got.header, { ch: "cam", seq: 3, ts: 3.5, delivery: "latest" });
  assertEquals(got.reserved, false);
});

Deno.test("readRobotFrames: back-to-back frames on one stream, split anywhere", async () => {
  const frames = [1, 2, 3].map((seq) => dataFrame("odom", seq, "reliable"));
  const whole = new Uint8Array([...PREAMBLE, ...frames.flatMap((f) => [...f])]);
  const expected: FrameHeader[] = frames.map((_, i) => ({
    ch: "odom",
    seq: i + 1,
    ts: i + 1.5,
    delivery: "reliable",
  }));
  const one = await readAll(byteStream(whole.slice()));
  assertEquals(one.map((f) => f.header), expected);
  assertEquals(one.map((f) => f.bytes), frames);
  const bytes = Array.from(whole, (b) => new Uint8Array([b]));
  const slow = await readAll(byteStream(...bytes));
  assertEquals(slow.map((f) => f.bytes), frames);
  // A boundary in the middle of a frame's prefix and header.
  const cut = PREAMBLE.length + frames[0].byteLength + 5;
  const split = await readAll(byteStream(whole.slice(0, cut), whole.slice(cut)));
  assertEquals(split.map((f) => f.bytes), frames);
});

Deno.test("readRobotFrames: a frame is handed over before the stream ends", async () => {
  const frame = dataFrame("odom", 1, "reliable");
  const whole = new Uint8Array([...PREAMBLE, ...frame]);
  let controller!: ReadableByteStreamController;
  const rs = new ReadableStream({
    type: "bytes",
    start(c) {
      controller = c;
    },
  });
  const seen: number[] = [];
  const done = readRobotFrames(rs, (f) => seen.push(f.header!.seq));
  controller.enqueue(whole.slice());
  for (let i = 0; i < 5 && seen.length === 0; i++) await tick();
  assertEquals(seen, [1]); // a persistent stream never ends; the frame is out
  controller.enqueue(dataFrame("odom", 2, "reliable"));
  for (let i = 0; i < 5 && seen.length === 1; i++) await tick();
  assertEquals(seen, [1, 2]);
  controller.close();
  await done; // a clean end between frames
});

Deno.test("readRobotFrames: a payload past one buffer grows the frame", async () => {
  const header: FrameHeader = { ch: "cloud", seq: 1, ts: 1.5, delivery: "latest" };
  const frame = encodeDataFrame(header, new Uint8Array(300 * 1024).fill(5));
  const whole = new Uint8Array([...PREAMBLE, ...frame]);
  const chunks: Uint8Array[] = [];
  for (let at = 0; at < whole.byteLength; at += 1200) chunks.push(whole.slice(at, at + 1200));
  const [got] = await readAll(byteStream(...chunks));
  assertEquals(got.bytes, frame);
});

Deno.test("readRobotFrames: a header longer than the first read still parses", async () => {
  const header: FrameHeader = {
    ch: "cam",
    seq: 1,
    ts: 1.5,
    delivery: "latest",
    meta: { pad: "x".repeat(6000) },
  };
  const frame = encodeDataFrame(header, new Uint8Array([1, 2, 3]));
  const [got] = await readAll(byteStream(new Uint8Array([...PREAMBLE, ...frame])));
  assertEquals(got.header?.meta, header.meta);
  assertEquals(got.bytes, frame);
});

Deno.test("readRobotFrames: a malformed header yields null, frame bytes intact", async () => {
  const bad = encodeDataFrame(
    { ch: "cam", seq: 1, ts: 1.5, delivery: "bogus" } as unknown as FrameHeader,
    new Uint8Array([7]),
  );
  const [got] = await readAll(byteStream(PREAMBLE.slice(), bad.slice()));
  assertEquals(got.header, null);
  assertEquals(got.bytes, bad);
  assertEquals(got.reserved, false);
});

Deno.test("readRobotFrames: reserved classification survives an invalid header", async () => {
  // The raw ch decides reserved-ness, so a malformed @-frame still routes to
  // the control rejection path instead of passing as ordinary data.
  const bad = encodeDataFrame(
    { ch: CONTROL_CHANNEL, seq: 1, ts: 1.5, delivery: "bogus" } as unknown as FrameHeader,
    new Uint8Array([7]),
  );
  const [got] = await readAll(byteStream(PREAMBLE.slice(), bad.slice()));
  assertEquals(got.header, null);
  assertEquals(got.reserved, true);
});

Deno.test("readRobotFrames: @-frame payload cap is exact at 64 KiB", async () => {
  const helloHeader: FrameHeader = { ch: CONTROL_CHANNEL, seq: 1, ts: 1.5, delivery: "reliable" };
  const atCap = encodeDataFrame(helloHeader, new Uint8Array(MAX_CONTROL_PAYLOAD_BYTES));
  const [got] = await readAll(byteStream(PREAMBLE.slice(), atCap.slice()));
  assertEquals(got.header?.ch, CONTROL_CHANNEL);
  assertEquals(got.bytes.byteLength, atCap.byteLength);

  const overCap = encodeDataFrame(helloHeader, new Uint8Array(MAX_CONTROL_PAYLOAD_BYTES + 1));
  await assertRejects(
    () => readAll(byteStream(PREAMBLE.slice(), overCap.slice())),
    ControlPayloadTooLargeError,
  );
});

Deno.test("readRobotFrames: the cap keys off the raw ch, not the validated header", async () => {
  // A hostile @-frame with an otherwise-invalid header (bogus delivery) must
  // still be capped before its payload is buffered.
  const sneaky = encodeDataFrame(
    { ch: "@evil", seq: 1, ts: 1.5, delivery: "bogus" } as unknown as FrameHeader,
    new Uint8Array(MAX_CONTROL_PAYLOAD_BYTES + 1),
  );
  await assertRejects(
    () => readAll(byteStream(PREAMBLE.slice(), sneaky.slice())),
    ControlPayloadTooLargeError,
  );
});

Deno.test("readRobotFrames: ordinary channels are not control-capped", async () => {
  const big = encodeDataFrame(
    { ch: "cam", seq: 1, ts: 1.5, delivery: "latest" },
    new Uint8Array(MAX_CONTROL_PAYLOAD_BYTES + 1),
  );
  const [got] = await readAll(byteStream(PREAMBLE.slice(), big.slice()));
  assertEquals(got.header?.ch, "cam");
  assertEquals(got.bytes.byteLength, big.byteLength);
});

Deno.test("readRobotFrames: multi-byte varint session id", async () => {
  // session id 0x14c as a 2-byte varint (0x40 | 0x01, 0x4c)
  const frame = dataFrame("cam", 1, "latest");
  const [got] = await readAll(
    byteStream(new Uint8Array([0x40, 0x41, 0x41, 0x4c]), frame.slice()),
  );
  assertEquals(got.header?.seq, 1);
});

Deno.test("readRobotFrames: non-WebTransport stream type rejects", async () => {
  await assertRejects(
    () => readAll(byteStream(new Uint8Array([0x17, 0x00]))),
    Error,
    "not a WebTransport data stream",
  );
});

Deno.test("readRobotFrames: ending mid-frame is incomplete, between frames is clean", async () => {
  await assertRejects(
    () => readAll(byteStream(new Uint8Array([0x40, 0x41]))),
    Error,
    "stream ended mid-preamble",
  );
  const frame = dataFrame("cam", 1, "latest");
  await assertRejects(
    () => readAll(byteStream(PREAMBLE.slice(), frame.slice(0, frame.byteLength - 1))),
    IncompleteFrameError,
  );
  assertEquals(await readAll(byteStream(PREAMBLE.slice())), []); // nothing sent: clean
  // A reset of an idle persistent stream is routine; mid-frame it is a loss.
  const idleReset = new ReadableStream({
    type: "bytes",
    start(c) {
      c.enqueue(new Uint8Array([...PREAMBLE, ...frame]));
      c.error(new Error("reset"));
    },
  });
  assertEquals((await readAll(idleReset)).length, 1);
  const midReset = new ReadableStream({
    type: "bytes",
    start(c) {
      c.enqueue(new Uint8Array([...PREAMBLE, ...frame.slice(0, 10)]));
      c.error(new Error("reset"));
    },
  });
  await assertRejects(() => readAll(midReset), IncompleteFrameError);
});

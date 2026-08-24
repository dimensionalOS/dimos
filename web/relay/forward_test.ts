import { assert, assertEquals, assertRejects } from "@std/assert";
import { encodeDataFrame, type FrameHeader } from "@dimos/shared";
import {
  type FrameSend,
  type FrameWriter,
  LatestChannel,
  parseRobotFrameHeader,
  Rate,
  readDataFrameBytes,
  readWebTransportPreamble,
  ReliableChannel,
  type ViewerSink,
} from "./forward.ts";

class FakeSend implements FrameSend {
  aborted = false;
  settled = false;
  readonly done: Promise<void>;
  #resolve!: () => void;
  #reject!: (e: Error) => void;

  constructor(
    public bytes: Uint8Array,
    readonly sink: FakeSink,
    /** Position in sink.sent[], captured at sendFrame time. */
    readonly index: number,
    /** Mirrors the real sink: false only while wedged in stream creation. */
    public writeStarted: boolean,
  ) {
    this.done = new Promise<void>((resolve, reject) => {
      this.#resolve = resolve;
      this.#reject = reject;
    });
  }

  accept(): void {
    if (!this.settled) {
      this.writeStarted = true;
      this.settled = true;
      this.#resolve();
    }
  }

  /** The stream got created; the (still unaccepted) write is in progress. */
  beginWrite(): void {
    this.writeStarted = true;
  }

  fail(e: Error): void {
    if (!this.settled) {
      this.settled = true;
      this.#reject(e);
    }
  }

  abort(): void {
    if (this.aborted) return;
    this.aborted = true;
    this.sink.sendsAborted++;
    this.fail(new Error("frame send aborted"));
  }

  supersede(bytes: Uint8Array): boolean {
    if (this.writeStarted || this.aborted) return false;
    this.bytes = bytes;
    // sent[] holds the latest binding, so deep-equals assertions read what
    // would actually go on the wire.
    this.sink.sent[this.index] = bytes;
    return true;
  }
}

class FakeSink implements ViewerSink {
  /** Every frame handed to the sink, in order (latest path). */
  sent: Uint8Array[] = [];
  sends: FakeSend[] = [];
  /** FrameSend aborts (latest streams reset), settled or not. */
  sendsAborted = 0;
  kicked: string | null = null;
  kicks = 0;
  streamsOpened = 0;
  /** Persistent-stream (reliable) aborts. */
  streamsAborted = 0;
  auto: boolean;
  manualOpen = false;
  /** Model sends stuck in createUnidirectionalStream (credit exhausted):
   * their write has not started, so they stay supersedable. */
  wedgeCreate = false;
  #waiters: { resolve: () => void; reject: (e: Error) => void }[] = [];
  #openWaiters: (() => void)[] = [];

  constructor(auto = true) {
    this.auto = auto;
  }

  sendFrame(bytes: Uint8Array): FrameSend {
    const send = new FakeSend(bytes, this, this.sent.length, !this.wedgeCreate);
    this.sent.push(bytes);
    this.sends.push(send);
    if (this.auto) send.accept();
    return send;
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

  /** Accept the oldest unsettled send (latest) or write (reliable). */
  release(n = 1): void {
    while (n-- > 0) {
      const pending = this.sends.find((s) => !s.settled);
      if (pending !== undefined) pending.accept();
      else this.#waiters.shift()?.resolve();
    }
  }

  rejectWrite(): void {
    const pending = this.sends.find((s) => !s.settled);
    if (pending !== undefined) pending.fail(new Error("stream aborted"));
    else this.#waiters.shift()?.reject(new Error("stream aborted"));
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

Deno.test("reliable: overflow kicks once and empties the FIFO", async () => {
  const sink = new FakeSink(false);
  const ch = new ReliableChannel(sink);
  // Nothing drains (the persistent stream is still opening), so the 65th
  // queued frame overflows; the channel kicks once, self-disposes, and
  // ignores the rest of the burst instead of re-queueing + re-kicking.
  for (let i = 0; i < 200; i++) ch.offer(frame(i));
  await tick();
  assertEquals(sink.kicks, 1);
  assertEquals(sink.kicked, "reliable channel overflow");
  assertEquals(ch.queued(), 0);
  assertEquals(sink.streamsAborted, 1);
  ch.offer(frame(200)); // still a no-op until transport teardown completes
  assertEquals(ch.queued(), 0);
  assertEquals(sink.kicks, 1);
});

Deno.test("latest: dispose resets every outstanding send without kicking", async () => {
  const sink = new FakeSink(false);
  const ch = new LatestChannel(sink);
  ch.offer(frame(1)); // send in flight
  sink.release();
  await tick();
  ch.offer(frame(2)); // second send in flight (first still outstanding)
  ch.offer(frame(3)); // parked in the pending slot
  ch.dispose();
  assertEquals(ch.queued(), 0);
  assertEquals(ch.inflight(), 0);
  // Both the accepted stream and the in-flight one are reset (an accepted
  // stream may still hold undelivered buffered data).
  assertEquals(sink.sendsAborted, 2);
  assertEquals([ch.aborted, ch.expired], [0, 0]); // disposal resets are not counted
  await tick();
  ch.offer(frame(4)); // ignored after dispose
  await tick();
  assertEquals(sink.sent, [frame(1), frame(2)]);
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

// ---------------------------------------------------------------------------
// Reaping (the T1-review issue-12 fix): accepted-but-possibly-undelivered
// latest streams are reset once stale, with counters that tell a suspended
// viewer (aborted) apart from routine end-of-life resets (expired).

function timedChannel(
  sink: FakeSink,
  start = 100_000,
): { ch: LatestChannel; tick(ms: number): void; now(): number } {
  let now = start;
  const ch = new LatestChannel(sink, { now: () => now });
  return { ch, tick: (ms: number) => now += ms, now: () => now };
}

Deno.test("latest: a stale accepted send is reset by the next offer (expired)", async () => {
  const sink = new FakeSink(false);
  const { ch, tick: advance } = timedChannel(sink);
  ch.offer(frame(1));
  sink.release(); // accepted; its stream stays open (never FIN'd)
  await tick();
  assertEquals(ch.inflight(), 1);

  advance(600); // past LATEST_STALE_MS
  ch.offer(frame(2));
  assertEquals(sink.sendsAborted, 1); // frame 1's stream reset
  assertEquals(sink.sends[0].aborted, true);
  assertEquals([ch.aborted, ch.expired], [0, 1]); // no backpressure: routine expiry
  sink.release();
  await tick();
  assertEquals(ch.sent, 2);
  assertEquals(ch.bytesOut, 2);
  assertEquals(ch.inflight(), 1); // only frame 2's stream remains
  assertEquals(sink.kicked, null);
});

Deno.test("latest: reaps under backpressure count as aborted", async () => {
  const sink = new FakeSink(false);
  const { ch, tick: advance } = timedChannel(sink);
  ch.offer(frame(1));
  sink.release(); // frame 1 accepted at t0
  await tick();
  advance(400);
  ch.offer(frame(2)); // frame 2 in flight, never accepted: the viewer wedged
  await tick();

  advance(200); // frame 1's stream is now 600 ms old, the carrier only 200
  ch.offer(frame(3));
  // The stale accepted stream is reset and counted aborted (the channel is
  // backpressured); the still-young write-wedged carrier is left alone.
  assertEquals([ch.aborted, ch.expired], [1, 0]);
  assertEquals(sink.sendsAborted, 1);
  assertEquals(sink.sends[1].aborted, false);
});

Deno.test("latest: offers supersede the create-wedged carrier; release delivers the newest", async () => {
  // Resetting a create-wedged send would only spawn a replacement wedged on
  // the same exhausted stream credit (resets do not replenish credit while
  // the viewer's app is not reading), growing an unbounded zombie chain.
  // Instead newer offers replace its payload in place.
  const sink = new FakeSink(false);
  sink.wedgeCreate = true;
  const { ch, tick: advance } = timedChannel(sink);
  ch.offer(frame(1)); // handed to the transport, wedged in stream creation
  await tick();
  advance(60_000);
  ch.offer(frame(2)); // supersedes the carrier's payload
  ch.offer(frame(3)); // supersedes again
  assertEquals(sink.sends.length, 1); // no zombie chain even 60 s stale
  assertEquals(sink.sendsAborted, 0);
  assertEquals(ch.dropped, 2);
  assertEquals(ch.queued(), 0);
  assertEquals([ch.aborted, ch.expired], [0, 0]);

  sink.release(); // the viewer resumes: the carrier is accepted
  await tick();
  assertEquals(sink.sent, [frame(3)]); // exactly the newest payload went out
  assertEquals(ch.sent, 1);
  assertEquals(ch.bytesOut, frame(3).byteLength); // 3 offers = 1 sent + 2 dropped
});

Deno.test("latest: a stale write-wedged carrier is reset and the newest resent", async () => {
  // Once the write has begun the payload can no longer change, so a stale
  // carrier is reset and the newest goes out on a fresh stream.
  const sink = new FakeSink(false); // default: the write is already in progress
  const { ch, tick: advance } = timedChannel(sink);
  ch.offer(frame(1)); // write begins, wedged in flow control
  await tick();
  advance(600); // past LATEST_STALE_MS
  ch.offer(frame(2)); // supersede refused -> the stale carrier is reset
  assertEquals(sink.sends[0].aborted, true);
  assertEquals(ch.aborted, 1);
  assertEquals(ch.dropped, 1); // frame 1's payload never reached the wire
  await tick(); // the drain swallows the abort and picks up the parked frame
  assertEquals(ch.queued(), 0);
  assertEquals(ch.inflight(), 1); // only the fresh carrier

  sink.release();
  await tick();
  assertEquals(ch.sent, 1); // frame 1 was handed to the transport, never accepted
  assertEquals(sink.sent, [frame(1), frame(2)]);
});

Deno.test("latest: supersede stops once the write begins", async () => {
  const sink = new FakeSink(false);
  sink.wedgeCreate = true;
  const ch = new LatestChannel(sink);
  ch.offer(frame(1)); // wedged in stream creation
  ch.offer(frame(2)); // supersedes in place
  assertEquals(ch.dropped, 1);
  sink.sends[0].beginWrite(); // stream created; the write is now in progress
  ch.offer(frame(3)); // the payload can no longer change: parks instead
  assertEquals(ch.queued(), 1);
  assertEquals(ch.dropped, 1); // parking in an empty slot drops nothing

  sink.release();
  await tick();
  sink.release();
  await tick();
  assertEquals(sink.sent, [frame(2), frame(3)]);
  assertEquals(ch.sent, 2);
});

Deno.test("latest: a healthy viewer expires streams but never aborts", async () => {
  const sink = new FakeSink();
  const { ch, tick: advance, now } = timedChannel(sink);
  for (let i = 0; i < 5; i++) {
    ch.offer(frame(i)); // auto sink: accepted immediately
    await tick();
    advance(600);
  }
  assertEquals(ch.sent, 5);
  assertEquals(ch.aborted, 0);
  assertEquals(ch.expired, 4); // every reap saw an accepted newest send
  assertEquals(ch.inflight(), 1);
  assertEquals(sink.kicked, null);

  // The input idled; the relay's periodic reap (registry.reapAll) ends the
  // last stream too, still as routine expiry.
  ch.reap(now()); // the loop's final advance left the last acceptance 600 ms old
  assertEquals(ch.expired, 5);
  assertEquals(ch.inflight(), 0);
  assertEquals(ch.aborted, 0);
});

Deno.test("latest: a real transport failure still kicks exactly once", async () => {
  const sink = new FakeSink(false);
  const ch = new LatestChannel(sink);
  ch.offer(frame(1));
  sink.release();
  await tick();
  ch.offer(frame(2));
  sink.rejectWrite(); // connection died: not an abort()
  await tick();
  assertEquals(sink.kicked, "write failed");
  // The kick's dispose sweeps both outstanding streams (harmless on the dead
  // frame-2 stream; the real sink swallows abort errors).
  assertEquals(sink.sendsAborted, 2);
  ch.offer(frame(3)); // ignored after the dispose
  await tick();
  assertEquals(sink.sent, [frame(1), frame(2)]);
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

import { assertEquals, assertRejects } from "@std/assert";
import { encodeDataFrame, type FrameHeader } from "@dimos/shared";
import {
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
  auto: boolean;
  #waiters: (() => void)[] = [];

  constructor(auto = true) {
    this.auto = auto;
  }

  sendFrame(bytes: Uint8Array): Promise<void> {
    this.sent.push(bytes);
    if (this.auto) return Promise.resolve();
    return new Promise<void>((resolve) => this.#waiters.push(resolve));
  }

  release(n = 1): void {
    while (n-- > 0) this.#waiters.shift()?.();
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
  for (let i = 0; i < 10; i++) {
    sink.release();
    await tick();
  }
  assertEquals(sink.sent, Array.from({ length: 10 }, (_, i) => frame(i)));
  assertEquals(ch.sent, 10);
  assertEquals(ch.dropped, 0);
  assertEquals(sink.kicked, null);
});

Deno.test("reliable: queue overflow kicks the viewer", async () => {
  const sink = new FakeSink(false);
  const ch = new ReliableChannel(sink);
  // 1 in flight + 64 queued is accepted; the next one overflows.
  for (let i = 0; i < 66 && sink.kicked === null; i++) ch.offer(frame(i));
  await tick();
  assertEquals(sink.kicked, "reliable channel overflow");
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

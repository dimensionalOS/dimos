import { assertEquals } from "@std/assert";
import { CONTROL_CHANNEL, DataFrameStreamReader, decodeDatagram, type Msg } from "@dimos/shared";
import { type CarrierSink, RobotCarrier } from "./carrier.ts";
import type { FrameWriter } from "./forward.ts";

class FakeCarrierSink implements CarrierSink {
  /** Every frame written to the persistent stream, in order. */
  written: Uint8Array[] = [];
  streamsOpened = 0;
  streamsAborted = 0;
  failed: string | null = null;
  fails = 0;
  auto: boolean;
  manualOpen = false;
  #waiters: { resolve: () => void; reject: (e: Error) => void }[] = [];
  #openWaiters: (() => void)[] = [];

  constructor(auto = true) {
    this.auto = auto;
  }

  openStream(): Promise<FrameWriter> {
    this.streamsOpened++;
    const writer: FrameWriter = {
      write: (bytes: Uint8Array) => {
        this.written.push(bytes);
        if (this.auto) return Promise.resolve();
        return new Promise<void>((resolve, reject) => this.#waiters.push({ resolve, reject }));
      },
      abort: () => {
        this.streamsAborted++;
        return Promise.resolve();
      },
    };
    if (!this.manualOpen) return Promise.resolve(writer);
    return new Promise((resolve) => this.#openWaiters.push(() => resolve(writer)));
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

  fail(reason: string): void {
    this.fails++;
    this.failed = reason;
  }
}

function tick(): Promise<void> {
  return new Promise((resolve) => setTimeout(resolve, 0));
}

function subs(n: number, ...chs: string[]): Msg {
  return { t: "subs", chs, n };
}

Deno.test("carrier: control frames arrive in order, byte-decodable, on one lazy stream", async () => {
  const sink = new FakeCarrierSink();
  const carrier = new RobotCarrier(sink);
  assertEquals(sink.streamsOpened, 0); // no stream before the first send
  carrier.sendControl(subs(1));
  carrier.sendControl(subs(2, "odom"));
  await tick();
  carrier.sendControl(subs(3, "odom", "color_image"));
  await tick();
  assertEquals(sink.streamsOpened, 1);
  const reader = new DataFrameStreamReader();
  const frames = sink.written.flatMap((chunk) => reader.push(chunk));
  assertEquals(frames.length, 3);
  frames.forEach((frame, i) => {
    assertEquals(frame.header.ch, CONTROL_CHANNEL);
    assertEquals(frame.header.seq, i + 1); // carrier-monotonic
    assertEquals(frame.header.delivery, "reliable");
  });
  assertEquals(frames.map((f) => decodeDatagram(f.payload)), [
    subs(1),
    subs(2, "odom"),
    subs(3, "odom", "color_image"),
  ]);
  assertEquals(carrier.stats(), {
    queued: 0,
    queuedBytes: 0,
    sent: 3,
    bytesOut: sink.written.reduce((total, f) => total + f.byteLength, 0),
  });
  assertEquals(sink.fails, 0);
});

Deno.test("carrier: overflow fails the session once; later sends are no-ops", async () => {
  const sink = new FakeCarrierSink(false);
  const carrier = new RobotCarrier(sink);
  // Nothing drains past the first pending write, so the 257th queued frame
  // overflows; the carrier fails once, self-disposes, and ignores the rest.
  for (let i = 0; i < 400; i++) carrier.sendControl(subs(i));
  await tick();
  assertEquals(sink.fails, 1);
  assertEquals(sink.failed, "carrier overflow");
  assertEquals(carrier.stats().queued, 0); // disposed: queue emptied
  carrier.sendControl(subs(999));
  await tick();
  assertEquals(sink.fails, 1);
});

Deno.test("carrier: a write failure fails the session", async () => {
  const sink = new FakeCarrierSink(false);
  const carrier = new RobotCarrier(sink);
  carrier.sendControl(subs(1));
  await tick();
  sink.rejectWrite();
  await tick();
  assertEquals(sink.fails, 1);
  assertEquals(sink.failed, "carrier write failed");
});

Deno.test("carrier: a write failure after dispose is not a session failure", async () => {
  const sink = new FakeCarrierSink(false);
  const carrier = new RobotCarrier(sink);
  carrier.sendControl(subs(1));
  await tick(); // write pending
  carrier.dispose();
  assertEquals(sink.streamsAborted, 1);
  sink.rejectWrite();
  await tick();
  assertEquals(sink.fails, 0);
});

Deno.test("carrier: dispose during stream open releases the stream", async () => {
  const sink = new FakeCarrierSink();
  sink.manualOpen = true;
  const carrier = new RobotCarrier(sink);
  carrier.sendControl(subs(1));
  carrier.dispose(); // no writer yet to abort
  sink.releaseOpen();
  await tick();
  assertEquals(sink.streamsAborted, 1);
  assertEquals(sink.written, []);
  assertEquals(sink.fails, 0);
});

Deno.test("carrier: an over-cap control payload fails the session, nothing queued", async () => {
  const sink = new FakeCarrierSink();
  const carrier = new RobotCarrier(sink);
  carrier.sendControl(subs(1, "x".repeat(65 * 1024)));
  await tick();
  assertEquals(sink.fails, 1);
  assertEquals(sink.streamsOpened, 0);
  assertEquals(carrier.stats().sent, 0);
});

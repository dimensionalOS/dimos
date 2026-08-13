import { afterEach, describe, expect, it } from "vitest";
import {
  type ChannelSpec,
  ControlFrameReader,
  encodeControlFrame,
  encodeDataFrame,
  type Msg,
  PROTOCOL_VERSION,
  type RobotInfo,
} from "@dimos/shared";
import {
  manifestsEqual,
  pickAutoWatch,
  type SessionHandle,
  startSession,
  subscribableChannels,
} from "./session.ts";
import type { RelayInfo, WebTransportLike } from "./transport.ts";

function spec(over: Partial<ChannelSpec> = {}): ChannelSpec {
  return { ch: "odom", encoding: "pose.json.v1", delivery: "reliable", maxHz: 20, ...over };
}

describe("manifestsEqual", () => {
  it("ignores channel order", () => {
    const a = [spec(), spec({ ch: "color_image", encoding: "jpeg.v1", delivery: "latest" })];
    expect(manifestsEqual(a, [...a].reverse())).toBe(true);
  });

  it("detects field changes and extra channels", () => {
    expect(manifestsEqual([spec()], [spec({ maxHz: 30 })])).toBe(false);
    expect(manifestsEqual([spec()], [spec({ encoding: "pose.json.v2" })])).toBe(false);
    expect(manifestsEqual([spec()], [spec({ delivery: "latest" })])).toBe(false);
    expect(manifestsEqual([spec()], [spec(), spec({ ch: "extra" })])).toBe(false);
    expect(manifestsEqual([], [])).toBe(true);
  });
});

describe("pickAutoWatch", () => {
  const robot = (id: string): RobotInfo => ({ id, name: id, model: "go2" });

  it("picks the robot only when it is the only one", () => {
    expect(pickAutoWatch([])).toBeNull();
    expect(pickAutoWatch([robot("a")])).toEqual(robot("a"));
    expect(pickAutoWatch([robot("a"), robot("b")])).toBeNull();
  });
});

describe("subscribableChannels", () => {
  it("keeps only channels with a decoder (undecodable ones waste bandwidth)", () => {
    const odom = spec();
    const jpeg = spec({ ch: "color_image", encoding: "jpeg.v1", delivery: "latest" });
    expect(subscribableChannels([odom, jpeg])).toEqual([odom]);
    expect(subscribableChannels([jpeg])).toEqual([]);
  });
});

// ---------------------------------------------------------------------------
// Integration tests: the real Session against a fake relay behind the
// WebTransportLike seam. The fake is the relay end of one connection: it
// collects viewer control messages and can push control messages and data
// frames, so robot lifecycle transitions run over the actual wire encoding
// while the relay and "viewer connection" stay up the whole time.

const INFO: RelayInfo = {
  wtUrl: "https://127.0.0.1:1/viewer",
  certHash: "aGFzaA==",
  v: PROTOCOL_VERSION,
};

const ROBOT_A: RobotInfo = { id: "a", name: "A", model: "go2" };
const ROBOT_B: RobotInfo = { id: "b", name: "B", model: "go2" };

class FakeRelayEnd {
  readonly sent: Msg[] = [];
  readonly wt: WebTransportLike;
  #control!: ReadableStreamDefaultController<Uint8Array>;
  #uni!: ReadableStreamDefaultController<ReadableStream<Uint8Array>>;

  constructor() {
    const inbound = new ControlFrameReader();
    const readable = new ReadableStream<Uint8Array>({
      start: (c) => {
        this.#control = c;
      },
    });
    const writable = new WritableStream<Uint8Array>({
      write: (chunk) => {
        this.sent.push(...inbound.push(chunk));
      },
    });
    let closeWt = () => {};
    const closed = new Promise<unknown>((resolve) => {
      closeWt = () => resolve({});
    });
    this.wt = {
      ready: Promise.resolve(),
      closed,
      close: () => closeWt(),
      createBidirectionalStream: () => Promise.resolve({ readable, writable }),
      incomingUnidirectionalStreams: new ReadableStream<ReadableStream<Uint8Array>>({
        start: (c) => {
          this.#uni = c;
        },
      }),
    };
  }

  push(msg: Msg): void {
    this.#control.enqueue(encodeControlFrame(msg));
  }

  /** One data frame on its own uni stream, JSON payload like the bridge's. */
  pushFrame(seq: number, value: unknown, ch = "odom"): void {
    const payload = new TextEncoder().encode(JSON.stringify(value));
    const frame = encodeDataFrame({ ch, seq, ts: seq, delivery: "reliable" }, payload);
    this.#uni.enqueue(
      new ReadableStream<Uint8Array>({
        start: (c) => {
          c.enqueue(frame);
          c.close();
        },
      }),
    );
  }

  watches(id: string): number {
    return this.sent.filter((m) => m.t === "watch" && m.robotId === id).length;
  }

  subs(): string[] {
    return this.sent.flatMap((m) => (m.t === "sub" ? [m.ch] : []));
  }
}

async function until(cond: () => boolean, what = "condition"): Promise<void> {
  const deadline = Date.now() + 2000;
  while (!cond()) {
    if (Date.now() > deadline) throw new Error(`timed out waiting for ${what}`);
    await new Promise((resolve) => setTimeout(resolve, 2));
  }
}

/** Lets already-queued streams/messages drain before a negative assertion. */
function settle(): Promise<void> {
  return new Promise((resolve) => setTimeout(resolve, 20));
}

describe("Session over a fake WebTransport", () => {
  const handles: SessionHandle[] = [];

  afterEach(() => {
    for (const handle of handles.splice(0)) handle.stop();
  });

  function start(): { relay: FakeRelayEnd; handle: SessionHandle } {
    const relay = new FakeRelayEnd();
    const handle = startSession({
      fetchInfo: () => Promise.resolve(INFO),
      createWebTransport: () => relay.wt,
    });
    handles.push(handle);
    return { relay, handle };
  }

  async function goLive(
    relay: FakeRelayEnd,
    handle: SessionHandle,
    robot = ROBOT_A,
    channels = [spec()],
  ): Promise<void> {
    relay.push({ t: "welcome", v: PROTOCOL_VERSION });
    relay.push({ t: "robots", robots: [robot] });
    await until(() => relay.watches(robot.id) === 1, "watch");
    relay.push({ t: "manifest", robotId: robot.id, channels });
    await until(() => handle.status.get().channels.length === channels.length, "manifest");
  }

  it("publishes connected only after the relay's welcome", async () => {
    const { relay, handle } = start();
    await until(() => relay.sent.some((m) => m.t === "hello"), "hello");
    expect(handle.status.get().transport.phase).toBe("connecting");

    relay.push({ t: "welcome", v: PROTOCOL_VERSION });
    await until(() => handle.status.get().transport.phase === "connected", "connected");
  });

  it("watches the sole robot and subs only decodable manifest channels", async () => {
    const { relay, handle } = start();
    await goLive(relay, handle, ROBOT_A, [
      spec(),
      spec({ ch: "color_image", encoding: "jpeg.v1", delivery: "latest" }),
    ]);
    expect(relay.watches("a")).toBe(1);
    expect(relay.subs()).toEqual(["odom"]);
    expect(handle.status.get().robot).toEqual(ROBOT_A);
  });

  it("retries the watch when the robot reappears after unknown_robot", async () => {
    const { relay, handle } = start();
    relay.push({ t: "welcome", v: PROTOCOL_VERSION });
    relay.push({ t: "robots", robots: [ROBOT_A] });
    await until(() => relay.watches("a") === 1, "first watch");

    // The robot died before the watch arrived: the relay answered with
    // unknown_robot and pushed an empty robots list.
    relay.push({ t: "robots", robots: [] });
    relay.push({ t: "error", code: "unknown_robot", message: "no robot a" });
    await until(() => handle.status.get().lastError === "unknown_robot: no robot a", "error");

    // The same id coming back must trigger a fresh watch, and its manifest
    // clears the stale error.
    relay.push({ t: "robots", robots: [ROBOT_A] });
    await until(() => relay.watches("a") === 2, "retried watch");
    relay.push({ t: "manifest", robotId: "a", channels: [spec()] });
    await until(() => handle.status.get().lastError === null, "error cleared");
    expect(handle.status.get().channels).toEqual([spec()]);
  });

  it("clears the stale error on the next welcome", async () => {
    const { relay, handle } = start();
    relay.push({ t: "welcome", v: PROTOCOL_VERSION });
    relay.push({ t: "error", code: "no_watch", message: "watch a robot before sub/unsub" });
    await until(() => handle.status.get().lastError !== null, "error");

    relay.push({ t: "welcome", v: PROTOCOL_VERSION });
    await until(() => handle.status.get().lastError === null, "error cleared");
  });

  it("survives a same-relay robot restart: rewatch, reset, restarted seqs win", async () => {
    const { relay, handle } = start();
    await goLive(relay, handle);
    relay.pushFrame(500, { x: 500 });
    await until(() => handle.channels.get("odom")?.seq === 500, "first frame");

    // Robot gone: channels, values, and the watch confirmation are dropped.
    relay.push({ t: "robots", robots: [] });
    await until(() => handle.status.get().channels.length === 0, "cleared channels");
    expect(handle.status.get().robotCount).toBe(0);
    expect(handle.status.get().epoch).toBe(1);
    expect(handle.channels.get("odom")).toBeNull();

    // A frame still draining out of the dead robot's relay queue is ignored.
    relay.pushFrame(501, { x: 501 });
    await settle();
    expect(handle.channels.get("odom")).toBeNull();

    // Same id returns: the watch must be re-sent even though the id never
    // changed, and the manifest re-adopted.
    relay.push({ t: "robots", robots: [ROBOT_A] });
    await until(() => relay.watches("a") === 2, "rewatch");
    relay.push({ t: "manifest", robotId: "a", channels: [spec()] });
    await until(() => handle.status.get().channels.length === 1, "manifest readopted");

    // A late high-seq frame from the dead producer must not lock out the
    // new producer's restarted counter.
    relay.pushFrame(502, { x: 502 });
    await until(() => handle.channels.get("odom")?.seq === 502, "late old frame");
    relay.pushFrame(1, { x: 1 });
    await until(() => handle.channels.get("odom")?.seq === 1, "restarted seq");
    expect(handle.channels.get("odom")?.value).toEqual({ x: 1 });
  });

  it("does not carry values into a replacement robot with an identical manifest", async () => {
    const { relay, handle } = start();
    await goLive(relay, handle);
    relay.pushFrame(900, { x: 900 });
    await until(() => handle.channels.get("odom")?.seq === 900, "value from A");

    relay.push({ t: "robots", robots: [] });
    await until(() => handle.status.get().channels.length === 0, "cleared");

    relay.push({ t: "robots", robots: [ROBOT_B] });
    await until(() => relay.watches("b") === 1, "watch B");
    relay.push({ t: "manifest", robotId: "b", channels: [spec()] });
    await until(() => handle.status.get().channels.length === 1, "manifest B");

    // Identical manifest, different robot: A's value must not show under B.
    expect(handle.channels.get("odom")).toBeNull();
    expect(handle.status.get().robot).toEqual(ROBOT_B);
    relay.pushFrame(1, { x: 1 });
    await until(() => handle.channels.get("odom")?.seq === 1, "value from B");
    expect(handle.channels.get("odom")?.value).toEqual({ x: 1 });
  });

  it("drops data and remounts when the watched robot's manifest changes", async () => {
    const { relay, handle } = start();
    await goLive(relay, handle);
    relay.pushFrame(900, { x: 900 });
    await until(() => handle.channels.get("odom")?.seq === 900, "value");

    const changed = spec({ ch: "status" });
    relay.push({ t: "manifest", robotId: "a", channels: [changed] });
    await until(() => handle.status.get().channels[0]?.ch === "status", "new manifest");
    expect(handle.status.get().epoch).toBe(1);
    expect(handle.channels.get("odom")).toBeNull();
  });

  it("clears the watch on multiple robots and ignores a stale manifest reply", async () => {
    const { relay, handle } = start();
    await goLive(relay, handle);
    relay.pushFrame(900, { x: 900 });
    await until(() => handle.channels.get("odom")?.seq === 900, "value from A");

    // A second robot appears: auto-watch is ambiguous, everything clears.
    relay.push({ t: "robots", robots: [ROBOT_A, ROBOT_B] });
    await until(() => handle.status.get().robotCount === 2, "two robots");
    expect(handle.status.get().robot).toBeNull();
    expect(handle.status.get().channels).toEqual([]);
    expect(handle.channels.get("odom")).toBeNull();

    // A manifest reply that raced the second registration is not adopted.
    relay.push({ t: "manifest", robotId: "a", channels: [spec()] });
    await settle();
    expect(handle.status.get().channels).toEqual([]);

    // A leaves; the survivor becomes the sole robot and gets watched.
    relay.push({ t: "robots", robots: [ROBOT_B] });
    await until(() => relay.watches("b") === 1, "watch B");
    relay.push({ t: "manifest", robotId: "b", channels: [spec({ ch: "status" })] });
    await until(() => handle.status.get().channels[0]?.ch === "status", "manifest B");
  });
});

import { afterEach, describe, expect, it } from "vitest";
import {
  type ChannelSpec,
  ControlFrameReader,
  encodeControlFrame,
  encodeDataFrame,
  type Msg,
  type PanelSpec,
  PROTOCOL_VERSION,
  type RobotInfo,
} from "@dimos/shared";
import type { Manifest } from "@dimos/shared/manifest";
import type { CostmapValue } from "./decoders/costmap.ts";
import {
  channelSubscribable,
  manifestsEqual,
  pickAutoWatch,
  type SessionHandle,
  startSession,
  subscribableChannels,
} from "./session.ts";
import type { RelayInfo, WebTransportLike } from "./transport.ts";

// Normalized specs on purpose: pushing them over the fake wire and parsing
// them back yields the identical objects, so adoption asserts stay exact.
function spec(over: Partial<ChannelSpec> = {}): ChannelSpec {
  return {
    ch: "odom",
    dir: "rx",
    encoding: "pose.json.v1",
    delivery: "reliable",
    maxHz: 20,
    params: {},
    ...over,
  };
}

function panel(over: Partial<PanelSpec> & { id: string; kind: string }): PanelSpec {
  return { title: "", channels: [], params: {}, ...over };
}

function manifest(
  channels: ChannelSpec[],
  panels: PanelSpec[] = [],
  layout: Manifest["layout"] = null,
  pages: string[] = [],
): Manifest {
  return { version: 1, channels, panels, layout, pages };
}

describe("manifestsEqual", () => {
  it("ignores channel order", () => {
    const a = [spec(), spec({ ch: "color_image", encoding: "jpeg.v1", delivery: "latest" })];
    expect(manifestsEqual(manifest(a), manifest([...a].reverse()))).toBe(true);
  });

  it("detects field changes and extra channels", () => {
    expect(manifestsEqual(manifest([spec()]), manifest([spec({ maxHz: 30 })]))).toBe(false);
    expect(manifestsEqual(manifest([spec()]), manifest([spec({ encoding: "pose.json.v2" })])))
      .toBe(false);
    expect(manifestsEqual(manifest([spec()]), manifest([spec({ delivery: "latest" })]))).toBe(
      false,
    );
    expect(manifestsEqual(manifest([spec()]), manifest([spec({ dir: "tx" })]))).toBe(false);
    expect(manifestsEqual(manifest([spec()]), manifest([spec({ params: { q: 1.5 } })]))).toBe(
      false,
    );
    expect(manifestsEqual(manifest([spec()]), manifest([spec(), spec({ ch: "extra" })]))).toBe(
      false,
    );
    expect(manifestsEqual(manifest([]), manifest([]))).toBe(true);
  });

  it("detects panel changes, including display order", () => {
    const video = panel({ id: "cam", kind: "video", channels: ["odom"] });
    const readout = panel({ id: "pose", kind: "readout", channels: ["odom"] });
    expect(manifestsEqual(manifest([spec()], [video]), manifest([spec()], [video]))).toBe(true);
    expect(manifestsEqual(manifest([spec()], [video]), manifest([spec()], []))).toBe(false);
    expect(
      manifestsEqual(
        manifest([spec()], [video]),
        manifest([spec()], [{ ...video, kind: "readout" }]),
      ),
    ).toBe(false);
    expect(
      manifestsEqual(
        manifest([spec()], [video]),
        manifest([spec()], [{ ...video, title: "Front" }]),
      ),
    ).toBe(false);
    expect(
      manifestsEqual(
        manifest([spec()], [video, readout]),
        manifest([spec()], [readout, video]),
      ),
    ).toBe(false); // panel order is display order
  });

  it("detects layout and pages changes (a layout-only edit must remount)", () => {
    const video = panel({ id: "cam", kind: "video", channels: ["odom"] });
    const withLayout = (layout: Manifest["layout"]) => manifest([spec()], [video], layout);
    expect(manifestsEqual(withLayout("cam"), withLayout("cam"))).toBe(true);
    expect(manifestsEqual(withLayout({ row: ["cam"] }), withLayout({ row: ["cam"] }))).toBe(true);
    expect(manifestsEqual(withLayout({ row: ["cam"] }), withLayout({ col: ["cam"] }))).toBe(false);
    expect(
      manifestsEqual(
        withLayout({ row: ["cam"], shares: [2.5] }),
        withLayout({ row: ["cam"] }),
      ),
    ).toBe(false);
    expect(manifestsEqual(withLayout("cam"), withLayout(null))).toBe(false);
    expect(
      manifestsEqual(
        manifest([spec()], [video], null, ["cam"]),
        manifest([spec()], [video], null, []),
      ),
    ).toBe(false);
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
  const odom = spec();
  const jpeg = spec({ ch: "color_image", encoding: "jpeg.v1", delivery: "latest" });
  const costmap = spec({ ch: "global_costmap", encoding: "costmap.zlib.v1", delivery: "latest" });
  const future = spec({ ch: "voxels", encoding: "voxels.bin.v9", delivery: "latest" });
  const videoPanel = panel({ id: "cam", kind: "video", channels: ["color_image"] });
  const mapPanel = panel({ id: "map", kind: "map2d", channels: ["global_costmap", "odom"] });

  it("keeps only channels with a decoder (undecodable ones waste bandwidth)", () => {
    expect(subscribableChannels([odom, jpeg, future], [videoPanel])).toEqual([odom, jpeg]);
    expect(subscribableChannels([future], [])).toEqual([]);
  });

  it("never subscribes tx channels", () => {
    expect(channelSubscribable(spec({ dir: "tx" }), [])).toBe(false);
  });

  it("gates panel-only encodings on a renderable panel binding them", () => {
    expect(channelSubscribable(jpeg, [])).toBe(false);
    expect(channelSubscribable(jpeg, [videoPanel])).toBe(true);
    // A panel kind this build cannot render does not justify the bandwidth
    // (and the UnknownPanel fallback must never leak into this gate).
    expect(channelSubscribable(jpeg, [{ ...videoPanel, kind: "hologram" }])).toBe(false);
    // Cheap JSON channels are subscribed with or without a panel.
    expect(channelSubscribable(odom, [])).toBe(true);
  });

  it("gates the costmap encoding like jpeg (grids nobody renders stay unencoded)", () => {
    expect(channelSubscribable(costmap, [])).toBe(false);
    expect(channelSubscribable(costmap, [mapPanel])).toBe(true);
    expect(channelSubscribable(costmap, [{ ...mapPanel, kind: "hologram" }])).toBe(false);
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
  /** Awaited per decoded viewer message: lets a test react at the exact
   * moment a message is observed while holding the session's control writer
   * (e.g. to land a data frame mid-sub-loop). */
  onMsg: ((msg: Msg) => void | Promise<void>) | null = null;
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
      write: async (chunk) => {
        for (const msg of inbound.push(chunk)) {
          this.sent.push(msg);
          await this.onMsg?.(msg);
        }
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

  pushManifest(robotId: string, m: Manifest): void {
    this.push({ t: "manifest", robotId, manifest: m as unknown as Record<string, unknown> });
  }

  /** One data frame on its own uni stream, JSON payload like the bridge's. */
  pushFrame(seq: number, value: unknown, ch = "odom"): void {
    this.pushRaw(seq, new TextEncoder().encode(JSON.stringify(value)), ch);
  }

  /** One data frame on its own uni stream, arbitrary payload bytes. */
  pushRaw(seq: number, payload: Uint8Array, ch: string, meta?: Record<string, unknown>): void {
    const frame = encodeDataFrame({ ch, seq, ts: seq, delivery: "reliable", meta }, payload);
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

  function adopted(handle: SessionHandle): ChannelSpec[] {
    return handle.status.get().manifest?.channels ?? [];
  }

  async function goLive(
    relay: FakeRelayEnd,
    handle: SessionHandle,
    robot = ROBOT_A,
    channels = [spec()],
    panels: PanelSpec[] = [],
  ): Promise<void> {
    relay.push({ t: "welcome", v: PROTOCOL_VERSION });
    relay.push({ t: "robots", robots: [robot] });
    await until(() => relay.watches(robot.id) === 1, "watch");
    relay.pushManifest(robot.id, manifest(channels, panels));
    await until(() => adopted(handle).length === channels.length, "manifest");
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
      spec({ ch: "voxels", encoding: "voxels.bin.v9", delivery: "latest" }),
    ]);
    expect(relay.watches("a")).toBe(1);
    // No panel binds color_image, so the jpeg channel stays unsubscribed too.
    expect(relay.subs()).toEqual(["odom"]);
    expect(handle.status.get().robot).toEqual(ROBOT_A);
  });

  it("subs the jpeg channel when a video panel binds it", async () => {
    const { relay, handle } = start();
    await goLive(
      relay,
      handle,
      ROBOT_A,
      [spec(), spec({ ch: "color_image", encoding: "jpeg.v1", delivery: "latest" })],
      [panel({ id: "cam", kind: "video", channels: ["color_image"] })],
    );
    expect(relay.subs()).toEqual(["odom", "color_image"]);
  });

  it("keeps the jpeg channel unsubscribed under an unrenderable panel kind", async () => {
    const { relay, handle } = start();
    await goLive(
      relay,
      handle,
      ROBOT_A,
      [spec(), spec({ ch: "color_image", encoding: "jpeg.v1", delivery: "latest" })],
      [panel({ id: "holo", kind: "hologram", channels: ["color_image"] })],
    );
    expect(relay.subs()).toEqual(["odom"]);
  });

  it("never subs a tx channel", async () => {
    const { relay, handle } = start();
    await goLive(relay, handle, ROBOT_A, [
      spec(),
      spec({ ch: "tele_cmd_vel", dir: "tx", encoding: "twist.json.v1", delivery: "latest" }),
    ]);
    expect(relay.subs()).toEqual(["odom"]);
  });

  it("subs the costmap channel when a map2d panel binds it", async () => {
    const { relay, handle } = start();
    await goLive(
      relay,
      handle,
      ROBOT_A,
      [spec(), spec({ ch: "global_costmap", encoding: "costmap.zlib.v1", delivery: "latest" })],
      [panel({ id: "map", kind: "map2d", channels: ["global_costmap", "odom"] })],
    );
    expect(relay.subs()).toEqual(["odom", "global_costmap"]);
  });

  it("keeps the costmap channel unsubscribed under an unrenderable panel kind", async () => {
    const { relay, handle } = start();
    await goLive(
      relay,
      handle,
      ROBOT_A,
      [spec(), spec({ ch: "global_costmap", encoding: "costmap.zlib.v1", delivery: "latest" })],
      [panel({ id: "holo", kind: "hologram", channels: ["global_costmap"] })],
    );
    expect(relay.subs()).toEqual(["odom"]);
  });

  it("stores a costmap frame that arrives while subs are still being sent", async () => {
    // The bridge replays the cached grid the moment a sub lands, so the frame
    // can beat the control loop's continuation. The hook injects it
    // synchronously at the sub and then holds the control writer for one
    // macrotask, letting the whole uni-stream ingest chain drain first: with
    // adoption after the sub loop, #ingest would drop it as manifest-less.
    const { relay, handle } = start();
    relay.push({ t: "welcome", v: PROTOCOL_VERSION });
    relay.push({ t: "robots", robots: [ROBOT_A] });
    await until(() => relay.watches("a") === 1, "watch");

    relay.onMsg = (msg) => {
      if (msg.t !== "sub" || msg.ch !== "global_costmap") return;
      relay.pushRaw(1, new Uint8Array([1, 2, 3]), "global_costmap", {
        w: 2,
        h: 2,
        res: 0.5,
        origin: [0, 0, 0],
      });
      return new Promise((resolve) => setTimeout(resolve, 0));
    };
    relay.pushManifest(
      "a",
      manifest(
        [
          spec(),
          spec({ ch: "global_costmap", encoding: "costmap.zlib.v1", delivery: "latest" }),
        ],
        [panel({ id: "map", kind: "map2d", channels: ["global_costmap", "odom"] })],
      ),
    );

    await until(() => handle.channels.get("global_costmap") !== null, "stored costmap");
    const slot = handle.channels.get("global_costmap")!;
    expect((slot.value as CostmapValue).w).toBe(2);
    expect(slot.seq).toBe(1);
  });

  it("counts a corrupt jpeg frame as a decode error instead of storing it", async () => {
    const { relay, handle } = start();
    await goLive(
      relay,
      handle,
      ROBOT_A,
      [spec({ ch: "color_image", encoding: "jpeg.v1", delivery: "latest" })],
      [panel({ id: "cam", kind: "video", channels: ["color_image"] })],
    );
    // Garbage bytes with no JPEG SOI: the decoder must throw at ingest.
    relay.pushRaw(1, new Uint8Array([1, 2, 3, 4, 5, 6]), "color_image");
    await until(() => {
      handle.channels.publishUi();
      return handle.channels.getUiSnapshot("color_image").stats.frames === 1;
    }, "corrupt frame counted");
    expect(handle.channels.get("color_image")).toBeNull(); // never stored
    const { stats } = handle.channels.getUiSnapshot("color_image");
    expect(stats.decodeErrors).toBe(1);
    expect(stats.decodeFailing).toBe(true);
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
    relay.pushManifest("a", manifest([spec()]));
    await until(() => handle.status.get().lastError === null, "error cleared");
    expect(adopted(handle)).toEqual([spec()]);
  });

  it("clears the stale error on the next welcome", async () => {
    const { relay, handle } = start();
    relay.push({ t: "welcome", v: PROTOCOL_VERSION });
    relay.push({ t: "error", code: "no_watch", message: "watch a robot before sub/unsub" });
    await until(() => handle.status.get().lastError !== null, "error");

    relay.push({ t: "welcome", v: PROTOCOL_VERSION });
    await until(() => handle.status.get().lastError === null, "error cleared");
  });

  it("adopts an empty manifest from a bare reply (manifest-less robot)", async () => {
    const { relay, handle } = start();
    relay.push({ t: "welcome", v: PROTOCOL_VERSION });
    relay.push({ t: "robots", robots: [ROBOT_A] });
    await until(() => relay.watches("a") === 1, "watch");
    relay.push({ t: "manifest", robotId: "a" });
    await until(() => handle.status.get().manifest !== null, "empty manifest adopted");
    expect(handle.status.get().manifest).toEqual({
      version: 1,
      channels: [],
      panels: [],
      layout: null,
      pages: [],
    });
    expect(handle.status.get().lastError).toBeNull();
  });

  it("shows the polite flag on an unsupported manifest version", async () => {
    const { relay, handle } = start();
    await goLive(relay, handle);
    relay.pushFrame(7, { x: 7 });
    await until(() => handle.channels.get("odom")?.seq === 7, "value");

    // The robot restarted with a manifest this build cannot parse: stale
    // panels/data drop and the polite flag replaces the raw error.
    relay.push({
      t: "manifest",
      robotId: "a",
      manifest: { version: 2, channels: { alien: true } },
    });
    await until(() => handle.status.get().manifestUnsupported, "unsupported flag");
    expect(handle.status.get().manifest).toBeNull();
    expect(handle.status.get().lastError).toBeNull();
    expect(handle.channels.get("odom")).toBeNull();
    expect(handle.status.get().epoch).toBe(1);

    // A downgrade back to v1 recovers without a reload.
    relay.pushManifest("a", manifest([spec()]));
    await until(() => adopted(handle).length === 1, "readopted");
    expect(handle.status.get().manifestUnsupported).toBe(false);
  });

  it("sets lastError on an invalid (same-version) manifest", async () => {
    const { relay, handle } = start();
    relay.push({ t: "welcome", v: PROTOCOL_VERSION });
    relay.push({ t: "robots", robots: [ROBOT_A] });
    await until(() => relay.watches("a") === 1, "watch");
    relay.push({
      t: "manifest",
      robotId: "a",
      manifest: { version: 1, channels: [spec(), spec()] },
    });
    await until(() => handle.status.get().lastError !== null, "error");
    expect(handle.status.get().lastError).toContain("duplicate_channel_id");
    expect(handle.status.get().manifestUnsupported).toBe(false);
  });

  it("survives a same-relay robot restart: rewatch, reset, restarted seqs win", async () => {
    const { relay, handle } = start();
    await goLive(relay, handle);
    relay.pushFrame(500, { x: 500 });
    await until(() => handle.channels.get("odom")?.seq === 500, "first frame");

    // Robot gone: manifest, values, and the watch confirmation are dropped.
    relay.push({ t: "robots", robots: [] });
    await until(() => handle.status.get().manifest === null, "cleared manifest");
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
    relay.pushManifest("a", manifest([spec()]));
    await until(() => adopted(handle).length === 1, "manifest readopted");

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
    await until(() => handle.status.get().manifest === null, "cleared");

    relay.push({ t: "robots", robots: [ROBOT_B] });
    await until(() => relay.watches("b") === 1, "watch B");
    relay.pushManifest("b", manifest([spec()]));
    await until(() => adopted(handle).length === 1, "manifest B");

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
    relay.pushManifest("a", manifest([changed]));
    await until(() => adopted(handle)[0]?.ch === "status", "new manifest");
    expect(handle.status.get().epoch).toBe(1);
    expect(handle.channels.get("odom")).toBeNull();
  });

  it("drops data and remounts when only the layout changes", async () => {
    const { relay, handle } = start();
    const video = panel({ id: "cam", kind: "video", channels: ["color_image"] });
    const channels = [spec({ ch: "color_image", encoding: "jpeg.v1", delivery: "latest" })];
    await goLive(relay, handle, ROBOT_A, channels, [video]);
    expect(handle.status.get().epoch).toBe(0);

    relay.pushManifest("a", manifest(channels, [video], { row: ["cam"], shares: [2.5] }));
    await until(() => handle.status.get().epoch === 1, "layout-only remount");
    expect(handle.status.get().manifest?.layout).toEqual({ row: ["cam"], shares: [2.5] });
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
    expect(handle.status.get().manifest).toBeNull();
    expect(handle.channels.get("odom")).toBeNull();

    // A manifest reply that raced the second registration is not adopted.
    relay.pushManifest("a", manifest([spec()]));
    await settle();
    expect(handle.status.get().manifest).toBeNull();

    // A leaves; the survivor becomes the sole robot and gets watched.
    relay.push({ t: "robots", robots: [ROBOT_B] });
    await until(() => relay.watches("b") === 1, "watch B");
    relay.pushManifest("b", manifest([spec({ ch: "status" })]));
    await until(() => adopted(handle)[0]?.ch === "status", "manifest B");
  });
});

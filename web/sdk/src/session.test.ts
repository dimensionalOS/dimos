import { afterEach, describe, expect, it, vi } from "vitest";
import {
  type Msg,
  PROTOCOL_VERSION,
  type RobotInfo,
  TX_DATA_MAX_BYTES,
  txDataBytes,
} from "@dimos/shared";
import type { CostmapValue } from "./decoders/costmap.ts";
import { teleopHooks } from "./internal/teleopMachine.ts";
import {
  connect,
  type ConnectOptions,
  manifestsEqual,
  pickAutoWatch,
  type Session,
} from "./session.ts";
import {
  FakeRelayEnd,
  INFO,
  manifest,
  panel,
  ROBOT_A,
  ROBOT_B,
  settle,
  spec,
  until,
} from "./testing/fakeRelay.ts";

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
    const withLayout = (layout: Parameters<typeof manifest>[2]) =>
      manifest([spec()], [video], layout);
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

// ---------------------------------------------------------------------------
// Integration tests: the real Session against a fake relay behind the
// WebTransportLike seam (testing/fakeRelay.ts): robot lifecycle transitions
// run over the actual wire encoding while the relay and "viewer connection"
// stay up the whole time.

describe("Session over a fake WebTransport", () => {
  const handles: Session[] = [];

  afterEach(() => {
    for (const handle of handles.splice(0)) handle.close();
  });

  function start(options?: ConnectOptions): { relay: FakeRelayEnd; handle: Session } {
    const relay = new FakeRelayEnd();
    const handle = connect(options, {
      fetchInfo: () => Promise.resolve(INFO),
      createWebTransport: () => relay.wt,
    });
    handles.push(handle);
    return { relay, handle };
  }

  /** Like start(), but every (re)connection gets its own fresh relay end. */
  function startReconnecting(): { relays: FakeRelayEnd[]; handle: Session } {
    const relays: FakeRelayEnd[] = [];
    const handle = connect(undefined, {
      fetchInfo: () => Promise.resolve(INFO),
      createWebTransport: () => {
        const relay = new FakeRelayEnd();
        relays.push(relay);
        return relay.wt;
      },
    });
    handles.push(handle);
    return { relays, handle };
  }

  function adopted(handle: Session) {
    return handle.status.get().manifest?.channels ?? [];
  }

  async function goLive(
    relay: FakeRelayEnd,
    handle: Session,
    robot = ROBOT_A,
    channels = [spec()],
    panels: Parameters<typeof manifest>[1] = [],
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

  it("watches the sole robot and sends no subs without consumers", async () => {
    const { relay, handle } = start();
    await goLive(relay, handle, ROBOT_A, [
      spec(),
      spec({ ch: "color_image", encoding: "jpeg.v1", delivery: "latest" }),
    ]);
    expect(relay.watches("a")).toBe(1);
    await settle();
    expect(relay.subs()).toEqual([]);
    expect(handle.status.get().watchedRobot).toEqual(ROBOT_A);
    expect(handle.status.get().robots).toEqual([ROBOT_A]);
  });

  it("connect({robot}) pins the watch among multiple robots", async () => {
    const { relay, handle } = start({ robot: "b" });
    relay.push({ t: "welcome", v: PROTOCOL_VERSION });
    relay.push({ t: "robots", robots: [ROBOT_A, ROBOT_B] });
    await until(() => relay.watches("b") === 1, "watch b");
    expect(relay.watches("a")).toBe(0);
    expect(handle.status.get().watchedRobot).toEqual(ROBOT_B);
  });

  it("sends one sub for the first consumer and one unsub after the last", async () => {
    const { relay, handle } = start();
    await goLive(relay, handle);
    const first = handle.subscribe("odom", () => {});
    const second = handle.subscribe("odom", () => {});
    await until(() => relay.subs().length === 1, "sub");
    expect(relay.subs()).toEqual(["odom"]);

    first();
    await settle();
    expect(relay.unsubs()).toEqual([]);

    second();
    await until(() => relay.unsubs().length === 1, "unsub");
    expect(relay.unsubs()).toEqual(["odom"]);
    expect(relay.subs()).toEqual(["odom"]);

    second(); // releasing twice is a no-op
    await settle();
    expect(relay.unsubs()).toEqual(["odom"]);
  });

  it("stays silent when a subscription is released before any manifest", async () => {
    const { relay, handle } = start();
    relay.push({ t: "welcome", v: PROTOCOL_VERSION });
    relay.push({ t: "robots", robots: [ROBOT_A] });
    await until(() => relay.watches("a") === 1, "watch");

    const unsubscribe = handle.subscribe("odom", () => {});
    unsubscribe();
    relay.pushManifest("a", manifest([spec()]));
    await until(() => adopted(handle).length === 1, "manifest");
    await settle();
    expect(relay.subs()).toEqual([]);
    expect(relay.unsubs()).toEqual([]);
  });

  it("replays desired subscriptions after a reconnect", async () => {
    const { relays, handle } = startReconnecting();
    await until(() => relays.length === 1, "first connection");
    await goLive(relays[0], handle);
    handle.subscribe("odom", () => {});
    await until(() => relays[0].subs().length === 1, "sub on first connection");

    relays[0].wt.close();
    await until(() => relays.length === 2, "second connection");
    const relay = relays[1];
    relay.push({ t: "welcome", v: PROTOCOL_VERSION });
    relay.push({ t: "robots", robots: [ROBOT_A] });
    await until(() => relay.watches("a") === 1, "rewatch");
    relay.pushManifest("a", manifest([spec()]));
    await until(() => relay.subs().length === 1, "sub replayed");
    expect(relay.subs()).toEqual(["odom"]);
  });

  it("switches robots on manual watch and re-subs after the new adoption", async () => {
    const { relay, handle } = start();
    relay.push({ t: "welcome", v: PROTOCOL_VERSION });
    relay.push({ t: "robots", robots: [ROBOT_A, ROBOT_B] });
    await until(() => handle.status.get().robots.length === 2, "robots");
    expect(handle.status.get().watchedRobot).toBeNull(); // ambiguous: no auto watch

    const adoptionA = handle.watch("a");
    await until(() => relay.watches("a") === 1, "watch a");
    relay.pushManifest("a", manifest([spec()]));
    expect(await adoptionA).toEqual(manifest([spec()]));

    handle.subscribe("odom", () => {});
    await until(() => relay.subs().length === 1, "sub under a");

    const adoptionB = handle.watch("b");
    await until(() => relay.watches("b") === 1, "watch b");
    // Switching drops the old producer locally; no unsub is sent because the
    // relay clears this viewer's subscriptions on the watch switch itself.
    expect(handle.status.get().manifest).toBeNull();
    expect(relay.unsubs()).toEqual([]);

    relay.pushManifest("b", manifest([spec()]));
    expect(await adoptionB).toEqual(manifest([spec()]));
    await until(() => relay.subs().length === 2, "re-sub under b");
    expect(relay.subs()).toEqual(["odom", "odom"]);
  });

  it("supersedes a pending watch with a newer one", async () => {
    const { relay, handle } = start();
    relay.push({ t: "welcome", v: PROTOCOL_VERSION });
    relay.push({ t: "robots", robots: [ROBOT_A, ROBOT_B] });
    await until(() => handle.status.get().robots.length === 2, "robots");

    const first = handle.watch("a");
    const second = handle.watch("b");
    await expect(first).rejects.toMatchObject({
      name: "WatchRejectedError",
      reason: "superseded",
    });
    await until(() => relay.watches("b") === 1, "watch b");
    relay.pushManifest("b", manifest([]));
    expect(await second).toEqual(manifest([]));
  });

  it("rejects a pending watch on close", async () => {
    const { handle } = start();
    const pending = handle.watch("a");
    handle.close();
    await expect(pending).rejects.toMatchObject({ reason: "closed" });
  });

  it("flags a desired channel missing from the manifest and subs it later", async () => {
    const { relay, handle } = start();
    await goLive(relay, handle); // manifest carries only odom
    handle.subscribe("gps", () => {});
    await until(() => handle.status.get().lastError !== null, "unknown_channel");
    expect(handle.status.get().lastError).toEqual({
      code: "unknown_channel",
      ch: "gps",
      message: "channel gps is not an rx channel of the adopted manifest",
    });
    await settle();
    expect(relay.subs()).toEqual([]);

    // A later manifest carrying the channel satisfies the desire and clears
    // the error.
    relay.pushManifest("a", manifest([spec(), spec({ ch: "gps" })]));
    await until(() => relay.subs().includes("gps"), "gps subbed");
    expect(relay.subs()).toEqual(["gps"]);
    expect(handle.status.get().lastError).toBeNull();
  });

  it("revalidates wired subscriptions against every new manifest", async () => {
    const { relay, handle } = start();
    await goLive(relay, handle); // manifest carries odom
    handle.subscribe("odom", () => {});
    await until(() => relay.subs().length === 1, "sub");

    // The channel leaves the manifest: the wire interest is dropped
    // relay-side while the desire stays and surfaces as unknown_channel.
    relay.pushManifest("a", manifest([spec({ ch: "status" })]));
    await until(() => relay.unsubs().length === 1, "unsub");
    expect(relay.unsubs()).toEqual(["odom"]);
    expect(handle.status.get().lastError?.code).toBe("unknown_channel");

    // It returns: the desired set re-subscribes and the error clears.
    relay.pushManifest("a", manifest([spec()]));
    await until(() => relay.subs().length === 2, "re-sub");
    expect(relay.subs()).toEqual(["odom", "odom"]);
    expect(handle.status.get().lastError).toBeNull();
  });

  it("flags a tx channel as unsubscribable instead of subbing it", async () => {
    const { relay, handle } = start();
    await goLive(relay, handle, ROBOT_A, [
      spec(),
      spec({ ch: "tele_cmd_vel", dir: "tx", encoding: "twist.json.v1", delivery: "latest" }),
    ]);
    handle.subscribe("tele_cmd_vel", () => {});
    await until(() => handle.status.get().lastError?.code === "unknown_channel", "error");
    await settle();
    expect(relay.subs()).toEqual([]);
  });

  it("keeps notifying other subscribers when one callback throws", async () => {
    const { relay, handle } = start({ uiTickMs: 3_600_000 });
    await goLive(relay, handle);
    const seen: number[] = [];
    const errorSpy = vi.spyOn(console, "error").mockImplementation(() => {});
    try {
      handle.subscribe("odom", () => {
        throw new Error("boom");
      });
      handle.subscribe("odom", (snapshot) => {
        if (snapshot.slot !== null) seen.push(snapshot.slot.seq);
      });
      relay.pushFrame(1, { x: 1 });
      await until(() => handle.store.get("odom")?.seq === 1, "frame");
      handle.store.publishUi();
      expect(seen).toEqual([1]);
      expect(errorSpy).toHaveBeenCalled();
    } finally {
      errorSpy.mockRestore();
    }
  });

  it("keeps a reliable stream alive when a direct store subscriber throws", async () => {
    const { relay, handle } = start();
    await goLive(relay, handle);
    const errorSpy = vi.spyOn(console, "error").mockImplementation(() => {});
    try {
      handle.store.subscribe("odom", () => {
        throw new Error("boom");
      });
      // Both frames ride one uni stream, like a reliable channel's
      // persistent stream: the throw on the first frame must not abandon
      // the stream reader before the second.
      relay.pushFrames([{ seq: 1, value: { x: 1 } }, { seq: 2, value: { x: 2 } }]);
      await until(() => handle.store.get("odom")?.seq === 2, "both frames ingested");
      expect(errorSpy).toHaveBeenCalled();
    } finally {
      errorSpy.mockRestore();
    }
  });

  it("connect({url}) fetches the absolute /api/info", async () => {
    const urls: string[] = [];
    vi.stubGlobal(
      "fetch",
      vi.fn((input: string | URL | Request) => {
        urls.push(String(input));
        return Promise.resolve(new Response(JSON.stringify(INFO)));
      }),
    );
    try {
      const relay = new FakeRelayEnd();
      const handle = connect({ url: "https://relay.example:7780" }, {
        createWebTransport: () => relay.wt,
      });
      handles.push(handle);
      await until(() => urls.length === 1, "info fetch");
      expect(urls[0]).toBe("https://relay.example:7780/api/info");
    } finally {
      vi.unstubAllGlobals();
    }
  });

  it("stores a costmap frame that arrives while subs are still being sent", async () => {
    // The bridge replays the cached grid the moment a sub lands, so the frame
    // can beat the control loop's continuation. The hook injects it
    // synchronously at the sub and then holds the control writer for one
    // macrotask, letting the whole uni-stream ingest chain drain first: with
    // adoption after the sub reconcile, #ingest would drop it as
    // manifest-less.
    const { relay, handle } = start();
    relay.push({ t: "welcome", v: PROTOCOL_VERSION });
    relay.push({ t: "robots", robots: [ROBOT_A] });
    await until(() => relay.watches("a") === 1, "watch");
    handle.subscribe("global_costmap", () => {}); // desired before any manifest

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
      manifest([
        spec(),
        spec({ ch: "global_costmap", encoding: "costmap.zlib.v1", delivery: "latest" }),
      ]),
    );

    await until(() => handle.store.get("global_costmap") !== null, "stored costmap");
    const slot = handle.store.get("global_costmap")!;
    expect((slot.value as CostmapValue).w).toBe(2);
    expect(slot.seq).toBe(1);
  });

  it("counts a corrupt jpeg frame as a decode error instead of storing it", async () => {
    const { relay, handle } = start();
    await goLive(relay, handle, ROBOT_A, [
      spec({ ch: "color_image", encoding: "jpeg.v1", delivery: "latest" }),
    ]);
    // Garbage bytes with no JPEG SOI: the decoder must throw at ingest.
    relay.pushRaw(1, new Uint8Array([1, 2, 3, 4, 5, 6]), "color_image");
    await until(() => {
      handle.store.publishUi();
      return handle.store.getUiSnapshot("color_image").stats.frames === 1;
    }, "corrupt frame counted");
    expect(handle.store.get("color_image")).toBeNull(); // never stored
    const { stats } = handle.store.getUiSnapshot("color_image");
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
    await until(
      () => handle.status.get().lastError?.message === "unknown_robot: no robot a",
      "error",
    );
    expect(handle.status.get().lastError?.code).toBe("relay_error");

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
    await until(() => handle.store.get("odom")?.seq === 7, "value");

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
    expect(handle.store.get("odom")).toBeNull();
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
    expect(handle.status.get().lastError?.code).toBe("invalid_manifest");
    expect(handle.status.get().lastError?.message).toContain("duplicate_channel_id");
    expect(handle.status.get().manifestUnsupported).toBe(false);
  });

  it("survives a same-relay robot restart: rewatch, reset, restarted seqs win", async () => {
    const { relay, handle } = start();
    await goLive(relay, handle);
    relay.pushFrame(500, { x: 500 });
    await until(() => handle.store.get("odom")?.seq === 500, "first frame");

    // Robot gone: manifest, values, and the watch confirmation are dropped.
    relay.push({ t: "robots", robots: [] });
    await until(() => handle.status.get().manifest === null, "cleared manifest");
    expect(handle.status.get().robots).toEqual([]);
    expect(handle.status.get().epoch).toBe(1);
    expect(handle.store.get("odom")).toBeNull();

    // A frame still draining out of the dead robot's relay queue is ignored.
    relay.pushFrame(501, { x: 501 });
    await settle();
    expect(handle.store.get("odom")).toBeNull();

    // Same id returns: the watch must be re-sent even though the id never
    // changed, and the manifest re-adopted.
    relay.push({ t: "robots", robots: [ROBOT_A] });
    await until(() => relay.watches("a") === 2, "rewatch");
    relay.pushManifest("a", manifest([spec()]));
    await until(() => adopted(handle).length === 1, "manifest readopted");

    // A late high-seq frame from the dead producer must not lock out the
    // new producer's restarted counter.
    relay.pushFrame(502, { x: 502 });
    await until(() => handle.store.get("odom")?.seq === 502, "late old frame");
    relay.pushFrame(1, { x: 1 });
    await until(() => handle.store.get("odom")?.seq === 1, "restarted seq");
    expect(handle.store.get("odom")?.value).toEqual({ x: 1 });
  });

  it("does not carry values into a replacement robot with an identical manifest", async () => {
    const { relay, handle } = start();
    await goLive(relay, handle);
    relay.pushFrame(900, { x: 900 });
    await until(() => handle.store.get("odom")?.seq === 900, "value from A");

    relay.push({ t: "robots", robots: [] });
    await until(() => handle.status.get().manifest === null, "cleared");

    relay.push({ t: "robots", robots: [ROBOT_B] });
    await until(() => relay.watches("b") === 1, "watch B");
    relay.pushManifest("b", manifest([spec()]));
    await until(() => adopted(handle).length === 1, "manifest B");

    // Identical manifest, different robot: A's value must not show under B.
    expect(handle.store.get("odom")).toBeNull();
    expect(handle.status.get().watchedRobot).toEqual(ROBOT_B);
    relay.pushFrame(1, { x: 1 });
    await until(() => handle.store.get("odom")?.seq === 1, "value from B");
    expect(handle.store.get("odom")?.value).toEqual({ x: 1 });
  });

  it("drops data and remounts when the watched robot's manifest changes", async () => {
    const { relay, handle } = start();
    await goLive(relay, handle);
    relay.pushFrame(900, { x: 900 });
    await until(() => handle.store.get("odom")?.seq === 900, "value");

    const changed = spec({ ch: "status" });
    relay.pushManifest("a", manifest([changed]));
    await until(() => adopted(handle)[0]?.ch === "status", "new manifest");
    expect(handle.status.get().epoch).toBe(1);
    expect(handle.store.get("odom")).toBeNull();
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
    await until(() => handle.store.get("odom")?.seq === 900, "value from A");

    // A second robot appears: auto-watch is ambiguous, everything clears.
    relay.push({ t: "robots", robots: [ROBOT_A, ROBOT_B] });
    await until(() => handle.status.get().robots.length === 2, "two robots");
    expect(handle.status.get().watchedRobot).toBeNull();
    expect(handle.status.get().manifest).toBeNull();
    expect(handle.store.get("odom")).toBeNull();

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

  it("teleop control rides the control stream and twists ride datagrams", async () => {
    const { relay, handle } = start();
    await goLive(relay, handle);
    const teleop = teleopHooks(handle);
    teleop.control({ t: "teleop_start" });
    await until(() => relay.sent.some((m) => m.t === "teleop_start"), "teleop_start");
    const twist: Msg = { t: "twist", vx: 0.5, vy: 0.25, wz: -0.25, seq: 1, ts: 1.5 };
    teleop.datagram(twist);
    await until(() => relay.sentDatagrams.length === 1, "twist datagram");
    expect(relay.sentDatagrams[0]).toEqual(twist);
    expect(relay.sent.some((m) => m.t === "twist")).toBe(false); // not on the stream
  });

  it("routes teleop_started and teleop_held to the facade, not lastError", async () => {
    const { relay, handle } = start();
    await goLive(relay, handle);
    const received: Msg[] = [];
    const unsubscribe = teleopHooks(handle).onMsg((msg) => received.push(msg));
    relay.push({ t: "teleop_started" });
    await until(() => received.length === 1, "teleop_started routed");
    relay.push({ t: "error", code: "teleop_held", message: "held by viewer 1" });
    await until(() => received.length === 2, "teleop_held routed");
    expect(received.map((m) => m.t)).toEqual(["teleop_started", "error"]);
    expect(handle.status.get().lastError).toBeNull();
    unsubscribe();
    relay.push({ t: "teleop_started" });
    await settle();
    expect(received).toHaveLength(2);
  });

  it("other error codes still land in lastError", async () => {
    const { relay, handle } = start();
    await goLive(relay, handle);
    relay.push({ t: "error", code: "unknown_channel", message: "no channel x" });
    await until(() => handle.status.get().lastError !== null, "lastError");
    expect(handle.status.get().lastError?.message).toContain("unknown_channel");
  });

  // --- Generic tx command (TeleopHooks.tx) ---

  const TX_CHANNELS = [
    spec(),
    spec({ ch: "tele_cmd_vel", dir: "tx", encoding: "twist.json.v1", delivery: "latest" }),
    spec({ ch: "human_input", dir: "tx", encoding: "text.json.v1", delivery: "reliable" }),
    spec({ ch: "goal_request", dir: "tx", encoding: "pose_goal.json.v1", delivery: "reliable" }),
  ];

  function sentTx(relay: FakeRelayEnd): Msg[] {
    return relay.sent.filter((m) => m.t === "tx");
  }

  it("tx sends {t, ch, seq, data} on the control stream once live", async () => {
    const { relay, handle } = start();
    await goLive(relay, handle, ROBOT_A, TX_CHANNELS);
    const data = { text: "go to the kitchen" };
    expect(teleopHooks(handle).tx("human_input", data)).toEqual({ ok: true, seq: 1 });
    await until(() => sentTx(relay).length === 1, "tx on stream");
    // Decoded by the wire reader, which rejects a tx with any extra key.
    const msg = sentTx(relay)[0];
    expect(msg).toEqual({ t: "tx", ch: "human_input", seq: 1, data });
    expect(Object.keys(msg)).toEqual(["t", "ch", "seq", "data"]);
    expect(relay.sentDatagrams).toEqual([]); // never a datagram
  });

  it("tx seq counts up per channel from 1 and only on accepted sends", async () => {
    const { relay, handle } = start();
    await goLive(relay, handle, ROBOT_A, TX_CHANNELS);
    const teleop = teleopHooks(handle);
    expect(teleop.tx("human_input", { text: "a" })).toEqual({ ok: true, seq: 1 });
    expect(teleop.tx("human_input", { text: "b" })).toEqual({ ok: true, seq: 2 });
    expect(teleop.tx("goal_request", { x: 1, y: 2 })).toEqual({ ok: true, seq: 1 });
    // A refused send spends no seq on either channel.
    expect(teleop.tx("human_input", { text: "x".repeat(TX_DATA_MAX_BYTES) }).ok).toBe(false);
    expect(teleop.tx("human_input", { text: "c" })).toEqual({ ok: true, seq: 3 });
    expect(teleop.tx("goal_request", { x: 3, y: 4 })).toEqual({ ok: true, seq: 2 });
    await until(() => sentTx(relay).length === 5, "five tx");
    expect(sentTx(relay).map((m) => (m.t === "tx" ? [m.ch, m.seq] : null))).toEqual([
      ["human_input", 1],
      ["human_input", 2],
      ["goal_request", 1],
      ["human_input", 3],
      ["goal_request", 2],
    ]);
  });

  it("tx is refused with disconnected before the relay's welcome", async () => {
    const { relay, handle } = start();
    await until(() => relay.sent.some((m) => m.t === "hello"), "hello");
    expect(teleopHooks(handle).tx("human_input", { text: "hi" })).toEqual({
      ok: false,
      reason: "disconnected",
    });
    await settle();
    expect(sentTx(relay)).toEqual([]);
  });

  it("tx is refused with no_manifest while connected but before adoption", async () => {
    const { relay, handle } = start();
    relay.push({ t: "welcome", v: PROTOCOL_VERSION });
    relay.push({ t: "robots", robots: [ROBOT_A] });
    await until(() => relay.watches("a") === 1, "watch");
    expect(handle.status.get().transport.phase).toBe("connected");
    expect(teleopHooks(handle).tx("human_input", { text: "hi" })).toEqual({
      ok: false,
      reason: "no_manifest",
    });
    await settle();
    expect(sentTx(relay)).toEqual([]);
  });

  it("tx refuses a malformed channel name before consulting the manifest", async () => {
    const { relay, handle } = start();
    await goLive(relay, handle, ROBOT_A, TX_CHANNELS);
    const teleop = teleopHooks(handle);
    for (const ch of ["Human_Input", "human-input", "1st", "", "x".repeat(65)]) {
      expect(teleop.tx(ch, { text: "hi" })).toEqual({ ok: false, reason: "bad_channel" });
    }
    await settle();
    expect(sentTx(relay)).toEqual([]);
  });

  it("tx to a channel the manifest lacks is unknown_channel", async () => {
    const { relay, handle } = start();
    await goLive(relay, handle, ROBOT_A, TX_CHANNELS);
    expect(teleopHooks(handle).tx("mystery", { text: "hi" })).toEqual({
      ok: false,
      reason: "unknown_channel",
    });
    await settle();
    expect(sentTx(relay)).toEqual([]);
  });

  it("tx to an rx channel or the twist channel is not_tx", async () => {
    const { relay, handle } = start();
    await goLive(relay, handle, ROBOT_A, TX_CHANNELS);
    const teleop = teleopHooks(handle);
    expect(teleop.tx("odom", { x: 0 })).toEqual({ ok: false, reason: "not_tx" });
    expect(teleop.tx("tele_cmd_vel", { vx: 1, vy: 0, wz: 0 })).toEqual({
      ok: false,
      reason: "not_tx",
    });
    await settle();
    expect(sentTx(relay)).toEqual([]);
  });

  it("tx enforces the shared data cap: over is too_large, at the cap goes", async () => {
    const { relay, handle } = start();
    await goLive(relay, handle, ROBOT_A, TX_CHANNELS);
    const teleop = teleopHooks(handle);
    // {"text":"..."} is 11 bytes of framing around the string.
    const atCap = { text: "x".repeat(TX_DATA_MAX_BYTES - 11) };
    const over = { text: "x".repeat(TX_DATA_MAX_BYTES - 10) };
    expect(txDataBytes(atCap)).toBe(TX_DATA_MAX_BYTES);
    expect(teleop.tx("human_input", over)).toEqual({ ok: false, reason: "too_large" });
    expect(teleop.tx("human_input", atCap)).toEqual({ ok: true, seq: 1 });
    await until(() => sentTx(relay).length === 1, "tx at cap");
    expect(sentTx(relay)[0]).toEqual({ t: "tx", ch: "human_input", seq: 1, data: atCap });
  });

  it("tx seq survives a reconnect instead of restarting at 1", async () => {
    const { relays, handle } = startReconnecting();
    await until(() => relays.length === 1, "first connection");
    await goLive(relays[0], handle, ROBOT_A, TX_CHANNELS);
    const teleop = teleopHooks(handle);
    expect(teleop.tx("human_input", { text: "a" })).toEqual({ ok: true, seq: 1 });
    expect(teleop.tx("human_input", { text: "b" })).toEqual({ ok: true, seq: 2 });
    await until(() => sentTx(relays[0]).length === 2, "two tx");

    relays[0].wt.close();
    await until(() => relays.length === 2, "second connection");
    expect(teleop.tx("human_input", { text: "lost" })).toEqual({
      ok: false,
      reason: "disconnected",
    });
    await goLive(relays[1], handle, ROBOT_A, TX_CHANNELS);
    expect(teleop.tx("human_input", { text: "c" })).toEqual({ ok: true, seq: 3 });
    await until(() => sentTx(relays[1]).length === 1, "tx after reconnect");
    expect(sentTx(relays[1])[0]).toMatchObject({ ch: "human_input", seq: 3 });
  });
});

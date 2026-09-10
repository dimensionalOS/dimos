// Registry unit tests over fake peers: subscription snapshot transitions,
// duplicate-id rejection, watch switching, robots pushes, and frame routing -
// no QUIC.
import { assert, assertEquals } from "@std/assert";
import {
  type ChannelSpec,
  encodeDataFrame,
  type FrameHeader,
  type JsonValue,
  type ManifestMsg,
  type Msg,
  PROTOCOL_VERSION,
  type RobotInfo,
  type RobotManifest,
  type SubsMsg,
} from "@dimos/shared";
import type { CarrierStats } from "./carrier.ts";
import {
  type ChannelPolicy,
  type FrameSend,
  type FrameWriter,
  LATEST_STALE_MS,
  LatestChannel,
  ReliableChannel,
  type ViewerSink,
} from "./forward.ts";
import { PUBLISH_TIMEOUT_MS, Registry, type RobotPeer, type ViewerPeer } from "./registry.ts";

class FakeSink implements ViewerSink {
  sent: Uint8Array[] = [];
  kicked: string | null = null;
  streamsOpened = 0;
  streamsAborted = 0;
  auto = true;
  #waiters: (() => void)[] = [];

  sendFrame(bytes: Uint8Array): FrameSend {
    this.sent.push(bytes);
    let settle!: () => void;
    let fail!: (e: Error) => void;
    const done = new Promise<void>((resolve, reject) => {
      settle = resolve;
      fail = reject;
    });
    if (this.auto) settle();
    else this.#waiters.push(settle);
    let aborted = false;
    return {
      done,
      get aborted() {
        return aborted;
      },
      abort() {
        if (aborted) return;
        aborted = true;
        fail(new Error("frame send aborted"));
      },
      // Registry tests exercise the parking path, not the create wedge.
      supersede: () => false,
    };
  }

  openStream(): Promise<FrameWriter> {
    this.streamsOpened++;
    return Promise.resolve({
      write: (bytes: Uint8Array) => {
        this.sent.push(bytes);
        if (this.auto) return Promise.resolve();
        return new Promise<void>((resolve) => this.#waiters.push(resolve));
      },
      abort: () => {
        this.streamsAborted++;
        return Promise.resolve();
      },
    });
  }

  release(n = 1): void {
    while (n-- > 0) this.#waiters.shift()?.();
  }

  kick(reason: string): void {
    this.kicked = reason;
  }
}

class FakeRobot implements RobotPeer {
  info: RobotInfo | null;
  channels: ChannelSpec[];
  manifest: RobotManifest | null;
  /** Datagram control (welcome/error/pong/teleop). */
  msgs: Msg[] = [];
  /** Robot control carrier messages (subs snapshots). */
  control: Msg[] = [];
  /** Forwarded publishes (tx data frames on the carrier). */
  pubs: { ch: string; payload: Uint8Array; meta: Record<string, unknown> }[] = [];
  closed: string | null = null;

  constructor(id: string, channels: ChannelSpec[] = [], manifest: RobotManifest | null = null) {
    this.info = { id, name: id, model: "test" };
    this.channels = channels;
    this.manifest = manifest;
  }

  sendMsg(msg: Msg): void {
    this.msgs.push(msg);
  }

  sendControl(msg: Msg): void {
    this.control.push(msg);
  }

  sendPub(ch: string, payload: Uint8Array, meta: Record<string, unknown>): void {
    this.pubs.push({ ch, payload, meta });
  }

  carrierStats(): CarrierStats {
    return { queued: 0, queuedBytes: 0, sent: this.control.length, bytesOut: 0 };
  }

  subs(): SubsMsg[] {
    return this.control.filter((m): m is SubsMsg => m.t === "subs");
  }

  lastSubs(): SubsMsg {
    const all = this.subs();
    assert(all.length > 0, "expected at least one subs snapshot");
    return all[all.length - 1];
  }
}

class FakeViewer implements ViewerPeer {
  static nextId = 1;
  readonly id = FakeViewer.nextId++;
  watched: string | null = null;
  readonly subs = new Set<string>();
  readonly policies = new Map<string, ChannelPolicy>();
  readonly sink = new FakeSink();
  greeted = false;
  pushed: Msg[] = [];
  replies: Msg[] = [];

  sendMsg(msg: Msg): void {
    this.pushed.push(msg);
  }
}

function send(reg: Registry, viewer: FakeViewer, msg: Msg): boolean {
  return reg.onViewerMsg(viewer, msg, (m) => viewer.replies.push(m));
}

/** hello -> watch -> sub for each channel; returns the greeted viewer. */
function attach(reg: Registry, robotId: string, chs: string[]): FakeViewer {
  const viewer = new FakeViewer();
  reg.addViewer(viewer);
  send(reg, viewer, { t: "hello", v: PROTOCOL_VERSION, role: "viewer" });
  send(reg, viewer, { t: "watch", robotId });
  for (const ch of chs) send(reg, viewer, { t: "sub", ch });
  return viewer;
}

function frame(ch: string, seq: number, delivery: "latest" | "reliable" = "latest"): Uint8Array {
  const header: FrameHeader = { ch, seq, ts: seq + 0.5, delivery };
  return encodeDataFrame(header, new Uint8Array([seq]));
}

function tick(): Promise<void> {
  return new Promise((resolve) => setTimeout(resolve, 0));
}

function rxSpec(ch: string, encoding: string, delivery: "latest" | "reliable"): ChannelSpec {
  return {
    ch,
    dir: "rx",
    encoding,
    delivery,
    maxHz: 15,
    params: {},
    publish: "none",
    requiredScope: null,
  };
}

const SPECS: ChannelSpec[] = [
  rxSpec("color_image", "jpeg.v1", "latest"),
  rxSpec("odom", "pose.json.v1", "reliable"),
];

Deno.test("snapshots fire on 0->1 and ->0, not on redundant subs", () => {
  const reg = new Registry();
  const robot = new FakeRobot("r1", SPECS);
  reg.registerRobot(robot);
  assertEquals(robot.lastSubs(), { t: "subs", chs: [], n: 1 }); // forced baseline

  const v1 = attach(reg, "r1", ["color_image"]);
  assertEquals(robot.lastSubs(), { t: "subs", chs: ["color_image"], n: 2 });

  const before = robot.subs().length;
  const v2 = attach(reg, "r1", ["color_image"]); // same channel: set unchanged
  assertEquals(robot.subs().length, before);

  send(reg, v2, { t: "unsub", ch: "color_image" }); // still one subscriber
  assertEquals(robot.subs().length, before);

  send(reg, v1, { t: "unsub", ch: "color_image" }); // ->0
  assertEquals(robot.lastSubs(), { t: "subs", chs: [], n: 3 });
});

Deno.test("viewer disconnect behaves like unsub", () => {
  const reg = new Registry();
  const robot = new FakeRobot("r1", SPECS);
  reg.registerRobot(robot);
  const viewer = attach(reg, "r1", ["odom", "color_image"]);
  assertEquals(robot.lastSubs().chs, ["color_image", "odom"]);
  reg.viewerClosed(viewer);
  assertEquals(robot.lastSubs(), { t: "subs", chs: [], n: robot.subs().length });
});

Deno.test("watch switch moves subscriptions between robots", () => {
  const reg = new Registry();
  const r1 = new FakeRobot("r1", SPECS);
  const r2 = new FakeRobot("r2", SPECS);
  reg.registerRobot(r1);
  reg.registerRobot(r2);
  const viewer = attach(reg, "r1", ["odom"]);
  assertEquals(r1.lastSubs().chs, ["odom"]);

  send(reg, viewer, { t: "watch", robotId: "r2" }); // subs cleared on switch
  assertEquals(r1.lastSubs().chs, []);
  assertEquals(viewer.subs.size, 0);
  send(reg, viewer, { t: "sub", ch: "color_image" });
  assertEquals(r2.lastSubs().chs, ["color_image"]);
});

Deno.test("re-watching the same robot keeps subscriptions", () => {
  const reg = new Registry();
  // Deliberately not normalized: the reply must carry the robot's manifest
  // verbatim, not a re-serialized subset.
  const raw: RobotManifest = {
    version: 1,
    channels: [{ ch: "odom", encoding: "pose.json.v1", delivery: "reliable", maxHz: 20 }],
    panels: [{ id: "color_image", kind: "video", channels: ["color_image"] }],
    layout: "color_image",
  };
  const robot = new FakeRobot("r1", SPECS, raw);
  reg.registerRobot(robot);
  const viewer = attach(reg, "r1", ["odom"]);
  send(reg, viewer, { t: "watch", robotId: "r1" });
  assertEquals(viewer.subs, new Set(["odom"]));
  const manifests = viewer.replies.filter((m): m is ManifestMsg => m.t === "manifest");
  assertEquals(manifests.length, 2);
  assertEquals(manifests[1].manifest, raw); // forwarded verbatim
});

Deno.test("duplicate live robot id is rejected; reconnect works after close", () => {
  const reg = new Registry(() => 0);
  const first = new FakeRobot("r1", [...SPECS, pubSpec("chat", 1)]);
  assertEquals(reg.registerRobot(first), true);
  const watcher = attach(reg, "r1", ["odom"]);

  const second = new FakeRobot("r1", [...SPECS, pubSpec("chat", 100)]);
  assertEquals(reg.registerRobot(second), false);
  assertEquals(first.closed, null);
  assertEquals(second.subs(), []);
  assertEquals(watcher.pushed, []);
  assertEquals((reg.robotsMsg() as { robots: RobotInfo[] }).robots, [first.info!]);

  // A publisher empties its 1 Hz chat bucket against the first registration
  // (capacity 1, zero refill under the frozen clock).
  const publisher = attach(reg, "r1", []);
  pub(reg, publisher, "p-1", "chat", 1.5);
  assertEquals(first.pubs.length, 1);
  pub(reg, publisher, "p-2", "chat", 1.5);
  assertEquals(lastError(publisher).code, "rate_limited");

  reg.robotClosed(first);
  assertEquals(reg.registerRobot(second), true);
  // The returning robot reattaches the surviving viewer state and announces
  // itself only after the old live registration is gone.
  assertEquals(watcher.pushed, [
    { t: "robots", robots: [] },
    { t: "robots", robots: [second.info!] },
  ]);
  assertEquals(second.lastSubs(), { t: "subs", chs: ["odom"], n: 1 });

  // The restart raised chat's maxHz 1 -> 100: the surviving viewer's stale
  // (empty) bucket must be rebuilt at the new rate, not keep limiting at the
  // dead registration's 1 Hz.
  for (let i = 0; i < 5; i++) pub(reg, publisher, `q${i}`, "chat", 1.5);
  assertEquals(second.pubs.length, 5);
  assertEquals(pubStats(reg).rejected.rate_limited, 1); // only the p-2 probe
});

Deno.test("hello replies welcome + robots; register/close push robots", () => {
  const reg = new Registry();
  const viewer = new FakeViewer();
  reg.addViewer(viewer);
  send(reg, viewer, { t: "hello", v: PROTOCOL_VERSION, role: "viewer" });
  assertEquals(viewer.replies[0], { t: "welcome", v: PROTOCOL_VERSION });
  assertEquals(viewer.replies[1], { t: "robots", robots: [] });

  const robot = new FakeRobot("r1", SPECS);
  reg.registerRobot(robot);
  assertEquals(viewer.pushed, [{ t: "robots", robots: [robot.info!] }]);

  // A viewer that never helloed gets no pushes.
  const silent = new FakeViewer();
  reg.addViewer(silent);
  reg.robotClosed(robot);
  assertEquals(viewer.pushed[1], { t: "robots", robots: [] });
  assertEquals(silent.pushed, []);
});

Deno.test("viewer hello rejects wrong version and role", () => {
  const reg = new Registry();
  const badVersion = new FakeViewer();
  reg.addViewer(badVersion);
  assertEquals(send(reg, badVersion, { t: "hello", v: 99, role: "viewer" }), false);
  assertEquals((badVersion.replies[0] as { code: string }).code, "version_mismatch");
  assertEquals(badVersion.greeted, false);

  // A v1 (T2-era) viewer would decode one frame per reliable stream and then
  // silently freeze on the v2 persistent stream; it must be rejected too.
  const v1Viewer = new FakeViewer();
  reg.addViewer(v1Viewer);
  assertEquals(send(reg, v1Viewer, { t: "hello", v: 1, role: "viewer" }), false);
  assertEquals((v1Viewer.replies[0] as { code: string }).code, "version_mismatch");
  assertEquals(v1Viewer.greeted, false);

  const badRole = new FakeViewer();
  reg.addViewer(badRole);
  assertEquals(
    send(reg, badRole, { t: "hello", v: PROTOCOL_VERSION, role: "robot" }),
    false,
  );
  assertEquals((badRole.replies[0] as { code: string }).code, "role_mismatch");
  assertEquals(badRole.greeted, false);
});

Deno.test("viewer commands before hello are rejected without changing state", () => {
  const reg = new Registry();
  const robot = new FakeRobot("r1", SPECS);
  reg.registerRobot(robot);
  const viewer = new FakeViewer();
  reg.addViewer(viewer);

  assertEquals(send(reg, viewer, { t: "watch", robotId: "r1" }), false);
  assertEquals((viewer.replies[0] as { code: string }).code, "hello_required");
  assertEquals(viewer.watched, null);
  assertEquals(viewer.subs, new Set());
});

Deno.test("frames route only to watching+subscribed viewers", async () => {
  const reg = new Registry();
  const r1 = new FakeRobot("r1", SPECS);
  const r2 = new FakeRobot("r2", SPECS);
  reg.registerRobot(r1);
  reg.registerRobot(r2);
  const subscribed = attach(reg, "r1", ["odom"]);
  const otherChannel = attach(reg, "r1", ["color_image"]);
  const otherRobot = attach(reg, "r2", ["odom"]);
  const noSub = attach(reg, "r1", []);

  reg.onRobotFrame(r1, frame("odom", 1));
  // The reliable policy opens its persistent stream before the first write.
  await tick();
  assertEquals(subscribed.sink.sent.length, 1);
  assertEquals(otherChannel.sink.sent.length, 0);
  assertEquals(otherRobot.sink.sent.length, 0);
  assertEquals(noSub.sink.sent.length, 0);
});

Deno.test("watch switch disposes the old robot's policies", async () => {
  const reg = new Registry();
  const r1 = new FakeRobot("r1", SPECS);
  const r2 = new FakeRobot("r2", SPECS);
  reg.registerRobot(r1);
  reg.registerRobot(r2);
  const viewer = attach(reg, "r1", ["odom"]);
  viewer.sink.auto = false;
  reg.onRobotFrame(r1, frame("odom", 1));
  reg.onRobotFrame(r1, frame("odom", 2));
  await tick(); // stream opened; frame 1 in flight, frame 2 queued
  send(reg, viewer, { t: "watch", robotId: "r2" });
  assertEquals(viewer.policies.size, 0);
  assertEquals(viewer.sink.streamsAborted, 1);
  viewer.sink.release(); // the in-flight write completes after the switch
  await tick();
  assertEquals(viewer.sink.sent.length, 1); // the queued r1 frame never went out
  assertEquals(viewer.sink.kicked, null);
});

Deno.test("rapid watch switches release every abandoned persistent stream", async () => {
  const reg = new Registry();
  const robots: Record<string, FakeRobot> = {
    r1: new FakeRobot("r1", SPECS),
    r2: new FakeRobot("r2", SPECS),
  };
  reg.registerRobot(robots.r1);
  reg.registerRobot(robots.r2);
  const viewer = attach(reg, "r1", ["odom"]);
  reg.onRobotFrame(robots.r1, frame("odom", 0));
  let watching = "r1";
  for (let i = 1; i <= 6; i++) {
    watching = watching === "r1" ? "r2" : "r1";
    send(reg, viewer, { t: "watch", robotId: watching });
    send(reg, viewer, { t: "sub", ch: "odom" });
    reg.onRobotFrame(robots[watching], frame("odom", i));
  }
  await tick();
  assertEquals(viewer.sink.streamsOpened, 7);
  assertEquals(viewer.sink.streamsAborted, 6); // only the live stream remains
  assertEquals(viewer.sink.kicked, null);
});

Deno.test("unsub disposes the channel's policy and releases its stream", async () => {
  const reg = new Registry();
  const robot = new FakeRobot("r1", SPECS);
  reg.registerRobot(robot);
  const viewer = attach(reg, "r1", ["odom"]);
  reg.onRobotFrame(robot, frame("odom", 1));
  await tick();
  send(reg, viewer, { t: "unsub", ch: "odom" });
  assertEquals(viewer.policies.size, 0);
  assertEquals(viewer.sink.streamsAborted, 1);
  // Re-subscribing starts a fresh policy on a fresh stream.
  send(reg, viewer, { t: "sub", ch: "odom" });
  reg.onRobotFrame(robot, frame("odom", 2));
  await tick();
  assertEquals(viewer.sink.streamsOpened, 2);
  assertEquals(viewer.sink.sent.length, 2);
});

Deno.test("a delivery change disposes the superseded policy", async () => {
  const reg = new Registry();
  const robot = new FakeRobot("r1", []); // no manifest: the frame header rules
  reg.registerRobot(robot);
  const viewer = attach(reg, "r1", ["tele"]);
  reg.onRobotFrame(robot, frame("tele", 1, "reliable"));
  await tick();
  assert(viewer.policies.get("tele") instanceof ReliableChannel);
  reg.onRobotFrame(robot, frame("tele", 2, "latest"));
  assert(viewer.policies.get("tele") instanceof LatestChannel);
  assertEquals(viewer.sink.streamsAborted, 1); // the reliable writer released
  await tick();
  assertEquals(viewer.sink.sent.length, 2);
});

Deno.test("viewer teardown disposes its policies", async () => {
  const reg = new Registry();
  const robot = new FakeRobot("r1", SPECS);
  reg.registerRobot(robot);
  const viewer = attach(reg, "r1", ["odom"]);
  reg.onRobotFrame(robot, frame("odom", 1));
  await tick();
  reg.viewerClosed(viewer);
  assertEquals(viewer.policies.size, 0);
  assertEquals(viewer.sink.streamsAborted, 1);
});

Deno.test("manifest delivery wins over the frame header's", () => {
  const reg = new Registry();
  const robot = new FakeRobot("r1", SPECS); // odom declared reliable
  reg.registerRobot(robot);
  const viewer = attach(reg, "r1", ["odom"]);
  reg.onRobotFrame(robot, frame("odom", 1, "latest")); // header says latest
  assert(viewer.policies.get("odom") instanceof ReliableChannel);
});

Deno.test("sub to an undeclared channel is rejected", () => {
  const reg = new Registry();
  const robot = new FakeRobot("r1", SPECS);
  reg.registerRobot(robot);
  const viewer = attach(reg, "r1", []);
  const before = robot.subs().length;
  send(reg, viewer, { t: "sub", ch: "mystery" });
  assertEquals((viewer.replies.at(-1) as { code: string }).code, "unknown_channel");
  assertEquals(viewer.subs.size, 0);
  assertEquals(robot.subs().length, before); // no new snapshot went out
});

Deno.test("a robot that declared no manifest accepts any sub", () => {
  // The transport e2e tests hello without a manifest and steer delivery via
  // frame headers; with nothing to validate against, subs pass through.
  const reg = new Registry();
  const robot = new FakeRobot("r1", []);
  reg.registerRobot(robot);
  const viewer = attach(reg, "r1", ["anything"]);
  assertEquals(robot.lastSubs().chs, ["anything"]);
  assertEquals(viewer.replies.filter((m) => m.t === "error"), []);
});

Deno.test("sub to a reserved @-channel is rejected even without a manifest", () => {
  // @-ids are protocol control; they must never enter subs or snapshots.
  const reg = new Registry();
  const robot = new FakeRobot("r1", []);
  reg.registerRobot(robot);
  const viewer = attach(reg, "r1", []);
  const before = robot.subs().length;
  send(reg, viewer, { t: "sub", ch: "@control" });
  assertEquals((viewer.replies.at(-1) as { code: string }).code, "unknown_channel");
  assertEquals(viewer.subs.size, 0);
  assertEquals(robot.subs().length, before); // no new snapshot went out
});

Deno.test("sub with an over-long channel id is rejected even without a manifest", () => {
  // Manifest ids are length-bounded; undeclared subs get the same cap so a
  // manifest-less robot's snapshot cannot outgrow the @control payload cap
  // (which would fail its carrier and kill the session).
  const reg = new Registry();
  const robot = new FakeRobot("r1", []);
  reg.registerRobot(robot);
  const viewer = attach(reg, "r1", []);
  const before = robot.subs().length;
  send(reg, viewer, { t: "sub", ch: "x".repeat(65) });
  assertEquals((viewer.replies.at(-1) as { code: string }).code, "unknown_channel");
  assertEquals(viewer.subs.size, 0);
  assertEquals(robot.subs().length, before); // no new snapshot went out
});

Deno.test("sub while the watched robot is offline is rejected", () => {
  const reg = new Registry();
  const robot = new FakeRobot("r1", SPECS);
  reg.registerRobot(robot);
  const viewer = attach(reg, "r1", []);
  reg.robotClosed(robot);
  send(reg, viewer, { t: "sub", ch: "odom" });
  assertEquals((viewer.replies.at(-1) as { code: string }).code, "unknown_robot");
  assertEquals(viewer.subs.size, 0);
});

Deno.test("reconnect-stale sub is filtered from snapshots, frames fall back to header delivery", () => {
  const reg = new Registry();
  const withMystery: ChannelSpec[] = [...SPECS, rxSpec("mystery", "x", "latest")];
  const first = new FakeRobot("r1", withMystery);
  reg.registerRobot(first);
  const viewer = attach(reg, "r1", ["mystery"]);
  assertEquals(first.lastSubs().chs, ["mystery"]);

  const second = new FakeRobot("r1", SPECS); // manifest shrank across restart
  reg.robotClosed(first);
  reg.registerRobot(second);
  // The surviving sub is no longer in the manifest: snapshots must not carry
  // it (the bridge has no input to activate for it) ...
  assertEquals(second.lastSubs().chs, []);
  // ... but if the robot still sends the channel, routing falls back to the
  // frame header's delivery.
  reg.onRobotFrame(second, frame("mystery", 1, "latest"));
  assert(viewer.policies.get("mystery") instanceof LatestChannel);
});

Deno.test("invalid and unregistered frames are dropped and counted", () => {
  const reg = new Registry();
  const ghost = new FakeRobot("ghost", []);
  reg.onRobotFrame(ghost, frame("odom", 1)); // never registered
  const robot = new FakeRobot("r1", SPECS);
  reg.registerRobot(robot);
  const viewer = attach(reg, "r1", ["odom"]);
  reg.onRobotFrame(robot, new Uint8Array(16)); // headerLen=0 -> invalid
  assertEquals(viewer.sink.sent.length, 0);
  const stats = reg.stats() as { framesDropped: number; framesFromUnregistered: number };
  assertEquals(stats.framesDropped, 1);
  assertEquals(stats.framesFromUnregistered, 1);
});

Deno.test("reapAll resets stale accepted latest streams", async () => {
  // What server.ts drives on an interval: an idle input stops offering, so
  // without this the last accepted stream would stay open indefinitely.
  const reg = new Registry();
  const robot = new FakeRobot("r1", SPECS);
  reg.registerRobot(robot);
  const viewer = attach(reg, "r1", ["color_image"]);
  reg.onRobotFrame(robot, frame("color_image", 1, "latest"));
  await tick();
  const policy = viewer.policies.get("color_image")!;
  assertEquals(policy.inflight(), 1);
  // Entries carry real Date.now timestamps; fabricate a clock past staleMs.
  reg.reapAll(Date.now() + LATEST_STALE_MS + 100);
  assertEquals(policy.inflight(), 0);
  assertEquals(policy.expired, 1);
  assertEquals(policy.aborted, 0);
});

Deno.test("stats project exact per-channel key sets with counters and rates", async () => {
  const reg = new Registry();
  const robot = new FakeRobot("r1", SPECS);
  reg.registerRobot(robot);
  const viewer = attach(reg, "r1", ["color_image", "odom"]);
  reg.onRobotFrame(robot, frame("color_image", 1, "latest"));
  reg.onRobotFrame(robot, frame("odom", 1, "reliable"));
  await tick();

  const stats = reg.stats() as {
    perRobot: Record<string, {
      carrier: Record<string, unknown>;
      channels: Record<string, Record<string, unknown>>;
      undeclared: Record<string, unknown>;
    }>;
    perViewer: { channels: Record<string, Record<string, unknown>> }[];
  };
  assertEquals(Object.keys(stats.perRobot.r1).sort(), [
    "carrier",
    "channels",
    "subs",
    "teleop",
    "undeclared",
  ]);
  assertEquals(Object.keys(stats.perRobot.r1.carrier).sort(), [
    "bytesOut",
    "queued",
    "queuedBytes",
    "sent",
  ]);
  const inKeys = Object.keys(stats.perRobot.r1.channels.color_image).sort();
  assertEquals(inKeys, ["bps", "bytesIn", "delivery", "fps", "framesIn"]);
  // Declared-only traffic leaves the aggregate bucket zeroed.
  assertEquals(stats.perRobot.r1.undeclared, { framesIn: 0, bytesIn: 0, fps: 0, bps: 0 });
  const out = stats.perViewer[0].channels;
  const outKeys = Object.keys(out.color_image).sort();
  assertEquals(outKeys, [
    "aborted",
    "bps",
    "bytesOut",
    "delivery",
    "dropped",
    "expired",
    "fps",
    "inflight",
    "queued",
    "sent",
  ]);
  assertEquals(out.color_image.delivery, "latest");
  assertEquals(out.color_image.sent, 1);
  assertEquals(out.color_image.inflight, 1); // its stream stays open until reaped
  assertEquals(out.odom.delivery, "reliable");
  assertEquals(out.odom.inflight, 0); // one persistent stream, nothing to reset
  assert((out.odom.bytesOut as number) > 0);
  assertEquals(viewer.sink.kicked, null);
});

Deno.test("undeclared channel frames aggregate into one per-robot bucket", async () => {
  const reg = new Registry();
  const declared = new FakeRobot("r1", SPECS);
  reg.registerRobot(declared);
  // Novel header ch strings on a declared robot must not grow channels.
  reg.onRobotFrame(declared, frame("novel_a", 1, "latest"));
  reg.onRobotFrame(declared, frame("novel_b", 2, "latest"));
  let stats = reg.stats() as {
    perRobot: Record<string, {
      channels: Record<string, unknown>;
      undeclared: { framesIn: number; bytesIn: number };
    }>;
  };
  assertEquals(stats.perRobot.r1.channels, {});
  assertEquals(stats.perRobot.r1.undeclared.framesIn, 2);
  assert(stats.perRobot.r1.undeclared.bytesIn > 0);

  // A manifest-less robot still forwards to subscribed viewers while all its
  // traffic lands in the aggregate.
  const bare = new FakeRobot("r2", []);
  reg.registerRobot(bare);
  const viewer = attach(reg, "r2", ["tele"]);
  reg.onRobotFrame(bare, frame("tele", 1, "latest"));
  await tick();
  assertEquals(viewer.sink.sent.length, 1);
  stats = reg.stats() as typeof stats;
  assertEquals(stats.perRobot.r2.channels, {});
  assertEquals(stats.perRobot.r2.undeclared.framesIn, 1);
});

// --- Teleop lease ---

const TWIST: Msg = { t: "twist", vx: 0.5, vy: 0.25, wz: -0.25, seq: 1, ts: 1.5 };

function teleopMsgs(robot: FakeRobot): Msg[] {
  return robot.msgs.filter((m) =>
    m.t === "twist" || m.t === "stop" || m.t === "teleop_start" || m.t === "teleop_stop"
  );
}

Deno.test("teleop_start grants the lease and acks; re-start by the holder is idempotent", () => {
  const reg = new Registry();
  const robot = new FakeRobot("r1", SPECS);
  reg.registerRobot(robot);
  const viewer = attach(reg, "r1", []);
  assert(send(reg, viewer, { t: "teleop_start" }));
  assertEquals(viewer.replies.at(-1), { t: "teleop_started" });
  assert(send(reg, viewer, { t: "teleop_start" }));
  assertEquals(viewer.replies.at(-1), { t: "teleop_started" });
  // The robot-bound announcement is resent with the SAME gen (no bump): a
  // re-arm heals a lost start datagram without voiding the holder's twists.
  assertEquals(teleopMsgs(robot), [
    { t: "teleop_start", gen: 1 },
    { t: "teleop_start", gen: 1 },
  ]);
});

Deno.test("teleop_start while held replies teleop_held without closing", () => {
  const reg = new Registry();
  const robot = new FakeRobot("r1", SPECS);
  reg.registerRobot(robot);
  const holder = attach(reg, "r1", []);
  send(reg, holder, { t: "teleop_start" });
  const second = attach(reg, "r1", []);
  assert(send(reg, second, { t: "teleop_start" })); // non-fatal
  const reply = second.replies.at(-1);
  assert(reply !== undefined && reply.t === "error" && reply.code === "teleop_held");
  // The holder keeps driving; the loser stays gated.
  send(reg, holder, TWIST);
  send(reg, second, TWIST);
  assertEquals(teleopMsgs(robot), [{ t: "teleop_start", gen: 1 }, { ...TWIST, gen: 1 }]);
});

Deno.test("teleop_start needs a watch and a live robot", () => {
  const reg = new Registry();
  const robot = new FakeRobot("r1", SPECS);
  reg.registerRobot(robot);
  const unwatched = new FakeViewer();
  reg.addViewer(unwatched);
  send(reg, unwatched, { t: "hello", v: PROTOCOL_VERSION, role: "viewer" });
  assert(send(reg, unwatched, { t: "teleop_start" }));
  let reply = unwatched.replies.at(-1);
  assert(reply !== undefined && reply.t === "error" && reply.code === "no_watch");

  const watcher = attach(reg, "r1", []);
  reg.robotClosed(robot);
  assert(send(reg, watcher, { t: "teleop_start" }));
  reply = watcher.replies.at(-1);
  assert(reply !== undefined && reply.t === "error" && reply.code === "unknown_robot");
});

Deno.test("twist and stop forward robot-ward only from the lease holder", () => {
  const reg = new Registry();
  const robot = new FakeRobot("r1", SPECS);
  reg.registerRobot(robot);
  const bystander = attach(reg, "r1", []);
  send(reg, bystander, TWIST); // no lease at all yet
  const holder = attach(reg, "r1", []);
  send(reg, holder, { t: "teleop_start" });
  send(reg, holder, TWIST);
  send(reg, holder, { t: "stop", seq: 2, ts: 2.5 });
  send(reg, bystander, TWIST);
  assertEquals(teleopMsgs(robot), [
    { t: "teleop_start", gen: 1 },
    { ...TWIST, gen: 1 },
    { t: "stop", seq: 2, ts: 2.5, gen: 1 },
  ]);
  const stats = reg.stats() as { teleopForwarded: number; teleopDropped: number };
  assertEquals(stats.teleopForwarded, 2);
  assertEquals(stats.teleopDropped, 2);
});

Deno.test("teleop_stop from the holder releases and forwards teleop_stop robot-ward", () => {
  const reg = new Registry();
  const robot = new FakeRobot("r1", SPECS);
  reg.registerRobot(robot);
  const holder = attach(reg, "r1", []);
  send(reg, holder, { t: "teleop_start" });
  send(reg, holder, { t: "teleop_stop" });
  assertEquals(teleopMsgs(robot), [{ t: "teleop_start", gen: 1 }, { t: "teleop_stop", gen: 1 }]);
  // Released: the ex-holder is gated again, and another viewer can take over.
  send(reg, holder, TWIST);
  assertEquals(teleopMsgs(robot), [{ t: "teleop_start", gen: 1 }, { t: "teleop_stop", gen: 1 }]);
  const next = attach(reg, "r1", []);
  send(reg, next, { t: "teleop_start" });
  assertEquals(next.replies.at(-1), { t: "teleop_started" });
});

Deno.test("teleop_stop from a non-holder does not disturb the lease", () => {
  const reg = new Registry();
  const robot = new FakeRobot("r1", SPECS);
  reg.registerRobot(robot);
  const holder = attach(reg, "r1", []);
  send(reg, holder, { t: "teleop_start" });
  const other = attach(reg, "r1", []);
  send(reg, other, { t: "teleop_stop" });
  assertEquals(teleopMsgs(robot), [{ t: "teleop_start", gen: 1 }]); // no stop sent
  send(reg, holder, TWIST);
  assertEquals(teleopMsgs(robot), [{ t: "teleop_start", gen: 1 }, { ...TWIST, gen: 1 }]);
});

Deno.test("holder disconnect releases the lease and sends teleop_stop robot-ward", () => {
  const reg = new Registry();
  const robot = new FakeRobot("r1", SPECS);
  reg.registerRobot(robot);
  const holder = attach(reg, "r1", []);
  send(reg, holder, { t: "teleop_start" });
  reg.viewerClosed(holder);
  assertEquals(teleopMsgs(robot), [{ t: "teleop_start", gen: 1 }, { t: "teleop_stop", gen: 1 }]);
  const next = attach(reg, "r1", []);
  send(reg, next, { t: "teleop_start" });
  assertEquals(next.replies.at(-1), { t: "teleop_started" });
});

Deno.test("holder watch-switch releases the old robot's lease", () => {
  const reg = new Registry();
  const r1 = new FakeRobot("r1", SPECS);
  const r2 = new FakeRobot("r2", SPECS);
  reg.registerRobot(r1);
  reg.registerRobot(r2);
  const holder = attach(reg, "r1", []);
  send(reg, holder, { t: "teleop_start" });
  send(reg, holder, { t: "watch", robotId: "r2" });
  assertEquals(teleopMsgs(r1), [{ t: "teleop_start", gen: 1 }, { t: "teleop_stop", gen: 1 }]);
  // r2's lease was never granted: twists stay gated until teleop_start.
  send(reg, holder, TWIST);
  assertEquals(teleopMsgs(r2), []);
});

Deno.test("robot death drops the lease; a re-registered robot starts free", () => {
  const reg = new Registry();
  const robot = new FakeRobot("r1", SPECS);
  reg.registerRobot(robot);
  const holder = attach(reg, "r1", []);
  send(reg, holder, { t: "teleop_start" });
  reg.robotClosed(robot);
  const reborn = new FakeRobot("r1", SPECS);
  reg.registerRobot(reborn);
  // The ex-holder's twists are dropped until it re-arms.
  send(reg, holder, TWIST);
  assertEquals(teleopMsgs(reborn), []);
  send(reg, holder, { t: "teleop_start" });
  assertEquals(holder.replies.at(-1), { t: "teleop_started" });
  send(reg, holder, TWIST);
  // The reborn entry restarts at gen 1: safe, because the bridge's floor
  // reset with its session.
  assertEquals(teleopMsgs(reborn), [{ t: "teleop_start", gen: 1 }, { ...TWIST, gen: 1 }]);
});

Deno.test("lease generations increment per grant and stamp robot-bound teleop", () => {
  const reg = new Registry();
  const robot = new FakeRobot("r1", SPECS);
  reg.registerRobot(robot);
  const first = attach(reg, "r1", []);
  send(reg, first, { t: "teleop_start" });
  // A viewer-supplied gen must lose to the relay's stamp.
  send(reg, first, { ...TWIST, gen: 99 });
  send(reg, first, { t: "teleop_stop" });
  const second = attach(reg, "r1", []);
  send(reg, second, { t: "teleop_start" });
  send(reg, second, TWIST);
  assertEquals(teleopMsgs(robot), [
    { t: "teleop_start", gen: 1 },
    { ...TWIST, gen: 1 },
    { t: "teleop_stop", gen: 1 },
    { t: "teleop_start", gen: 2 },
    { ...TWIST, gen: 2 },
  ]);
});

Deno.test("stats expose the lease holder", () => {
  const reg = new Registry();
  const robot = new FakeRobot("r1", SPECS);
  reg.registerRobot(robot);
  const holder = attach(reg, "r1", []);
  let stats = reg.stats() as { perRobot: Record<string, { teleop: number | null }> };
  assertEquals(stats.perRobot.r1.teleop, null);
  send(reg, holder, { t: "teleop_start" });
  stats = reg.stats() as typeof stats;
  assertEquals(stats.perRobot.r1.teleop, holder.id);
});

// ---------- generic publish (W7) ----------

function pubSpec(ch: string, maxHz = 5): ChannelSpec {
  return {
    ch,
    dir: "tx",
    encoding: "text.json.v1",
    delivery: "reliable",
    maxHz,
    params: {},
    publish: "shared",
    requiredScope: null,
  };
}

function pub(reg: Registry, viewer: FakeViewer, id: string, ch: string, data: JsonValue): void {
  send(reg, viewer, { t: "pub", id, ch, data });
}

function lastError(viewer: FakeViewer): { code: string; requestId?: string } {
  const last = viewer.replies.at(-1) as { t: string; code: string; requestId?: string };
  assertEquals(last.t, "error");
  return last;
}

function pubStats(reg: Registry): {
  accepted: number;
  acked: number;
  timedOut: number;
  lateAcks: number;
  pending: number;
  rejected: Record<string, number>;
} {
  return (reg.stats() as { pub: ReturnType<typeof pubStats> }).pub;
}

Deno.test("pub forwards with stamped meta and routes exactly one ack", () => {
  const now = 10_000;
  const reg = new Registry(() => now);
  const robot = new FakeRobot("r1", [...SPECS, pubSpec("human_input")]);
  reg.registerRobot(robot);
  const sender = attach(reg, "r1", []);
  const bystander = attach(reg, "r1", []);
  send(reg, sender, {
    t: "pub",
    id: "v-1",
    ch: "human_input",
    data: { text: "salut β" },
    clientTs: 9.5,
  });
  assertEquals(robot.pubs.length, 1);
  const { ch, payload, meta } = robot.pubs[0];
  assertEquals(ch, "human_input");
  assertEquals(JSON.parse(new TextDecoder().decode(payload)), { text: "salut β" });
  // Relay-authored token, never the viewer's id; the synthetic local
  // principal marks the no-auth relay.
  assertEquals(meta, { id: "p1", principal: "local", relayTs: now / 1000, clientTs: 9.5 });
  assertEquals(pubStats(reg).pending, 1);

  reg.onRobotPubResult(robot, {
    t: "pub_ack",
    id: "p1",
    ch: "human_input",
    relayTs: 10,
    bridgeTs: 10.5,
  });
  // The ack restores the viewer's own request id and reaches only its sender.
  assertEquals(sender.pushed.at(-1), {
    t: "pub_ack",
    id: "v-1",
    ch: "human_input",
    relayTs: 10,
    bridgeTs: 10.5,
  });
  assertEquals(bystander.pushed.filter((m) => m.t === "pub_ack"), []);
  const stats = pubStats(reg);
  assertEquals([stats.accepted, stats.acked, stats.pending], [1, 1, 0]);
  // clientTs is optional and never stamped as undefined/null.
  pub(reg, sender, "v-2", "human_input", 1.5);
  assertEquals("clientTs" in robot.pubs[1].meta, false);
  assertEquals(robot.pubs[1].meta.id, "p2");
  // null is a publishable value: it forwards as the literal JSON "null".
  pub(reg, sender, "v-3", "human_input", null);
  assertEquals(new TextDecoder().decode(robot.pubs[2].payload), "null");
});

Deno.test("pub validation failures reply correlated errors", () => {
  const reg = new Registry(() => 0);
  const robot = new FakeRobot("r1", [
    ...SPECS,
    pubSpec("human_input"),
    { ...pubSpec("goal"), publish: "exclusive" },
    // The teleop shape: tx but publish="none".
    { ...pubSpec("tele_cmd_vel"), encoding: "twist.json.v1", delivery: "latest", publish: "none" },
  ]);
  reg.registerRobot(robot);

  const unwatched = new FakeViewer();
  reg.addViewer(unwatched);
  send(reg, unwatched, { t: "hello", v: PROTOCOL_VERSION, role: "viewer" });
  pub(reg, unwatched, "a", "human_input", 1.5);
  assertEquals(
    lastError(unwatched),
    {
      t: "error",
      code: "no_watch",
      message: "watch a robot before publishing",
      requestId: "a",
    } as never,
  );

  const viewer = attach(reg, "r1", []);
  pub(reg, viewer, "b", "nope", 1.5);
  assertEquals(lastError(viewer).code, "unknown_channel");
  pub(reg, viewer, "c", "odom", 1.5); // rx channel
  assertEquals(lastError(viewer).code, "not_publishable");
  pub(reg, viewer, "d", "goal", 1.5); // exclusive until W8
  assertEquals(lastError(viewer).code, "not_publishable");
  pub(reg, viewer, "e", "tele_cmd_vel", 1.5); // specialized tx
  assertEquals(lastError(viewer).code, "not_publishable");
  pub(reg, viewer, "f", "human_input", "x".repeat(33 * 1024));
  assertEquals(lastError(viewer).code, "publish_too_large");

  // Duplicate ids are per-viewer and only while pending: settling frees the id.
  pub(reg, viewer, "g", "human_input", 1.5);
  pub(reg, viewer, "g", "human_input", 2.5);
  assertEquals(lastError(viewer).code, "duplicate_request");
  assertEquals(lastError(viewer).requestId, "g");
  reg.onRobotPubResult(robot, {
    t: "pub_ack",
    id: "p1",
    ch: "human_input",
    relayTs: 1,
    bridgeTs: 2,
  });
  pub(reg, viewer, "g", "human_input", 2.5);
  assertEquals(robot.pubs.length, 2);

  // A manifest-less robot declared no publishable channels either.
  const bare = new FakeRobot("r2", []);
  reg.registerRobot(bare);
  const bareViewer = attach(reg, "r2", []);
  pub(reg, bareViewer, "h", "human_input", 1.5);
  assertEquals(lastError(bareViewer).code, "unknown_channel");

  const rejected = pubStats(reg).rejected;
  assertEquals(rejected.no_watch, 1);
  assertEquals(rejected.unknown_channel, 2);
  assertEquals(rejected.not_publishable, 3);
  assertEquals(rejected.publish_too_large, 1);
  assertEquals(rejected.duplicate_request, 1);
});

Deno.test("pub rate limits: per-viewer bucket, then the aggregate across viewers", () => {
  let now = 0;
  const reg = new Registry(() => now);
  const robot = new FakeRobot("r1", [pubSpec("human_input", 2)]); // capacity 2
  reg.registerRobot(robot);
  const a = attach(reg, "r1", []);
  const b = attach(reg, "r1", []);

  pub(reg, a, "a1", "human_input", 1.5);
  pub(reg, a, "a2", "human_input", 1.5);
  pub(reg, a, "a3", "human_input", 1.5);
  assertEquals(lastError(a).code, "rate_limited");
  assertEquals(robot.pubs.length, 2);

  // Viewer B has fresh per-viewer tokens, but the aggregate is exhausted:
  // more sessions cannot multiply the accepted robot/channel rate.
  pub(reg, b, "b1", "human_input", 1.5);
  assertEquals(lastError(b).code, "rate_limited");
  assertEquals(robot.pubs.length, 2);

  // Refill at 2/s: half a second buys one token (viewer and aggregate).
  now += 500;
  pub(reg, a, "a4", "human_input", 1.5);
  assertEquals(robot.pubs.length, 3);
});

Deno.test("pub pending caps bound per-viewer and per-robot state", () => {
  const reg = new Registry(() => 0);
  const robot = new FakeRobot("r1", [pubSpec("human_input", 10_000)]);
  reg.registerRobot(robot);
  const first = attach(reg, "r1", []);
  for (let i = 0; i < 16; i++) pub(reg, first, `f${i}`, "human_input", 1.5);
  assertEquals(robot.pubs.length, 16);
  pub(reg, first, "f16", "human_input", 1.5);
  assertEquals(lastError(first).code, "pending_limit");

  // Three more viewers reach the 64-entry robot cap; a fresh fifth viewer
  // then fails on the robot budget, not its own.
  for (let v = 0; v < 3; v++) {
    const viewer = attach(reg, "r1", []);
    for (let i = 0; i < 16; i++) pub(reg, viewer, `x${i}`, "human_input", 1.5);
  }
  assertEquals(robot.pubs.length, 64);
  const fresh = attach(reg, "r1", []);
  pub(reg, fresh, "y0", "human_input", 1.5);
  assertEquals(lastError(fresh).code, "pending_limit");
  assertEquals(pubStats(reg).pending, 64);
});

Deno.test("pending-capped publishes do not drain the aggregate pub bucket", () => {
  const reg = new Registry(() => 0);
  // Capacity ceil(32) with zero refill under the frozen clock: if rejected
  // requests spent tokens, 16 accepted + 16 pending_limit rejections would
  // empty the aggregate and starve the second viewer.
  const robot = new FakeRobot("r1", [pubSpec("human_input", 32)]);
  reg.registerRobot(robot);
  const capped = attach(reg, "r1", []);
  for (let i = 0; i < 16; i++) pub(reg, capped, `a${i}`, "human_input", 1.5);
  assertEquals(robot.pubs.length, 16);
  for (let i = 16; i < 32; i++) {
    pub(reg, capped, `a${i}`, "human_input", 1.5);
    assertEquals(lastError(capped).code, "pending_limit");
  }
  const other = attach(reg, "r1", []);
  pub(reg, other, "b0", "human_input", 1.5);
  assertEquals(robot.pubs.length, 17);
  assertEquals(pubStats(reg).rejected.rate_limited, undefined);
});

Deno.test("pub byte budget bounds pending payload volume per viewer", () => {
  const reg = new Registry(() => 0);
  const robot = new FakeRobot("r1", [pubSpec("human_input", 10_000)]);
  reg.registerRobot(robot);
  const viewer = attach(reg, "r1", []);
  // 30 KiB serialized each: the 9th crosses the 256 KiB viewer byte budget
  // well before the 16-entry count cap.
  const big = "x".repeat(30 * 1024);
  for (let i = 0; i < 8; i++) pub(reg, viewer, `b${i}`, "human_input", big);
  assertEquals(robot.pubs.length, 8);
  pub(reg, viewer, "b8", "human_input", big);
  assertEquals(lastError(viewer).code, "pending_limit");
});

Deno.test("pub timeout settles pending with publish_timeout; a late ack is dropped", () => {
  let now = 0;
  const reg = new Registry(() => now);
  const robot = new FakeRobot("r1", [pubSpec("human_input")]);
  reg.registerRobot(robot);
  const viewer = attach(reg, "r1", []);
  pub(reg, viewer, "v-1", "human_input", 1.5);
  now += PUBLISH_TIMEOUT_MS - 1;
  reg.reapAll(now);
  assertEquals(pubStats(reg).pending, 1);
  now += 1;
  reg.reapAll(now);
  assertEquals(viewer.pushed.at(-1), {
    t: "error",
    code: "publish_timeout",
    message: `no bridge acknowledgement within ${PUBLISH_TIMEOUT_MS} ms`,
    requestId: "v-1",
  });
  const stats = pubStats(reg);
  assertEquals([stats.timedOut, stats.pending], [1, 0]);
  // The bridge's ack arriving after the timeout is counted, not routed.
  reg.onRobotPubResult(robot, {
    t: "pub_ack",
    id: "p1",
    ch: "human_input",
    relayTs: 1,
    bridgeTs: 2,
  });
  assertEquals(pubStats(reg).lateAcks, 1);
  assertEquals(viewer.pushed.filter((m) => m.t === "pub_ack"), []);
});

Deno.test("viewer disconnect releases its pending publishes", () => {
  const reg = new Registry(() => 0);
  const robot = new FakeRobot("r1", [pubSpec("human_input")]);
  reg.registerRobot(robot);
  const viewer = attach(reg, "r1", []);
  pub(reg, viewer, "v-1", "human_input", 1.5);
  reg.viewerClosed(viewer);
  assertEquals(pubStats(reg).pending, 0);
  reg.onRobotPubResult(robot, {
    t: "pub_ack",
    id: "p1",
    ch: "human_input",
    relayTs: 1,
    bridgeTs: 2,
  });
  assertEquals(pubStats(reg).lateAcks, 1);
});

Deno.test("robot disconnect fails pending publishes to live viewers", () => {
  const reg = new Registry(() => 0);
  const robot = new FakeRobot("r1", [pubSpec("human_input")]);
  reg.registerRobot(robot);
  const viewer = attach(reg, "r1", []);
  pub(reg, viewer, "v-1", "human_input", 1.5);
  reg.robotClosed(robot);
  // The error precedes the robots-update push that closes out robotClosed.
  assertEquals(viewer.pushed.filter((m) => m.t === "error"), [{
    t: "error",
    code: "robot_disconnected",
    message: "robot r1 disconnected before acknowledging the publish",
    requestId: "v-1",
  }]);
  assertEquals(pubStats(reg).pending, 0);
});

Deno.test("pub_nack maps to a correlated error with a clamped message", () => {
  const reg = new Registry(() => 0);
  const robot = new FakeRobot("r1", [pubSpec("human_input")]);
  reg.registerRobot(robot);
  const viewer = attach(reg, "r1", []);
  pub(reg, viewer, "v-1", "human_input", 1.5);
  reg.onRobotPubResult(robot, {
    t: "pub_nack",
    id: "p1",
    code: "decode_failed",
    message: "x".repeat(400),
  });
  const err = viewer.pushed.at(-1) as { code: string; message: string; requestId: string };
  assertEquals(err.code, "decode_failed");
  assertEquals(err.message.length, 256);
  assertEquals(err.requestId, "v-1");
  assertEquals(pubStats(reg).rejected.decode_failed, 1);
});

Deno.test("watch switch clears pub buckets but keeps pending routable", () => {
  const reg = new Registry(() => 0);
  const robotA = new FakeRobot("ra", [pubSpec("human_input", 1)]); // capacity 1
  const robotB = new FakeRobot("rb", [pubSpec("human_input", 1)]);
  reg.registerRobot(robotA);
  reg.registerRobot(robotB);
  const viewer = attach(reg, "ra", []);
  pub(reg, viewer, "v-1", "human_input", 1.5); // pending on A; bucket empty
  pub(reg, viewer, "v-2", "human_input", 1.5);
  assertEquals(lastError(viewer).code, "rate_limited");
  send(reg, viewer, { t: "watch", robotId: "rb" });
  // Fresh per-viewer bucket for the new robot (the aggregate is per robot).
  pub(reg, viewer, "v-3", "human_input", 1.5);
  assertEquals(robotB.pubs.length, 1);
  // The pending publish on A still routes its ack to the live viewer.
  reg.onRobotPubResult(robotA, {
    t: "pub_ack",
    id: "p1",
    ch: "human_input",
    relayTs: 1,
    bridgeTs: 2,
  });
  assertEquals(
    viewer.pushed.filter((m) => m.t === "pub_ack").map((m) => (m as { id: string }).id),
    ["v-1"],
  );
});

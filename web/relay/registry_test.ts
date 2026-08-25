// Registry unit tests over fake peers: subscription snapshot transitions,
// duplicate-id rejection, watch switching, robots pushes, and frame routing -
// no QUIC.
import { assert, assertEquals } from "@std/assert";
import {
  type ChannelSpec,
  encodeDataFrame,
  type FrameHeader,
  type ManifestMsg,
  type Msg,
  PROTOCOL_VERSION,
  type RobotInfo,
  type RobotManifest,
  type SubsMsg,
} from "@dimos/shared";
import {
  type ChannelPolicy,
  type FrameSend,
  type FrameWriter,
  LATEST_STALE_MS,
  LatestChannel,
  ReliableChannel,
  type ViewerSink,
} from "./forward.ts";
import { Registry, type RobotPeer, type ViewerPeer } from "./registry.ts";

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
  msgs: Msg[] = [];
  closed: string | null = null;

  constructor(id: string, channels: ChannelSpec[] = [], manifest: RobotManifest | null = null) {
    this.info = { id, name: id, model: "test" };
    this.channels = channels;
    this.manifest = manifest;
  }

  sendMsg(msg: Msg): void {
    this.msgs.push(msg);
  }

  subs(): SubsMsg[] {
    return this.msgs.filter((m): m is SubsMsg => m.t === "subs");
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

const SPECS: ChannelSpec[] = [
  { ch: "color_image", dir: "rx", encoding: "jpeg.v1", delivery: "latest", maxHz: 15, params: {} },
  { ch: "odom", dir: "rx", encoding: "pose.json.v1", delivery: "reliable", maxHz: 20, params: {} },
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
  const reg = new Registry();
  const first = new FakeRobot("r1", SPECS);
  assertEquals(reg.registerRobot(first), true);
  const watcher = attach(reg, "r1", ["odom"]);

  const second = new FakeRobot("r1", SPECS);
  assertEquals(reg.registerRobot(second), false);
  assertEquals(first.closed, null);
  assertEquals(second.subs(), []);
  assertEquals(watcher.pushed, []);
  assertEquals((reg.robotsMsg() as { robots: RobotInfo[] }).robots, [first.info!]);

  reg.robotClosed(first);
  assertEquals(reg.registerRobot(second), true);
  // The returning robot reattaches the surviving viewer state and announces
  // itself only after the old live registration is gone.
  assertEquals(watcher.pushed, [
    { t: "robots", robots: [] },
    { t: "robots", robots: [second.info!] },
  ]);
  assertEquals(second.lastSubs(), { t: "subs", chs: ["odom"], n: 1 });
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
  const withMystery: ChannelSpec[] = [
    ...SPECS,
    { ch: "mystery", dir: "rx", encoding: "x", delivery: "latest", maxHz: 1, params: {} },
  ];
  const first = new FakeRobot("r1", withMystery);
  reg.registerRobot(first);
  const viewer = attach(reg, "r1", ["mystery"]);
  assertEquals(first.lastSubs().chs, ["mystery"]);

  const second = new FakeRobot("r1", SPECS); // manifest shrank across restart
  reg.robotClosed(first);
  reg.registerRobot(second);
  // The surviving sub is no longer in the manifest: snapshots must not carry
  // it (an out-of-manifest union could outgrow the datagram budget) ...
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

Deno.test("resendSnapshots repeats the last set with a fresh n", () => {
  const reg = new Registry();
  const robot = new FakeRobot("r1", SPECS);
  reg.registerRobot(robot);
  attach(reg, "r1", ["odom"]);
  const last = robot.lastSubs();
  reg.resendSnapshots();
  assertEquals(robot.lastSubs(), { t: "subs", chs: last.chs, n: last.n + 1 });
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
      channels: Record<string, Record<string, unknown>>;
      undeclared: Record<string, unknown>;
    }>;
    perViewer: { channels: Record<string, Record<string, unknown>> }[];
  };
  assertEquals(Object.keys(stats.perRobot.r1).sort(), ["channels", "subs", "undeclared"]);
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

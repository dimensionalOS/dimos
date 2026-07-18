// Registry unit tests over fake peers: subscription snapshot transitions,
// takeover, watch switching, robots pushes, and frame routing - no QUIC.
import { assert, assertEquals } from "@std/assert";
import {
  type ChannelSpec,
  encodeDataFrame,
  type FrameHeader,
  type Msg,
  PROTOCOL_VERSION,
  type RobotInfo,
  type SubsMsg,
} from "@dimos/shared";
import { type ChannelPolicy, LatestChannel, ReliableChannel, type ViewerSink } from "./forward.ts";
import { Registry, type RobotPeer, type ViewerPeer } from "./registry.ts";

class FakeSink implements ViewerSink {
  sent: Uint8Array[] = [];
  kicked: string | null = null;

  sendFrame(bytes: Uint8Array): Promise<void> {
    this.sent.push(bytes);
    return Promise.resolve();
  }

  kick(reason: string): void {
    this.kicked = reason;
  }
}

class FakeRobot implements RobotPeer {
  info: RobotInfo | null;
  channels: ChannelSpec[];
  msgs: Msg[] = [];
  closed: string | null = null;

  constructor(id: string, channels: ChannelSpec[] = []) {
    this.info = { id, name: id, model: "test" };
    this.channels = channels;
  }

  sendMsg(msg: Msg): void {
    this.msgs.push(msg);
  }

  close(reason: string): void {
    this.closed = reason;
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

const SPECS: ChannelSpec[] = [
  { ch: "color_image", encoding: "jpeg.v1", delivery: "latest", maxHz: 15 },
  { ch: "odom", encoding: "pose.json.v1", delivery: "reliable", maxHz: 20 },
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
  const robot = new FakeRobot("r1", SPECS);
  reg.registerRobot(robot);
  const viewer = attach(reg, "r1", ["odom"]);
  send(reg, viewer, { t: "watch", robotId: "r1" });
  assertEquals(viewer.subs, new Set(["odom"]));
  const manifests = viewer.replies.filter((m) => m.t === "manifest");
  assertEquals(manifests.length, 2);
  assertEquals((manifests[1] as { channels: ChannelSpec[] }).channels, SPECS);
});

Deno.test("takeover closes the old session, keeps n, reattaches watchers", () => {
  const reg = new Registry();
  const first = new FakeRobot("r1", SPECS);
  reg.registerRobot(first);
  const watcher = attach(reg, "r1", ["odom"]);
  const nBefore = first.lastSubs().n;

  const second = new FakeRobot("r1", SPECS);
  reg.registerRobot(second);
  assertEquals(first.closed, "replaced by new robot");
  // Viewers hear about the takeover: this push is what makes debug.html
  // re-watch and refresh a manifest that changed across the restart.
  assertEquals(watcher.pushed, [{ t: "robots", robots: [second.info!] }]);
  // Forced snapshot to the new session carries the surviving watcher's subs
  // and continues the old counter (the bridge's stale-n filter keeps working
  // even though it is a fresh client session).
  assertEquals(second.lastSubs().chs, ["odom"]);
  assert(second.lastSubs().n > nBefore, "n must continue past the old session's");

  // The old session's close callback must not unregister the new one, and a
  // delayed hello resend from the replaced session must not re-register it
  // (that would kick the live session).
  reg.robotClosed(first);
  reg.registerRobot(first);
  assertEquals(second.closed, null);
  const viewer2 = attach(reg, "r1", ["color_image"]);
  assertEquals(second.lastSubs().chs, ["color_image", "odom"]);
  assertEquals(viewer2.replies.filter((m) => m.t === "error"), []);
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

Deno.test("version mismatch rejects the session; bad watch/sub do not", () => {
  const reg = new Registry();
  const viewer = new FakeViewer();
  reg.addViewer(viewer);
  assertEquals(send(reg, viewer, { t: "hello", v: 99, role: "viewer" }), false);
  assertEquals((viewer.replies[0] as { code: string }).code, "version_mismatch");

  assertEquals(send(reg, viewer, { t: "watch", robotId: "ghost" }), true);
  assertEquals((viewer.replies[1] as { code: string }).code, "unknown_robot");
  assertEquals(send(reg, viewer, { t: "sub", ch: "odom" }), true);
  assertEquals((viewer.replies[2] as { code: string }).code, "no_watch");
});

Deno.test("frames route only to watching+subscribed viewers", () => {
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
  assertEquals(subscribed.sink.sent.length, 1);
  assertEquals(otherChannel.sink.sent.length, 0);
  assertEquals(otherRobot.sink.sent.length, 0);
  assertEquals(noSub.sink.sent.length, 0);
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

Deno.test("takeover-stale sub is filtered from snapshots, frames fall back to header delivery", () => {
  const reg = new Registry();
  const withMystery: ChannelSpec[] = [
    ...SPECS,
    { ch: "mystery", encoding: "x", delivery: "latest", maxHz: 1 },
  ];
  const first = new FakeRobot("r1", withMystery);
  reg.registerRobot(first);
  const viewer = attach(reg, "r1", ["mystery"]);
  assertEquals(first.lastSubs().chs, ["mystery"]);

  const second = new FakeRobot("r1", SPECS); // manifest shrank across restart
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

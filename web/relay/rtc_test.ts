// RtcHub over a fake SFU client and fake peers. No network.
import { assert, assertEquals } from "@std/assert";
import type { ChannelSpec, IceServer, Msg, RobotInfo, RtcOfferMsg } from "@dimos/shared";
import { TRACK_ENCODING } from "@dimos/shared/manifest";
import type { CarrierStats } from "./carrier.ts";
import {
  CloudflareError,
  SessionGoneError,
  STUN_ONLY,
  type TrackRef,
  type TrackResult,
  type TracksResponse,
} from "./cloudflare.ts";
import type { ChannelPolicy, ViewerSink } from "./forward.ts";
import type { RobotPeer, ViewerPeer } from "./registry.ts";
import { MAX_CLOSE_ATTEMPTS, RtcHub, type RtcHubOptions, type SfuClient } from "./rtc.ts";

interface Call {
  method: string;
  args: unknown[];
}

type Scripted = TracksResponse | Error | (() => TracksResponse | Error);

class FakeSfu implements SfuClient {
  calls: Call[] = [];
  /** Scripted addTracks outcomes, consumed in call order (a thunk runs at
   * call time); default after. */
  scripted: Scripted[] = [];
  /** One-shot errors, consumed by the next call of their method (a permanent
   * error would spin the instant-sleep loop and starve settle()). */
  newSessionError: Error | null = null;
  closeError: Error | null = null;
  /** Scripted closeTracks results, consumed in call order; default after:
   * every mid confirmed. */
  closeResults: TrackResult[][] = [];
  renegotiateError: Error | null = null;
  iceError: Error | null = null;
  ice: IceServer[] = [{ urls: ["turn:t"], username: "u", credential: "c" }];
  #sessions = 0;
  #mids = 0;

  newSession(sdp: string): Promise<{ sessionId: string; answerSdp: string }> {
    this.calls.push({ method: "newSession", args: [sdp] });
    const error = this.newSessionError;
    if (error !== null) {
      this.newSessionError = null;
      return Promise.reject(error);
    }
    return Promise.resolve({ sessionId: `s${++this.#sessions}`, answerSdp: `answer:${sdp}` });
  }

  addTracks(sessionId: string, tracks: TrackRef[]): Promise<TracksResponse> {
    this.calls.push({ method: "addTracks", args: [sessionId, tracks] });
    let next = this.scripted.shift();
    if (typeof next === "function") next = next();
    if (next instanceof Error) return Promise.reject(next);
    if (next !== undefined) return Promise.resolve(next);
    if (tracks[0]?.location === "local") {
      return Promise.resolve({
        tracks: tracks.map((t) => ({ mid: t.mid, trackName: t.trackName })),
        sdp: null,
      });
    }
    return Promise.resolve({
      tracks: tracks.map((t) => ({ mid: `p${++this.#mids}`, trackName: t.trackName })),
      sdp: `offer:${sessionId}`,
    });
  }

  closeTracks(sessionId: string, mids: string[]): Promise<TrackResult[]> {
    this.calls.push({ method: "closeTracks", args: [sessionId, mids] });
    const error = this.closeError;
    if (error !== null) {
      this.closeError = null;
      return Promise.reject(error);
    }
    return Promise.resolve(this.closeResults.shift() ?? mids.map((mid) => ({ mid })));
  }

  renegotiate(sessionId: string, answerSdp: string): Promise<void> {
    this.calls.push({ method: "renegotiate", args: [sessionId, answerSdp] });
    const error = this.renegotiateError;
    if (error !== null) {
      this.renegotiateError = null;
      return Promise.reject(error);
    }
    return Promise.resolve();
  }

  iceServers(): Promise<IceServer[]> {
    this.calls.push({ method: "iceServers", args: [] });
    const error = this.iceError;
    if (error !== null) {
      this.iceError = null;
      return Promise.reject(error);
    }
    return Promise.resolve(this.ice);
  }

  of(method: string): unknown[][] {
    return this.calls.filter((c) => c.method === method).map((c) => c.args);
  }

  /** The addTracks calls that declared local tracks / pulled remote ones. */
  locals(): unknown[][] {
    return this.of("addTracks").filter((a) => (a[1] as TrackRef[])[0]?.location === "local");
  }

  pulls(): unknown[][] {
    return this.of("addTracks").filter((a) => (a[1] as TrackRef[])[0]?.location === "remote");
  }

  /** The method sequence, for ordering assertions. */
  methods(): string[] {
    return this.calls.map((c) => c.method);
  }
}

class FakeRobot implements RobotPeer {
  info: RobotInfo | null;
  channels: ChannelSpec[];
  manifest = null;
  closed: string | null = null;
  control: Msg[] = [];
  msgs: Msg[] = [];

  constructor(id: string, channels: ChannelSpec[]) {
    this.info = { id, name: id, model: "test" };
    this.channels = channels;
  }

  sendMsg(msg: Msg): void {
    this.msgs.push(msg);
  }

  sendControl(msg: Msg): void {
    this.control.push(msg);
  }

  sendPub(): void {}

  carrierStats(): CarrierStats {
    return { queued: 0, queuedBytes: 0, sent: 0, bytesOut: 0 };
  }
}

class FakeViewer implements ViewerPeer {
  static nextId = 1;
  readonly id = FakeViewer.nextId++;
  watched: string | null = null;
  readonly subs = new Set<string>();
  readonly policies = new Map<string, ChannelPolicy>();
  // Never touched by the hub.
  readonly sink = {} as ViewerSink;
  greeted = true;
  name: string | null = null;
  pushed: Msg[] = [];

  sendMsg(msg: Msg): void {
    this.pushed.push(msg);
  }

  offers(): RtcOfferMsg[] {
    return this.pushed.filter((m): m is RtcOfferMsg => m.t === "rtc_offer");
  }

  errors(): string[] {
    return this.pushed.filter((m) => m.t === "error").map((m) => (m as { code: string }).code);
  }
}

function spec(ch: string, encoding: string): ChannelSpec {
  return {
    ch,
    dir: "rx",
    encoding,
    delivery: "latest",
    maxHz: 15,
    params: {},
    publish: "none",
    requiredScope: null,
  };
}

const SPECS = [
  spec("color_image", TRACK_ENCODING),
  spec("rear", TRACK_ENCODING),
  spec("odom", "pose.json.v1"),
];
const ROBOT_OFFER: RtcOfferMsg = {
  t: "rtc_offer",
  sdp: "robot-sdp",
  // odom is not a track channel: its mid must be ignored.
  tracks: [{ ch: "color_image", mid: "0" }, { ch: "rear", mid: "1" }, { ch: "odom", mid: "2" }],
};
const LOCAL_CAM: TrackRef = { location: "local", mid: "0", trackName: "color_image" };
const LOCAL_REAR: TrackRef = { location: "local", mid: "1", trackName: "rear" };
/** The default declaration result for both track channels. */
const DECLARED_BOTH: TracksResponse = {
  tracks: [{ mid: "0", trackName: "color_image" }, { mid: "1", trackName: "rear" }],
  sdp: null,
};
const DECLARED_CAM: TracksResponse = {
  tracks: [{ mid: "0", trackName: "color_image" }],
  sdp: null,
};
const NOT_FOUND: TracksResponse = {
  tracks: [{ trackName: "color_image", errorCode: "not_found_track_error" }],
  sdp: null,
};

const instant = () => Promise.resolve();

/** Let every chained microtask (fake API calls, hub loops) run. */
async function settle(): Promise<void> {
  for (let i = 0; i < 10; i++) await new Promise((resolve) => setTimeout(resolve, 0));
}

function hub(sfu: FakeSfu, options: RtcHubOptions = {}): RtcHub {
  return new RtcHub(sfu, { sleep: instant, answerTimeoutMs: 1000, ...options });
}

/** Tear the hub down like shutdown() does (a pending exchange is rejected,
 * clearing its answer timer) and let the loops exit. */
async function done(h: RtcHub): Promise<void> {
  h.dispose();
  await settle();
}

/** Robot offered and answered; viewer (watching, subscribed to `chs`)
 * offered and answered. Returns once both SFU sessions exist. */
async function connected(
  h: RtcHub,
  chs: string[],
): Promise<{ robot: FakeRobot; viewer: FakeViewer }> {
  const robot = new FakeRobot("r1", SPECS);
  h.robotOffer(robot, ROBOT_OFFER);
  const viewer = new FakeViewer();
  viewer.watched = "r1";
  for (const ch of chs) viewer.subs.add(ch);
  h.viewerOffer(viewer, "viewer-sdp");
  await settle();
  assertEquals(robot.control, [{ t: "rtc_answer", sdp: "answer:robot-sdp" }]);
  assertEquals(viewer.pushed[0], { t: "rtc_answer", sdp: "answer:viewer-sdp" });
  return { robot, viewer };
}

/** Move the viewer's watch to `robotId` the way the registry does: subs
 * cleared with the switch, re-subscribed by the SDK afterwards. */
function switchWatch(h: RtcHub, viewer: FakeViewer, robotId: string, chs: string[]): void {
  viewer.watched = robotId;
  viewer.subs.clear();
  h.viewerChanged(viewer);
  for (const ch of chs) viewer.subs.add(ch);
  h.viewerChanged(viewer);
}

Deno.test("declare per channel on first need, pull on sub, renegotiate the answer, force-close on unsub", async () => {
  const sfu = new FakeSfu();
  const h = hub(sfu);
  const { viewer } = await connected(h, ["color_image"]);
  // Declare (local, robot session s1, the needed channel only) then pull
  // (remote, viewer session s2).
  assertEquals(sfu.of("addTracks"), [
    ["s1", [LOCAL_CAM]],
    ["s2", [{ location: "remote", sessionId: "s1", trackName: "color_image" }]],
  ]);
  assertEquals(viewer.offers(), [{
    t: "rtc_offer",
    sdp: "offer:s2",
    robotId: "r1",
    tracks: [{ ch: "color_image", mid: "p1" }],
  }]);
  assertEquals(sfu.of("renegotiate"), []); // waiting for the viewer's answer

  h.viewerAnswer(viewer, "viewer-answer");
  await settle();
  assertEquals(sfu.of("renegotiate"), [["s2", "viewer-answer"]]);
  assertEquals(h.stats().pulls, 1);

  // A second channel declares only itself.
  viewer.subs.add("rear");
  h.viewerChanged(viewer);
  await settle();
  assertEquals(sfu.of("addTracks").slice(2), [
    ["s1", [LOCAL_REAR]],
    ["s2", [{ location: "remote", sessionId: "s1", trackName: "rear" }]],
  ]);
  h.viewerAnswer(viewer, "a2");
  await settle();
  assertEquals(h.stats().pulls, 2);

  viewer.subs.delete("color_image");
  h.viewerChanged(viewer);
  await settle();
  assertEquals(sfu.of("closeTracks"), [["s2", ["p1"]]]);
  assertEquals(h.stats().pulls, 1);
  // A re-sub pulls again; the declaration is not repeated.
  viewer.subs.add("color_image");
  h.viewerChanged(viewer);
  await settle();
  assertEquals(sfu.locals().length, 2);
  assertEquals(viewer.offers().length, 3);
  await done(h);
});

Deno.test("several missing channels declare, pull and renegotiate in one call each", async () => {
  const sfu = new FakeSfu();
  const h = hub(sfu);
  const { viewer } = await connected(h, ["color_image", "rear", "odom"]);
  assertEquals(sfu.locals(), [["s1", [LOCAL_CAM, LOCAL_REAR]]]);
  const pulls = sfu.pulls();
  assertEquals(pulls.length, 1);
  assertEquals((pulls[0][1] as TrackRef[]).map((t) => t.trackName).sort(), ["color_image", "rear"]);
  assertEquals(viewer.offers()[0].tracks?.length, 2);
  h.viewerAnswer(viewer, "a");
  await settle();
  assertEquals(h.stats().pulls, 2);
});

Deno.test("a second viewer reuses the robot's declaration", async () => {
  const sfu = new FakeSfu();
  const h = hub(sfu);
  const { viewer } = await connected(h, ["color_image"]);
  h.viewerAnswer(viewer, "a");
  const other = new FakeViewer();
  other.watched = "r1";
  other.subs.add("color_image");
  h.viewerOffer(other, "other-sdp");
  await settle();
  assertEquals(sfu.locals().length, 1);
  assertEquals(other.offers().length, 1);
  await done(h);
});

Deno.test("a viewer that offers before the robot pulls once the robot's session exists", async () => {
  const sfu = new FakeSfu();
  const h = hub(sfu);
  const viewer = new FakeViewer();
  viewer.watched = "r1";
  viewer.subs.add("color_image");
  h.viewerOffer(viewer, "viewer-sdp");
  await settle();
  assertEquals(sfu.of("addTracks"), []); // nothing to pull from yet
  const robot = new FakeRobot("r1", SPECS);
  h.robotOffer(robot, ROBOT_OFFER);
  await settle();
  assertEquals(sfu.of("addTracks").length, 2);
  assertEquals(viewer.offers().length, 1);
  await done(h);
});

Deno.test("a robot re-offer closes pulls from its old session and re-pulls from the new one", async () => {
  const sfu = new FakeSfu();
  const h = hub(sfu);
  const { robot, viewer } = await connected(h, ["color_image"]);
  h.viewerAnswer(viewer, "a");
  await settle();
  assertEquals(h.stats().pulls, 1);
  // The bridge reconnected: robotClosed for the old session, then a fresh
  // registration offers again.
  h.robotClosed(robot);
  await settle();
  assertEquals(sfu.of("closeTracks"), [["s2", ["p1"]]]);
  assertEquals(h.stats().pulls, 0);
  const again = new FakeRobot("r1", SPECS);
  h.robotOffer(again, ROBOT_OFFER);
  await settle();
  // Declared again on the new session s3, pulled again onto the viewer.
  assertEquals(sfu.locals().map((a) => a[0]), ["s1", "s3"]);
  assertEquals(viewer.offers().length, 2);
  h.viewerAnswer(viewer, "b");
  await settle();
  assertEquals(h.stats().pulls, 1);
});

Deno.test("a watch switch closes pulls from the old robot and pulls from the new one", async () => {
  const sfu = new FakeSfu();
  const h = hub(sfu);
  const { viewer } = await connected(h, ["color_image"]);
  h.viewerAnswer(viewer, "a");
  const other = new FakeRobot("r2", SPECS);
  h.robotOffer(other, ROBOT_OFFER);
  await settle();
  switchWatch(h, viewer, "r2", ["color_image"]);
  await settle();
  assertEquals(sfu.of("closeTracks"), [["s2", ["p1"]]]);
  assertEquals((sfu.pulls()[1][1] as TrackRef[])[0].sessionId, "s3");
  assertEquals(viewer.offers()[1].robotId, "r2");
  await done(h);
});

Deno.test("a watch switch mid-exchange: the old robot's pull is answered, closed, and the new robot's is offered with its id", async () => {
  const sfu = new FakeSfu();
  const h = hub(sfu);
  const { viewer } = await connected(h, ["color_image"]); // r1's offer pending
  const other = new FakeRobot("r2", SPECS);
  h.robotOffer(other, ROBOT_OFFER);
  await settle();
  switchWatch(h, viewer, "r2", ["color_image"]);
  h.viewerAnswer(viewer, "a");
  await settle();
  // The negotiated pull is recorded, then closed as stale by the next run.
  assertEquals(sfu.of("renegotiate"), [["s2", "a"]]);
  assertEquals(sfu.of("closeTracks"), [["s2", ["p1"]]]);
  assertEquals(viewer.offers().map((o) => o.robotId), ["r1", "r2"]);
  h.viewerAnswer(viewer, "b");
  await settle();
  assertEquals(h.stats().pulls, 1);
});

Deno.test("a pull allocated while the watch moved is released, never offered", async () => {
  const sfu = new FakeSfu();
  const h = hub(sfu);
  h.robotOffer(new FakeRobot("r1", SPECS), ROBOT_OFFER); // s1
  h.robotOffer(new FakeRobot("r2", SPECS), ROBOT_OFFER); // s2
  const viewer = new FakeViewer();
  viewer.watched = "r1";
  viewer.subs.add("color_image");
  sfu.scripted = [
    DECLARED_CAM,
    () => {
      // The pull on the viewer's session (s3): the watch moves meanwhile.
      switchWatch(h, viewer, "r2", ["color_image"]);
      return { tracks: [{ mid: "p9", trackName: "color_image" }], sdp: "offer:s3" };
    },
  ];
  h.viewerOffer(viewer, "viewer-sdp");
  await settle();
  assertEquals(sfu.of("closeTracks"), [["s3", ["p9"]]]);
  assertEquals(viewer.offers().map((o) => o.robotId), ["r2"]);
  await done(h);
});

Deno.test("not found right after the declaration is awaited silently; not found past the track TTL re-declares", async () => {
  let t = 0;
  const sfu = new FakeSfu();
  sfu.scripted = [DECLARED_CAM, NOT_FOUND, NOT_FOUND];
  const h = hub(sfu, { now: () => t });
  const { viewer } = await connected(h, ["color_image"]);
  const locations = () => sfu.of("addTracks").map((a) => (a[1] as TrackRef[])[0].location);
  assertEquals(locations(), ["local", "remote", "remote", "remote"]);
  assertEquals(viewer.errors(), []);
  h.viewerAnswer(viewer, "a");
  await settle();
  viewer.subs.delete("color_image");
  h.viewerChanged(viewer);
  await settle();
  assertEquals(sfu.of("closeTracks"), [["s2", ["p1"]]]);
  // 60 s after the declaration the SFU has collected the unfed track: the
  // pull finds nothing, the track is declared again, then pulled.
  t = 60_000;
  sfu.scripted = [NOT_FOUND];
  viewer.subs.add("color_image");
  h.viewerChanged(viewer);
  await settle();
  assertEquals(locations().slice(4), ["remote", "local", "remote"]);
  assertEquals(viewer.offers().length, 2);
  await done(h);
});

Deno.test("a partially allocated pull negotiates what it got and pulls the rest later", async () => {
  const sfu = new FakeSfu();
  sfu.scripted = [
    DECLARED_BOTH,
    {
      tracks: [
        { mid: "p9", trackName: "color_image" },
        { trackName: "rear", errorCode: "not_found_track_error" },
      ],
      sdp: "offer:s2",
    },
  ];
  const h = hub(sfu);
  const { viewer } = await connected(h, ["color_image", "rear"]);
  assertEquals(viewer.offers()[0].tracks, [{ ch: "color_image", mid: "p9" }]);
  h.viewerAnswer(viewer, "a");
  await settle();
  // rear alone is pulled next; nothing was released.
  assertEquals(sfu.pulls().length, 2);
  assertEquals((sfu.pulls()[1][1] as TrackRef[]).map((t) => t.trackName), ["rear"]);
  assertEquals(sfu.of("closeTracks"), []);
  assertEquals(viewer.offers()[1].tracks, [{ ch: "rear", mid: "p1" }]);
  h.viewerAnswer(viewer, "b");
  await settle();
  assertEquals(h.stats().pulls, 2);
});

Deno.test("a hard per-track error reports rtc_failed for that track; the others stay negotiated", async () => {
  const sfu = new FakeSfu();
  sfu.scripted = [
    DECLARED_BOTH,
    {
      tracks: [
        { mid: "p9", trackName: "color_image" },
        { trackName: "rear", errorCode: "invalid_track_error" },
      ],
      sdp: "offer:s2",
    },
  ];
  const h = hub(sfu);
  const { viewer } = await connected(h, ["color_image", "rear"]);
  h.viewerAnswer(viewer, "a");
  await settle();
  assertEquals(sfu.of("closeTracks"), []);
  assertEquals(viewer.errors(), ["rtc_failed"]);
  // The backoff (instant here) re-ran and pulled the failed one alone.
  assertEquals(viewer.offers().length, 2);
  assertEquals(viewer.offers()[1].tracks, [{ ch: "rear", mid: "p1" }]);
  h.viewerAnswer(viewer, "b");
  await settle();
  assertEquals(h.stats().pulls, 2);
});

Deno.test("no rtc_answer in time: the pulls are released and the retry re-pulls", async () => {
  const sfu = new FakeSfu();
  const h = hub(sfu, { answerTimeoutMs: 20 });
  const { viewer } = await connected(h, ["color_image"]);
  assertEquals(viewer.offers().length, 1);
  await new Promise((resolve) => setTimeout(resolve, 60));
  await settle();
  assertEquals(sfu.of("closeTracks")[0], ["s2", ["p1"]]);
  assertEquals(viewer.errors()[0], "rtc_failed");
  assert(viewer.offers().length >= 2, "re-pulled after the timeout");
  assertEquals(sfu.of("renegotiate"), []);
  // Answering the latest offer completes it.
  h.viewerAnswer(viewer, "late-but-fine");
  await settle();
  assertEquals(sfu.of("renegotiate").length, 1);
  assertEquals(h.stats().pulls, 1);
});

Deno.test("a failed renegotiation releases the pulls and retries", async () => {
  const sfu = new FakeSfu();
  sfu.renegotiateError = new Error("boom");
  const h = hub(sfu);
  const { viewer } = await connected(h, ["color_image"]);
  h.viewerAnswer(viewer, "a");
  await settle();
  assertEquals(sfu.of("closeTracks"), [["s2", ["p1"]]]);
  assertEquals(viewer.errors(), ["rtc_failed"]);
  assertEquals(viewer.offers().length, 2);
  h.viewerAnswer(viewer, "b");
  await settle();
  assertEquals(h.stats().pulls, 1);
});

Deno.test("a failed viewer session replies rtc_failed and is retried with the same offer", async () => {
  const sfu = new FakeSfu();
  sfu.newSessionError = new Error("sfu down");
  const h = hub(sfu);
  const viewer = new FakeViewer();
  h.viewerOffer(viewer, "v");
  await settle();
  assertEquals(viewer.errors(), ["rtc_failed"]);
  assertEquals(sfu.of("newSession"), [["v"], ["v"]]);
  assertEquals(viewer.pushed.at(-1), { t: "rtc_answer", sdp: "answer:v" });
});

Deno.test("a failed robot session stays silent", async () => {
  const sfu = new FakeSfu();
  sfu.newSessionError = new Error("sfu down");
  const h = hub(sfu);
  const robot = new FakeRobot("r1", SPECS);
  h.robotOffer(robot, ROBOT_OFFER);
  await settle();
  assertEquals(robot.control, []);
  assertEquals(robot.msgs, []);
  assertEquals(h.stats().robots, 0);
});

Deno.test("the viewer's session gone on a pull drops it and asks for a new offer; the re-offer pulls without re-declaring", async () => {
  const sfu = new FakeSfu();
  sfu.scripted = [DECLARED_CAM, new SessionGoneError(410, null, "gone", "s2")];
  const h = hub(sfu);
  const { viewer } = await connected(h, ["color_image"]);
  assertEquals(viewer.errors(), ["rtc_session_gone"]);
  assertEquals(sfu.of("closeTracks"), []);
  assertEquals(h.stats().viewers, 0);
  h.viewerOffer(viewer, "again");
  await settle();
  assertEquals(sfu.locals().length, 1);
  assertEquals(sfu.pulls().map((a) => a[0]), ["s2", "s3"]);
  assertEquals(viewer.offers().length, 1);
  await done(h);
});

Deno.test("the robot's session gone on a declaration drops the robot; viewers get one rtc_failed and settle; a fresh offer declares again", async () => {
  const sfu = new FakeSfu();
  sfu.scripted = [new SessionGoneError(410, null, "gone", "s1")];
  const h = hub(sfu);
  const { viewer } = await connected(h, ["color_image"]);
  assertEquals(h.stats().robots, 0);
  assertEquals(sfu.of("addTracks").length, 1);
  const again = new FakeRobot("r1", SPECS);
  h.robotOffer(again, ROBOT_OFFER);
  await settle();
  assertEquals(sfu.locals().map((a) => a[0]), ["s1", "s3"]);
  assertEquals(viewer.offers().length, 1);
  assertEquals(viewer.errors(), ["rtc_failed"]);
  await done(h);
});

Deno.test("a declaration failing several tracks unmarks every failed one; the retry declares them all", async () => {
  const sfu = new FakeSfu();
  sfu.scripted = [
    {
      tracks: [
        { trackName: "color_image", errorCode: "invalid_track_error" },
        { trackName: "rear", errorCode: "invalid_track_error" },
      ],
      sdp: null,
    },
  ];
  const h = hub(sfu);
  const { viewer } = await connected(h, ["color_image", "rear"]);
  assertEquals(sfu.locals().map((a) => (a[1] as TrackRef[]).map((t) => t.trackName)), [
    ["color_image", "rear"],
    ["color_image", "rear"],
  ]);
  assertEquals(viewer.errors(), ["rtc_failed"]);
  h.viewerAnswer(viewer, "a");
  await settle();
  assertEquals(h.stats().pulls, 2);
  await done(h);
});

Deno.test("a declaration that fails per track (or returns no result) reports rtc_failed and is retried, never marked", async () => {
  const sfu = new FakeSfu();
  sfu.scripted = [
    { tracks: [], sdp: null },
    { tracks: [{ trackName: "color_image", errorCode: "invalid_track_error" }], sdp: null },
  ];
  const h = hub(sfu);
  const { viewer } = await connected(h, ["color_image"]);
  assertEquals(
    sfu.of("addTracks").map((a) => (a[1] as TrackRef[])[0].location),
    ["local", "local", "local", "remote"],
  );
  assertEquals(viewer.errors(), ["rtc_failed", "rtc_failed"]);
  assertEquals(viewer.offers().length, 1);
  await done(h);
});

Deno.test("a failed close is retried before the next pull; the new robot's pull waits for it", async () => {
  const sfu = new FakeSfu();
  const h = hub(sfu);
  const { viewer } = await connected(h, ["color_image"]);
  h.viewerAnswer(viewer, "a");
  const other = new FakeRobot("r2", SPECS);
  h.robotOffer(other, ROBOT_OFFER);
  await settle();
  sfu.closeError = new Error("timeout");
  switchWatch(h, viewer, "r2", ["color_image"]);
  await settle();
  assertEquals(sfu.of("closeTracks"), [["s2", ["p1"]], ["s2", ["p1"]]]);
  assertEquals(viewer.errors(), ["rtc_failed"]);
  assertEquals(sfu.pulls().map((a) => (a[1] as TrackRef[])[0].sessionId), ["s1", "s3"]);
  const methods = sfu.methods();
  assert(methods.lastIndexOf("closeTracks") < methods.lastIndexOf("addTracks"), "closed first");
  await done(h);
});

Deno.test("a 4xx on a close keeps the mid pending; the retry confirms it before the next pull", async () => {
  const sfu = new FakeSfu();
  const h = hub(sfu);
  const { viewer } = await connected(h, ["color_image"]);
  h.viewerAnswer(viewer, "a");
  await settle();
  sfu.closeError = new CloudflareError(400, "invalid_mid", "bad", "s2");
  viewer.subs.delete("color_image");
  h.viewerChanged(viewer);
  await settle();
  // The instant backoff re-ran and the retry was confirmed.
  assertEquals(sfu.of("closeTracks"), [["s2", ["p1"]], ["s2", ["p1"]]]);
  assertEquals(viewer.errors(), ["rtc_failed"]);
  viewer.subs.add("color_image");
  h.viewerChanged(viewer);
  await settle();
  assertEquals(sfu.of("closeTracks").length, 2); // nothing left to close
  assertEquals(viewer.offers().length, 2);
  await done(h);
});

Deno.test("a mid the SFU failed to close in a 200 stays pending without holding up the pull, and is retried until confirmed", async () => {
  const sfu = new FakeSfu();
  const h = hub(sfu);
  const { viewer } = await connected(h, ["color_image"]);
  h.viewerAnswer(viewer, "a");
  await settle();
  sfu.closeResults = [[{ mid: "p1", errorCode: "close_error" }]];
  viewer.subs.delete("color_image");
  viewer.subs.add("rear");
  h.viewerChanged(viewer);
  await settle();
  // The failed close is kept and the run goes on to pull rear.
  assertEquals(sfu.of("closeTracks"), [["s2", ["p1"]]]);
  assertEquals(viewer.errors(), []);
  assertEquals(viewer.offers().length, 2);
  assertEquals(viewer.offers()[1].tracks, [{ ch: "rear", mid: "p2" }]);
  h.viewerAnswer(viewer, "b");
  await settle();
  // The run ended pending: its (instant) re-run confirmed the close.
  assertEquals(sfu.of("closeTracks"), [["s2", ["p1"]], ["s2", ["p1"]]]);
  const methods = sfu.methods();
  assert(methods.lastIndexOf("closeTracks") > methods.lastIndexOf("renegotiate"), "after");
  assertEquals(h.stats().pulls, 1);
  await done(h);
});

Deno.test("a close that keeps failing is given up after MAX_CLOSE_ATTEMPTS tries", async () => {
  const sfu = new FakeSfu();
  const h = hub(sfu);
  const { viewer } = await connected(h, ["color_image"]);
  h.viewerAnswer(viewer, "a");
  await settle();
  sfu.closeResults = Array.from({ length: MAX_CLOSE_ATTEMPTS + 2 }, () => [
    { mid: "p1", errorCode: "close_error" },
  ]);
  viewer.subs.delete("color_image");
  h.viewerChanged(viewer);
  await settle();
  assertEquals(sfu.of("closeTracks").length, MAX_CLOSE_ATTEMPTS);
  assertEquals(viewer.errors(), []);
  assertEquals(h.stats().pulls, 0);
  await done(h);
});

Deno.test("a stalled track (the bridge's rtc_stalled) closes its pulls and pulls it again; a not_found re-declares at once", async () => {
  let t = 0;
  const sfu = new FakeSfu();
  const h = hub(sfu, { now: () => t });
  const { robot, viewer } = await connected(h, ["color_image", "rear"]);
  h.viewerAnswer(viewer, "a");
  await settle();
  assertEquals(h.stats().pulls, 2);
  t = 5_000; // well inside the declaration's track lifetime
  sfu.scripted = [NOT_FOUND]; // the re-pull: the SFU collected the track
  h.robotStalled(robot, "color_image");
  await settle();
  // The dead pull is closed; the re-pull finds nothing, the aged declaration
  // is repeated at once and the track pulled again. rear is untouched.
  assertEquals(sfu.of("closeTracks"), [["s2", ["p1"]]]);
  const after = sfu.of("addTracks").slice(2).map((a) => {
    const [ref] = a[1] as TrackRef[];
    return [ref.location, ref.trackName];
  });
  assertEquals(after, [
    ["remote", "color_image"],
    ["local", "color_image"],
    ["remote", "color_image"],
  ]);
  assertEquals(viewer.errors(), []);
  assertEquals(viewer.offers().length, 2);
  assertEquals(viewer.offers()[1].tracks, [{ ch: "color_image", mid: "p3" }]);
  h.viewerAnswer(viewer, "b");
  await settle();
  assertEquals(h.stats().pulls, 2);
  // A channel that is not one of the robot's tracks is ignored.
  h.robotStalled(robot, "odom");
  await settle();
  assertEquals(sfu.of("closeTracks").length, 1);
  await done(h);
});

Deno.test("an event during a pending wait reconciles at once", async () => {
  const sleeps: (() => void)[] = [];
  const sfu = new FakeSfu();
  sfu.scripted = [DECLARED_CAM, NOT_FOUND];
  const h = new RtcHub(sfu, {
    sleep: () => new Promise<void>((resolve) => sleeps.push(resolve)),
    answerTimeoutMs: 1000,
  });
  h.robotOffer(new FakeRobot("r1", SPECS), ROBOT_OFFER); // s1
  h.robotOffer(new FakeRobot("r2", SPECS), ROBOT_OFFER); // s2
  await settle();
  const viewer = new FakeViewer();
  viewer.watched = "r1";
  viewer.subs.add("color_image");
  h.viewerOffer(viewer, "viewer-sdp"); // s3
  await settle();
  assertEquals(sleeps.length, 1);
  assertEquals(viewer.errors(), []);
  assertEquals(viewer.offers(), []);
  switchWatch(h, viewer, "r2", ["color_image"]);
  await settle();
  assertEquals(sleeps.length, 1); // the switch ran without waiting
  assertEquals(viewer.offers().length, 1);
  assertEquals(viewer.offers()[0].robotId, "r2");
  sleeps[0]();
  h.viewerAnswer(viewer, "a");
  await settle();
  assertEquals(sfu.pulls().length, 2); // the wakeup found nothing missing
  assertEquals(h.stats().pulls, 1);
});

Deno.test("a viewer re-offer replaces its session; old pulls are forgotten, not closed", async () => {
  const sfu = new FakeSfu();
  const h = hub(sfu);
  const { viewer } = await connected(h, ["color_image"]);
  h.viewerAnswer(viewer, "a");
  await settle();
  h.viewerOffer(viewer, "viewer-sdp-2");
  await settle();
  assertEquals(sfu.of("closeTracks"), []);
  // Pulled afresh onto the new session s3.
  assertEquals(sfu.pulls().map((a) => a[0]), ["s2", "s3"]);
  await done(h);
});

Deno.test("refreshIce mints new credentials and notifies; a failed mint keeps the set, silently", async () => {
  const sfu = new FakeSfu();
  const h = hub(sfu);
  let notified = 0;
  await h.start(() => notified++);
  assertEquals(notified, 0); // the first mint precedes every peer
  sfu.ice = [{ urls: ["turn:t"], username: "u2", credential: "c2" }];
  await h.refreshIce();
  assertEquals(notified, 1);
  assertEquals(h.iceMsg(), { t: "rtc_ice", iceServers: sfu.ice });
  sfu.iceError = new Error("HTTP 500");
  await h.refreshIce();
  assertEquals(notified, 1);
  assertEquals(h.iceMsg(), { t: "rtc_ice", iceServers: sfu.ice });
  assertEquals(h.stats().apiErrors, 1);
  h.dispose();
});

Deno.test("start() resolves once the first mint landed; a failed mint leaves STUN only", async () => {
  const sfu = new FakeSfu();
  const h = hub(sfu);
  assertEquals(h.iceMsg(), { t: "rtc_ice", iceServers: STUN_ONLY });
  await h.start();
  assertEquals(h.iceMsg(), { t: "rtc_ice", iceServers: sfu.ice });
  h.dispose();
  const failing = new FakeSfu();
  failing.iceError = new Error("HTTP 500");
  const h2 = hub(failing);
  await h2.start();
  assertEquals(h2.iceMsg(), { t: "rtc_ice", iceServers: STUN_ONLY });
  assertEquals(h2.stats().apiErrors, 1);
  h2.dispose();
});

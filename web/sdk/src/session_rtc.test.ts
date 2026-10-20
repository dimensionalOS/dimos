// The session's WebRTC leg over the fake relay and a fake PeerConnection.
import { afterEach, describe, expect, it } from "vitest";
import { type IceServer, MAX_SDP_LEN, type Msg, PROTOCOL_VERSION } from "@dimos/shared";
import { TRACK_ENCODING } from "@dimos/shared/manifest";
import { FakePeerConnection } from "./rtc.test.ts";
import { connect, type Session } from "./session.ts";
import {
  FakeRelayEnd,
  INFO,
  manifest,
  ROBOT_A,
  ROBOT_B,
  settle,
  spec,
  until,
} from "./testing/fakeRelay.ts";
import { backoffDelayMs } from "./transport.ts";

const STUN: IceServer[] = [{ urls: ["stun:stun.cloudflare.com:3478"] }];
const ICE: Msg = { t: "rtc_ice", iceServers: STUN };
const CAM = spec({ ch: "cam", encoding: TRACK_ENCODING, delivery: "latest" });

describe("Session WebRTC leg", () => {
  const handles: Session[] = [];

  afterEach(() => {
    for (const handle of handles.splice(0)) handle.close();
  });

  function start(makePc = () => new FakePeerConnection()): {
    relays: FakeRelayEnd[];
    pcs: FakePeerConnection[];
    /** The ICE servers each PeerConnection was created with. */
    configs: IceServer[][];
    handle: Session;
  } {
    const relays: FakeRelayEnd[] = [];
    const pcs: FakePeerConnection[] = [];
    const configs: IceServer[][] = [];
    const handle = connect(undefined, {
      fetchInfo: () => Promise.resolve({ ...INFO, rtc: true }),
      createWebTransport: () => {
        const relay = new FakeRelayEnd();
        relays.push(relay);
        return relay.wt;
      },
      createPeerConnection: (config) => {
        configs.push(config.iceServers);
        const pc = makePc();
        pcs.push(pc);
        return pc;
      },
    });
    handles.push(handle);
    return { relays, pcs, configs, handle };
  }

  /** welcome, robots, rtc_ice (the relay's order), then the manifest. */
  async function goLive(relay: FakeRelayEnd, handle: Session, channels = [CAM, spec()]) {
    relay.push({ t: "welcome", v: PROTOCOL_VERSION });
    relay.push({ t: "robots", robots: [ROBOT_A] });
    relay.push(ICE);
    await until(() => relay.watches(ROBOT_A.id) === 1, "watch");
    relay.pushManifest(ROBOT_A.id, manifest(channels));
    await until(() => handle.status.get().manifest !== null, "manifest");
  }

  function offers(relay: FakeRelayEnd): Msg[] {
    return relay.sent.filter((m) => m.t === "rtc_offer");
  }

  function answers(relay: FakeRelayEnd): Msg[] {
    return relay.sent.filter((m) => m.t === "rtc_answer");
  }

  /** A relay pull offer naming robot A's track. */
  function pull(sdp: string, mid: string, robotId = ROBOT_A.id): Msg {
    return { t: "rtc_offer", sdp, robotId, tracks: [{ ch: "cam", mid }] };
  }

  it("creates no PeerConnection without a track subscription", async () => {
    const { relays, pcs, handle } = start();
    await until(() => relays.length === 1, "connection");
    handle.subscribe("odom", () => {});
    await goLive(relays[0], handle);
    await until(() => relays[0].subs().includes("odom"), "sub");
    await settle();
    expect(pcs).toHaveLength(0);
    expect(offers(relays[0])).toEqual([]);
  });

  it("the first track sub creates the PeerConnection and offers; the answer is applied", async () => {
    const { relays, pcs, handle } = start();
    await until(() => relays.length === 1, "connection");
    handle.subscribe("cam", () => {});
    handle.subscribe("odom", () => {});
    await goLive(relays[0], handle);
    await until(() => offers(relays[0]).length === 1, "rtc_offer");
    expect(pcs).toHaveLength(1);
    expect(offers(relays[0])).toEqual([{ t: "rtc_offer", sdp: pcs[0].offerSdp }]);
    relays[0].push({ t: "rtc_answer", sdp: "v=0\r\nsfu-answer\r\n" });
    await until(() => pcs[0].remote.length === 1, "answer applied");
    expect(pcs[0].remote[0]).toEqual({ type: "answer", sdp: "v=0\r\nsfu-answer\r\n" });
    // A second track consumer adds nothing: one PeerConnection per connection.
    handle.subscribe("cam", () => {});
    await settle();
    expect(pcs).toHaveLength(1);
  });

  it("rtc_ice arriving after the track sub still creates the PeerConnection", async () => {
    const { relays, pcs, handle } = start();
    await until(() => relays.length === 1, "connection");
    handle.subscribe("cam", () => {});
    const relay = relays[0];
    relay.push({ t: "welcome", v: PROTOCOL_VERSION });
    relay.push({ t: "robots", robots: [ROBOT_A] });
    await until(() => relay.watches(ROBOT_A.id) === 1, "watch");
    relay.pushManifest(ROBOT_A.id, manifest([CAM]));
    await until(() => relay.subs().includes("cam"), "sub");
    await settle();
    expect(pcs).toHaveLength(0);
    relay.push(ICE);
    await until(() => offers(relay).length === 1, "rtc_offer");
    expect(pcs).toHaveLength(1);
  });

  it("a relay rtc_offer is answered and its tracks become the channels' values", async () => {
    const { relays, pcs, handle } = start();
    await until(() => relays.length === 1, "connection");
    handle.subscribe("cam", () => {});
    await goLive(relays[0], handle);
    await until(() => offers(relays[0]).length === 1, "rtc_offer");
    relays[0].push({ t: "rtc_answer", sdp: "v=0\r\nsfu-answer\r\n" });
    relays[0].push(pull("v=0\r\nm=video\r\n", "0"));
    await until(() => relays[0].sent.some((m) => m.t === "rtc_answer"), "rtc_answer");
    expect(relays[0].sent.filter((m) => m.t === "rtc_answer")).toEqual([
      { t: "rtc_answer", sdp: pcs[0].answerSdp },
    ]);
    expect(handle.store.get("cam")?.value).toBe(pcs[0].transceivers[0].receiver.track);

    // A later pull (a re-sub after an unsub) supersedes.
    relays[0].push(pull("v=0\r\nm=video\r\nm=video\r\n", "1"));
    await until(
      () => handle.store.get("cam")?.value === pcs[0].transceivers[1]?.receiver.track,
      "second pull",
    );
  });

  it("an offer naming an unknown mid reports rtc_failed and keeps the session", async () => {
    const { relays, handle } = start();
    await until(() => relays.length === 1, "connection");
    handle.subscribe("cam", () => {});
    await goLive(relays[0], handle);
    await until(() => offers(relays[0]).length === 1, "rtc_offer");
    relays[0].push(pull("v=0\r\nm=video\r\n", "9"));
    await until(() => handle.status.get().lastError?.code === "rtc_failed", "error");
    expect(handle.status.get().transport).toEqual({ phase: "connected" });
  });

  it("refuses an oversize offer locally instead of killing the control stream", async () => {
    const { relays, handle } = start(() => {
      const pc = new FakePeerConnection();
      pc.offerSdp = "x".repeat(MAX_SDP_LEN + 1);
      return pc;
    });
    await until(() => relays.length === 1, "connection");
    handle.subscribe("cam", () => {});
    await goLive(relays[0], handle, [CAM]);
    await until(() => handle.status.get().lastError?.code === "rtc_failed", "error");
    expect(offers(relays[0])).toHaveLength(0);
    expect(handle.status.get().transport).toEqual({ phase: "connected" });
  });

  it("a pull for a robot no longer watched is answered but not ingested", async () => {
    const { relays, pcs, handle } = start();
    await until(() => relays.length === 1, "connection");
    handle.subscribe("cam", () => {});
    const relay = relays[0];
    relay.push({ t: "welcome", v: PROTOCOL_VERSION });
    relay.push({ t: "robots", robots: [ROBOT_A, ROBOT_B] });
    relay.push(ICE);
    const watchA = handle.watch(ROBOT_A.id);
    await until(() => relay.watches(ROBOT_A.id) === 1, "watch a");
    relay.pushManifest(ROBOT_A.id, manifest([CAM]));
    await watchA;
    await until(() => offers(relay).length === 1, "rtc_offer");
    relay.push({ t: "rtc_answer", sdp: "v=0\r\nsfu-answer\r\n" });
    await until(() => pcs[0].remote.length === 1, "answer applied");
    // A's pull lands after the switch to B: answered (an SFU offer must be),
    // not ingested (the channel name is B's too), on the same PeerConnection.
    const watchB = handle.watch(ROBOT_B.id);
    relay.push(pull("v=0\r\nm=video\r\n", "0"));
    await until(() => answers(relay).length === 1, "rtc_answer");
    await settle();
    expect(handle.store.get("cam")).toBeNull();
    // B's own pull is ingested.
    await until(() => relay.watches(ROBOT_B.id) === 1, "watch b");
    relay.pushManifest(ROBOT_B.id, manifest([CAM]));
    await watchB;
    await until(() => relay.subs().length === 2, "re-sub");
    relay.push(pull("v=0\r\nm=video\r\nm=video\r\n", "1", ROBOT_B.id));
    await until(() => handle.store.get("cam") !== null, "b's track");
    expect(handle.store.get("cam")?.value).toBe(pcs[0].transceivers[1].receiver.track);
  });

  it("a failed offer is closed and offered again after a backoff; rtc_session_gone drops the peer and re-offers", async () => {
    let first = true;
    const { relays, pcs, handle } = start(() => {
      const pc = new FakePeerConnection();
      if (first) {
        first = false;
        pc.createOffer = () => Promise.reject(new Error("no offer"));
      }
      return pc;
    });
    await until(() => relays.length === 1, "connection");
    handle.subscribe("cam", () => {});
    await goLive(relays[0], handle, [CAM]);
    await until(() => handle.status.get().lastError?.code === "rtc_failed", "error");
    expect(pcs[0].closed).toBe(1);
    await settle();
    expect(pcs).toHaveLength(1); // not before the backoff
    await until(() => offers(relays[0]).length === 1, "re-offer"); // the 500 ms rung
    expect(pcs).toHaveLength(2);
    relays[0].push({ t: "rtc_answer", sdp: "v=0\r\nsfu-answer\r\n" });
    await until(() => pcs[1].remote.length === 1, "answer applied");
    relays[0].push({ t: "error", code: "rtc_session_gone", message: "SFU session gone" });
    await until(() => pcs[1].closed === 1, "peer dropped");
    expect(handle.status.get().lastError).toEqual({
      code: "rtc_failed",
      message: expect.stringContaining("SFU session gone"),
    });
    await until(() => offers(relays[0]).length === 2, "third offer");
  });

  it("a rejected answer or renegotiation is a peer failure too", async () => {
    let n = 0;
    const { relays, pcs, handle } = start(() => {
      const pc = new FakePeerConnection();
      const real = pc.setRemoteDescription.bind(pc);
      n++;
      if (n === 1) pc.setRemoteDescription = () => Promise.reject(new Error("bad answer"));
      if (n === 2) {
        pc.setRemoteDescription = (d) =>
          d.type === "offer" ? Promise.reject(new Error("bad offer")) : real(d);
      }
      return pc;
    });
    await until(() => relays.length === 1, "connection");
    handle.subscribe("cam", () => {});
    await goLive(relays[0], handle, [CAM]);
    await until(() => offers(relays[0]).length === 1, "first offer");
    relays[0].push({ t: "rtc_answer", sdp: "v=0\r\nsfu-answer\r\n" });
    await until(() => pcs[0].closed === 1, "first peer dropped");
    await until(() => offers(relays[0]).length === 2, "second offer");
    relays[0].push({ t: "rtc_answer", sdp: "v=0\r\nsfu-answer\r\n" });
    await until(() => pcs[1].remote.length === 1, "answer applied");
    relays[0].push(pull("v=0\r\nm=video\r\n", "0"));
    await until(() => pcs[1].closed === 1, "second peer dropped");
    expect(answers(relays[0])).toEqual([]);
    expect(handle.status.get().lastError?.code).toBe("rtc_failed");
  });

  it("a connection that fails after the answer is dropped and offered again", async () => {
    const { relays, pcs, handle } = start();
    await until(() => relays.length === 1, "connection");
    handle.subscribe("cam", () => {});
    await goLive(relays[0], handle, [CAM]);
    await until(() => offers(relays[0]).length === 1, "first offer");
    relays[0].push({ t: "rtc_answer", sdp: "v=0\r\nsfu-answer\r\n" });
    await until(() => pcs[0].remote.length === 1, "answer applied");
    pcs[0].connection("failed");
    expect(pcs[0].closed).toBe(1);
    expect(handle.status.get().lastError).toEqual({
      code: "rtc_failed",
      message: expect.stringContaining("connection failed"),
    });
    await until(() => offers(relays[0]).length === 2, "re-offer");
    expect(pcs).toHaveLength(2);
    pcs[0].connection("failed"); // the dropped peer: superseded, ignored
    await settle();
    expect(pcs).toHaveLength(2);
  });

  it("no re-offer once the track channel is released", async () => {
    const { relays, pcs, handle } = start(() => {
      const pc = new FakePeerConnection();
      pc.createOffer = () => Promise.reject(new Error("no offer"));
      return pc;
    });
    await until(() => relays.length === 1, "connection");
    const release = handle.subscribe("cam", () => {});
    await goLive(relays[0], handle, [CAM]);
    await until(() => handle.status.get().lastError?.code === "rtc_failed", "error");
    release();
    await new Promise((resolve) => setTimeout(resolve, backoffDelayMs(1) + 50));
    expect(pcs).toHaveLength(1);
  });

  it("closes the PeerConnection with the connection and offers again on the next one", async () => {
    const { relays, pcs, handle } = start();
    await until(() => relays.length === 1, "connection");
    handle.subscribe("cam", () => {});
    await goLive(relays[0], handle);
    await until(() => offers(relays[0]).length === 1, "first offer");
    relays[0].endControl();
    await until(() => relays.length === 2, "reconnect");
    expect(pcs[0].closed).toBe(1);
    await goLive(relays[1], handle);
    await until(() => offers(relays[1]).length === 1, "second offer");
    expect(pcs).toHaveLength(2);
  });

  it("keeps the pulled track through an ambiguous robot list and restores it once the robot is watched again", async () => {
    const { relays, pcs, handle } = start();
    await until(() => relays.length === 1, "connection");
    handle.subscribe("cam", () => {});
    const relay = relays[0];
    await goLive(relay, handle, [CAM]);
    await until(() => offers(relay).length === 1, "rtc_offer");
    relay.push({ t: "rtc_answer", sdp: "v=0\r\nsfu-answer\r\n" });
    relay.push(pull("v=0\r\nm=video\r\n", "0"));
    const track = pcs[0].transceivers[0].receiver.track;
    await until(() => handle.store.get("cam")?.value === track, "track");
    // A second robot makes the auto-watch ambiguous: the producer is dropped
    // locally while the relay keeps the watch, the subscription and the pull.
    relay.push({ t: "robots", robots: [ROBOT_A, ROBOT_B] });
    await until(() => handle.status.get().manifest === null, "producer cleared");
    expect(handle.store.get("cam")).toBeNull();
    relay.push({ t: "robots", robots: [ROBOT_A] });
    await until(() => relay.watches(ROBOT_A.id) === 2, "watch again");
    relay.pushManifest(ROBOT_A.id, manifest([CAM]));
    await until(() => handle.status.get().manifest !== null, "manifest again");
    expect(handle.store.get("cam")?.value).toBe(track);
    // Nothing was re-subscribed or re-negotiated: the relay's pull is live.
    await settle();
    expect(relay.subs()).toEqual(["cam"]);
    expect(offers(relay)).toHaveLength(1);
    expect(pcs).toHaveLength(1);
  });

  it("drops the previous robot's tracks on a watch switch: switching back waits for a new pull", async () => {
    const { relays, pcs, handle } = start();
    await until(() => relays.length === 1, "connection");
    handle.subscribe("cam", () => {});
    const relay = relays[0];
    relay.push({ t: "welcome", v: PROTOCOL_VERSION });
    relay.push({ t: "robots", robots: [ROBOT_A, ROBOT_B] });
    relay.push(ICE);
    const watchA = handle.watch(ROBOT_A.id);
    await until(() => relay.watches(ROBOT_A.id) === 1, "watch a");
    relay.pushManifest(ROBOT_A.id, manifest([CAM]));
    await watchA;
    await until(() => offers(relay).length === 1, "rtc_offer");
    relay.push({ t: "rtc_answer", sdp: "v=0\r\nsfu-answer\r\n" });
    relay.push(pull("v=0\r\nm=video\r\n", "0"));
    const trackA = pcs[0].transceivers[0].receiver.track;
    await until(() => handle.store.get("cam")?.value === trackA, "a's track");
    const watchB = handle.watch(ROBOT_B.id);
    await until(() => relay.watches(ROBOT_B.id) === 1, "watch b");
    relay.pushManifest(ROBOT_B.id, manifest([CAM]));
    await watchB;
    const watchAgain = handle.watch(ROBOT_A.id);
    await until(() => relay.watches(ROBOT_A.id) === 2, "watch a again");
    relay.pushManifest(ROBOT_A.id, manifest([CAM]));
    await watchAgain;
    await settle();
    // The relay closed A's pull on the switch: the old track is not shown.
    expect(handle.store.get("cam")).toBeNull();
    relay.push(pull("v=0\r\nm=video\r\nm=video\r\n", "1"));
    await until(
      () => handle.store.get("cam")?.value === pcs[0].transceivers[1]?.receiver.track,
      "new pull",
    );
  });

  it("a refreshed rtc_ice is used by the next PeerConnection", async () => {
    const { relays, pcs, configs, handle } = start();
    await until(() => relays.length === 1, "connection");
    handle.subscribe("cam", () => {});
    await goLive(relays[0], handle, [CAM]);
    await until(() => offers(relays[0]).length === 1, "first offer");
    const turn = [{ urls: ["turn:turn.cloudflare.com:3478"], username: "u2", credential: "c2" }];
    relays[0].push({ t: "rtc_ice", iceServers: turn });
    await settle();
    expect(pcs).toHaveLength(1); // a refresh alone replaces nothing
    pcs[0].connection("failed");
    await until(() => offers(relays[0]).length === 2, "re-offer");
    expect(configs).toEqual([STUN, turn]);
  });
});

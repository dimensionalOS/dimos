import { afterEach, describe, expect, it, vi } from "vitest";
import type { Msg, TopicSettings, TopicSubMsg, TopicUpdateMsg } from "@dimos/shared";
import { FakePeerConnection } from "./testing/fakePeerConnection.ts";
import { TopicClient, type TopicSample, type TopicStatus } from "./topics.ts";
import {
  RTC_ANSWER_TIMEOUT_MS,
  RTC_CONNECT_TIMEOUT_MS,
  RTC_MAX_FAILURES,
  RTC_RETRY_MS,
  type TopicVideoOptions,
  VIDEO_FALLBACK_PROFILE,
  VIDEO_PROFILE,
} from "./topicVideo.ts";

const PROFILE_OPTIONS = {
  raw: { maxHz: { kind: "number", max: 1000, default: 1000 } },
  [VIDEO_PROFILE]: {
    maxHz: { kind: "number", max: 1000, default: 30 },
    maxResolution: { kind: "size", min: 16, max: 4096, default: [1920, 1080] },
    maxBitrate: { kind: "int", min: 100000, max: 4500000, default: 4500000 },
  },
  [VIDEO_FALLBACK_PROFILE]: {
    maxHz: { kind: "number", max: 1000, default: 30 },
    maxResolution: { kind: "size", min: 16, max: 4096, default: [1280, 720] },
    quality: { kind: "int", min: 1, max: 100, default: 75 },
  },
} as const;

async function flush() {
  for (let i = 0; i < 10; i++) await Promise.resolve();
}

describe("direct topic video", () => {
  const owned: { close(): void }[] = [];
  afterEach(() => {
    for (const item of owned.splice(0)) item.close();
    vi.useRealTimers();
  });
  function setup(
    profiles: string[] = ["raw", VIDEO_PROFILE, VIDEO_FALLBACK_PROFILE],
    profileOptions: Record<string, unknown> = PROFILE_OPTIONS,
  ) {
    const pcs: FakePeerConnection[] = [];
    const client = new TopicClient("workspace", () => {
      const pc = new FakePeerConnection();
      pcs.push(pc);
      return pc;
    });
    owned.push(client);
    const sent: Msg[] = [];
    const connect = () => client.connected((m) => sent.push(m), vi.fn());
    connect();
    const sources = (gen = 1) =>
      client.onMessage({
        t: "topic_sources",
        v: 1,
        sources: [{
          source: "workspace",
          gen,
          read: true,
          write: false,
          profiles,
          profileOptions: profileOptions as never,
        }],
      });
    const subs = () => sent.filter((m): m is TopicSubMsg => m.t === "topic_sub");
    const ready = (
      route = "@t:v1",
      gen = 1,
      sub = subs().at(-1)!,
      settings: TopicSettings = { maxHz: 30, maxResolution: [1920, 1080] },
    ) => {
      client.onMessage({
        t: "topic_ready",
        id: sub.id,
        source: "workspace",
        route,
        gen,
        settings,
      });
      return { id: sub.id, route, gen };
    };
    const offers = () => sent.filter((m) => m.t === "topic_rtc_offer");
    const rtc = () => sent.filter((m) => m.t.startsWith("topic_rtc_"));
    const tracks: (MediaStreamTrack | null)[] = [];
    const statuses: TopicStatus[] = [];
    const video = (options: TopicVideoOptions = {}) => {
      const handle = client.video("sensor/cam", options, (track) => tracks.push(track));
      handle.onStatus((status) => statuses.push(status));
      owned.push(handle);
      return handle;
    };
    return {
      client,
      pcs,
      sent,
      connect,
      sources,
      subs,
      ready,
      offers,
      rtc,
      tracks,
      statuses,
      video,
    };
  }

  it("offers once the route is ready, trickles candidates, applies the answer and reports the track", async () => {
    const h = setup();
    const video = h.video({ maxHz: 15 });
    h.sources();
    expect(h.subs()).toMatchObject([{ profile: VIDEO_PROFILE, delivery: "latest", maxHz: 15 }]);
    expect(video.mode()).toBe("none");
    const ref = h.ready();
    await flush();
    const [pc] = h.pcs;
    expect(pc.added).toEqual([{ kind: "video", init: { direction: "recvonly" } }]);
    expect(h.offers()).toEqual([{ t: "topic_rtc_offer", ...ref, sdp: pc.offerSdp }]);
    pc.candidate("host");
    pc.candidate(null);
    expect(h.rtc().slice(1)).toEqual([
      {
        t: "topic_rtc_candidate",
        ...ref,
        candidate: "candidate:1 1 udp 1 10.0.0.2 5000 typ host",
        mid: "0",
      },
      { t: "topic_rtc_candidate", ...ref, candidate: "", mid: "" },
    ]);
    h.client.onMessage({ t: "topic_rtc_answer", ...ref, sdp: "v=0 answer" });
    h.client.onMessage({
      t: "topic_rtc_candidate",
      ...ref,
      candidate: "candidate:9 host",
      mid: "0",
    });
    h.client.onMessage({ t: "topic_rtc_candidate", ...ref, candidate: "", mid: "0" });
    await flush();
    expect(pc.remote).toEqual([{ type: "answer", sdp: "v=0 answer" }]);
    expect(pc.remoteCandidates).toEqual([{ candidate: "candidate:9 host", sdpMid: "0" }, null]);
    expect(video.status().phase).toBe("connecting");
    const track = pc.track();
    expect(h.tracks).toEqual([track]);
    expect(video.track()).toBe(track);
    expect(video.receiver()).toBe(pc.transceivers[0].receiver);
    // A track only says the answer applied: it plays once the first packet arrives.
    expect(video.mode()).toBe("none");
    expect(video.status().phase).toBe("connecting");
    pc.media();
    expect(video.mode()).toBe("rtc");
    await expect(video.ready()).resolves.toBeUndefined();
    expect(video.settings()).toEqual({ maxHz: 30, maxResolution: [1920, 1080] });
    // A second handle on the same camera owns its own route and peer.
    h.video();
    expect(h.subs()).toHaveLength(2);
    expect(h.subs()[1].id).not.toBe(h.subs()[0].id);
    // A stray signal for another id or generation is ignored.
    h.client.onMessage({ t: "topic_rtc_close", ...ref, gen: 2, reason: "ice_failed" });
    h.client.onMessage({ t: "topic_rtc_close", id: "nope", route: ref.route, gen: 1, reason: "x" });
    expect(video.status().phase).toBe("ready");
  });

  it("adapts from receiver statistics and keeps the track on an in-place acknowledgement", async () => {
    vi.useFakeTimers();
    const h = setup();
    const video = h.video({ maxResolution: [1280, 720] });
    h.sources();
    const ref = h.ready("@t:v1", 1, undefined, {
      maxHz: 30,
      maxResolution: [1280, 720],
      maxBitrate: 2500000,
    });
    await flush();
    h.client.onMessage({ t: "topic_rtc_answer", ...ref, sdp: "answer" });
    await flush();
    const pc = h.pcs[0];
    const track = pc.track();
    pc.media();
    expect(video.stats()).toBeNull();
    for (let i = 0; i < 4; i++) {
      // Firefox-style stats: no freeze counters or bitrate estimate.
      pc.stats = [{
        type: "inbound-rtp",
        kind: "video",
        packetsReceived: 97 * i,
        packetsLost: 3 * i,
        framesDecoded: 15 * i,
        bytesReceived: 1000 * i,
      }];
      await vi.advanceTimersByTimeAsync(500);
    }
    const update = h.sent.find((m): m is TopicUpdateMsg => m.t === "topic_update")!;
    expect(update.options).toEqual({ maxBitrate: 1500000 });
    const changed = vi.fn();
    video.onSettings(changed);
    h.client.onMessage({
      t: "topic_applied",
      id: ref.id,
      revision: update.revision,
      settings: { maxHz: 30, maxResolution: [1280, 720], maxBitrate: 1500000 },
    });
    await flush();
    expect(changed).toHaveBeenCalledExactlyOnceWith(video.settings());
    expect(video.stats()).toMatchObject({
      adapted: true,
      rung: { index: 1 },
      video: { lossFraction: 0.03, freezeMs: 0, bitrate: 16000, fps: 30 },
    });
    expect(video.track()).toBe(track);
    expect(h.tracks).toEqual([track]);
    expect(h.offers()).toHaveLength(1);
    expect(pc.closed).toBe(0);
  });

  it("backs off and offers again after an answer timeout, ICE failure or provider close, and stays failed after the third", async () => {
    vi.useFakeTimers();
    const h = setup();
    const video = h.video();
    h.sources();
    const ref = h.ready();
    await flush();
    expect(h.offers()).toHaveLength(1);
    await vi.advanceTimersByTimeAsync(RTC_ANSWER_TIMEOUT_MS);
    expect(video.status()).toMatchObject({
      phase: "interrupted",
      error: { code: "rtc_failed", message: "rtc_answer_timeout" },
    });
    expect(h.pcs[0].closed).toBe(1);
    await vi.advanceTimersByTimeAsync(RTC_RETRY_MS[0]);
    expect(h.offers()).toHaveLength(2);
    h.client.onMessage({ t: "topic_rtc_answer", ...ref, sdp: "a" });
    await flush();
    h.pcs[1].connection("failed");
    expect(video.status()).toMatchObject({
      phase: "interrupted",
      error: { code: "rtc_failed", message: "no_direct_path" },
    });
    await vi.advanceTimersByTimeAsync(RTC_RETRY_MS[1]);
    expect(h.offers()).toHaveLength(3);
    h.client.onMessage({ t: "topic_rtc_close", ...ref, reason: "ice_failed" });
    expect(video.status()).toMatchObject({
      phase: "rejected",
      error: { code: "rtc_failed", message: "ice_failed" },
    });
    expect(video.mode()).toBe("none");
    await expect(video.ready()).rejects.toMatchObject({ code: "rtc_failed" });
    await vi.advanceTimersByTimeAsync(30_000);
    expect(h.offers()).toHaveLength(3);
    expect(h.tracks).toEqual([]);
  });

  it("keeps the failure budget until media plays, so failures after a track reach the fallback", async () => {
    vi.useFakeTimers();
    const h = setup();
    const video = h.video({ fallback: "jpeg" });
    h.sources();
    const ref = h.ready();
    for (let attempt = 0; attempt < RTC_MAX_FAILURES; attempt++) {
      await flush();
      h.client.onMessage({ t: "topic_rtc_answer", ...ref, sdp: "a" });
      await flush();
      const pc = h.pcs[attempt];
      pc.track(); // browsers fire this on the answer, before ICE connects
      expect(video.mode()).toBe("none");
      expect(video.status().phase).not.toBe("ready");
      pc.connection("failed");
      if (attempt < RTC_RETRY_MS.length) await vi.advanceTimersByTimeAsync(RTC_RETRY_MS[attempt]);
    }
    expect(h.offers()).toHaveLength(RTC_MAX_FAILURES);
    expect(video.mode()).toBe("jpeg");
    expect(h.subs().at(-1)).toMatchObject({ profile: VIDEO_FALLBACK_PROFILE });
    expect(video.status()).toMatchObject({
      phase: "connecting",
      error: { code: "rtc_failed", message: "no_direct_path" },
    });
  });

  it("fails a peer that never connects after its answer, waits on an idle camera, and resets the budget on media", async () => {
    vi.useFakeTimers();
    const h = setup();
    const video = h.video();
    h.sources();
    const ref = h.ready();
    await flush();
    h.client.onMessage({ t: "topic_rtc_answer", ...ref, sdp: "a" });
    await flush();
    await vi.advanceTimersByTimeAsync(RTC_CONNECT_TIMEOUT_MS - 1);
    expect(video.status().phase).toBe("connecting");
    await vi.advanceTimersByTimeAsync(1);
    expect(video.status()).toMatchObject({
      phase: "interrupted",
      error: { code: "rtc_failed", message: "no_direct_path" },
    });
    expect(h.pcs[0].closed).toBe(1);
    await vi.advanceTimersByTimeAsync(RTC_RETRY_MS[0]);
    expect(h.offers()).toHaveLength(2);
    // Connected with nothing to play is a quiet camera, not a failure.
    h.client.onMessage({ t: "topic_rtc_answer", ...ref, sdp: "b" });
    await flush();
    const pc = h.pcs[1];
    const track = pc.track();
    pc.connection("connected");
    await vi.advanceTimersByTimeAsync(10 * RTC_CONNECT_TIMEOUT_MS);
    expect(pc.closed).toBe(0);
    expect(h.offers()).toHaveLength(2);
    expect(video.mode()).toBe("none");
    expect(video.track()).toBe(track);
    pc.media();
    expect(video.mode()).toBe("rtc");
    expect(video.status().phase).toBe("ready");
    // A peer that played starts over: the next failure waits the first delay again.
    pc.connection("failed");
    expect(video.status().phase).toBe("interrupted");
    await vi.advanceTimersByTimeAsync(RTC_RETRY_MS[0]);
    expect(h.offers()).toHaveLength(3);
  });

  it("does not offer on the fallback route when the rejection lands during a backoff", async () => {
    vi.useFakeTimers();
    const h = setup();
    const video = h.video({ fallback: "jpeg" });
    h.sources();
    const ref = h.ready();
    await flush();
    await vi.advanceTimersByTimeAsync(RTC_ANSWER_TIMEOUT_MS);
    expect(video.status().phase).toBe("interrupted");
    h.client.onMessage({
      t: "topic_error",
      id: ref.id,
      code: "unsupported_profile",
      message: "unsupported profile",
      outcome: "rejected",
    });
    expect(video.mode()).toBe("jpeg");
    h.ready("@t:v3");
    await vi.advanceTimersByTimeAsync(RTC_RETRY_MS[0]);
    expect(h.offers()).toHaveLength(1);
  });

  it("ends the track on disconnect and offers again on the new route, replacing it", async () => {
    const h = setup();
    const video = h.video();
    h.sources();
    const ref = h.ready();
    await flush();
    h.client.onMessage({ t: "topic_rtc_answer", ...ref, sdp: "a" });
    const first = h.pcs[0].track();
    h.pcs[0].media();
    expect(video.mode()).toBe("rtc");
    h.client.disconnected();
    expect(h.tracks).toEqual([first, null]);
    expect(video.mode()).toBe("none");
    expect(video.status().phase).toBe("interrupted");
    expect(h.pcs[0].closed).toBe(1);
    h.connect();
    h.sources(2);
    const again = h.ready("@t:v9", 2);
    await flush();
    expect(h.offers().at(-1)).toMatchObject({ ...again, sdp: h.pcs[1].offerSdp });
    h.client.onMessage({ t: "topic_rtc_answer", ...again, sdp: "b" });
    const second = h.pcs[1].track();
    h.pcs[1].media();
    expect(h.tracks).toEqual([first, null, second]);
    await expect(video.ready()).resolves.toBeUndefined();
    expect(video.track()).toBe(second);
  });

  it("offers again at once when the relay moves the peer to another encoder, and update() rides the subscription", async () => {
    const h = setup();
    const video = h.video();
    h.sources();
    const ref = h.ready();
    await flush();
    h.client.onMessage({ t: "topic_rtc_answer", ...ref, sdp: "a" });
    const first = h.pcs[0].track();
    await expect(video.update({ profile: "raw" })).rejects.toMatchObject({
      code: "unsupported_option",
    });
    const applied = video.update({ maxResolution: [640, 360] });
    expect(h.sent.at(-1)).toEqual({
      t: "topic_update",
      id: ref.id,
      revision: 1,
      options: { maxResolution: [640, 360] },
    });
    h.client.onMessage({ t: "topic_rtc_close", ...ref, reason: "settings_changed" });
    h.client.onMessage({
      t: "topic_applied",
      id: ref.id,
      revision: 1,
      settings: { maxHz: 30, maxResolution: [640, 360] },
    });
    await expect(applied).resolves.toEqual({ maxHz: 30, maxResolution: [640, 360] });
    await flush();
    expect(h.tracks).toEqual([first, null]);
    expect(h.offers()).toHaveLength(2);
    expect(video.status().phase).toBe("connecting");
    h.client.onMessage({ t: "topic_rtc_answer", ...ref, sdp: "b" });
    const second = h.pcs[1].track();
    h.pcs[1].media();
    expect(h.tracks).toEqual([first, null, second]);
    expect(video.mode()).toBe("rtc");
    expect(video.settings()).toEqual({ maxHz: 30, maxResolution: [640, 360] });
  });

  it("reports rtc_unavailable for a source without the profile, and falls back to JPEG only when asked", async () => {
    // A provider without video advertises neither the profile nor its settings.
    const { [VIDEO_PROFILE]: _video, ...jpegOnly } = PROFILE_OPTIONS;
    const h = setup(["raw", VIDEO_FALLBACK_PROFILE], jpegOnly);
    const plain = h.video();
    h.sources();
    // Refused before any topic_sub, as the relay would refuse it.
    expect(h.subs()).toEqual([]);
    expect(plain.status()).toMatchObject({ phase: "rejected", error: { code: "rtc_unavailable" } });
    expect(plain.mode()).toBe("none");
    await expect(plain.ready()).rejects.toMatchObject({ code: "rtc_unavailable" });
    expect(h.offers()).toEqual([]);

    const samples: TopicSample[] = [];
    // A picture size must not fail the settings check first: the missing
    // profile has no ranges, and that reason would not reach the fallback.
    const fallback = h.video({
      maxResolution: [640, 360],
      fallback: "jpeg",
      onSample: (sample) => samples.push(sample),
    });
    expect(fallback.mode()).toBe("jpeg");
    expect(h.subs()).toHaveLength(1);
    expect(h.subs()[0]).toMatchObject({
      profile: VIDEO_FALLBACK_PROFILE,
      delivery: "latest",
      settings: { maxResolution: [640, 360] },
    });
    expect(fallback.status()).toMatchObject({
      phase: "connecting",
      error: { code: "rtc_unavailable" },
    });
    h.client.onMessage({
      t: "topic_ready",
      id: h.subs()[0].id,
      source: "workspace",
      route: "@t:v3",
      gen: 1,
      settings: { maxHz: 30, maxResolution: [640, 360], quality: 75 },
    });
    await expect(fallback.ready()).resolves.toBeUndefined();
    expect(fallback.status()).toMatchObject({ phase: "ready", error: { code: "rtc_unavailable" } });
    expect(fallback.settings()).toEqual({ maxHz: 30, maxResolution: [640, 360], quality: 75 });
    h.client.ingest({
      header: {
        ch: "@t:v3",
        seq: 1,
        ts: 3,
        delivery: "latest",
        meta: { key: "sensor/cam", gen: 1, encoding: "image/jpeg", ingressTs: 2 },
      },
      payload: new Uint8Array([255, 216]),
    });
    expect(samples.map((s) => s.encoding)).toEqual(["image/jpeg"]);
    expect(h.tracks).toEqual([]);
  });

  it("falls back when the camera has no variant left for this handle's settings", () => {
    const h = setup();
    const video = h.video({ fallback: "jpeg", maxHz: 7 });
    h.sources();
    h.client.onMessage({
      t: "topic_error",
      id: h.subs()[0].id,
      code: "variant_limit",
      message: "variant limit",
      outcome: "rejected",
    });
    expect(video.mode()).toBe("jpeg");
    expect(video.status().error).toMatchObject({
      code: "rtc_unavailable",
      message: "variant_limit",
    });
    expect(h.subs().at(-1)).toMatchObject({ profile: VIDEO_FALLBACK_PROFILE, maxHz: 7 });
  });

  it("falls back with the settings in force, leaving out a rejected change", async () => {
    const h = setup();
    const video = h.video({ fallback: "jpeg", maxHz: 30 });
    h.sources();
    const ref = h.ready();
    await flush();
    const applied = video.update({ maxHz: 5, maxResolution: [640, 360] });
    h.client.onMessage({
      t: "topic_applied",
      id: ref.id,
      revision: 1,
      settings: { maxHz: 5, maxResolution: [640, 360] },
    });
    await applied;
    const refused = video.update({ maxHz: 7 });
    h.client.onMessage({
      t: "topic_error",
      id: ref.id,
      revision: 2,
      code: "variant_limit",
      message: "variant limit",
      outcome: "rejected",
    });
    await expect(refused).rejects.toMatchObject({ code: "variant_limit" });
    h.client.onMessage({ t: "topic_rtc_close", ...ref, reason: "rtc_limit" });
    expect(video.mode()).toBe("jpeg");
    expect(h.subs().at(-1)).toMatchObject({
      source: "workspace",
      profile: VIDEO_FALLBACK_PROFILE,
      delivery: "latest",
      maxHz: 5,
      settings: { maxResolution: [640, 360] },
    });
  });

  it("a source change ends the old peer, and the fallback keeps the new source", async () => {
    const h = setup();
    h.sources();
    const info = {
      gen: 1,
      read: true,
      write: false,
      profiles: [VIDEO_PROFILE, VIDEO_FALLBACK_PROFILE],
    };
    h.client.onMessage({
      t: "topic_sources",
      v: 1,
      sources: [
        { source: "a", profileOptions: PROFILE_OPTIONS as never, ...info },
        { source: "b", profileOptions: PROFILE_OPTIONS as never, ...info },
      ],
    });
    const video = h.video({ fallback: "jpeg", source: "a", maxHz: 30 });
    expect(h.subs().at(-1)).toMatchObject({ source: "a", maxHz: 30 });
    const ready = (route: string) => {
      const { id } = h.subs().at(-1)!;
      h.client.onMessage({
        t: "topic_ready",
        id,
        source: h.subs().at(-1)!.source,
        route,
        gen: 1,
        settings: { maxHz: 5 },
      });
      return { id, route, gen: 1 };
    };
    const first = ready("@t:v1");
    await flush();
    h.client.onMessage({ t: "topic_rtc_answer", ...first, sdp: "a" });
    h.pcs[0].track();
    h.pcs[0].media();
    const moved = video.update({ source: "b", maxHz: 5, maxResolution: [640, 360] });
    expect(h.pcs[0].closed).toBe(1);
    expect(video.track()).toBeNull();
    expect(video.mode()).toBe("none");
    expect(video.status().phase).toBe("connecting");
    expect(h.subs().at(-1)).toMatchObject({
      source: "b",
      profile: VIDEO_PROFILE,
      maxHz: 5,
      settings: { maxResolution: [640, 360] },
    });
    const second = ready("@t:v2");
    await moved;
    await flush();
    expect(h.offers().at(-1)).toMatchObject(second);
    h.client.onMessage({ t: "topic_rtc_close", ...second, reason: "rtc_limit" });
    expect(h.subs().at(-1)).toMatchObject({
      source: "b",
      profile: VIDEO_FALLBACK_PROFILE,
      maxHz: 5,
      settings: { maxResolution: [640, 360] },
    });
  });

  it("a fallback keeps the source it was picked on when another source joins meanwhile", async () => {
    const client = new TopicClient(undefined, () => new FakePeerConnection());
    owned.push(client);
    const sent: Msg[] = [];
    client.connected((m) => sent.push(m), vi.fn());
    const announce = (names: string[]) =>
      client.onMessage({
        t: "topic_sources",
        v: 1,
        sources: names.map((source) => ({
          source,
          gen: 1,
          read: true,
          write: false,
          profiles: ["raw", VIDEO_PROFILE, VIDEO_FALLBACK_PROFILE],
          profileOptions: PROFILE_OPTIONS as never,
        })),
      });
    announce(["cam-a"]);
    const video = client.video("sensor/cam", { fallback: "jpeg" }, vi.fn());
    owned.push(video);
    const subs = () => sent.filter((m): m is TopicSubMsg => m.t === "topic_sub");
    expect(subs()[0].source).toBe("cam-a");
    client.onMessage({
      t: "topic_ready",
      id: subs()[0].id,
      source: "cam-a",
      route: "@t:v1",
      gen: 1,
      settings: { maxHz: 30, maxResolution: [1920, 1080] },
    });
    await flush();
    announce(["cam-a", "cam-b"]);
    client.onMessage({
      t: "topic_rtc_close",
      id: subs()[0].id,
      route: "@t:v1",
      gen: 1,
      reason: "rtc_limit",
    });
    expect(video.mode()).toBe("jpeg");
    expect(subs()).toHaveLength(2);
    expect(subs()[1]).toMatchObject({ source: "cam-a", profile: VIDEO_FALLBACK_PROFILE });
    expect(video.status()).toMatchObject({
      phase: "connecting",
      error: { code: "rtc_unavailable" },
    });
  });

  it("falls back after rtc_limit from the provider and closes cleanly", async () => {
    const h = setup();
    const video = h.video({ fallback: "jpeg" });
    h.sources();
    const ref = h.ready();
    await flush();
    h.client.onMessage({ t: "topic_rtc_close", ...ref, reason: "rtc_limit" });
    expect(h.pcs[0].closed).toBe(1);
    expect(video.mode()).toBe("jpeg");
    expect(video.status()).toMatchObject({
      phase: "connecting",
      error: { code: "rtc_unavailable", message: "rtc_limit" },
    });
    expect(h.sent.filter((m) => m.t === "topic_unsub")).toEqual([{ t: "topic_unsub", id: ref.id }]);
    expect(h.subs().at(-1)).toMatchObject({ profile: VIDEO_FALLBACK_PROFILE });
    video.close();
    expect(video.status().phase).toBe("closed");
    expect(h.sent.filter((m) => m.t === "topic_unsub")).toHaveLength(2);

    const live = h.video();
    const second = h.ready("@t:v7");
    await flush();
    h.client.onMessage({ t: "topic_rtc_answer", ...second, sdp: "a" });
    const track = h.pcs[1].track();
    live.close();
    expect(h.tracks).toEqual([track, null]);
    expect(h.pcs[1].closed).toBe(1);
    expect(h.sent.at(-1)).toEqual({ t: "topic_unsub", id: second.id });
    // Nothing after close reaches the handle.
    h.client.onMessage({ t: "topic_rtc_close", ...second, reason: "ice_failed" });
    expect(live.status().phase).toBe("closed");
  });
});

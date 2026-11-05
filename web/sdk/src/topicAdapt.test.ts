import { afterEach, describe, expect, it, vi } from "vitest";
import {
  type Msg,
  normalizeTopicSettings,
  type OptionRanges,
  TOPIC_SETTING_NAMES,
  type TopicSettings,
  type TopicSubMsg,
  type TopicUpdateMsg,
} from "@dimos/shared";
import {
  adaptationLadder,
  adaptationPolicy,
  type AdaptTarget,
  applyCap,
  TopicAdaptation,
} from "./topicAdapt.ts";
import { TopicClient, type TopicOptions } from "./topics.ts";

const JPEG = "image.jpeg.balanced.v1";
const CLOUD = "cloud.q16.balanced.v1";
const VIDEO = "video.h264.v1";
const ranges: Record<string, OptionRanges> = {
  raw: {},
  [JPEG]: {
    maxHz: { kind: "number", default: 30 },
    quality: { kind: "int", min: 1, max: 100, default: 75 },
    maxResolution: { kind: "size", min: 16, max: 4096, default: [1280, 720] },
  },
  [CLOUD]: {
    maxHz: { kind: "number", default: 20 },
    keepEvery: { kind: "int", min: 1, max: 64, default: 1 },
    fields: { kind: "enum", values: ["intensity", "color"], default: ["intensity", "color"] },
  },
  [VIDEO]: {
    maxHz: { kind: "number", default: 30 },
    maxResolution: { kind: "size", min: 16, max: 4096, default: [1920, 1080] },
    maxBitrate: { kind: "int", min: 100000, max: 4500000, default: 4500000 },
  },
  "depth.lossless.v1": { maxHz: { kind: "number", default: 30 } },
};
const owned: { close(): void }[] = [];
afterEach(() => {
  for (const item of owned.splice(0)) item.close();
  vi.useRealTimers();
  vi.unstubAllGlobals();
});
function controller() {
  vi.useFakeTimers();
  vi.setSystemTime(100000);
  const adapt = new TopicAdaptation();
  owned.push(adapt);
  const changes: { profile: string; at: number; cap: Partial<TopicSettings> }[] = [];
  function target(profile = JPEG, options: TopicOptions = {}) {
    const ceiling = normalizeTopicSettings(
      ranges[profile],
      Object.fromEntries(
        Object.entries(options).filter(([key]) =>
          (TOPIC_SETTING_NAMES as readonly string[]).includes(key)
        ),
      ),
    ).settings!;
    let settings = ceiling;
    const policy = adaptationPolicy(options, profile, options.delivery ?? "latest");
    const value: AdaptTarget = {
      profile,
      policy,
      ready: () => true,
      ceiling: () => ceiling,
      settings: () => settings,
      receiver: () => null,
      apply: vi.fn((cap) => {
        changes.push({ profile, at: Date.now(), cap });
        settings = applyCap(ceiling, cap);
        return Promise.resolve(settings);
      }),
    };
    adapt.add(value);
    return value;
  }
  const tick = (ms = 500) => vi.advanceTimersByTimeAsync(ms);
  const sample = (value: AdaptTarget, age = 0, bytes = 100) =>
    adapt.sample(value, (Date.now() - age) / 1000, bytes);
  return { adapt, changes, target, tick, sample };
}

describe("adaptation decisions", () => {
  it("confirms age excess for two ticks, respects cooldown, and ignores an idle low-rate source", async () => {
    const h = controller();
    const camera = h.target();
    h.sample(camera, 5000);
    await h.tick(); // absorbs offset and base latency
    h.sample(camera, 5200);
    await h.tick();
    expect(h.changes).toHaveLength(0);
    h.sample(camera, 5250);
    await h.tick();
    expect(h.changes).toHaveLength(0);
    h.sample(camera, 5250);
    await h.tick();
    expect(h.changes).toHaveLength(1);
    for (let i = 0; i < 3; i++) {
      h.sample(camera, 5250);
      await h.tick();
    }
    expect(h.changes).toHaveLength(1);
    h.sample(camera, 5250);
    await h.tick();
    expect(h.changes).toHaveLength(2);
    await h.tick(3000);
    expect(h.changes).toHaveLength(2); // silence is not a stall
    expect(h.adapt.stats(camera)).toMatchObject({
      deliveredHz: 0,
      ageMs: { p50: 0, p95: 0 },
      adapted: true,
    });
  });

  it("uses abort deltas for total stalls, sheds lower priority first, then the larger byte rate", async () => {
    const h = controller();
    const camera = h.target(JPEG, { priority: 3 });
    const cloud = h.target(CLOUD, { priority: 5 });
    h.adapt.pong(1);
    await h.tick(1000);
    expect(h.changes.map((c) => c.profile)).toEqual([CLOUD]);
    expect(h.adapt.stats(camera)?.aborted).toBe(1);
    h.adapt.pong(1);
    await h.tick(500);
    expect(h.changes).toHaveLength(1);
    h.adapt.pong(2);
    await h.tick(1000);
    expect(h.changes[1].profile).toBe(JPEG); // cloud cooling
    h.adapt.setEnabled(false);
    await h.tick(4000);
    expect(h.adapt.stats(cloud)).toBeNull();
    expect(camera.settings()).toEqual(camera.ceiling());
    expect(cloud.settings()).toEqual(cloud.ceiling());
  });

  it("steps the highest priority degraded route up after each ten clean seconds", async () => {
    const h = controller();
    h.target(JPEG, { priority: 3 });
    h.target(CLOUD, { priority: 5 });
    h.adapt.pong(1);
    await h.tick(1000);
    h.adapt.pong(2);
    await h.tick(1000);
    expect(h.changes.map((c) => c.profile)).toEqual([CLOUD, JPEG]);
    await h.tick(9500);
    expect(h.changes).toHaveLength(2);
    await h.tick();
    expect(h.changes.at(-1)).toMatchObject({ profile: JPEG, cap: {} });
    await h.tick(10000);
    expect(h.changes.at(-1)).toMatchObject({ profile: CLOUD, cap: {} });
  });

  it("breaks ties by delivered bytes and never changes opted-out or reliable handles", async () => {
    const h = controller();
    const small = h.target();
    const large = h.target();
    const off = h.target(CLOUD, { adapt: false });
    const reliable = h.target(CLOUD, { delivery: "reliable", adapt: true });
    const raw = h.target("raw");
    h.sample(small, 0, 10);
    h.sample(large, 0, 1000);
    h.adapt.pong(10);
    await h.tick(1000);
    expect(large.apply).toHaveBeenCalledOnce();
    expect(small.apply).not.toHaveBeenCalled();
    for (const target of [off, reliable, raw]) expect(target.apply).not.toHaveBeenCalled();
  });

  it("retries variant_limit after cooldown without claiming the rejected cap took effect", async () => {
    const h = controller();
    const camera = h.target();
    vi.mocked(camera.apply).mockRejectedValueOnce({ code: "variant_limit" });
    h.adapt.pong(1);
    await h.tick(1000);
    await h.tick();
    expect(h.adapt.stats(camera)).toMatchObject({
      adapted: false,
      rung: null,
      rejected: "variant_limit",
    });
    await h.tick(1000);
    expect(camera.apply).toHaveBeenCalledTimes(1);
    await h.tick(500);
    expect(camera.apply).toHaveBeenCalledTimes(2);
    await h.tick();
    expect(h.adapt.stats(camera)).toMatchObject({ adapted: true, rung: { index: 1 } });
    expect(h.adapt.stats(camera)?.rejected).toBeUndefined();
  });

  it("pauses and clears measurement windows while hidden", async () => {
    const doc = Object.assign(new EventTarget(), { hidden: false });
    vi.stubGlobal("document", doc);
    const h = controller();
    const camera = h.target();
    h.sample(camera);
    await h.tick();
    h.sample(camera, 300);
    await h.tick();
    doc.hidden = true;
    doc.dispatchEvent(new Event("visibilitychange"));
    h.adapt.pong(5);
    await h.tick(20000);
    expect(h.changes).toHaveLength(0);
    doc.hidden = false;
    doc.dispatchEvent(new Event("visibilitychange"));
    h.sample(camera, 300);
    await h.tick();
    expect(h.changes).toHaveLength(0);
    expect(h.adapt.stats(camera)?.ageMs.p95).toBe(0);
  });

  it.each([
    { packetsLost: 3, packetsReceived: 97, framesDecoded: 15, bytesReceived: 1000 },
    {
      packetsLost: 0,
      packetsReceived: 100,
      framesDecoded: 15,
      bytesReceived: 1000,
      totalFreezesDuration: 0.2,
    },
    { packetsLost: 0, packetsReceived: 100, framesDecoded: 0, bytesReceived: 1000 },
    {
      packetsLost: 0,
      packetsReceived: 100,
      framesDecoded: 15,
      bytesReceived: 1000,
      estimate: 1000000,
    },
  ])("uses receiver deltas for video congestion: %j", async (delta) => {
    const h = controller();
    const camera = h.target(VIDEO);
    let n = 0;
    const receiver = {
      getStats: vi.fn(() => {
        const i = n++;
        return Promise.resolve(
          new Map([
            ["in", {
              type: "inbound-rtp",
              kind: "video",
              packetsLost: i * delta.packetsLost,
              packetsReceived: i * delta.packetsReceived,
              framesDecoded: i * delta.framesDecoded,
              bytesReceived: i * delta.bytesReceived,
              totalFreezesDuration: i *
                ("totalFreezesDuration" in delta ? delta.totalFreezesDuration! : 0),
            }],
            ["pair", {
              type: "candidate-pair",
              nominated: true,
              state: "succeeded",
              availableIncomingBitrate: "estimate" in delta ? delta.estimate : undefined,
            }],
          ]),
        );
      }),
    } as unknown as RTCRtpReceiver;
    camera.receiver = () => receiver;
    await h.tick(2000);
    expect(h.changes).toHaveLength(1);
    await h.tick(3500);
    expect(h.changes).toHaveLength(1);
    await h.tick();
    expect(h.changes).toHaveLength(2);
    expect(h.adapt.stats(camera)?.video?.bitrate).toBe(16000);
  });

  it("does not treat exact video thresholds as congestion", async () => {
    const h = controller();
    const camera = h.target(VIDEO);
    let n = 0;
    camera.receiver = () => receiver;
    const receiver = {
      getStats: () => {
        const i = n++;
        return Promise.resolve(
          new Map([
            ["in", {
              type: "inbound-rtp",
              kind: "video",
              packetsLost: i * 2,
              packetsReceived: i * 98,
              framesDecoded: i * 15,
              bytesReceived: i * 1000,
              totalFreezesDuration: i * 0.1,
            }],
            ["pair", {
              type: "candidate-pair",
              selected: true,
              state: "succeeded",
              availableIncomingBitrate: 3600000,
            }],
          ]),
        );
      },
    } as unknown as RTCRtpReceiver;
    await h.tick(4000);
    expect(h.changes).toEqual([]);
    expect(h.adapt.stats(camera)?.video).toMatchObject({ lossFraction: 0.02, fps: 30 });
  });

  it("leaves a healthy loopback screen unchanged for ten minutes", async () => {
    const h = controller();
    const camera = h.target();
    h.target(CLOUD);
    for (let i = 0; i < 1200; i++) {
      h.sample(camera, 50);
      h.adapt.pong(0);
      await h.tick();
    }
    expect(h.changes).toEqual([]);
  });

  it("adapts an explicitly enabled raw route by rate alone", async () => {
    const h = controller();
    const raw = h.target("raw", { adapt: true, maxHz: 12 });
    h.adapt.pong(1);
    await h.tick(1000);
    expect(raw.settings()).toEqual({ maxHz: 6 });
    expect(h.changes).toEqual([{ profile: "raw", at: 101000, cap: { maxHz: 6 } }]);
  });
});

describe("adaptation ladders", () => {
  it.each([JPEG, CLOUD, VIDEO, "depth.lossless.v1", "raw"])(
    "bounds every %s rung by the request and the floors",
    (profile) => {
      const ceiling = normalizeTopicSettings(ranges[profile], { maxHz: 30 }).settings!;
      for (const keep of ["rate", "quality"] as const) {
        const ladder = adaptationLadder(profile, ceiling, keep);
        for (const rung of ladder) {
          const settings = applyCap(ceiling, rung.cap);
          expect(settings.maxHz).toBeGreaterThanOrEqual(profile === VIDEO ? 5 : 1);
          expect(settings.maxHz).toBeLessThanOrEqual(ceiling.maxHz);
          if (settings.quality) expect(settings.quality).toBeGreaterThanOrEqual(30);
          if (settings.maxResolution) {
            expect(settings.maxResolution[0]).toBeGreaterThanOrEqual(
              profile === VIDEO ? 640 : 160,
            );
          }
          if (settings.keepEvery) expect(settings.keepEvery).toBeLessThanOrEqual(64);
          if (settings.maxBitrate) expect(settings.maxBitrate).toBeGreaterThanOrEqual(200000);
        }
        expect(applyCap(ceiling, ladder.at(-1)!.cap).maxHz).toBe(profile === VIDEO ? 5 : 1);
        if (keep === "quality") expect(ladder[1].name).toMatch(/^rate:/);
      }
    },
  );
  it("uses requested fractions and skips duplicate rungs below the floors", () => {
    const ladder = adaptationLadder(
      JPEG,
      { maxHz: 10, quality: 60, maxResolution: [640, 360] },
      "rate",
    );
    expect(ladder.map((r) => r.cap)).toEqual([
      {},
      { quality: 45 },
      { quality: 30 },
      { quality: 30, maxResolution: [320, 180] },
      { quality: 30, maxResolution: [160, 90] },
      { quality: 30, maxResolution: [160, 90], maxHz: 5 },
      { quality: 30, maxResolution: [160, 90], maxHz: 2.5 },
      { quality: 30, maxResolution: [160, 90], maxHz: 1 },
    ]);
    expect(adaptationLadder(JPEG, { maxHz: 0.5, quality: 20, maxResolution: [80, 64] }, "rate"))
      .toHaveLength(1);
  });
  it("never raises the video bitrate further down the ladder", () => {
    const ceiling: TopicSettings = { maxHz: 30, maxResolution: [1920, 1080], maxBitrate: 500000 };
    const ladder = adaptationLadder(VIDEO, ceiling, "rate");
    expect(ladder.map((r) => r.name)).toEqual([
      "requested",
      "bitrate:0.6",
      "bitrate:0.35",
      "resolution:720p",
      "resolution:360p",
      "rate:15",
      "rate:10",
      "rate:5",
    ]);
    expect(ladder.map((r) => applyCap(ceiling, r.cap).maxBitrate)).toEqual([
      500000,
      300000,
      200000,
      200000,
      200000,
      200000,
      200000,
      200000,
    ]);
  });
});

function clientSetup() {
  vi.useFakeTimers();
  vi.setSystemTime(100000);
  const client = new TopicClient("test");
  owned.push(client);
  const sent: Msg[] = [];
  const requested = new Map<string, Record<string, unknown>>();
  const profiles = new Map<string, string>();
  const ready = (sub: TopicSubMsg) => {
    profiles.set(sub.id, sub.profile);
    requested.set(sub.id, { maxHz: sub.maxHz, ...sub.settings });
    client.onMessage({
      t: "topic_ready",
      id: sub.id,
      source: "test",
      gen: 1,
      route: `@t:${sub.id}`,
      settings: normalizeTopicSettings(ranges[sub.profile], requested.get(sub.id)!).settings!,
    });
  };
  client.connected((msg) => {
    sent.push(msg);
    if (msg.t === "topic_sub") queueMicrotask(() => ready(msg));
    if (msg.t === "topic_update") {
      queueMicrotask(() => {
        const request = requested.get(msg.id)!;
        for (const [k, v] of Object.entries(msg.options)) {
          if (v === null) delete request[k];
          else request[k] = v;
        }
        client.onMessage({
          t: "topic_applied",
          id: msg.id,
          revision: msg.revision,
          settings: normalizeTopicSettings(ranges[profiles.get(msg.id)!], request).settings!,
        });
      });
    }
  }, vi.fn());
  client.onMessage({
    t: "topic_sources",
    v: 1,
    sources: [{
      source: "test",
      gen: 1,
      read: true,
      write: false,
      profiles: Object.keys(ranges),
      profileOptions: ranges,
    }],
  });
  const updates = () => sent.filter((m): m is TopicUpdateMsg => m.t === "topic_update");
  return { client, sent, updates, tick: (ms = 500) => vi.advanceTimersByTimeAsync(ms) };
}

describe("acknowledged cap layer", () => {
  it("caps a shared group in place, preserves page requests, notifies settings, and lets a joiner inherit", async () => {
    const h = clientSetup();
    const one = h.client.subscribe("camera", { profile: JPEG, quality: 60 }, vi.fn());
    const two = h.client.subscribe("camera", { profile: JPEG, quality: 60 }, vi.fn());
    const settings = vi.fn();
    one.onSettings(settings);
    await h.tick(0);
    expect(one.stats()).toBeNull();
    h.client.pong(1);
    await h.tick(1000);
    expect(h.updates()).toHaveLength(1);
    expect(one.settings().quality).toBe(45);
    expect(two.settings()).toEqual(one.settings());
    const three = h.client.subscribe("camera", { profile: JPEG, quality: 60 }, vi.fn());
    expect(three.settings().quality).toBe(45);
    expect(h.sent.filter((m) => m.t === "topic_sub")).toHaveLength(1);
    two.close();
    three.close();
    const changed = one.update({ quality: 50, maxHz: 12 });
    await h.tick(0);
    await expect(changed).resolves.toMatchObject({ quality: 45, maxHz: 12 });
    expect(settings).toHaveBeenLastCalledWith(one.settings());
    h.client.setAdaptation(false);
    await h.tick(2000);
    expect(one.settings()).toMatchObject({ quality: 50, maxHz: 12 });
    expect(h.updates().at(-1)?.options.quality).toBe(50);
  });
  it("lifts to null for settings absent from the page and leaves adapt:false subscriptions alone", async () => {
    const h = clientSetup();
    const one = h.client.subscribe("camera", { profile: JPEG }, vi.fn());
    const off = h.client.subscribe("camera", { profile: JPEG, adapt: false }, vi.fn());
    await h.tick(0);
    h.client.pong(1);
    await h.tick(1000);
    expect(one.settings().quality).toBe(56);
    expect(off.settings().quality).toBe(75);
    h.client.setAdaptation(false);
    await h.tick(2000);
    expect(h.updates().at(-1)?.options.quality).toBeNull();
    expect(one.settings().quality).toBe(75);
    expect(
      h.sent.filter((m) => m.t === "topic_sub").every((m) =>
        !("adapt" in m) && !("priority" in m) && !("keep" in m)
      ),
    ).toBe(true);
  });

  it("preserves and lifts a cap when a page update splits a shared group", async () => {
    const h = clientSetup();
    const one = h.client.subscribe("camera", { profile: JPEG }, vi.fn());
    const two = h.client.subscribe("camera", { profile: JPEG }, vi.fn());
    await h.tick(0);
    h.client.pong(1);
    await h.tick(1000);
    const update = two.update({ maxHz: 12 });
    await h.tick(0);
    await expect(update).resolves.toMatchObject({ quality: 56, maxHz: 12 });
    expect(one.settings()).toMatchObject({ quality: 56, maxHz: 30 });
    h.client.setAdaptation(false);
    await h.tick(2000);
    expect(one.settings()).toMatchObject({ quality: 75, maxHz: 30 });
    expect(two.settings()).toMatchObject({ quality: 75, maxHz: 12 });
    expect(vi.getTimerCount()).toBe(0);
  });
  it("checks a page update against the ranges before the cap hides it", async () => {
    const h = clientSetup();
    const one = h.client.subscribe("camera", { profile: JPEG }, vi.fn());
    await h.tick(0);
    h.client.pong(1);
    await h.tick(1000);
    expect(one.settings().quality).toBe(56);
    await expect(one.update({ quality: 101 })).rejects.toMatchObject({
      code: "option_out_of_range",
    });
    expect(one.settings().quality).toBe(56);
    h.client.setAdaptation(false);
    await h.tick(2000);
    expect(one.settings().quality).toBe(75);
  });
});

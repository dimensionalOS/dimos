import { type TopicSettings, videoBitrate } from "@dimos/shared";

export const ADAPT_TICK_MS = 500;
export const DATA_COOLDOWN_MS = 2000;
export const VIDEO_COOLDOWN_MS = 4000;
export const ADAPT_RECOVERY_MS = 10000;
const STATS_MS = 2000;
const BASELINE_MS = 30000;
const AGE_THRESHOLD_MS = 200;

export interface AdaptOptions {
  adapt?: boolean | { priority?: number; keep?: "rate" | "quality" };
  priority?: number;
  keep?: "rate" | "quality";
}
export interface AdaptPolicy {
  adapt: boolean;
  priority: number;
  keep: "rate" | "quality";
}
export function adaptationPolicy(
  options: AdaptOptions,
  profile: string,
  delivery: string,
): AdaptPolicy {
  const nested = typeof options.adapt === "object" && options.adapt !== null ? options.adapt : {};
  return {
    adapt: delivery !== "reliable" &&
      (options.adapt !== undefined ? options.adapt !== false : profile !== "raw"),
    priority: options.priority ?? nested.priority ?? 4,
    keep: options.keep ?? nested.keep ?? "rate",
  };
}
export function validAdaptation(options: AdaptOptions): boolean {
  const adapt = options.adapt;
  if (
    adapt !== undefined && typeof adapt !== "boolean" &&
    (typeof adapt !== "object" || adapt === null || Array.isArray(adapt) ||
      Object.keys(adapt).some((k) => k !== "priority" && k !== "keep"))
  ) return false;
  const nested = typeof adapt === "object" ? adapt : {};
  return [options.priority, nested.priority].every((p) =>
    p === undefined || Number.isInteger(p) && p >= 1 && p <= 7
  ) &&
    [options.keep, nested.keep].every((k) => k === undefined || k === "rate" || k === "quality");
}
export interface TopicStats {
  deliveredHz: number;
  /** Arrival age above the rolling 30-second minimum, independent of clock offset. */
  ageMs: { p50: number; p95: number };
  aborted: number;
  rung: { name: string; index: number } | null;
  adapted: boolean;
  rejected?: string;
  video?: { lossFraction: number; freezeMs: number; bitrate: number; fps: number };
}
export interface AdaptTarget {
  profile: string;
  policy: AdaptPolicy;
  ready(): boolean;
  ceiling(): TopicSettings;
  settings(): TopicSettings;
  apply(cap: Partial<TopicSettings>): Promise<TopicSettings>;
  receiver(): RTCRtpReceiver | null;
}
interface Rung {
  name: string;
  cap: Partial<TopicSettings>;
}
/** Cumulative caps relative to the page's normalized request. */
export function adaptationLadder(
  profile: string,
  ceiling: TopicSettings,
  keep: "rate" | "quality",
): Rung[] {
  const video = profile === "video.h264.v1";
  const rateFloor = Math.min(ceiling.maxHz, video ? 5 : 1);
  const rates = video ? [15, 10, 5] : [ceiling.maxHz / 2, ceiling.maxHz / 4, 1];
  const detail: Rung[] = [];
  const rate: Rung[] = rates.map((hz) => ({
    name: `rate:${Math.max(rateFloor, Math.min(ceiling.maxHz, hz))}`,
    cap: { maxHz: Math.max(rateFloor, Math.min(ceiling.maxHz, hz)) },
  }));
  if (profile.startsWith("image.jpeg.") && ceiling.quality !== undefined && ceiling.maxResolution) {
    for (const fraction of [0.75, 0.5]) {
      detail.push({
        name: `quality:${fraction}`,
        cap: {
          quality: Math.min(ceiling.quality, Math.max(30, Math.round(ceiling.quality * fraction))),
        },
      });
    }
    const [w, h] = ceiling.maxResolution;
    for (const fraction of [0.5, 0.25]) {
      const scale = Math.min(1, Math.max(160 / w, fraction));
      detail.push({
        name: `resolution:${fraction}`,
        cap: { maxResolution: [Math.round(w * scale), Math.max(16, Math.round(h * scale))] },
      });
    }
  } else if (profile.startsWith("cloud.q16.") && ceiling.keepEvery !== undefined) {
    for (const factor of [2, 4, 8]) {
      detail.push({
        name: `stride:${factor}`,
        cap: { keepEvery: Math.min(64, ceiling.keepEvery * factor) },
      });
    }
    let fields = ceiling.fields ?? [];
    for (const field of ["color", "intensity"]) {
      fields = fields.filter((f) => f !== field);
      detail.push({ name: `without:${field}`, cap: { fields } });
    }
  } else if (video && ceiling.maxResolution && ceiling.maxBitrate !== undefined) {
    for (const fraction of [0.6, 0.35]) {
      detail.push({
        name: `bitrate:${fraction}`,
        cap: {
          maxBitrate: Math.max(
            Math.min(200000, ceiling.maxBitrate),
            Math.round(ceiling.maxBitrate * fraction),
          ),
        },
      });
    }
    for (const box of [[1280, 720], [640, 360]] as [number, number][]) {
      if (box[0] < ceiling.maxResolution[0]) {
        detail.push({
          name: `resolution:${box[1]}p`,
          cap: {
            maxResolution: [box[0], Math.min(box[1], ceiling.maxResolution[1])],
            maxBitrate: Math.min(ceiling.maxBitrate, videoBitrate(box) * 0.35),
          },
        });
      }
    }
  }
  const result: Rung[] = [{ name: "requested", cap: {} }];
  let cap: Partial<TopicSettings> = {};
  let bitrate = ceiling.maxBitrate;
  let previous = JSON.stringify(ceiling);
  for (const rung of keep === "quality" ? [...rate, ...detail] : [...detail, ...rate]) {
    cap = { ...cap, ...rung.cap };
    if (video && bitrate !== undefined) {
      if (rung.cap.maxBitrate !== undefined) bitrate = rung.cap.maxBitrate;
      cap.maxBitrate = Math.min(
        ceiling.maxBitrate!,
        Math.max(200000, Math.round(bitrate * (cap.maxHz ?? ceiling.maxHz) / ceiling.maxHz)),
      );
    }
    const settings = applyCap(ceiling, cap);
    const next = JSON.stringify(settings);
    if (next !== previous) result.push({ name: rung.name, cap: { ...cap } });
    previous = next;
  }
  return result;
}
/** Caps only restrict a request. Missing page values use the profile ceiling. */
export function applyCap<T extends Record<string, unknown> | TopicSettings>(
  request: T,
  cap: Partial<TopicSettings>,
): T {
  const result = { ...request } as Record<string, unknown>;
  for (const [name, value] of Object.entries(cap)) {
    const own = result[name];
    if (value === undefined) continue;
    if (own === undefined || own === null) result[name] = value;
    else if (name === "maxResolution") {
      result[name] = (value as number[]).map((v, i) => Math.min(v, (own as number[])[i]));
    } else if (name === "fields") {
      result[name] = (own as string[]).filter((f) => (value as string[]).includes(f));
    } else {result[name] = name === "keepEvery"
        ? Math.max(own as number, value as number)
        : Math.min(own as number, value as number);}
  }
  return result as T;
}
interface Sample {
  at: number;
  age: number;
  bytes: number;
}
interface VideoSample {
  at: number;
  seconds: number;
  lost: number;
  packets: number;
  freeze: number;
  bytes: number;
  frames: number;
  estimate?: number;
}
interface VideoCounters {
  at: number;
  lost: number;
  packets: number;
  freeze: number;
  bytes: number;
  frames: number;
}
interface State {
  target: AdaptTarget;
  samples: Sample[];
  baseline: { at: number; age: number }[];
  video: VideoSample[];
  receiver: RTCRtpReceiver | null;
  counters: VideoCounters | null;
  polling: boolean;
  epoch: number;
  index: number;
  rung: string;
  cooldown: number;
  pending: boolean;
  rejected?: string;
  retry?: number;
  stats: TopicStats | null;
}
const percentile = (values: number[], q: number): number => {
  if (!values.length) return 0;
  values.sort((a, b) => a - b);
  return values[Math.min(values.length - 1, Math.floor(q * values.length))];
};

/** One controller per TopicClient. No inferred source rate or bandwidth budget. */
export class TopicAdaptation {
  #states = new Map<AdaptTarget, State>();
  #timer: ReturnType<typeof setInterval> | null = null;
  #enabled = true;
  #visible = typeof document === "undefined" || !document.hidden;
  #lastStall = Date.now();
  #stalledTicks = 0;
  #aborted: number | null = null;
  #aborts: { at: number; count: number }[] = [];
  #abortUntil = 0;
  #visibility = () => {
    this.#visible = !document.hidden;
    this.clear();
    this.#schedule();
  };
  constructor() {
    if (typeof document !== "undefined") {
      document.addEventListener("visibilitychange", this.#visibility);
    }
  }
  add(target: AdaptTarget): void {
    this.#states.set(target, {
      target,
      samples: [],
      baseline: [],
      video: [],
      receiver: null,
      counters: null,
      polling: false,
      epoch: 0,
      index: 0,
      rung: "requested",
      cooldown: 0,
      pending: false,
      stats: null,
    });
    this.#schedule();
  }
  remove(target: AdaptTarget): void {
    const state = this.#states.get(target);
    if (state) state.epoch++;
    this.#states.delete(target);
    this.#schedule();
  }
  inherit(target: AdaptTarget, previous: AdaptTarget): void {
    const next = this.#states.get(target), prior = this.#states.get(previous);
    if (next && prior) {
      next.index = prior.index;
      next.rung = prior.rung;
      next.cooldown = prior.cooldown;
    }
  }
  stats(target: AdaptTarget): TopicStats | null {
    const stats = this.#states.get(target)?.stats;
    const state = this.#states.get(target);
    return stats && state
      ? structuredClone({
        ...stats,
        rung: state.index ? { name: state.rung, index: state.index } : null,
        adapted: state.index > 0,
        rejected: state.rejected,
      })
      : null;
  }
  sample(target: AdaptTarget, ingressTs: number, bytes: number): void {
    const state = this.#states.get(target);
    if (!state || !this.#visible) return;
    const at = Date.now();
    const age = at - ingressTs * 1000;
    const bucket = Math.floor(at / ADAPT_TICK_MS) * ADAPT_TICK_MS;
    state.baseline = state.baseline.filter((b) => b.at > at - BASELINE_MS);
    const last = state.baseline.at(-1);
    if (last?.at === bucket) last.age = Math.min(last.age, age);
    else state.baseline.push({ at: bucket, age });
    state.samples.push({ at, age, bytes });
    state.samples = state.samples.filter((s) => s.at > at - STATS_MS).slice(-4096);
  }
  pong(aborted?: number): void {
    if (aborted === undefined) return;
    const delta = Math.max(0, aborted - (this.#aborted ?? 0));
    this.#aborted = aborted;
    if (delta && this.#visible) {
      const at = Date.now();
      this.#aborts.push({ at, count: delta });
      // A pong is one measured congestion window. Retain it for the two
      // confirming ticks, not forever until the next (possibly lost) pong.
      this.#abortUntil = at + 2 * ADAPT_TICK_MS;
    }
  }
  clear(): void {
    this.#lastStall = Date.now();
    this.#stalledTicks = 0;
    this.#aborts = [];
    this.#abortUntil = 0;
    for (const s of this.#states.values()) {
      s.epoch++;
      s.samples = [];
      s.baseline = [];
      s.video = [];
      s.counters = null;
      s.stats = null;
    }
  }
  disconnected(): void {
    this.#aborted = null;
    this.clear();
  }
  setEnabled(enabled: boolean): void {
    this.#enabled = enabled;
    if (!enabled) { for (const state of this.#states.values()) state.retry = undefined; }
    this.clear();
    this.#schedule();
    if (!enabled) this.#lift();
  }
  #schedule(): void {
    const lifting = [...this.#states.values()].some((s) => s.index > 0 || s.pending);
    const running = this.#states.size &&
      (this.#enabled && this.#visible || !this.#enabled && lifting);
    if (running && this.#timer === null) {
      this.#timer = setInterval(() => this.#tick(), ADAPT_TICK_MS);
    } else if (!running && this.#timer !== null) {
      clearInterval(this.#timer);
      this.#timer = null;
    }
  }

  #lift(): void {
    for (const s of this.#states.values()) {
      if (s.index > 0 && !s.pending && s.cooldown <= Date.now() && s.target.ready()) {
        this.#change(s, 0);
      }
    }
    this.#schedule();
  }
  #poll(s: State): void {
    const receiver = s.target.receiver();
    if (receiver !== s.receiver) {
      s.receiver = receiver;
      s.counters = null;
      s.video = [];
      s.epoch++;
    }
    if (!receiver || s.polling) return;
    s.polling = true;
    const epoch = s.epoch;
    receiver.getStats().then((report) => {
      if (epoch !== s.epoch) return;
      const now = Date.now();
      let inbound: RTCInboundRtpStreamStats | undefined;
      let estimate: number | undefined;
      report.forEach((stat) => {
        if (stat.type === "inbound-rtp" && (stat.kind === "video" || stat.mediaType === "video")) {
          inbound = stat;
        }
        if (
          stat.type === "candidate-pair" && (stat.nominated || stat.selected) &&
          stat.state === "succeeded"
        ) estimate = stat.availableIncomingBitrate;
      });
      if (!inbound) return;
      const current: VideoCounters = {
        at: now,
        lost: inbound.packetsLost ?? 0,
        packets: inbound.packetsReceived ?? 0,
        freeze: inbound.totalFreezesDuration ?? 0,
        bytes: inbound.bytesReceived ?? 0,
        frames: inbound.framesDecoded ?? 0,
      };
      const prev = s.counters;
      s.counters = current;
      if (!prev || now <= prev.at || current.bytes < prev.bytes || current.frames < prev.frames) {
        return;
      }
      s.video.push({
        at: now,
        seconds: (now - prev.at) / 1000,
        lost: Math.max(0, current.lost - prev.lost),
        packets: Math.max(0, current.packets - prev.packets),
        freeze: Math.max(0, current.freeze - prev.freeze) * 1000,
        bytes: current.bytes - prev.bytes,
        frames: current.frames - prev.frames,
        estimate,
      });
      s.video = s.video.filter((v) => v.at > now - STATS_MS);
    }).catch(() => {
      s.counters = null;
    }).finally(() => {
      s.polling = false;
    });
  }
  #tick(): void {
    if (!this.#enabled) {
      this.#lift();
      return;
    }
    if (!this.#visible) return;
    const now = Date.now();
    this.#aborts = this.#aborts.filter((a) => a.at > now - STATS_MS);
    const stalled: State[] = [];
    const eligible: State[] = [];
    const rates = new Map<State, number>();
    for (const s of this.#states.values()) {
      if (!s.target.ready()) continue;
      this.#poll(s);
      s.samples = s.samples.filter((v) => v.at > now - STATS_MS);
      s.baseline = s.baseline.filter((v) => v.at > now - BASELINE_MS);
      s.video = s.video.filter((v) => v.at > now - STATS_MS);
      const baseline = Math.min(...s.baseline.map((b) => b.age));
      const ages = s.samples.map((v) => Math.max(0, v.age - baseline));
      const tickAges = s.samples.filter((v) => v.at >= now - ADAPT_TICK_MS).map((v) =>
        Math.max(0, v.age - baseline)
      );
      let bad = percentile(tickAges, 0.5) > AGE_THRESHOLD_MS;
      const totals = s.video.reduce(
        (a, v) => ({
          seconds: a.seconds + v.seconds,
          lost: a.lost + v.lost,
          packets: a.packets + v.packets,
          freeze: a.freeze + v.freeze,
          bytes: a.bytes + v.bytes,
          frames: a.frames + v.frames,
        }),
        { seconds: 0, lost: 0, packets: 0, freeze: 0, bytes: 0, frames: 0 },
      );
      const seconds = totals.seconds || STATS_MS / 1000;
      const video = s.video.length
        ? {
          lossFraction: totals.lost / Math.max(1, totals.packets + totals.lost),
          freezeMs: totals.freeze,
          bitrate: totals.bytes * 8 / seconds,
          fps: totals.frames / seconds,
        }
        : undefined;
      if (video) {
        const estimate = s.video.at(-1)?.estimate;
        bad ||= video.lossFraction > 0.02 || video.freezeMs > seconds * 1000 * 0.2 ||
          totals.bytes > 0 && totals.frames === 0 ||
          estimate !== undefined && estimate < 0.8 * (s.target.settings().maxBitrate ?? 0);
      }
      s.stats = {
        deliveredHz: video?.fps ?? s.samples.length / (STATS_MS / 1000),
        ageMs: { p50: percentile(ages, 0.5), p95: percentile(ages, 0.95) },
        aborted: this.#aborts.reduce((n, a) => n + a.count, 0),
        rung: s.index ? { name: s.rung, index: s.index } : null,
        adapted: s.index > 0,
        ...(s.rejected ? { rejected: s.rejected } : {}),
        ...(video ? { video } : {}),
      };
      rates.set(
        s,
        video ? video.bitrate / 8 : s.samples.reduce((n, v) => n + v.bytes, 0) / (STATS_MS / 1000),
      );
      if (bad) stalled.push(s);
      if (s.target.policy.adapt && !s.pending && s.cooldown <= now) eligible.push(s);
    }
    const abort = this.#abortUntil >= now && this.#abortUntil > 0;
    const bad = stalled.length > 0 || abort;
    if (bad) {
      this.#lastStall = now;
      this.#stalledTicks++;
    } else this.#stalledTicks = 0;
    const retry = eligible.filter((s) => s.retry !== undefined)
      .sort((a, b) => b.target.policy.priority - a.target.policy.priority)[0];
    if (retry) {
      this.#change(retry, retry.retry!);
      return;
    }
    if (this.#stalledTicks >= 2) {
      const candidates = eligible.filter((s) =>
        (!stalled.length || stalled.includes(s)) && s.index < this.#ladder(s).length - 1
      );
      candidates.sort((a, b) =>
        b.target.policy.priority - a.target.policy.priority ||
        (rates.get(b) ?? 0) - (rates.get(a) ?? 0)
      );
      if (candidates[0]) this.#change(candidates[0], candidates[0].index + 1);
    } else if (!bad && now - this.#lastStall >= ADAPT_RECOVERY_MS) {
      const candidates = eligible.filter((s) => s.index > 0).sort((a, b) =>
        a.target.policy.priority - b.target.policy.priority
      );
      if (candidates[0]) {
        this.#lastStall = now;
        this.#change(candidates[0], candidates[0].index - 1);
      }
    }
  }
  #ladder(s: State): Rung[] {
    return adaptationLadder(s.target.profile, s.target.ceiling(), s.target.policy.keep);
  }
  #change(s: State, index: number): void {
    const ladder = this.#ladder(s);
    const rung = ladder[Math.min(index, ladder.length - 1)];
    s.pending = true;
    s.cooldown = Date.now() +
      (s.target.profile === "video.h264.v1" ? VIDEO_COOLDOWN_MS : DATA_COOLDOWN_MS);
    s.target.apply(rung.cap).then(() => {
      s.cooldown = Date.now() +
        (s.target.profile === "video.h264.v1" ? VIDEO_COOLDOWN_MS : DATA_COOLDOWN_MS);
      s.index = Math.min(index, ladder.length - 1);
      s.rung = rung.name;
      s.rejected = undefined;
      s.retry = undefined;
    }, (error: { code?: string }) => {
      s.rejected = error.code ?? "update_failed";
      s.retry = index;
    }).finally(() => {
      s.pending = false;
      if (!this.#enabled) this.#lift();
    });
  }
  close(): void {
    if (this.#timer !== null) clearInterval(this.#timer);
    this.#timer = null;
    if (typeof document !== "undefined") {
      document.removeEventListener("visibilitychange", this.#visibility);
    }
    for (const s of this.#states.values()) s.epoch++;
    this.#states.clear();
  }
}

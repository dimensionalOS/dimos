// Channel + status stores. React-free by design: the session layer writes here
// at wire rate, while React reads through two decoupled paths (hooks.ts) so
// high-rate channels never cause per-frame renders. Canvas consumers (T4) use
// the direct subscribe/get path instead.
//
// Two clocks meet here. Header `ts` is the source's Unix time in seconds (the
// bridge stamps time.time()); everything else is browser milliseconds. Rate
// and age are computed in source time so a burst-delivered backlog reads as
// its true source rate and age, not as fresh high-rate data. The offset
// between the clocks is estimated per channel as min(arrival - header ts)
// over received frames; rebaseline() re-arms the estimate because a new
// producer brings a new clock, and published ages clamp at 0 when a slot
// predates the current estimate.

import type { FrameHeader, RobotInfo } from "@dimos/shared";
import type { Manifest } from "@dimos/shared/manifest";
import type { SessionError } from "./errors.ts";
import type { TransportPhase } from "./transport.ts";

/** Latest successfully decoded frame of one channel (latest-wins by header seq). */
export interface Slot {
  value: unknown;
  /** Bounded text form of `value` for the raw channel UI (decoder-provided). */
  preview?: string;
  seq: number;
  ts: number;
  version: number;
}

export interface ChannelStats {
  frames: number;
  decodeErrors: number;
  /** Received-frame rate over the last HZ_WINDOW_MS of source time (header ts). */
  hz: number;
  /** Skew-corrected source age of the slot's frame; null while there is no slot. */
  ageMs: number | null;
  /** Browser arrival time of the newest frame (receive silence, not telemetry age). */
  lastFrameAtMs: number;
  /** Header seq/ts of the newest received frame, decoded or not (-1/0 before data). */
  lastSeq: number;
  lastTs: number;
  /** True while the newest received frame failed to decode. */
  decodeFailing: boolean;
}

export interface ChannelSnapshot {
  slot: Slot | null;
  stats: ChannelStats;
}

const HZ_WINDOW_MS = 2000;
// The rate window is a ring of fixed source-time buckets, pruned on ingest and
// slid by projected source time on publish, so storage stays bounded no matter
// how long publishUi() is starved (background tabs throttle timers).
const HZ_BUCKET_MS = 100;
const HZ_BUCKET_COUNT = HZ_WINDOW_MS / HZ_BUCKET_MS;
// Staleness must keep rising on screen while a channel is silent; bucketing
// the published age makes publishUi() emit a fresh snapshot once per bucket.
const AGE_BUCKET_MS = 500;

// After rebaseline() the latest-wins seq guard is suspended for this long:
// long enough to outlive frames of a dead robot session still draining out
// of the relay's queues (whose high seqs would lock out the new producer's
// restarted counter), short enough that ordinary out-of-order latest streams
// can only wobble the slot briefly after a producer change.
export const REBASELINE_WINDOW_MS = 3000;

const EMPTY_SNAPSHOT: ChannelSnapshot = {
  slot: null,
  stats: {
    frames: 0,
    decodeErrors: 0,
    hz: 0,
    ageMs: null,
    lastFrameAtMs: 0,
    lastSeq: -1,
    lastTs: 0,
    decodeFailing: false,
  },
};

interface ChannelState {
  slot: Slot | null;
  frames: number;
  decodeErrors: number;
  decodeFailing: boolean;
  lastFrameAtMs: number;
  lastSeq: number;
  lastTs: number;
  skewMs: number; // min(arrival ms - header ts ms) since the last re-arm
  skewRearm: boolean; // next frame replaces skewMs and clears the rate ring
  rate: number[]; // frame counts per HZ_BUCKET_MS of source time
  rateHead: number; // absolute bucket index of the ring's newest slot; -1 = empty
  dirty: boolean;
  ageBucket: number;
  snapshot: ChannelSnapshot;
  direct: Set<() => void>;
  ui: Set<() => void>;
}

/** Notify every subscriber, isolating failures: one throwing callback must
 * not silence the rest, and an exception must never travel up into the
 * ingest path (a reliable stream's reader would abandon the stream). */
function notifyAll(cbs: Set<() => void>): void {
  for (const cb of cbs) {
    try {
      cb();
    } catch (e) {
      console.error("channel store subscriber threw", e);
    }
  }
}

function rateSlot(bucket: number): number {
  return ((bucket % HZ_BUCKET_COUNT) + HZ_BUCKET_COUNT) % HZ_BUCKET_COUNT;
}

/** Slide the ring forward so `bucket` is its newest slot, zeroing what it passes. */
function advanceRate(state: ChannelState, bucket: number): void {
  if (state.rateHead === -1 || bucket - state.rateHead >= HZ_BUCKET_COUNT) {
    state.rate.fill(0);
  } else {
    for (let b = state.rateHead + 1; b <= bucket; b++) state.rate[rateSlot(b)] = 0;
  }
  state.rateHead = bucket;
}

export class ChannelStore {
  #channels = new Map<string, ChannelState>();
  #now: () => number;
  #rebaselineUntilMs = 0;

  constructor(now: () => number = Date.now) {
    this.#now = now;
  }

  #state(ch: string): ChannelState {
    let state = this.#channels.get(ch);
    if (state === undefined) {
      state = {
        slot: null,
        frames: 0,
        decodeErrors: 0,
        decodeFailing: false,
        lastFrameAtMs: 0,
        lastSeq: -1,
        lastTs: 0,
        skewMs: Infinity,
        skewRearm: false,
        rate: new Array<number>(HZ_BUCKET_COUNT).fill(0),
        rateHead: -1,
        dirty: false,
        ageBucket: -1,
        snapshot: EMPTY_SNAPSHOT,
        direct: new Set(),
        ui: new Set(),
      };
      this.#channels.set(ch, state);
    }
    return state;
  }

  /**
   * Record an arrived frame. `value` is the decoded payload (undefined for
   * encodings without a decoder); `decodeOk` is false when a decoder threw;
   * `preview` is the decoder's bounded text form for the raw UI. A failed
   * decode only updates stats: the slot always describes one successfully
   * decoded frame. Decoded frames with seq <= the current slot's are dropped
   * from the slot (streams arrive out of order by design) but still counted
   * in the stats - except inside a rebaseline window, where arrival order
   * wins.
   */
  ingest(
    ch: string,
    header: FrameHeader,
    value: unknown,
    decodeOk: boolean,
    preview?: string,
  ): void {
    const state = this.#state(ch);
    const now = this.#now();
    state.frames += 1;
    state.lastFrameAtMs = now;
    state.lastSeq = header.seq;
    state.lastTs = header.ts;
    state.dirty = true;
    const tsMs = header.ts * 1000;
    const skew = now - tsMs;
    if (state.skewRearm) {
      state.skewRearm = false;
      state.skewMs = skew;
      state.rate.fill(0);
      state.rateHead = -1;
    } else if (skew < state.skewMs) {
      state.skewMs = skew;
    }
    const bucket = Math.floor(tsMs / HZ_BUCKET_MS);
    if (bucket > state.rateHead) advanceRate(state, bucket);
    if (bucket > state.rateHead - HZ_BUCKET_COUNT) state.rate[rateSlot(bucket)] += 1;
    if (!decodeOk) {
      state.decodeErrors += 1;
      state.decodeFailing = true;
      return;
    }
    state.decodeFailing = false;
    const prev = state.slot;
    if (prev === null || now < this.#rebaselineUntilMs || header.seq > prev.seq) {
      state.slot = {
        value,
        preview,
        seq: header.seq,
        ts: header.ts,
        version: (prev?.version ?? 0) + 1,
      };
      notifyAll(state.direct);
    }
  }

  /** Always-current slot (canvas/direct consumers; no snapshot indirection). */
  get(ch: string): Slot | null {
    return this.#channels.get(ch)?.slot ?? null;
  }

  /** Synchronous per-ingest notifications (canvas/direct consumers, T4). */
  subscribe(ch: string, cb: () => void): () => void {
    const state = this.#state(ch);
    state.direct.add(cb);
    return () => state.direct.delete(cb);
  }

  /** Notified only from publishUi(); pair with getUiSnapshot for React. */
  subscribeUi(ch: string, cb: () => void): () => void {
    const state = this.#state(ch);
    state.ui.add(cb);
    return () => state.ui.delete(cb);
  }

  /** Stable between publishUi() calls (useSyncExternalStore contract). */
  getUiSnapshot(ch: string): ChannelSnapshot {
    return this.#channels.get(ch)?.snapshot ?? EMPTY_SNAPSHOT;
  }

  /** Rebuild and publish snapshots for channels whose visible state changed. */
  publishUi(): void {
    const now = this.#now();
    for (const state of this.#channels.values()) {
      // Slide the rate window along projected source time so silence decays hz.
      if (state.rateHead !== -1 && state.skewMs !== Infinity) {
        const bucket = Math.floor((now - state.skewMs) / HZ_BUCKET_MS);
        if (bucket > state.rateHead) advanceRate(state, bucket);
      }
      let inWindow = 0;
      for (const n of state.rate) inWindow += n;
      const hz = (inWindow * 1000) / HZ_WINDOW_MS;
      const ageMs = state.slot === null
        ? null
        : Math.max(0, now - state.skewMs - state.slot.ts * 1000);
      const ageBucket = ageMs === null ? -1 : Math.floor(ageMs / AGE_BUCKET_MS);
      if (!state.dirty && hz === state.snapshot.stats.hz && ageBucket === state.ageBucket) {
        continue;
      }
      state.dirty = false;
      state.ageBucket = ageBucket;
      state.snapshot = {
        slot: state.slot,
        stats: {
          frames: state.frames,
          decodeErrors: state.decodeErrors,
          hz,
          ageMs,
          lastFrameAtMs: state.lastFrameAtMs,
          lastSeq: state.lastSeq,
          lastTs: state.lastTs,
          decodeFailing: state.decodeFailing,
        },
      };
      notifyAll(state.ui);
    }
  }

  /**
   * The producer behind the channels may have changed (a manifest was
   * (re)confirmed): suspend the seq guard for REBASELINE_WINDOW_MS so a
   * restarted counter is accepted, and a late high-seq frame from the old
   * producer is displaced by the next arrival instead of locking it out.
   * The clock behind the header timestamps may have changed with it, so each
   * channel re-learns its skew (and drops rate history recorded against the
   * old clock) at its next frame.
   */
  rebaseline(): void {
    this.#rebaselineUntilMs = this.#now() + REBASELINE_WINDOW_MS;
    for (const state of this.#channels.values()) state.skewRearm = true;
  }

  /** Drop all data (the producer was invalidated); subscribers stay registered. */
  reset(): void {
    for (const state of this.#channels.values()) {
      state.slot = null;
      state.frames = 0;
      state.decodeErrors = 0;
      state.decodeFailing = false;
      state.lastFrameAtMs = 0;
      state.lastSeq = -1;
      state.lastTs = 0;
      state.skewMs = Infinity;
      state.skewRearm = false;
      state.rate.fill(0);
      state.rateHead = -1;
      state.dirty = false;
      state.ageBucket = -1;
      state.snapshot = EMPTY_SNAPSHOT;
      notifyAll(state.direct);
      notifyAll(state.ui);
    }
  }
}

export interface SessionStatus {
  transport: TransportPhase;
  /** Every robot the relay announced, watched or not. */
  robots: RobotInfo[];
  watchedRobot: RobotInfo | null;
  /** The adopted (normalized) manifest; null until a watch is confirmed. */
  manifest: Manifest | null;
  /** True when the robot's manifest version is newer than this build parses;
   * the App shows a polite update notice instead of panels. */
  manifestUnsupported: boolean;
  epoch: number;
  lastError: SessionError | null;
}

export class StatusStore {
  #status: SessionStatus = {
    transport: { phase: "connecting", attempt: 1 },
    robots: [],
    watchedRobot: null,
    manifest: null,
    manifestUnsupported: false,
    epoch: 0,
    lastError: null,
  };
  #subscribers = new Set<() => void>();

  get = (): SessionStatus => this.#status;

  update(patch: Partial<SessionStatus>): void {
    this.#status = { ...this.#status, ...patch };
    for (const cb of this.#subscribers) cb();
  }

  subscribe = (cb: () => void): () => void => {
    this.#subscribers.add(cb);
    return () => this.#subscribers.delete(cb);
  };
}

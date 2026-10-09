// Battery page state: the run's charge history folded from battery.json.v1
// frames, and the time left at the drain observed over the recent window.

export interface BatteryFrame {
  ts: number;
  percentage: number | null;
  voltage: number | null;
  current: number | null;
  temperature: number | null;
}

export interface BatterySample {
  /** Seconds since the first sample of the run. */
  t: number;
  /** Charge, 0..100. */
  pct: number;
}

export interface BatteryHistory {
  frame: BatteryFrame | null;
  /** Source time of the first sample; the chart's origin. */
  startTs: number | null;
  samples: readonly BatterySample[];
  unreadable: boolean;
}

export const EMPTY_HISTORY: BatteryHistory = {
  frame: null,
  startTs: null,
  samples: [],
  unreadable: false,
};

/** One sample per frame at 1-2 Hz: a day of running stays under this. */
export const MAX_SAMPLES = 100_000;
/** The drain is fitted over this much recent history. */
export const FIT_WINDOW_S = 600;
/** No estimate before this much history or this much observed drop. */
export const MIN_FIT_SPAN_S = 60;
export const MIN_FIT_DROP_PCT = 0.5;

function num(value: unknown): number | null {
  return typeof value === "number" && Number.isFinite(value) ? value : null;
}

export function parseBatteryFrame(value: unknown): BatteryFrame | null {
  if (typeof value !== "object" || value === null) return null;
  const v = value as Record<string, unknown>;
  const ts = num(v.ts);
  if (ts === null) return null;
  return {
    ts,
    percentage: num(v.percentage),
    voltage: num(v.voltage),
    current: num(v.current),
    temperature: num(v.temperature),
  };
}

export function foldBattery(history: BatteryHistory, value: unknown): BatteryHistory {
  const frame = parseBatteryFrame(value);
  if (frame === null) return { ...history, unreadable: true };
  const startTs = history.startTs ?? frame.ts;
  let samples = history.samples;
  if (frame.percentage !== null) {
    const sample = { t: frame.ts - startTs, pct: 100 * frame.percentage };
    samples = samples.length >= MAX_SAMPLES ? [...samples.slice(1), sample] : [...samples, sample];
  }
  return { frame, startTs, samples, unreadable: false };
}

/** Seconds of charge left at the drain of the last FIT_WINDOW_S (least
 * squares on pct vs t), or null while charging, idle, or too early to say. */
export function estimateRemainingS(samples: readonly BatterySample[]): number | null {
  if (samples.length < 2) return null;
  const last = samples[samples.length - 1];
  const window = samples.filter((s) => s.t >= last.t - FIT_WINDOW_S);
  const span = last.t - window[0].t;
  const drop = window[0].pct - last.pct;
  if (span < MIN_FIT_SPAN_S || drop < MIN_FIT_DROP_PCT) return null;
  let st = 0, sp = 0, stt = 0, stp = 0;
  for (const { t, pct } of window) {
    st += t;
    sp += pct;
    stt += t * t;
    stp += t * pct;
  }
  const n = window.length;
  const slope = (n * stp - st * sp) / (n * stt - st * st); // pct per second
  if (!(slope < 0)) return null;
  return last.pct / -slope;
}

export function fmtDuration(seconds: number): string {
  const s = Math.max(0, Math.round(seconds));
  const h = Math.floor(s / 3600);
  const m = Math.floor((s % 3600) / 60);
  if (h > 0) return `${h} h ${String(m).padStart(2, "0")} min`;
  if (m > 0) return `${m} min`;
  return `${s} s`;
}

export function fmtClock(seconds: number): string {
  const s = Math.max(0, Math.floor(seconds));
  const h = Math.floor(s / 3600);
  const m = Math.floor((s % 3600) / 60);
  const r = s % 60;
  return `${String(h).padStart(2, "0")}:${String(m).padStart(2, "0")}:${
    String(r).padStart(2, "0")
  }`;
}

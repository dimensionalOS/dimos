// The Stats page's data: a stats.json.v1 frame is the resource monitor's
// /resource_stats dict (dimos/core/resource_monitor/stats.py, published at
// 1 Hz) with the keys the page reads, under the producer's own names, so the
// tab and the dtop TUI consume the same thing. The formatters and the CPU
// heat scale are dtop's (dimos/cli/dtop.py). Nothing here touches the DOM.

export interface ProcessStats {
  pid: number;
  alive: boolean;
  cpu_percent: number;
  cpu_time_user: number;
  cpu_time_system: number;
  cpu_time_iowait: number;
  pss: number;
  num_threads: number;
  num_children: number;
  num_fds: number;
  io_read_bytes: number;
  io_write_bytes: number;
}

export interface ChildStats {
  pid: number;
  name: string;
  cpu_percent: number;
}

export interface WorkerStats extends ProcessStats {
  worker_id: number;
  modules: string[];
  dedicated: boolean;
  /** Direct children; the worker's cpu_percent already includes theirs. */
  children: ChildStats[];
}

export interface StatsFrame {
  coordinator: ProcessStats;
  workers: WorkerStats[];
}

const PROCESS_NUMBERS = [
  "pid",
  "cpu_percent",
  "cpu_time_user",
  "cpu_time_system",
  "cpu_time_iowait",
  "pss",
  "num_threads",
  "num_children",
  "num_fds",
  "io_read_bytes",
  "io_write_bytes",
] as const;

type Rec = Record<string, unknown>;

function isRec(v: unknown): v is Rec {
  return typeof v === "object" && v !== null && !Array.isArray(v);
}

function readProcess(v: unknown): ProcessStats | null {
  if (!isRec(v) || typeof v.alive !== "boolean") return null;
  for (const key of PROCESS_NUMBERS) {
    if (typeof v[key] !== "number") return null;
  }
  return v as unknown as ProcessStats;
}

function readChild(v: unknown): ChildStats | null {
  if (
    !isRec(v) || typeof v.pid !== "number" || typeof v.name !== "string" ||
    typeof v.cpu_percent !== "number"
  ) {
    return null;
  }
  return { pid: v.pid, name: v.name, cpu_percent: v.cpu_percent };
}

function readWorker(v: unknown): WorkerStats | null {
  const process = readProcess(v);
  if (process === null) return null;
  const rec = v as Rec;
  if (typeof rec.worker_id !== "number" || typeof rec.dedicated !== "boolean") return null;
  if (!Array.isArray(rec.modules) || !rec.modules.every((m) => typeof m === "string")) return null;
  if (!Array.isArray(rec.children)) return null;
  const children: ChildStats[] = [];
  for (const c of rec.children) {
    const child = readChild(c);
    if (child === null) return null;
    children.push(child);
  }
  return {
    ...process,
    worker_id: rec.worker_id,
    modules: rec.modules,
    dedicated: rec.dedicated,
    children,
  };
}

/** The frame, or null when the payload is not stats.json.v1-shaped. */
export function parseStats(value: unknown): StatsFrame | null {
  if (!isRec(value) || !Array.isArray(value.workers)) return null;
  const coordinator = readProcess(value.coordinator);
  if (coordinator === null) return null;
  const workers: WorkerStats[] = [];
  for (const w of value.workers) {
    const worker = readWorker(w);
    if (worker === null) return null;
    workers.push(worker);
  }
  return { coordinator, workers };
}

const MIB = 1048576;

/** dtop's `{v:3.0f}%`, unpadded. */
export function fmtPct(v: number): string {
  return `${Math.round(v)}%`;
}

/** dtop's _fmt_mem: MB below a GiB, GB from there, one decimal. */
export function fmtMem(bytes: number): string {
  const mb = bytes / MIB;
  return mb >= 1024 ? `${(mb / 1024).toFixed(1)} GB` : `${mb.toFixed(1)} MB`;
}

/** dtop's _fmt_secs: s, m past a minute, h past an hour, one decimal. */
export function fmtSecs(s: number): string {
  if (s >= 3600) return `${(s / 3600).toFixed(1)}h`;
  if (s >= 60) return `${(s / 60).toFixed(1)}m`;
  return `${s.toFixed(1)}s`;
}

/** dtop's `IO r/w` compound: whole MB read and written. */
export function fmtIo(read: number, write: number): string {
  return `${Math.round(read / MIB)}/${Math.round(write / MIB)} MB`;
}

// dtop's gradient stops (dimos/cli/theme.py CYAN, YELLOW, RED).
const HEAT_CYAN = [0x00, 0xee, 0xee];
const HEAT_YELLOW = [0xff, 0xcc, 0x00];
const HEAT_RED = [0xff, 0x00, 0x00];

/** Color for a 0..1 load ratio: cyan -> yellow -> red, dtop's scale. */
export function heat(ratio: number): string {
  const r = Math.min(Math.max(ratio, 0), 1);
  const lower = r <= 0.5;
  const from = lower ? HEAT_CYAN : HEAT_YELLOW;
  const to = lower ? HEAT_YELLOW : HEAT_RED;
  const t = lower ? r * 2 : (r - 0.5) * 2;
  const hex = from.map((a, i) => Math.round(a + (to[i] - a) * t).toString(16).padStart(2, "0"));
  return `#${hex.join("")}`;
}

/** CPU percent -> heat color; dtop colors CPU on the absolute 0..100 scale. */
export function cpuColor(percent: number): string {
  return heat(percent / 100);
}

/** `samples` plus `value`, keeping the newest `window`. A new array: the
 * page compares identities (state, effect deps). */
export function pushSample(samples: readonly number[], value: number, window: number): number[] {
  const next = samples.slice(Math.max(samples.length - window + 1, 0));
  next.push(value);
  return next;
}

/** Sparkline length in samples: 60 s at the resource monitor's 1 Hz. */
export const WINDOW = 60;

export interface StatsSnapshot {
  /** The newest readable frame; null before the first. */
  frame: StatsFrame | null;
  /** True when the newest frame was not stats.json.v1-shaped. */
  unreadable: boolean;
  /** CPU samples per row key (see the key helpers), oldest first. */
  cpu: ReadonlyMap<string, readonly number[]>;
}

export const EMPTY_SNAPSHOT: StatsSnapshot = { frame: null, unreadable: false, cpu: new Map() };

export const COORDINATOR_KEY = "coordinator";
export function workerKey(id: number): string {
  return `worker:${id}`;
}
export function childKey(pid: number): string {
  return `child:${pid}`;
}

/** `previous` folded with one more frame payload: one CPU sample per row
 * (dtop samples its UI clock and doubles up), the newest WINDOW kept, rows
 * absent from the frame dropped with their history. An off-contract payload
 * only raises the flag. */
export function foldSnapshot(previous: StatsSnapshot, value: unknown): StatsSnapshot {
  const frame = parseStats(value);
  if (frame === null) return { ...previous, unreadable: true };
  const cpu = new Map<string, number[]>();
  const push = (key: string, sample: number): void => {
    cpu.set(key, pushSample(previous.cpu.get(key) ?? [], sample, WINDOW));
  };
  push(COORDINATOR_KEY, frame.coordinator.cpu_percent);
  for (const worker of frame.workers) {
    push(workerKey(worker.worker_id), worker.cpu_percent);
    for (const child of worker.children) push(childKey(child.pid), child.cpu_percent);
  }
  return { frame, unreadable: false, cpu };
}

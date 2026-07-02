// URL-persisted bench config — one declarative spec per knob, consumed by state init, URL
// reflection, and the repro-URL builder, so default-omission logic exists exactly once.
// Built on the shared urlState.ts helpers (not modified here); every write is per-key, so
// foreign params (?tab, topics/worldview state) survive by construction.
import { STREAM_PROFILES } from "@dimos/web";
import { getParam, setUrlParam } from "../../urlState";

export interface BenchUrlConfig {
  profiles: string[];
  durMs: number;
  /** Matrix axis; 0 = ∞. */
  maxHz: number[];
  /** Netem matrix axis — raw ids; the caller filters against the gateway-reported profiles. */
  net: string[];
  reps: number;
  /** Sweep drives the GO2Load generator per heavy profile (default on when the RPC exists). */
  autoDrive: boolean;
  /** Selected flood profiles also subscribe the pose lanes (scenario `<tier>+pose`) —
   *  measures whether the bulk stream delays /load/fast. Zero extra cells. */
  coex: boolean;
  genKind: "image" | "cloud";
  genHz: number;
  genBytes: number;
}

export const BENCH_DEFAULTS: BenchUrlConfig = {
  profiles: ["pose"],
  durMs: 4000,
  maxHz: [0],
  net: [],
  reps: 1,
  autoDrive: true,
  coex: false,
  genKind: "image",
  genHz: 20,
  genBytes: 1_000_000,
};

/** Bench-owned query keys — auto-open and reflection key strictly off this list, so a URL
 *  carrying only foreign params (?tab, ?topics, …) never pops the drawer. */
export const BENCH_KEYS = [
  "profiles",
  "dur",
  "maxHz",
  "net",
  "reps",
  "drive",
  "coex",
  "load",
  "loadHz",
  "loadBytes",
  "loadKind",
] as const;

/** True when the URL carries any bench knob OR the run flag — the app then lands on the
 *  Topics tab and the drawer opens. `run` stays out of BENCH_KEYS (never reflected), but a
 *  default-config repro link is exactly `?gw=…&transport=…&run=1` and must still land here. */
export const hasBenchParams = () => BENCH_KEYS.some((k) => getParam(k) !== null) || getParam("run") !== null;

/** One-shot auto-run flag (?run=1) — cleared after firing/cancel so reloads don't re-run. */
export const readRunFlag = () => getParam("run") === "1";
export const clearRunFlag = () => setUrlParam("run", null);

const PROFILE_IDS = new Set(STREAM_PROFILES.map((p) => p.id));
/** The old generator ladder used "light" for today's `lidar` profile. */
const LOAD_ALIASES: Record<string, string> = { light: "lidar" };

const intList = (raw: string | null, def: number[]): number[] => {
  if (raw === null) return def;
  const ns = [...new Set(raw.split(",").map(Number).filter((n) => Number.isInteger(n) && n >= 0))];
  return ns.length ? ns.sort((a, b) => a - b) : def;
};
const int = (raw: string | null, def: number, min = 0): number => {
  const n = raw === null ? NaN : Number(raw);
  return Number.isInteger(n) && n >= min ? n : def;
};
const sameList = <T>(a: T[], b: T[]) => a.length === b.length && a.every((v, i) => v === b[i]);

/** Read the whole config from the URL once (lazy useState initializers). */
export function readBenchConfig(): BenchUrlConfig {
  const d = BENCH_DEFAULTS;
  const profiles = (getParam("profiles") ?? "").split(",").filter((id) => PROFILE_IDS.has(id));
  const loadRaw = getParam("load");
  const loadId = loadRaw ? LOAD_ALIASES[loadRaw] ?? loadRaw : null;
  const tier = STREAM_PROFILES.find((p) => p.id === loadId && p.gen);
  const kindRaw = getParam("loadKind");
  return {
    profiles: profiles.length ? profiles : d.profiles,
    durMs: int(getParam("dur"), d.durMs, 100),
    maxHz: intList(getParam("maxHz"), d.maxHz),
    net: (getParam("net") ?? "").split(",").filter(Boolean),
    reps: int(getParam("reps"), d.reps, 1),
    autoDrive: getParam("drive") !== "0",
    coex: getParam("coex") === "1",
    genKind: kindRaw === "cloud" ? "cloud" : tier?.gen?.kind ?? d.genKind,
    genHz: int(getParam("loadHz"), tier?.gen?.hz ?? d.genHz, 1),
    genBytes: int(getParam("loadBytes"), tier?.gen?.bytes ?? d.genBytes, 1),
  };
}

/** Serialize every knob (null = omit — the default). Shared by reflection and the repro builder. */
function serializeAll(cfg: BenchUrlConfig): Record<(typeof BENCH_KEYS)[number], string | null> {
  const d = BENCH_DEFAULTS;
  // A (hz, bytes) pair matching a gen profile serializes as its id; free values go numeric.
  const tier = STREAM_PROFILES.find((p) => p.gen?.hz === cfg.genHz && p.gen?.bytes === cfg.genBytes);
  const genDefault = cfg.genKind === d.genKind && cfg.genHz === d.genHz && cfg.genBytes === d.genBytes;
  return {
    profiles: sameList(cfg.profiles, d.profiles) ? null : cfg.profiles.join(","),
    dur: cfg.durMs === d.durMs ? null : String(cfg.durMs),
    maxHz: sameList(cfg.maxHz, d.maxHz) ? null : cfg.maxHz.join(","),
    net: cfg.net.length ? cfg.net.join(",") : null,
    reps: cfg.reps === d.reps ? null : String(cfg.reps),
    drive: cfg.autoDrive ? null : "0",
    coex: cfg.coex ? "1" : null,
    load: genDefault || !tier ? null : tier.id,
    loadHz: genDefault || tier ? null : String(cfg.genHz),
    loadBytes: genDefault || tier ? null : String(cfg.genBytes),
    loadKind: cfg.genKind === "cloud" ? "cloud" : null,
  };
}

/** Reflect the knobs into the URL in place (replaceState — shareable, no history spam). */
export function reflectBenchConfig(cfg: BenchUrlConfig): void {
  const all = serializeAll(cfg);
  for (const k of BENCH_KEYS) setUrlParam(k, all[k]);
}

/** Paste-to-reproduce URL: the CURRENT URL's params (foreign topics/worldview state rides
 *  along — background subscriptions and renderers shift results, so carrying them makes the
 *  link reproduce the session, not just the knobs), with gw/transport pinned explicitly,
 *  the bench knobs re-serialized, and run=1 appended. */
export function buildReproUrl(cfg: BenchUrlConfig, live: { gw: string; transport: string }): string {
  const u = new URL(globalThis.location.href);
  u.searchParams.set("gw", live.gw);
  u.searchParams.set("transport", live.transport);
  const all = serializeAll(cfg);
  for (const k of BENCH_KEYS) {
    if (all[k] === null) u.searchParams.delete(k);
    else u.searchParams.set(k, all[k]!);
  }
  u.searchParams.set("run", "1");
  return u.toString();
}

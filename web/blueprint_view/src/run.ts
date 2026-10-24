// The run this blueprint is in (GET /active's launch), and whether the saved config moved on since it started.
import { sameValue } from "dimos-react";
import type { Saved } from "./config.tsx";

export interface Launch {
  blueprint: string;
  phase: "starting" | "running" | "stopping" | "stopped" | "failed";
  pid: number;
  output?: string;
  /** why it failed, in one line, when the gateway could tell */
  error?: string | null;
  runId: string | null;
  config?: { global?: Record<string, unknown>; modules?: Record<string, Record<string, unknown>> };
  steps?: { code: string; state: string; data?: { deployed?: number } }[];
}

/** GET /active: the running blueprint (Desktop's, or one started in a terminal) and the gateway's last launch. */
export interface Active {
  running: boolean;
  runId: string | null;
  pid: number | null;
  blueprint: string | null;
  startedAt: string | null;
  logDir: string | null;
  launch: Launch | null;
}

/** The live run in dimos's run registry, as GET /active tells it. */
export interface RegistryRun {
  run_id: string;
  pid: number;
  blueprint: string;
  started_at: string;
  log_dir: string;
}

export function activeRuns(active: Active): RegistryRun[] {
  return active.running && active.runId && active.blueprint && active.pid !== null
    ? [{
      run_id: active.runId,
      pid: active.pid,
      blueprint: active.blueprint,
      started_at: active.startedAt ?? "",
      log_dir: active.logDir ?? "",
    }]
    : [];
}

/** This blueprint's live run, however it was started: the one asked for (&run=), else the newest. */
export function liveRunOf(
  runs: RegistryRun[],
  name: string,
  wanted: string | null,
): RegistryRun | null {
  const mine = runs.filter((run) => run.blueprint === name);
  return mine.find((run) => run.run_id === wanted) ?? mine[0] ?? null;
}

// dimos prints its tracebacks in rich's boxes, so the last line of a failed launch is a box edge and every line
// inside one carries gutters. Both have to come off before any of it reads as a reason.
const GUTTER = /^[\s|│┃]+|[\s|│┃]+$/g;
const DECORATION = /^[╭╮╰╯┌┐└┘├┤┬┴┼─━═│┃|\s]*$/;
/** `RuntimeError: ...`, `dimos.Error: ...`, and dimos's own `Error: ...` lines. */
const RAISED = /^(?:[A-Za-z_][\w.]*(?:Error|Exception|Interrupt)|Error|ExceptionGroup)\b.*?:\s*\S/;
/** A traceback's own furniture: the frames and code it quotes, not the exception. */
const FRAME = /^(?:File ["']|\/|[\w./-]+\.py[:\s]|\d+\s|[❱>]\s|at\s|Traceback\b)/;

const meaningful = (text: string): string[] =>
  text.split(/\r?\n/).map((line) => line.replace(GUTTER, "")).filter((line) =>
    line && !DECORATION.test(line)
  );

/**
 * Why a launch failed, in one line: the exception it ended with, else the last thing it printed. rich wraps a long
 * exception over several lines, so the lines under it are taken back onto it until the traceback's furniture starts.
 */
export function failedWhy(launch: Launch | null): string {
  const said = meaningful(launch?.error ?? "");
  const printed = said.length ? said : meaningful(launch?.output ?? "");
  const raised = printed.findLastIndex((line) => RAISED.test(line));
  if (raised < 0) {
    return printed.at(-1) ?? "it printed nothing";
  }
  const wrapped = [printed[raised]];
  for (const line of printed.slice(raised + 1)) {
    if (RAISED.test(line) || FRAME.test(line) || wrapped.join(" ").length > 300) {
      break;
    }
    wrapped.push(line);
  }
  return wrapped.join(" ");
}

/** A relaunch in flight: stopping the live run, then waiting for the new one. */
export type Relaunching = false | "stopping" | "starting";

const STEP_LABELS: Record<string, string> = {
  starting: "Starting dimOS",
  building: "Building the blueprint",
  starting_modules: "Starting modules",
  downloading_data: "Downloading the recording",
};

/** Whether the run started with other config than what's saved now (secrets, shown masked on both sides, aside). */
export function launchedWithOther(launch: Launch | null, saved: Saved | null): boolean {
  if (!launch?.config || !saved) {
    return false;
  }
  const masked = (value: unknown) => typeof value === "string" && /^•+$/.test(value);
  const differs = (
    left: Record<string, unknown>,
    right: Record<string, unknown>,
    base: Record<string, unknown> = {},
  ) =>
    [...new Set([...Object.keys(left), ...Object.keys(right)])].some((key) =>
      !masked(left[key]) && !masked(right[key]) &&
      !sameValue(key in left ? left[key] : base[key], key in right ? right[key] : base[key])
    );
  if (differs(saved.global, launch.config.global ?? {}, saved.defaults)) {
    return true;
  }
  const ran = launch.config.modules ?? {};
  const names = new Set([...Object.keys(saved.modules), ...Object.keys(ran)]);
  return [...names].some((name) => differs(saved.modules[name] ?? {}, ran[name] ?? {}));
}

function startingStep(
  launch: Launch | null,
  relaunching: Relaunching,
  relaunchedPid: number | null,
) {
  if (!launch || relaunching && launch.pid === relaunchedPid) {
    return STEP_LABELS.starting;
  }
  const download = (launch.output ?? "").split(/[\r\n]+/)
    .filter((line) => line.includes("Downloading LFS objects")).at(-1);
  if (download && !/\b100%|\bdone\b/i.test(download)) {
    return STEP_LABELS.downloading_data;
  }
  const step = (launch.steps ?? []).find((each) => each.state === "now");
  const deployed = step?.code === "starting_modules" ? step.data?.deployed : undefined;
  const label = STEP_LABELS[step?.code ?? ""] ?? STEP_LABELS.starting;
  return typeof deployed === "number" && deployed ? `${label} (${deployed} started)` : label;
}

export function relaunchLabel(
  launch: Launch | null,
  relaunching: Relaunching,
  relaunchedPid: number | null,
  stale: boolean,
): string {
  const phase = launch?.phase;
  if (
    relaunching === "stopping" && launch?.pid === relaunchedPid ||
    !relaunching && phase === "stopping"
  ) {
    return "Stopping…";
  }
  if (relaunching || phase === "starting") {
    return `${startingStep(launch, relaunching, relaunchedPid)}…`;
  }
  return stale ? "Relaunch to apply" : "Relaunch";
}

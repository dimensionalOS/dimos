import type { DimosRun } from "#dimos/agents/shared/protocol/runtime/runs.ts";
import type { EpochMs } from "#dimos/agents/shared/protocol/time.ts";
import type { Level, Segment } from "#dimos/agents/shared/view-models/levels.ts";
import type { LinkStatus } from "#dimos/agents/shared/protocol/gateway/link.ts";
import type { LoadStatus } from "#dimos/agents/shared/protocol/runtime/load.ts";
import { clock } from "#dimos/agents/shared/view-models/time.ts";

/** Reconnect attempts back off 1, 2, 4, 8, 16 s: the fifth is past 30 s, the ninth past 5 min. */
const CONNECTING_WARN_ATTEMPT = 5;

const CONNECTING_CRITICAL_ATTEMPT = 9;

function linkSegment(link: LinkStatus, runs: readonly DimosRun[]): Segment {
  switch (link.state) {
    case "connecting": {
      const level: Level =
        link.attempt >= CONNECTING_CRITICAL_ATTEMPT
          ? "critical"
          : link.attempt >= CONNECTING_WARN_ATTEMPT
            ? "warn"
            : "ok";
      return {
        text: link.attempt <= 1 ? "DimOS ◌ connecting" : `DimOS ◌ connecting, attempt ${link.attempt}`,
        level,
      };
    }
    case "up":
      return { text: "DimOS ● up", level: "ok" };
    case "down":
      return {
        text: `DimOS ○ down, retry ${clock(link.retryAt)}`,
        level: runs.some((run) => run.alive) ? "critical" : "warn",
      };
  }
}

function loadSegment(load: LoadStatus): Segment | null {
  return load.state === "failed" ? { text: `runtime load failed, retry ${clock(load.retryAt)}`, level: "warn" } : null;
}

export function linkSegments(
  link: LinkStatus,
  load: LoadStatus,
  runs: readonly DimosRun[],
  _now: EpochMs,
): readonly Segment[] {
  const loaded = loadSegment(load);
  return loaded === null ? [linkSegment(link, runs)] : [linkSegment(link, runs), loaded];
}

/** "run unitree-go2-agentic pid 41823", "run file left by pid 41823 (not running)", or null with no run file. */
export function runSegment(runs: readonly DimosRun[]): Segment | null {
  const run = runs.find((candidate) => candidate.alive) ?? runs[0];
  if (run === undefined) return null;
  return run.alive
    ? { text: `run ${run.blueprint} pid ${run.pid}`, level: "ok" }
    : { text: `run file left by pid ${run.pid} (not running)`, level: "warn" };
}

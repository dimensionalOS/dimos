import type { EpochMs } from "../time.ts";

/** One `~/.local/state/dimos/runs/<run_id>.json` (`dimos/core/run_registry.py`), with a liveness check of its pid. */
export type DimosRun = {
  readonly runId: string;
  readonly pid: number;
  readonly blueprint: string;
  readonly startedAt: EpochMs;
  /** `kill -0 pid` succeeded at the last read; a run that exited uncleanly leaves its file with `alive: false`. */
  readonly alive: boolean;
};

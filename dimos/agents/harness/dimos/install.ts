import { readRuns } from "./runs.ts";

/** Where DimOS lives and how to run and reach it. `dimos agent gateway` sets these; nothing reads config.json for them. */
export type DimosInstall = {
  /** The DimOS checkout; bash's DIMOS_ROOT and the default workspace. */
  readonly root: string;
  /** The venv's Python; its directory goes first on bash's PATH. */
  readonly python: string;
  /** The venv's `dimos` executable, which runs `dimos agent load`. */
  readonly cli: string;
  /** McpServer's endpoint, from its module settings: `McpServerConfig` `host` and `port` (`MCPSERVER__*`). */
  readonly mcpUrl: URL;
  /** `~/.local/state/dimos/runs/`, one file per running system. */
  readonly runsDir: string;
  /** `$XDG_STATE_HOME/dimos`: agent module records, also read while DimOS is down. */
  readonly stateDir: string;
};

/**
 * Whether a `dimos run` system is running: it writes one file per run and removes it when the run stops, and a run
 * that exited uncleanly leaves a file whose pid is gone.
 */
export function hasLiveRun(install: DimosInstall): boolean {
  return readRuns(install.runsDir).some((run) => run.alive);
}

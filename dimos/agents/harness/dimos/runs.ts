import type { DimosRun } from "#dimos/agents/shared/protocol/runtime/runs.ts";
import type { FSWatcher } from "node:fs";
import type { Live } from "#dimos/agents/shared/protocol/status.ts";
import { Status } from "#dimos/agents/shared/protocol/status.ts";
import { Type } from "typebox";
import { Value } from "typebox/value";
import { epochMs } from "#dimos/agents/shared/protocol/time.ts";
import { join } from "node:path";
import { mkdirSync, readFileSync, readdirSync, watch } from "node:fs";

type RunWatcherOptions = {
  readonly dir: string;
  /** How often pid liveness is re-checked between directory changes. */
  readonly pollMs: number;
  readonly report: (error: unknown) => void;
};

/** The fields of `RunEntry` the gateway shows; `started_at` is `datetime.isoformat()`. */
const RunFile = Type.Object({
  run_id: Type.String(),
  pid: Type.Integer({ minimum: 1 }),
  blueprint: Type.String(),
  started_at: Type.String(),
});

export function isAlive(pid: number): boolean {
  try {
    process.kill(pid, 0);
    return true;
  } catch (error) {
    return (error as NodeJS.ErrnoException).code === "EPERM";
  }
}

/** A file still being written does not parse yet; it is left out until a later read. */
function readRun(path: string): DimosRun | null {
  let json: unknown;
  try {
    json = JSON.parse(readFileSync(path, "utf8"));
  } catch {
    return null;
  }
  if (!Value.Check(RunFile, json)) return null;
  const startedAt = Date.parse(json.started_at);
  if (Number.isNaN(startedAt)) return null;
  return {
    runId: json.run_id,
    pid: json.pid,
    blueprint: json.blueprint,
    startedAt: epochMs(startedAt),
    alive: isAlive(json.pid),
  };
}

/** Every run file in `dir` that parses, oldest first; a missing directory has none. */
export function readRuns(dir: string): readonly DimosRun[] {
  let names: string[];
  try {
    names = readdirSync(dir);
  } catch {
    return [];
  }
  return names
    .filter((name) => name.endsWith(".json"))
    .flatMap((name) => readRun(join(dir, name)) ?? [])
    .sort((a, b) => a.startedAt - b.startedAt || a.runId.localeCompare(b.runId));
}

/**
 * The runs directory, read on every change and every `pollMs` for pid liveness. The loader's own watcher moves here;
 * it subscribes instead of watching the directory itself.
 */
export class RunWatcher {
  private readonly list: Status<readonly DimosRun[]>;
  readonly status: Live<readonly DimosRun[]>;
  private watcher: FSWatcher | null = null;
  private timer: NodeJS.Timeout | null = null;
  private readonly options: RunWatcherOptions;

  constructor(options: RunWatcherOptions) {
    this.options = options;
    this.list = new Status<readonly DimosRun[]>([], options.report);
    this.status = this.list;
  }

  /** Reads once, watches the directory, polls liveness; a missing directory is an empty list until it appears. */
  start(): void {
    try {
      mkdirSync(this.options.dir, { recursive: true });
      this.watcher = watch(this.options.dir, () => this.refresh());
      this.watcher.unref();
    } catch (error) {
      this.options.report(error);
    }
    this.timer = setInterval(() => this.refresh(), this.options.pollMs);
    this.timer.unref();
    this.refresh();
  }

  /** Re-reads now; called by the loader's triggers and tests. */
  refresh(): void {
    const runs = readRuns(this.options.dir);
    if (JSON.stringify(runs) === JSON.stringify(this.list.value)) return;
    this.list.set(runs);
  }

  close(): void {
    this.watcher?.close();
    this.watcher = null;
    if (this.timer !== null) clearInterval(this.timer);
    this.timer = null;
  }
}

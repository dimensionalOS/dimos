import type { ChildProcess } from "node:child_process";
import type { DimosInstall } from "#dimos/agents/harness/dimos/install.ts";
import type { DimosRun } from "#dimos/agents/shared/protocol/runtime/runs.ts";
import type { Live } from "#dimos/agents/shared/protocol/status.ts";
import type { LoadResult, LoadStatus } from "#dimos/agents/shared/protocol/runtime/load.ts";
import type { RunWatcher } from "#dimos/agents/harness/dimos/runs.ts";
import { Status } from "#dimos/agents/shared/protocol/status.ts";
import { epochMs } from "#dimos/agents/shared/protocol/time.ts";
import { loadResultDecoder } from "#dimos/agents/harness/dimos/wire/load.ts";
import { spawn } from "node:child_process";

type LoadTrigger = Extract<LoadStatus, { state: "running" }>["trigger"];

type LoaderOptions = {
  readonly install: DimosInstall;
  /** The environment `dimos agent load` runs with: the gateway's, which names the bus. */
  readonly env: NodeJS.ProcessEnv;
  /** A new live run asks for a load; no live run means there is nothing to load into. */
  readonly runs: RunWatcher;
  readonly maxBackoffMs: number;
  readonly report: (error: unknown) => void;
};

/** Keeps HarnessRuntime loaded into whichever DimOS runs: one `dimos agent load` at a time, retried with backoff. */
export class RuntimeLoader {
  private readonly options: LoaderOptions;
  private readonly load: Status<LoadStatus>;
  readonly status: Live<LoadStatus>;
  private running: ChildProcess | null = null;
  private pending: LoadTrigger | null = null;
  private attempt = 0;
  private retryTimer: NodeJS.Timeout | null = null;
  private unsubscribe: (() => void) | null = null;
  private closed = false;

  constructor(options: LoaderOptions) {
    this.options = options;
    this.load = new Status<LoadStatus>({ state: "idle" }, options.report);
    this.status = this.load;
  }

  /** Asks for a load at once, and again whenever a live run appears or the last live run goes. */
  start(): void {
    let known: ReadonlySet<DimosRun["runId"]> | null = null;
    this.unsubscribe = this.options.runs.status.subscribe((runs) => {
      const previous = known;
      const live = runs.filter((run) => run.alive);
      known = new Set(live.map((run) => run.runId));
      if (previous === null) this.trigger("gateway_start");
      else if (live.some((run) => !previous.has(run.runId)) || (live.length === 0 && previous.size > 0))
        this.trigger("new_run");
    });
  }

  /**
   * Coalesces: while a load runs, any number of triggers schedule exactly one more. Missing tools wait for a retry
   * already due, so a load that cannot add them backs off.
   */
  trigger(reason: LoadTrigger): void {
    if (this.closed) return;
    if (this.running !== null) {
      if (this.pending === null || reason !== "tools_missing") this.pending = reason;
      return;
    }
    if (this.retryTimer !== null) {
      if (reason === "tools_missing") return;
      clearTimeout(this.retryTimer);
      this.retryTimer = null;
    }
    if (reason !== "tools_missing" && !this.options.runs.status.value.some((run) => run.alive)) {
      this.setStatus({ state: "no_system" });
      return;
    }
    this.setStatus({ state: "running", trigger: reason, since: epochMs(Date.now()) });
    const child = spawn(this.options.install.cli, ["agent", "load"], {
      cwd: this.options.install.root,
      env: this.options.env,
      stdio: ["ignore", "pipe", "pipe"],
    });
    this.running = child;
    let stdout = "";
    let stderr = "";
    child.stdout?.on("data", (chunk: Buffer) => (stdout += chunk.toString()));
    child.stderr?.on("data", (chunk: Buffer) => (stderr += chunk.toString()));
    let settled = false;
    const settle = (error: string | null): void => {
      if (settled) return;
      settled = true;
      this.running = null;
      if (this.closed) return;
      if (error === null) this.exited(stdout, reason);
      else this.fail(error, reason);
      if (this.pending !== null) {
        const next = this.pending;
        this.pending = null;
        this.trigger(next);
      }
    };
    child.on("error", (error) => settle(String(error)));
    // "exit" can come before the last of stdout; "close" also follows an "error" when the spawn fails.
    child.on("close", (code, signal) =>
      settle(
        code === 0
          ? null
          : `dimos agent load exited with ${code ?? signal}: ${stderr.trim().split("\n").slice(-5).join("\n")}`,
      ),
    );
  }

  async close(): Promise<void> {
    this.closed = true;
    this.unsubscribe?.();
    this.unsubscribe = null;
    if (this.retryTimer !== null) clearTimeout(this.retryTimer);
    this.retryTimer = null;
    const child = this.running;
    if (child !== null && child.exitCode === null && child.signalCode === null) {
      await new Promise<void>((resolve) => {
        child.once("exit", () => resolve());
        child.kill("SIGTERM");
      });
    }
  }

  /** `tools/list` lists every runtime tool: after a load that exited 0, the next failure backs off from the start. */
  toolsComplete(): void {
    if (this.load.value.state === "loaded") this.attempt = 0;
  }

  /** A load that exited 0 is `loaded`, unless it was asked for missing tools and loaded nothing, which adds none. */
  private exited(stdout: string, reason: LoadTrigger): void {
    const lastLine = stdout.trim().split("\n").at(-1) ?? "";
    let result: LoadResult;
    try {
      result = loadResultDecoder.decode(JSON.parse(lastLine));
    } catch (parseError) {
      this.fail(`dimos agent load printed ${JSON.stringify(lastLine)}: ${String(parseError)}`, reason);
      return;
    }
    if (reason === "tools_missing" && result.loaded.length === 0) {
      this.fail(
        `it loaded nothing: ${result.already.join(" and ")} already run, and the runtime tools are missing`,
        reason,
      );
      return;
    }
    this.setStatus({ state: "loaded", result, at: epochMs(Date.now()) });
  }

  private fail(error: string, reason: LoadTrigger): void {
    this.attempt += 1;
    const delay = Math.min(1_000 * 2 ** this.attempt, this.options.maxBackoffMs);
    this.options.report(
      new Error(`dimos agent load (${reason}, attempt ${this.attempt}) failed; retry in ${delay} ms: ${error}`),
    );
    this.setStatus({ state: "failed", error, attempt: this.attempt, retryAt: epochMs(Date.now() + delay) });
    this.retryTimer = setTimeout(() => {
      this.retryTimer = null;
      this.trigger(reason);
    }, delay);
    this.retryTimer.unref();
  }

  private setStatus(status: LoadStatus): void {
    this.load.set(status);
  }
}

import type { AssistantMessage } from "@earendil-works/pi-ai";
import type { ChildProcess } from "node:child_process";
import { existsSync, readFileSync, renameSync, writeFileSync } from "node:fs";
import { join } from "node:path";
import { spawn } from "node:child_process";

/** A command the "serve" phase of test/agent-child.ts carries out; each becomes `cmd-N.json` and answers in `cmd-N.done.json`. */
export type ChildCommand =
  /** Appends to the faux model's queue, or replaces what is still queued when `replace` is set. */
  | { readonly op: "script"; readonly messages: readonly AssistantMessage[]; readonly replace?: boolean }
  | { readonly op: "start" }
  | { readonly op: "submit"; readonly conversation: number; readonly text: string; readonly request: string }
  | { readonly op: "events"; readonly conversation: number }
  | { readonly op: "toolResults"; readonly conversation: number }
  | { readonly op: "status" }
  | { readonly op: "close" };

const CHILD = join(import.meta.dirname, "agent-child.ts");

function sleep(ms: number): Promise<void> {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

/** One agent process in its "serve" phase: the test sends it commands and watches that it never exits on its own. */
export class ChildAgent {
  private readonly scratch: string;
  private readonly process: ChildProcess;
  private sent = 0;
  private exitCode: number | null | undefined = undefined;
  readonly exited: Promise<number | null>;

  private constructor(scratch: string, process: ChildProcess) {
    this.scratch = scratch;
    this.process = process;
    this.exited = new Promise((resolve) =>
      process.once("exit", (code, signal) => {
        this.exitCode = code ?? (signal === null ? null : -1);
        resolve(this.exitCode);
      }),
    );
  }

  static start(scratch: string, env: NodeJS.ProcessEnv): ChildAgent {
    const child = spawn(process.execPath, [CHILD], {
      env: { ...env, AGENT_CHILD_SCRATCH: scratch, AGENT_CHILD_PHASE: "serve" },
      stdio: ["ignore", "inherit", "inherit"],
    });
    return new ChildAgent(scratch, child);
  }

  /** True until the process exits for any reason. */
  get alive(): boolean {
    return this.exitCode === undefined;
  }

  /** The crash the child recorded before exiting on an unhandled rejection or exception, or null. */
  get crash(): unknown {
    const path = join(this.scratch, "crash.json");
    return existsSync(path) ? JSON.parse(readFileSync(path, "utf8")) : null;
  }

  async call<T>(command: ChildCommand, timeoutMs = 60_000): Promise<T> {
    if (!this.alive) throw new Error(`agent child exited with ${this.exitCode}; crash: ${JSON.stringify(this.crash)}`);
    this.sent += 1;
    const n = this.sent;
    // Written whole, then renamed, so the child's poll never reads a partial command.
    const path = join(this.scratch, `cmd-${n}.json`);
    writeFileSync(`${path}.tmp`, JSON.stringify(command));
    renameSync(`${path}.tmp`, path);
    const done = join(this.scratch, `cmd-${n}.done.json`);
    const deadline = Date.now() + timeoutMs;
    while (!existsSync(done)) {
      if (!this.alive)
        throw new Error(
          `agent child exited with ${this.exitCode} during ${command.op}; crash: ${JSON.stringify(this.crash)}`,
        );
      if (Date.now() > deadline) throw new Error(`agent child did not answer ${command.op} within ${timeoutMs} ms`);
      await sleep(50);
    }
    const result = JSON.parse(readFileSync(done, "utf8")) as T | { error: string };
    if (typeof result === "object" && result !== null && "error" in result)
      throw new Error(`${command.op}: ${String(result.error)}`);
    return result as T;
  }

  /** Asks the child to close its agent and exit; resolves with its exit code. */
  async close(): Promise<number | null> {
    await this.call({ op: "close" });
    return this.exited;
  }

  kill(): void {
    this.process.kill("SIGKILL");
  }
}

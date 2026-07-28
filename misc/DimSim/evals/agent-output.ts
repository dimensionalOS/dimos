import { dirname, fromFileUrl, resolve } from "@std/path";

export interface AgentOutputEvent {
  type: "agent_output";
  text: string;
  hasToolCalls: boolean;
  timestampMs: number;
}

export interface AgentOutputObserver {
  readonly failure: Promise<Error>;
  start(onOutput: (event: AgentOutputEvent) => void): Promise<void>;
  stop(): Promise<void>;
}

interface Deferred<T> {
  promise: Promise<T>;
  resolve(value: T): void;
  reject(error: unknown): void;
}

function deferred<T>(): Deferred<T> {
  let resolve!: (value: T) => void;
  let reject!: (error: unknown) => void;
  const promise = new Promise<T>((res, rej) => {
    resolve = res;
    reject = rej;
  });
  return { promise, resolve, reject };
}

export function parseAgentOutputLine(
  line: string,
): AgentOutputEvent | "ready" | null {
  let value: unknown;
  try {
    value = JSON.parse(line);
  } catch {
    return null;
  }
  if (!value || typeof value !== "object") return null;
  const event = value as Record<string, unknown>;
  if (event.type === "ready") return "ready";
  if (
    event.type !== "agent_output" ||
    typeof event.text !== "string" ||
    typeof event.hasToolCalls !== "boolean" ||
    typeof event.timestampMs !== "number" ||
    !Number.isFinite(event.timestampMs)
  ) {
    return null;
  }
  return event as unknown as AgentOutputEvent;
}

const EVALS_DIR = dirname(fromFileUrl(import.meta.url));
const REPO_ROOT = resolve(EVALS_DIR, "../../..");

export class ProcessAgentOutputObserver implements AgentOutputObserver {
  private child: Deno.ChildProcess | null = null;
  private stopping = false;
  private ready = false;
  private readonly readyState = deferred<void>();
  private readonly failureState = deferred<Error>();

  readonly failure = this.failureState.promise;

  async start(onOutput: (event: AgentOutputEvent) => void): Promise<void> {
    if (this.child) throw new Error("agent output observer already started");
    try {
      this.child = new Deno.Command("uv", {
        args: [
          "run",
          "--project",
          REPO_ROOT,
          "python",
          "-m",
          "dimos.simulation.dimsim.agent_output_sidecar",
        ],
        cwd: REPO_ROOT,
        stdin: "null",
        stdout: "piped",
        stderr: "inherit",
      }).spawn();
    } catch (error) {
      const wrapped = new Error(
        `failed to start agent output sidecar: ${
          error instanceof Error ? error.message : String(error)
        }`,
      );
      this.readyState.reject(wrapped);
      this.failureState.resolve(wrapped);
      throw wrapped;
    }

    void this.readOutput(onOutput);
    void this.watchStatus();
    await this.readyState.promise;
  }

  private async readOutput(
    onOutput: (event: AgentOutputEvent) => void,
  ): Promise<void> {
    const child = this.child;
    if (!child) return;
    try {
      const reader = child.stdout.pipeThrough(new TextDecoderStream())
        .getReader();
      let buffered = "";
      while (true) {
        const { value, done } = await reader.read();
        if (done) break;
        buffered += value;
        let newline = buffered.indexOf("\n");
        while (newline >= 0) {
          const line = buffered.slice(0, newline);
          buffered = buffered.slice(newline + 1);
          this.handleLine(line, onOutput);
          newline = buffered.indexOf("\n");
        }
      }
      if (buffered) {
        this.handleLine(buffered, onOutput);
      }
    } catch (error) {
      if (!this.stopping) {
        this.fail(
          new Error(
            `agent output sidecar stream failed: ${
              error instanceof Error ? error.message : String(error)
            }`,
          ),
        );
      }
    }
  }

  private handleLine(
    line: string,
    onOutput: (event: AgentOutputEvent) => void,
  ): void {
    const event = parseAgentOutputLine(line);
    if (event === "ready") {
      if (!this.ready) {
        this.ready = true;
        this.readyState.resolve();
      }
    } else if (event) {
      onOutput(event);
    }
  }

  private async watchStatus(): Promise<void> {
    const child = this.child;
    if (!child) return;
    const status = await child.status;
    if (!this.stopping) {
      this.fail(
        new Error(
          `agent output sidecar exited unexpectedly with code ${status.code}`,
        ),
      );
    }
  }

  private fail(error: Error): void {
    if (!this.ready) this.readyState.reject(error);
    this.failureState.resolve(error);
  }

  async stop(): Promise<void> {
    const child = this.child;
    if (!child) return;
    this.stopping = true;
    try {
      child.kill("SIGTERM");
    } catch {
      // It may have already exited.
    }
    const graceful = await Promise.race([
      child.status.then(() => true),
      new Promise<false>((resolve) => setTimeout(() => resolve(false), 2_000)),
    ]);
    if (!graceful) {
      try {
        child.kill("SIGKILL");
      } catch {
        // It may have exited during the grace interval.
      }
      await child.status.catch(() => {});
    }
    this.child = null;
  }
}

export function createAgentOutputObserver(): AgentOutputObserver {
  return new ProcessAgentOutputObserver();
}

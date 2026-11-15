import type { ChildProcess } from "node:child_process";
import type { DimosInstall } from "#dimos/agents/harness/dimos/install.ts";
import { after, before } from "node:test";
import { createServer } from "node:net";
import { createSocket } from "node:dgram";
import { existsSync } from "node:fs";
import { join, resolve } from "node:path";
import { mkdir, mkdtemp, rm } from "node:fs/promises";
import { parseDimosInstall } from "#dimos/agents/gateway/config/environment.ts";
import { spawn } from "node:child_process";
import { tmpdir } from "node:os";

/** A DimOS system in its own process: HarnessProbe, McpServer and HarnessRuntime on a private bus. */
type TestDimos = {
  /** What `dimos agent gateway` would pass a gateway for this system: the checkout, its .venv, its McpServer's URL, the scratch runs directory. */
  readonly install: DimosInstall;
  /**
   * The environment a gateway process under test runs with: the DIMOS_* variables `parseDimosInstall` reads, and the
   * system's bus, McpServer endpoint (MCPSERVER__HOST, MCPSERVER__PORT) and state home, so the gateway's
   * `dimos agent load` reaches this system and no other.
   */
  readonly env: NodeJS.ProcessEnv;
  /** Stops the process (SIGINT, SIGKILL after 30 s): no McpServer answers until `start()`. */
  stop(): Promise<void>;
  /** Starts the system again without HarnessRuntime, which the gateway must load back. */
  start(): Promise<void>;
  /** `stop()` then `start()`. */
  restart(): Promise<void>;
};

const SERVE = "from dimos.agents.harness.runtime.test_runtime import serve_system; serve_system()";

function sleep(ms: number): Promise<void> {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

async function until<T>(check: () => Promise<T | null>, label: string, timeoutMs: number): Promise<T> {
  const deadline = Date.now() + timeoutMs;
  for (;;) {
    const value = await check();
    if (value !== null) return value;
    if (Date.now() > deadline) throw new Error(`timed out after ${timeoutMs} ms waiting for ${label}`);
    await sleep(250);
  }
}

function freeTcpPort(): Promise<number> {
  return new Promise((resolve, reject) => {
    const server = createServer();
    server.listen(0, "127.0.0.1", () => {
      const address = server.address();
      server.close(() =>
        typeof address === "object" && address !== null ? resolve(address.port) : reject(new Error("no port")),
      );
    });
  });
}

function freeUdpPort(): Promise<number> {
  return new Promise((resolve) => {
    const socket = createSocket("udp4");
    socket.bind(0, "127.0.0.1", () => {
      const { port } = socket.address();
      socket.close(() => resolve(port));
    });
  });
}

/** One JSON-RPC request to McpServer over plain HTTP; the current server needs no initialize or session. */
async function rpc(url: string, method: string, params: unknown): Promise<unknown> {
  const response = await fetch(url, {
    method: "POST",
    headers: { "content-type": "application/json", accept: "application/json, text/event-stream" },
    body: JSON.stringify({ jsonrpc: "2.0", id: 1, method, params }),
    signal: AbortSignal.timeout(10_000),
  });
  const body = (await response.json()) as { result?: unknown; error?: unknown };
  if (body.error !== undefined) throw new Error(`${method}: ${JSON.stringify(body.error)}`);
  return body.result;
}

function run(
  command: string,
  args: readonly string[],
  cwd: string,
  env: NodeJS.ProcessEnv,
  timeoutMs: number,
): Promise<string> {
  return new Promise((resolve, reject) => {
    const child = spawn(command, args, { cwd, env, stdio: ["ignore", "pipe", "pipe"] });
    let stdout = "";
    let stderr = "";
    child.stdout.on("data", (chunk: Buffer) => (stdout += chunk.toString()));
    child.stderr.on("data", (chunk: Buffer) => (stderr += chunk.toString()));
    const timer = setTimeout(() => child.kill("SIGKILL"), timeoutMs);
    child.on("error", (error) => {
      clearTimeout(timer);
      reject(error);
    });
    child.on("exit", (code, signal) => {
      clearTimeout(timer);
      if (code === 0) resolve(stdout);
      else
        reject(
          new Error(
            `${command} ${args.join(" ")} exited with ${code ?? signal}: ${stderr.trim().split("\n").slice(-10).join("\n")}`,
          ),
        );
    });
  });
}

/**
 * Registers node:test `before` and `after` hooks in the calling test file. `before` starts the system as
 * dimos/agents/harness/runtime/test_runtime.py does, its `serve_system()` in a .venv Python subprocess, but on zenoh
 * with a private scouting group (LCM needs multicast on `lo`, which this host lacks and only sudo can add), McpServer
 * on 127.0.0.1 at a free MCPSERVER__PORT, XDG_STATE_HOME in a scratch directory and N_WORKERS=1. It pins zenoh in peer
 * mode and empties ROBOT_IP, ROBOT_IPS and ZENOH_CONNECT, so no session dials a robot or a router: `dimos agent load`
 * loads the checkout's `.env` from any working directory, the checkout's names a real robot, and only a variable
 * already set wins over it. DIMOS_ROOT is an empty scratch directory. It waits until McpServer answers, runs
 * `dimos agent load`, and waits until `tools/list` has `inventory`. `after` stops the process and removes the scratch
 * directory.
 * The returned function gives the running system, or null when the checkout has no .venv, so the test skips.
 */
export function dimosFixture(): () => TestDimos | null {
  let system: TestDimos | null = null;
  let child: ChildProcess | null = null;
  let scratch = "";
  let env: NodeJS.ProcessEnv = {};
  let python = "";
  let mcpUrl = "";

  const stop = async (): Promise<void> => {
    const running = child;
    child = null;
    if (running === null || running.exitCode !== null) return;
    await new Promise<void>((resolve) => {
      const timer = setTimeout(() => running.kill("SIGKILL"), 30_000);
      running.once("exit", () => {
        clearTimeout(timer);
        resolve();
      });
      running.kill("SIGINT");
    });
  };

  const answers = async (): Promise<true | null> => {
    try {
      const result = (await rpc(mcpUrl, "tools/call", { name: "echo", arguments: { text: "up" } })) as {
        content?: { text?: string }[];
      };
      return result.content?.[0]?.text === "up" ? true : null;
    } catch {
      return null;
    }
  };

  const start = async (): Promise<void> => {
    child = spawn(python, ["-c", SERVE], { cwd: scratch, env, stdio: ["ignore", "inherit", "inherit"] });
    await until(answers, "McpServer to answer", 60_000);
  };

  before(
    async () => {
      const checkout = resolve(import.meta.dirname, "..", "..", "..");
      python = join(checkout, ".venv", "bin", "python");
      if (!existsSync(python)) return;
      scratch = await mkdtemp(join(tmpdir(), "dimos-agent-test-"));
      const mcpPort = await freeTcpPort();
      const scoutPort = await freeUdpPort();
      mcpUrl = `http://127.0.0.1:${mcpPort}/mcp`;
      env = {
        ...process.env,
        DIMOS_TRANSPORT: "zenoh",
        ZENOH_MODE: "peer",
        ZENOH_SCOUT_ADDR: `224.0.0.231:${scoutPort}`,
        ZENOH_CONNECT: "",
        MCPSERVER__HOST: "127.0.0.1",
        MCPSERVER__PORT: String(mcpPort),
        RELAYBRIDGEMODULE__LOCAL_PORT: "0",
        RELAYBRIDGEMODULE__WEB_BUILD: "false",
        RELAYBRIDGEMODULE__OPEN_BROWSER: "false",
        RELAYBRIDGEMODULE__RTC: "false",
        XDG_STATE_HOME: join(scratch, "state"),
        XDG_CONFIG_HOME: join(scratch, "config"),
        XDG_CACHE_HOME: join(scratch, "cache"),
        XDG_RUNTIME_DIR: join(scratch, "runtime"),
        ROBOT_IP: "",
        ROBOT_IPS: "",
        N_WORKERS: "1",
        DIMOS_ROOT: join(scratch, "root"),
        DIMOS_PYTHON: python,
        DIMOS_CLI: join(checkout, ".venv", "bin", "dimos"),
        DIMOS_MCP_URL: mcpUrl,
        DIMOS_RUNS_DIR: join(scratch, "state", "dimos", "runs"),
        DIMOS_STATE_DIR: join(scratch, "state", "dimos"),
      };
      await mkdir(join(scratch, "runtime"), { recursive: true, mode: 0o700 });
      await mkdir(join(scratch, "root"));
      await start();
      const loaded = await run(env["DIMOS_CLI"] ?? "", ["agent", "load"], scratch, env, 120_000);
      if (!loaded.includes("HarnessRuntime")) throw new Error(`dimos agent load printed ${JSON.stringify(loaded)}`);
      await until(
        async () => {
          const result = (await rpc(mcpUrl, "tools/list", {})) as { tools?: { name: string }[] };
          return result.tools?.some((tool) => tool.name === "inventory") ? true : null;
        },
        "the harness tools",
        20_000,
      );
      system = {
        install: parseDimosInstall(env),
        env,
        stop,
        start,
        restart: async () => {
          await stop();
          await start();
        },
      };
    },
    { timeout: 300_000 },
  );

  after(
    async () => {
      await stop();
      if (scratch !== "") await rm(scratch, { recursive: true, force: true });
    },
    { timeout: 60_000 },
  );

  return () => system;
}

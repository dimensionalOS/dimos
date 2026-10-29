import type { AgentPaths } from "../gateway/config/paths.ts";
import type { ChildProcess } from "node:child_process";
import type { GatewayProcess } from "../shared/protocol/lifecycle/contract.ts";
import type { Client } from "../sdk/main.ts";
import type { Ref } from "@earendil-works/chord";
import { LifecycleContract } from "../shared/protocol/lifecycle/contract.ts";
import { owner, refusalOfFailure } from "../sdk/failures.ts";
import { buildInfo } from "../gateway/config/build.ts";
import { connect } from "../sdk/main.ts";
import { unixSocket } from "../sdk/node.ts";
import { dirname } from "node:path";
import { epochMs } from "../shared/protocol/time.ts";
import { mkdir, rm, rmdir, stat, unlink } from "node:fs/promises";
import { reachable } from "../shared/socket.ts";
import { refusalText } from "../shared/view-models/outcome.ts";
import { setTimeout as delay } from "node:timers/promises";
import { spawn } from "node:child_process";

/** The gateway as seen from outside it, through the frozen Lifecycle object, so any build answers. */
export type LaunchState =
  | { readonly kind: "running"; readonly process: GatewayProcess }
  /** A gateway of another build. Its state values carry no schema on the wire, so a shape change would go unnoticed. */
  | { readonly kind: "other_build"; readonly process: GatewayProcess }
  | { readonly kind: "stopped" }
  /** No socket yet, but `ensureGateway` holds a fresh `<socket>.starting`: a gateway is loading its packages. */
  | { readonly kind: "starting" }
  /** A socket file with nothing answering, as a killed gateway leaves; `status` reports it instead of hanging. */
  | { readonly kind: "stale_socket" }
  /** A process accepts connections on the socket but held no Lifecycle in time: a gateway still opening its store, or a hung one. */
  | { readonly kind: "not_answering" };

const START_TIMEOUT_MS = 60_000;

const START_POLL_MS = 50;

const LOCK_STALE_MS = 30_000;

const EXIT_WAIT_MS = 30_000;

async function exists(path: string): Promise<boolean> {
  return stat(path).then(
    () => true,
    () => false,
  );
}

/** The directory `ensureGateway` holds from before it spawns a gateway until that gateway serves. */
function startLock(paths: AgentPaths): string {
  return `${paths.socket}.starting`;
}

/** Whether `ensureGateway` holds the start lock; one older than 30 s was left by a starter that died. */
async function startLockHeld(lock: string): Promise<boolean> {
  return stat(lock).then(
    (info) => Date.now() - info.mtimeMs <= LOCK_STALE_MS,
    () => false,
  );
}

function alive(pid: number): boolean {
  try {
    process.kill(pid, 0);
    return true;
  } catch (error) {
    return error instanceof Error && "code" in error && error.code === "EPERM";
  }
}

/**
 * Whether a WebSocket upgrade on the socket completes within `timeoutMs`. The client's redial never gives up on a
 * socket that accepts and never answers, or refuses the upgrade while the gateway is still opening its store, so the
 * upgrade is probed with a bounded wait before the client is started.
 */
function upgradeAnswers(socketPath: string, timeoutMs: number): Promise<boolean> {
  return new Promise((resolve) => {
    const socket = unixSocket({ path: socketPath, label: "status" })();
    const timer = setTimeout(() => finish(false), timeoutMs);
    socket.addEventListener("open", () => finish(true));
    socket.addEventListener("error", () => finish(false));
    socket.addEventListener("close", () => finish(false));
    function finish(answer: boolean): void {
      clearTimeout(timer);
      socket.close();
      resolve(answer);
    }
  });
}

type LifecycleAccess = { readonly client: Client; readonly ref: Ref<typeof LifecycleContract> };

/** The Lifecycle object, or null when the gateway does not answer within `timeoutMs`. */
async function openLifecycle(paths: AgentPaths, timeoutMs: number): Promise<LifecycleAccess | null> {
  if (!(await upgradeAnswers(paths.socket, timeoutMs))) return null;
  const client = await connect(unixSocket({ path: paths.socket, label: "status" }), {
    backoffMs: { min: 100, max: 500 },
    readyTimeoutMs: timeoutMs,
    now: () => epochMs(Date.now()),
  }).catch(() => null);
  if (client === null) return null;
  try {
    const ref = await Promise.race([client.host.get(LifecycleContract, {}), client.ended]);
    return { client, ref };
  } catch {
    await client.close();
    return null;
  }
}

/** The gateway as seen from outside it, with a bounded connect so a stale socket never hangs the CLI. */
export async function gatewayState(paths: AgentPaths, timeoutMs: number): Promise<LaunchState> {
  if (!(await exists(paths.socket)))
    return (await startLockHeld(startLock(paths))) ? { kind: "starting" } : { kind: "stopped" };
  if (!(await reachable(paths.socket, timeoutMs))) return { kind: "stale_socket" };
  const lifecycle = await openLifecycle(paths, timeoutMs);
  if (lifecycle === null) return { kind: "not_answering" };
  try {
    const running = lifecycle.ref.process.value;
    return { kind: running.build.build === buildInfo().build ? "running" : "other_build", process: running };
  } finally {
    lifecycle.ref.release();
    await lifecycle.client.close();
  }
}

/**
 * Polls until the gateway serves its Lifecycle object, since a gateway binds the socket before it opens its store and
 * exits when opening fails. Throws the restart hint for another build, and `failure` after 60 s, once `spawned` has
 * exited, or once a socket that accepted connections is gone.
 */
async function waitUntilServing(paths: AgentPaths, spawned: ChildProcess | null, failure: string): Promise<void> {
  const deadline = Date.now() + START_TIMEOUT_MS;
  let accepted = false;
  for (;;) {
    const state = await gatewayState(paths, 500);
    switch (state.kind) {
      case "running":
        return;
      case "other_build":
        throw new Error(refusalText({ code: "other_build", gatewayBuild: state.process.build.build }));
      case "not_answering":
        accepted = true;
        break;
      case "stopped":
      case "starting":
      case "stale_socket":
        if (accepted) throw new Error(failure);
    }
    const exited = spawned !== null && (spawned.exitCode !== null || spawned.signalCode !== null);
    if (exited || Date.now() > deadline) throw new Error(failure);
    await delay(START_POLL_MS);
  }
}

/**
 * Starts the gateway detached when none answers, under a `<socket>.starting` lock directory (older than 30 s: removed),
 * then waits until it serves. A gateway that already answers must be this build. `command` is how this process
 * re-runs itself, e.g. `[node, dist/cli.js, "gateway"]`.
 */
export async function ensureGateway(
  paths: AgentPaths,
  command: readonly string[],
  env: NodeJS.ProcessEnv,
): Promise<void> {
  const held = `The gateway on ${paths.socket} did not answer; read ${paths.logFile}, or find what holds the socket with \`ss -xlp\`.`;
  if (await reachable(paths.socket, 500)) return waitUntilServing(paths, null, held);
  const lock = startLock(paths);
  const lockAge = await stat(lock).then(
    (info) => Date.now() - info.mtimeMs,
    () => null,
  );
  if (lockAge !== null && lockAge > LOCK_STALE_MS) await rm(lock, { recursive: true, force: true });
  await mkdir(dirname(paths.socket), { recursive: true, mode: 0o700 });
  try {
    await mkdir(lock, { mode: 0o700 });
  } catch (error) {
    if (!(error instanceof Error && "code" in error && error.code === "EEXIST")) throw error;
    return waitUntilServing(
      paths,
      null,
      `Another terminal is starting the gateway and it did not answer; inspect ${lock} and ${paths.logFile}.`,
    );
  }
  try {
    if (await reachable(paths.socket, 500)) return await waitUntilServing(paths, null, held);
    const [executable, ...args] = command;
    if (executable === undefined) throw new Error("No command to start the gateway with");
    const gateway = spawn(executable, args, { detached: true, stdio: "ignore", env });
    gateway.unref();
    await waitUntilServing(
      paths,
      gateway,
      `The gateway did not start; run \`dimos agent gateway\` to see why, or read ${paths.logFile}.`,
    );
  } finally {
    await rmdir(lock).catch(() => {});
  }
}

/** `lifecycle.stop` over the socket, then waits for the gateway process to exit. A stale socket file is removed and reported as not running. */
export async function stopGateway(paths: AgentPaths): Promise<"stopped" | "not_running"> {
  const state = await gatewayState(paths, 500);
  switch (state.kind) {
    case "stopped":
      return "not_running";
    case "starting":
      throw new Error(`A gateway is starting (${startLock(paths)}); run \`dimos agent stop\` again once it answers.`);
    case "stale_socket":
      await unlink(paths.socket).catch(() => {});
      return "not_running";
    case "not_answering":
      throw new Error(`A process holds ${paths.socket} but does not answer; \`ss -xlp\` names it.`);
    case "running":
    case "other_build": {
      const lifecycle = await openLifecycle(paths, 5_000);
      if (lifecycle === null) throw new Error(`The gateway on ${paths.socket} stopped answering.`);
      try {
        let refusal;
        try {
          const stopped = await Promise.race([owner(lifecycle.ref).stop({}), lifecycle.client.ended]);
          refusal = stopped.ok ? null : stopped.refusal;
        } catch (error) {
          refusal = refusalOfFailure(error, epochMs(Date.now()));
        }
        // A link that dropped mid-call, or closed with `stopping`, is the gateway going away: the pid wait below tells.
        if (refusal !== null && refusal.code !== "unknown_outcome" && refusal.code !== "stopping")
          throw new Error(`The gateway refused to stop: ${refusal.code}`);
      } finally {
        lifecycle.ref.release();
        await lifecycle.client.close();
      }
      const deadline = Date.now() + EXIT_WAIT_MS;
      while (alive(state.process.pid)) {
        if (Date.now() > deadline)
          throw new Error(`The gateway (pid ${state.process.pid}) did not exit within ${EXIT_WAIT_MS / 1000} s.`);
        await delay(100);
      }
      return "stopped";
    }
  }
}

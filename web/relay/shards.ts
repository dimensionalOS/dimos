// Sharding: one relay process per UDP port (Deno Workers cannot share a
// QUIC endpoint and Deno.QuicEndpoint has no reusePort), with the shard
// chosen at discovery so a robot and its viewers land on the same process:
// GET /api/info?robot=<id> answers with that robot's shard. The process
// started with --shards N is shard 0; it spawns shards 1..N-1 as children
// of itself (same arguments, --port 0, no --shards) and answers discovery
// for all of them from their ready lines. A client that names no robot
// stays on shard 0.
import { PROTOCOL_VERSION } from "@dimos/shared";

export interface ShardInfo {
  httpPort: number;
  wtUrl: string;
  certHash?: string;
  instance?: string;
}

/** The shard (0..n-1) a robot id lands on: FNV-1a over the id's bytes. */
export function shardFor(robotId: string, n: number): number {
  let h = 0x811c9dc5;
  for (const byte of new TextEncoder().encode(robotId)) {
    h = Math.imul(h ^ byte, 0x01000193) >>> 0;
  }
  return h % n;
}

/** `args` without the flag `name` (both `--name v` and `--name=v`). */
export function withoutFlag(args: string[], name: string): string[] {
  const out: string[] = [];
  for (let i = 0; i < args.length; i++) {
    if (args[i] === `--${name}`) {
      i++; // its value
      continue;
    }
    if (args[i].startsWith(`--${name}=`)) continue;
    out.push(args[i]);
  }
  return out;
}

export interface SpawnedShards {
  infos: ShardInfo[];
  kill(): void;
}

// What a child relay's environment keeps: where deno's module cache lives
// and how it prints. Everything else is dropped, loader variables above all
// (LD_LIBRARY_PATH, LD_PRELOAD), which Deno refuses to pass through a
// scoped --allow-run; reading these needs --allow-env for their names.
const CHILD_ENV_NAMES = [
  "HOME",
  "DENO_DIR",
  "XDG_CACHE_HOME",
  "PATH",
  "NO_COLOR",
  "TMPDIR",
  "GLIBC_TUNABLES", // the launcher's malloc trimming (relay_process.py)
];

function childEnv(): Record<string, string> {
  const env: Record<string, string> = {};
  for (const name of CHILD_ENV_NAMES) {
    try {
      const value = Deno.env.get(name);
      if (value !== undefined) env[name] = value;
    } catch {
      // not granted: the child does without it
    }
  }
  return env;
}

/** Spawn `count` child relays (shards 1..count) with `childArgs` plus
 * `--port 0`, reading each one's ready line; `readScopes` are the paths the
 * children may read besides the web directory (the served roots, the
 * certificate, the auth and rtc files). */
export async function spawnShards(
  count: number,
  childArgs: string[],
  readScopes: string[],
): Promise<SpawnedShards> {
  const webDir = new URL("../", import.meta.url);
  const children: Deno.ChildProcess[] = [];
  const infos: ShardInfo[] = [];
  const kill = () => {
    for (const child of children) {
      try {
        child.kill("SIGTERM");
      } catch {
        // already gone
      }
    }
  };
  try {
    for (let k = 1; k <= count; k++) {
      const child = new Deno.Command(Deno.execPath(), {
        args: [
          "run",
          "--v8-flags=--expose-gc",
          "--frozen",
          "--node-modules-dir=none",
          `--allow-read=${[webDir.pathname, ...readScopes].join(",")}`,
          "--allow-net",
          "--config",
          new URL("deno.json", webDir).pathname,
          new URL("relay/main.ts", webDir).pathname,
          ...childArgs,
          "--port",
          "0",
        ],
        clearEnv: true,
        env: childEnv(),
        stdout: "piped",
        stderr: "inherit",
      }).spawn();
      children.push(child);
      infos.push(await readyLine(child, k));
    }
  } catch (e) {
    kill();
    throw e;
  }
  return { infos, kill };
}

/** The child's JSON ready line (its first stdout line); the rest of its
 * stdout is relayed here prefixed with the shard number. */
async function readyLine(child: Deno.ChildProcess, k: number): Promise<ShardInfo> {
  const reader = child.stdout.getReader();
  const decoder = new TextDecoder();
  let buffer = "";
  let ready: ShardInfo | null = null;
  const relay = (line: string) => {
    if (line !== "") console.log(`[shard ${k}] ${line}`);
  };
  while (ready === null) {
    const { value, done } = await reader.read();
    if (done) throw new Error(`shard ${k} exited before its ready line`);
    buffer += decoder.decode(value, { stream: true });
    const nl = buffer.indexOf("\n");
    if (nl === -1) continue;
    const line = buffer.slice(0, nl);
    buffer = buffer.slice(nl + 1);
    const parsed = JSON.parse(line) as Record<string, unknown>;
    if (
      parsed.event !== "ready" || typeof parsed.httpPort !== "number" ||
      typeof parsed.wtUrl !== "string" || parsed.v !== PROTOCOL_VERSION
    ) {
      throw new Error(`shard ${k} printed an unexpected ready line: ${line}`);
    }
    ready = {
      httpPort: parsed.httpPort,
      wtUrl: parsed.wtUrl,
      ...(typeof parsed.certHash === "string" ? { certHash: parsed.certHash } : {}),
      ...(typeof parsed.instance === "string" ? { instance: parsed.instance } : {}),
    };
  }
  (async () => {
    for (const line of buffer.split("\n")) relay(line);
    while (true) {
      const { value, done } = await reader.read();
      if (done) break;
      for (const line of decoder.decode(value, { stream: true }).split("\n")) relay(line);
    }
    console.log(`[relay] shard ${k} exited`);
  })().catch(() => {});
  return ready;
}

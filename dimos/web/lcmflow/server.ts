#!/usr/bin/env -S deno run --allow-net --allow-read --allow-env --unstable-net
// Copyright 2025-2026 Dimensional Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// lcmflow dashboard bridge.
//
// Subscribes to every LCM channel and forwards *metadata only*
// ({channel, count, bytes} batched every 50 ms) to browser clients over
// WebSocket — payloads never leave this process, so a 40 MiB/s image
// stream costs the browser a few hundred bytes per second.
//
// Module topology (which module reads/writes which topic, over which
// transport) is recovered from the running DimOS instance's structured
// log: the module coordinator emits one "Transport" event per stream
// binding at startup.
//
//   deno run --allow-net --allow-read --allow-env --unstable-net server.ts
//   dimos lcmflow dashboard          # same, via the dimos CLI

import { LCM } from "jsr:@dimos/lcm@^0.2.0";

const args = new Map<string, string>();
for (let i = 0; i < Deno.args.length - 1; i++) {
  if (Deno.args[i].startsWith("--")) args.set(Deno.args[i].slice(2), Deno.args[i + 1]);
}
const PORT = Number(args.get("port") ?? 8090);
const LOG_OVERRIDE = args.get("log"); // explicit path to a main.jsonl
const PYTHON = args.get("python"); // dimos venv python, for the direction sidecar

const BATCH_MS = 50;
const TOPOLOGY_RESCAN_MS = 5000;

// ---------------------------------------------------------------------------
// Topology: parse "Transport" events from the newest DimOS run log.
// ---------------------------------------------------------------------------

interface Edge {
  module: string;
  name: string; // original (pre-remap) stream name, for direction lookup
  topic: string;
  type: string;
  transport: string;
  direction: string; // "in" | "out" | "" (filled from the Python sidecar)
}

interface Topology {
  source: string;
  edges: Edge[];
}

let topology: Topology = { source: "", edges: [] };

// "<ModuleClass>|<stream_name>" -> "in" | "out", from the Python sidecar.
// The coordinator log records who talks to which topic but not direction;
// dimos.utils.cli.lcmflow.topology recovers it from the blueprint's
// In[]/Out[] declarations (it runs in the dimos venv; we don't).
let directions = new Map<string, string>();
let directionsSource = "";

async function refreshDirections(source: string): Promise<void> {
  if (!PYTHON || source === directionsSource) return;
  directionsSource = source; // mark attempted even on failure (avoid retry storms)
  try {
    const out = await new Deno.Command(PYTHON, {
      args: ["-m", "dimos.utils.cli.lcmflow.topology"],
      stdout: "piped",
      stderr: "null",
    }).output();
    const obj = JSON.parse(new TextDecoder().decode(out.stdout));
    directions = new Map(Object.entries(obj));
  } catch {
    // no python / import failure — dashboard falls back to undirected edges
  }
}

function candidateLogDirs(): string[] {
  const home = Deno.env.get("HOME") ?? "";
  return [
    `${Deno.cwd()}/logs`,
    `${home}/.local/state/dimos/logs`,
  ];
}

/** Log paths of registered (running) DimOS instances, via the run registry. */
function registryLogs(): { path: string; mtime: number }[] {
  const home = Deno.env.get("HOME") ?? "";
  const found: { path: string; mtime: number }[] = [];
  let entries: Iterable<Deno.DirEntry>;
  try {
    entries = Deno.readDirSync(`${home}/.local/state/dimos/runs`);
  } catch {
    return found;
  }
  for (const entry of entries) {
    if (!entry.name.endsWith(".json")) continue;
    try {
      const rec = JSON.parse(Deno.readTextFileSync(`${home}/.local/state/dimos/runs/${entry.name}`));
      if (!rec.log_dir) continue;
      const candidate = `${rec.log_dir}/main.jsonl`;
      const stat = Deno.statSync(candidate);
      found.push({ path: candidate, mtime: stat.mtime?.getTime() ?? 0 });
    } catch {
      // stale registry entry or unreadable log
    }
  }
  return found;
}

/** Newest run log: prefer the run registry (authoritative, cwd-independent),
 *  fall back to scanning known log roots. */
function findNewestRunLog(): string | null {
  if (LOG_OVERRIDE) return LOG_OVERRIDE;
  let newest: { path: string; mtime: number } | null = null;
  for (const c of registryLogs()) {
    if (!newest || c.mtime > newest.mtime) newest = c;
  }
  if (newest) return newest.path;
  for (const root of candidateLogDirs()) {
    let entries: Iterable<Deno.DirEntry>;
    try {
      entries = Deno.readDirSync(root);
    } catch {
      continue;
    }
    for (const entry of entries) {
      if (!entry.isDirectory) continue;
      const candidate = `${root}/${entry.name}/main.jsonl`;
      try {
        const stat = Deno.statSync(candidate);
        const mtime = stat.mtime?.getTime() ?? 0;
        if (!newest || mtime > newest.mtime) newest = { path: candidate, mtime };
      } catch {
        // no main.jsonl in this run dir
      }
    }
  }
  return newest?.path ?? null;
}

function parseTopology(path: string): Edge[] {
  const edges: Edge[] = [];
  const seen = new Set<string>();
  let text: string;
  try {
    text = Deno.readTextFileSync(path);
  } catch {
    return edges;
  }
  for (const line of text.split("\n")) {
    if (!line.includes('"Transport"')) continue;
    try {
      const rec = JSON.parse(line);
      if (rec.event !== "Transport" || !rec.module) continue;
      const name = rec.original_name ?? rec.name ?? "";
      const edge: Edge = {
        module: rec.module,
        name,
        topic: rec.topic ?? rec.name ?? "",
        type: rec.type ?? "",
        transport: rec.transport ?? "",
        direction: directions.get(`${rec.module}|${name}`) ?? "",
      };
      const key = `${edge.module}|${edge.topic}|${edge.direction}`;
      if (!seen.has(key)) {
        seen.add(key);
        edges.push(edge);
      }
    } catch {
      // not JSON / partial line
    }
  }
  return edges;
}

async function refreshTopology(): Promise<boolean> {
  const path = findNewestRunLog();
  if (!path) return false;
  // Load stream directions for this run before parsing (no-op if unchanged).
  await refreshDirections(path);
  const edges = parseTopology(path);
  if (edges.length === 0 && topology.edges.length > 0) return false;
  const changed =
    path !== topology.source || JSON.stringify(edges) !== JSON.stringify(topology.edges);
  topology = { source: path, edges };
  return changed;
}

// ---------------------------------------------------------------------------
// WebSocket fan-out
// ---------------------------------------------------------------------------

const clients = new Set<WebSocket>();

function broadcast(payload: string) {
  for (const client of clients) {
    if (client.readyState === WebSocket.OPEN) client.send(payload);
  }
}

function topologyMessage(): string {
  return JSON.stringify({ kind: "topology", ...topology });
}

// ---------------------------------------------------------------------------
// LCM: metadata-only packet feed, batched
// ---------------------------------------------------------------------------

// channel -> [count, bytes] accumulated since the last flush
const pending = new Map<string, [number, number]>();
let packetsSeen = 0;
const channelsSeen = new Set<string>();

function note(channel: string, bytes: number) {
  packetsSeen++;
  channelsSeen.add(channel);
  const entry = pending.get(channel);
  if (entry) {
    entry[0] += 1;
    entry[1] += bytes;
  } else {
    pending.set(channel, [1, bytes]);
  }
}

// Subscribe to every channel via the dimos LCM client. subscribeRaw fires
// once per fully-reassembled message with the channel and raw bytes — we
// read only the length, so payloads are never decoded.
//
// NB: the wildcard is glob-style ("*"), not the regex ".*" — the latter
// compiles to /^\..*$/ and matches nothing (channels start with "/").
async function sniff() {
  const lcm = new LCM();
  await lcm.start();
  lcm.subscribeRaw("*", (msg: { channel: string; data: Uint8Array }) => {
    note(msg.channel, msg.data.byteLength);
  });
  console.log("subscribed to all LCM channels via @dimos/lcm");
  await lcm.run();
}

setInterval(() => {
  if (pending.size === 0 || clients.size === 0) {
    pending.clear();
    return;
  }
  const events: [string, number, number][] = [];
  for (const [channel, [count, bytes]] of pending) events.push([channel, count, bytes]);
  pending.clear();
  broadcast(JSON.stringify({ kind: "packets", t: Date.now(), events }));
}, BATCH_MS);

await refreshTopology();
setInterval(async () => {
  if (await refreshTopology()) broadcast(topologyMessage());
}, TOPOLOGY_RESCAN_MS);

// ---------------------------------------------------------------------------
// HTTP
// ---------------------------------------------------------------------------

/** First free TCP port in [start, start+20) — lets several dashboards coexist. */
function pickPort(start: number): number {
  for (let p = start; p < start + 20; p++) {
    try {
      const probe = Deno.listen({ port: p });
      probe.close();
      return p;
    } catch {
      // busy, try the next one
    }
  }
  console.error(`lcmflow dashboard: ports ${start}-${start + 19} are all in use`);
  Deno.exit(1);
}

const BOUND_PORT = pickPort(PORT);
if (BOUND_PORT !== PORT) {
  console.log(`port ${PORT} busy — using ${BOUND_PORT}`);
}

Deno.serve({ port: BOUND_PORT }, async (req) => {
  const url = new URL(req.url);

  if (req.headers.get("upgrade") === "websocket") {
    const { socket, response } = Deno.upgradeWebSocket(req);
    socket.onopen = () => {
      clients.add(socket);
      socket.send(topologyMessage());
    };
    socket.onclose = () => clients.delete(socket);
    socket.onerror = () => clients.delete(socket);
    return response;
  }

  if (url.pathname === "/topology") {
    return new Response(topologyMessage(), {
      headers: { "content-type": "application/json" },
    });
  }

  if (url.pathname === "/stats") {
    return new Response(
      JSON.stringify({
        clients: clients.size,
        packetsSeen,
        channelsSeen: channelsSeen.size,
        topologySource: topology.source,
        topologyEdges: topology.edges.length,
      }),
      { headers: { "content-type": "application/json" } },
    );
  }

  if (url.pathname === "/" || url.pathname === "/index.html") {
    const html = await Deno.readTextFile(new URL("./index.html", import.meta.url));
    return new Response(html, { headers: { "content-type": "text/html" } });
  }

  return new Response("Not found", { status: 404 });
});

console.log(`lcmflow dashboard: http://localhost:${BOUND_PORT}`);
console.log(`topology source:   ${topology.source || "(no run log found yet)"}`);
console.log(`stop with:         dimos lcmflow dashboard stop`);

await sniff();

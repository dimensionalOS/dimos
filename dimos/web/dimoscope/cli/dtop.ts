#!/usr/bin/env -S deno run -A
// dtop — headless CLI for the @dimos/topics SDK. The primary, no-browser surface to drive
// and measure every delivery transport (the browser app is a second consumer of the same SDK).
// Wraps the SDK directly; `bench` delegates to the shared bench core (no reimplementation).
//
//   deno run -A cli/dtop.ts <cmd> [--transport gatewayWs|zenohTs|sse|httpPoll] [--url URL] [--dur MS]
//
// Commands:
//   list                 discovered topics + types
//   echo <topic>         decode + print messages (--count N to stop)
//   stats [topic]        live hz / kB/s / latency / dropped / loss% (from seq gaps)
//   probe                connect + verify data flows (exit 0/1) — reliability/health check
//   bench                run BENCH_SCENARIOS via the shared measurement core → markdown table
//   gen-types [--out F]  emit a typed DimosTopics map from live discovery
// Import the specific data-plane modules (not the index.ts barrel) so the DOM-only media
// plane never enters the graph — this CLI is headless/Deno, no DOM.
import { connect, type DimosClient } from "../packages/topics/src/client.ts";
import { createGatewayWsTransport } from "../packages/topics/src/adapters/gatewayWs.ts";
import { createZenohTsTransport } from "../packages/topics/src/adapters/zenohTs.ts";
import { createSseTransport } from "../packages/topics/src/adapters/sse.ts";
import { createHttpPollTransport } from "../packages/topics/src/adapters/httpPoll.ts";
import { BENCH_SCENARIOS, formatMarkdown, measureScenario } from "../packages/topics/src/bench.ts";
import type { Transport } from "../packages/topics/src/transport.ts";
import type { TopicInfo } from "../packages/topics/src/types.ts";
import { generateTypes } from "./genTypes.ts";

const args = Deno.args;
const cmd = args[0];

const flag = (name: string, def?: string): string | undefined => {
  const i = args.indexOf(`--${name}`);
  return i >= 0 && i + 1 < args.length ? args[i + 1] : def;
};
const positional = (n: number): string | undefined => {
  const v = args[n];
  return v && !v.startsWith("--") ? v : undefined;
};

const transport = flag("transport", "gatewayWs")!;
const dur = Number(flag("dur", "4000"));
const url = flag("url") ??
  (transport === "zenohTs" ? "ws://localhost:10000" : "ws://localhost:8090");

// `--decode server-json` (gatewayWs only) makes the gateway decode and ship JSON, so the CLI
// renders ANY type — incl. user-defined messages not in @dimos/msgs — with zero rebuild.
const decodeMode = flag("decode") === "server-json" ? ("server-json" as const) : undefined;

function buildTransport(): Transport {
  switch (transport) {
    case "gatewayWs":
      return createGatewayWsTransport({ url, reconnect: false, decode: decodeMode });
    case "zenohTs":
      return createZenohTsTransport({ remoteApiUrl: url, discoveryKey: flag("key", "dimos/**") });
    case "sse":
      return createSseTransport({ url });
    case "httpPoll":
      return createHttpPollTransport({ url });
    default:
      console.error(`dtop: unknown --transport "${transport}" (gatewayWs|zenohTs|sse|httpPoll)`);
      Deno.exit(2);
  }
}

function makeClient(): Promise<DimosClient> {
  return connect({ transport: buildTransport() });
}

const sleep = (ms: number) => new Promise<void>((r) => setTimeout(r, ms));

/** Connect, let discovery settle, return the discovered topics. */
async function discover(client: DimosClient, ms: number): Promise<TopicInfo[]> {
  await sleep(ms);
  return client.listTopics().sort((a, b) => a.topic.localeCompare(b.topic));
}

// ── commands ────────────────────────────────────────────────────────────────
async function cmdList() {
  const client = await makeClient();
  const topics = await discover(client, Math.min(dur, 2500));
  if (!topics.length) {
    console.error(
      "(no topics — is a publisher up? passive transports (sse/httpPoll) need traffic to discover)",
    );
  }
  for (const t of topics) console.log(`${t.topic}\t${t.type}`);
  client.close();
}

async function cmdEcho() {
  const topic = positional(1);
  if (!topic) {
    console.error("usage: dtop echo <topic>");
    Deno.exit(2);
  }
  const max = Number(flag("count", "0")); // 0 = unbounded
  let n = 0;
  const client = await makeClient();
  client.topic<unknown>(topic).subscribe((data, m) => {
    const body = JSON.stringify(data) ?? "undefined";
    const stamp = new Date().toISOString().slice(11, 23);
    const lat = m.latencyMs != null ? `${m.latencyMs.toFixed(1)}ms` : "?";
    console.log(
      `[${stamp}] ${m.topic} <${m.type}> ${m.sizeBytes}B lat=${lat}  ${
        body.length > 240 ? body.slice(0, 240) + "…" : body
      }`,
    );
    if (max && ++n >= max) {
      client.close();
      Deno.exit(0);
    }
  });
  await new Promise(() => {}); // run until --count hit or Ctrl+C
}

async function cmdStats() {
  const only = positional(1);
  const client = await makeClient();
  const topics = await discover(client, 1500);
  const names = only ? [only] : topics.map((t) => t.topic);
  if (!names.length) {
    console.error("(no topics to watch)");
    Deno.exit(1);
  }
  // Per-topic seq span → wire loss% (same pooling as bench.ts). TopicStats doesn't carry seq,
  // so track it here from the message meta. `n/a` when the source stamps no numeric seq.
  const seqStat = new Map<string, { min: number; max: number; recv: number }>();
  const handles = names.map((name) => {
    const h = client.topic<unknown>(name);
    h.subscribe((_d, m) => {
      if (m.seq == null) return;
      const s = seqStat.get(name);
      if (!s) seqStat.set(name, { min: m.seq, max: m.seq, recv: 1 });
      else {
        if (m.seq < s.min) s.min = m.seq;
        if (m.seq > s.max) s.max = m.seq;
        s.recv++;
      }
    });
    return h;
  });
  const lossOf = (name: string): string => {
    const s = seqStat.get(name);
    if (!s) return "n/a";
    const expected = s.max - s.min + 1;
    return expected > 0 ? Math.max(0, (1 - s.recv / expected) * 100).toFixed(1) : "n/a";
  };
  const render = () => {
    let out = `${"topic".padEnd(34)} ${"hz".padStart(6)} ${"kB/s".padStart(8)} ${
      "lat(ms)".padStart(9)
    } ${"drop".padStart(6)} ${"loss%".padStart(7)} ${"msgs".padStart(8)}\n`;
    for (const h of handles) {
      const s = h.stats();
      out +=
        `${h.name.padEnd(34)} ${String(s.hz).padStart(6)} ${
          (s.bytesPerSec / 1024).toFixed(1).padStart(8)
        } ` +
        `${(s.lastLatencyMs != null ? s.lastLatencyMs.toFixed(1) : "n/a").padStart(9)} ${
          String(s.dropped).padStart(6)
        } ${lossOf(h.name).padStart(7)} ${String(s.count).padStart(8)}\n`;
    }
    console.clear();
    console.log(
      out + `\n${transport} · ${url} · loss% from seq gaps (n/a = no seq) · Ctrl+C to stop`,
    );
  };
  const timer = setInterval(render, 1000);
  if (Number.isFinite(dur) && flag("dur") !== undefined) {
    setTimeout(() => {
      clearInterval(timer);
      client.close();
      Deno.exit(0);
    }, dur);
  }
  await new Promise(() => {});
}

async function cmdProbe() {
  const client = await makeClient();
  const topics = await discover(client, 2000);
  const seen = new Map<string, number>();
  for (const t of topics) {
    client.topic<unknown>(t.topic).subscribe(() => seen.set(t.topic, (seen.get(t.topic) ?? 0) + 1));
  }
  await sleep(dur);
  for (const [t, c] of [...seen].sort()) console.log(`  ${t}: ${c}`);
  const total = [...seen.values()].reduce((a, b) => a + b, 0);
  const ok = total > 0;
  console.log(
    ok
      ? `[probe] OK ✅ ${transport}: ${total} msgs / ${topics.length} topics over ${dur}ms`
      : `[probe] NO DATA ❌ (${transport} ${url})`,
  );
  client.close();
  Deno.exit(ok ? 0 : 1);
}

async function cmdBench() {
  console.log(`\n=== dtop bench: ${transport} (${dur}ms/scenario, ${url}) ===`);
  const rows = [];
  for (const scenario of BENCH_SCENARIOS) {
    const client = await makeClient();
    await sleep(250);
    const row = await measureScenario(client, scenario, dur, true); // endToEnd publish→client
    client.close();
    rows.push(row);
    console.log(
      `  ${row.scenario.padEnd(32)} hz=${row.hz} kB/s=${row.kbps} ` +
        `p50=${row.latP50} p95=${row.latP95} p99=${row.latP99} max=${row.latMax} std=${row.latStd} loss=${row.lossPct}%`,
    );
  }
  console.log(
    "\n" + formatMarkdown(`${transport} (dtop)`, url, dur, new Date().toISOString(), rows),
  );
}

async function cmdGenTypes() {
  const out = flag("out");
  // The authoritative set of types @dimos/msgs can decode → anything else is user-defined → `unknown`.
  const { getTypeNames } = await import("@dimos/msgs");
  const known = new Set<string>(getTypeNames());
  const client = await makeClient();
  const topics = await discover(client, Math.min(dur, 2500));
  const ts = generateTypes(topics, known);
  client.close();
  if (out) {
    await Deno.writeTextFile(out, ts);
    console.error(`dtop: wrote ${topics.length} topics → ${out}`);
  } else {
    console.log(ts);
  }
}

function printHelp() {
  console.log(
    `dtop — headless @dimos/topics CLI\n\n` +
      `  dtop <list|echo|stats|probe|bench|gen-types> [options]\n\n` +
      `  --transport  gatewayWs (default) | zenohTs | sse | httpPoll\n` +
      `  --url        gateway url (default ws://localhost:8090; zenohTs ws://localhost:10000)\n` +
      `  --dur        duration ms for bench/probe/stats (default 4000)\n` +
      `  --decode     gatewayWs only: "server-json" → gateway decodes (renders custom types, zero rebuild)\n` +
      `  --count      echo: stop after N messages\n` +
      `  --out        gen-types: write to file instead of stdout\n`,
  );
}

try {
  Deno.addSignalListener("SIGINT", () => Deno.exit(0));
} catch { /* signals unsupported on this platform */ }

switch (cmd) {
  case "list":
    await cmdList();
    break;
  case "echo":
    await cmdEcho();
    break;
  case "stats":
    await cmdStats();
    break;
  case "probe":
    await cmdProbe();
    break;
  case "bench":
    await cmdBench();
    break;
  case "gen-types":
    await cmdGenTypes();
    break;
  default:
    printHelp();
    Deno.exit(cmd ? 2 : 0);
}

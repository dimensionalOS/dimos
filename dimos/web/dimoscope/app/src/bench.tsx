// In-browser transport benchmark — runs the SAME scenarios/measurement as the headless CLI
// bench (bench/bench.ts), but in the REAL browser runtime across all 5 delivery mechanisms
// (WebSocket · SSE · HTTP-poll · WebRTC-data · WebTransport), so WebRTC/WebTransport are
// benched on the real browser stacks (the CLI uses aiortc/aioquic stand-ins). Latency is
// end-to-end (publish → browser). Reuses @dimos/topics/bench.
//
// Prereqs: `deno task serve` (the one service on :8080) + a data source (bench_publisher.py
// or a sim). Open http://localhost:5173/bench.html (or http://localhost:8080/bench.html).
//   ?gw=host:port  → route through netsim or a remote VPS   ·   ?dur=ms  ·  ?wt=8443
// Unsupported mechanisms (e.g. WebTransport on Safari/Firefox) fail gracefully → a NaN row,
// which doubles as the cross-browser support matrix. Results also at window.__benchResults.
import { useState } from "react";
import { createRoot } from "react-dom/client";
import {
  BENCH_SCENARIOS,
  type BenchRow,
  createDimosClient,
  type DimosClient,
  formatMarkdown,
  measureScenario,
  onDemandSaving,
  type TransportFactory,
  ws,
} from "@dimos/topics";
import "./styles.css";

/** Build + connect a client for one transport factory at `url` (the bench's per-mechanism endpoint). */
async function openClient(transport: TransportFactory, url: string): Promise<DimosClient> {
  const c = createDimosClient({ transport });
  await c.connect(url);
  return c;
}

const host = location.hostname || "localhost";
const params = new URLSearchParams(location.search);
const DUR = Number(params.get("dur") ?? 4000);
// Default origin = the single service on :8080. `?gw=host:port` overrides it (through netsim, or a
// remote VPS) — mirrors app/src/main.tsx so the bench hits the same paths the app does.
const origin = params.get("gw") ?? `${host}:8080`;
const wsProto = location.protocol === "https:" ? "wss" : "ws";
const httpProto = location.protocol === "https:" ? "https" : "http";
const wsBase = `${wsProto}://${origin}`;
const httpBase = `${httpProto}://${origin}`;
const wtHost = origin.split(":")[0];
const WT_PORT = Number(params.get("wt") ?? 8443);

interface TransportDef {
  id: string;
  label: string;
  url: string;
  open: () => Promise<DimosClient>;
}
// The 5 same-origin delivery mechanisms on serve.py. Lazy-imported so a browser-only API
// (RTCPeerConnection / WebTransport) that a given engine lacks only fails when that row runs.
const TRANSPORTS: TransportDef[] = [
  {
    id: "ws",
    label: "WebSocket",
    url: `${wsBase}/ws`,
    open: () => openClient(ws({ reconnect: false }), `${wsBase}/ws`),
  },
  {
    id: "sse",
    label: "SSE",
    url: `${httpBase}/sse`,
    open: async () => {
      const { sse } = await import("@dimos/topics/experimental");
      return openClient(sse(), httpBase);
    },
  },
  {
    id: "poll",
    label: "HTTP poll",
    url: `${httpBase}/poll`,
    open: async () => {
      const { poll } = await import("@dimos/topics/experimental");
      return openClient(poll(), httpBase);
    },
  },
  {
    id: "webrtc",
    label: "WebRTC data",
    url: `${wsBase}/rtc`,
    open: async () => {
      const { webrtc } = await import("@dimos/topics/experimental");
      return openClient(webrtc(), `${wsBase}/rtc`);
    },
  },
  {
    id: "webtransport",
    label: "WebTransport",
    url: `https://${wtHost}:${WT_PORT}`,
    open: async () => {
      const { webtransportData } = await import("@dimos/topics/experimental");
      return openClient(
        webtransportData({ certHashUrl: `${httpBase}/cert` }),
        `https://${wtHost}:${WT_PORT}`,
      );
    },
  },
];

interface Result {
  id: string;
  label: string;
  url: string;
  rows: BenchRow[];
  saving: number;
}

function Bench() {
  const [running, setRunning] = useState(false);
  const [progress, setProgress] = useState("idle");
  const [results, setResults] = useState<Result[]>([]);

  async function run() {
    setRunning(true);
    setResults([]);
    const out: Result[] = [];
    for (const t of TRANSPORTS) {
      const rows: BenchRow[] = [];
      for (const scenario of BENCH_SCENARIOS) {
        setProgress(`${t.label} · ${scenario.name}…`);
        try {
          const client = await t.open();
          await new Promise((r) => setTimeout(r, 250));
          rows.push(await measureScenario(client, scenario, DUR, true)); // endToEnd
          client.close();
        } catch (e) {
          setProgress(`${t.label} failed: ${String(e).slice(0, 80)}`);
          rows.push({
            scenario: scenario.name,
            topics: scenario.topics.length,
            msgs: 0,
            hz: 0,
            kbps: 0,
            latP50: NaN,
            latP95: NaN,
            latP99: NaN,
            latMax: NaN,
            latStd: NaN,
            lossPct: NaN,
          });
        }
      }
      out.push({ id: t.id, label: t.label, url: t.url, rows, saving: onDemandSaving(rows) });
      setResults([...out]);
    }
    (window as { __benchResults?: Result[] }).__benchResults = out;
    setProgress("done ✓");
    setRunning(false);
  }

  const stamp = new Date().toISOString().slice(0, 16).replace("T", " ");
  const md =
    `# dimoscope transport benchmark — in-browser (all 5 mechanisms)\n\n_${stamp} · ${DUR}ms/scenario · end-to-end latency (publish→browser) · origin ${origin}_\n\n` +
    results.map((r) => formatMarkdown(r.label, r.url, DUR, stamp, r.rows)).join("\n---\n\n");

  return (
    <div style={{ padding: 24, maxWidth: 1100, margin: "0 auto" }}>
      <h2 style={{ fontFamily: "ui-monospace, monospace" }}>
        dimoscope · transport benchmark{" "}
        <span className="muted">(in-browser, all 5 mechanisms)</span>
      </h2>
      <p className="muted small">
        Start the service + a source first: <code>deno task serve</code> +{" "}
        <code>bench_publisher.py</code> (or a sim). Origin <code>{origin}</code> ·{" "}
        {DUR}ms/scenario · latency is end-to-end (publish→browser), comparable across mechanisms.
      </p>
      <div style={{ display: "flex", gap: 8, marginBottom: 16, alignItems: "center" }}>
        <button className="tab" onClick={run} disabled={running}>
          {running ? "running…" : "▶ Run benchmark"}
        </button>
        <button
          className="tab"
          disabled={!results.length}
          onClick={() => navigator.clipboard.writeText(md)}
        >
          copy Markdown
        </button>
        <button
          className="tab"
          disabled={!results.length}
          onClick={() => {
            const blob = new Blob([JSON.stringify(results, null, 2)], { type: "application/json" });
            const a = document.createElement("a");
            a.href = URL.createObjectURL(blob);
            a.download = "bench-browser.json";
            a.click();
          }}
        >
          download JSON
        </button>
        <span className="muted small">{progress}</span>
      </div>
      {results.map((r) => (
        <div key={r.id} className="panel" style={{ marginBottom: 16 }}>
          <div className="panel-title">
            {r.label} · {r.url} · on-demand saving {r.saving}%
          </div>
          <table className="stats" style={{ width: "100%" }}>
            <thead>
              <tr>
                <th>scenario</th>
                <th>hz</th>
                <th>kB/s</th>
                <th>lat p50</th>
                <th>lat p95</th>
                <th>lat max</th>
              </tr>
            </thead>
            <tbody>
              {r.rows.map((row) => (
                <tr key={row.scenario}>
                  <td className="mono">{row.scenario}</td>
                  <td>{row.hz}</td>
                  <td>{row.kbps}</td>
                  <td>{row.latP50}</td>
                  <td>{row.latP95}</td>
                  <td>{row.latMax}</td>
                </tr>
              ))}
            </tbody>
          </table>
        </div>
      ))}
    </div>
  );
}

createRoot(document.getElementById("root")!).render(<Bench />);

// In-browser transport benchmark — runs the SAME scenarios/measurement as the headless
// Bun bench (bench/bench.ts), but in the real browser runtime across ALL THREE transports,
// so zenoh-ts (browser-only) is benched apples-to-apples with the gateways. Latency is
// measured end-to-end (publish → browser) for a fair compare. Reuses @dimos/topics/bench.
//
// Prereqs: `bash bench/serve-bench.sh` (3 servers + bench_publisher on both buses).
// Open http://localhost:5173/bench.html · results also at window.__benchResults.
import { useState } from "react";
import { createRoot } from "react-dom/client";
import {
  connect,
  BENCH_SCENARIOS,
  measureScenario,
  formatMarkdown,
  onDemandSaving,
  type BenchRow,
  type DimosClient,
} from "@dimos/topics";
import "./styles.css";

const host = location.hostname || "localhost";
const DUR = Number(new URLSearchParams(location.search).get("dur") ?? 4000);

interface TransportDef {
  id: string;
  label: string;
  url: string;
  open: () => Promise<DimosClient>;
}
const TRANSPORTS: TransportDef[] = [
  { id: "zenoh", label: "Python↔Zenoh", url: `ws://${host}:8088`, open: () => connect({ url: `ws://${host}:8088`, reconnect: false }) },
  { id: "lcm", label: "Bun↔LCM", url: `ws://${host}:8089`, open: () => connect({ url: `ws://${host}:8089`, reconnect: false }) },
  {
    id: "zenoh-ts",
    label: "zenoh-ts (direct)",
    url: `ws://${host}:10000`,
    open: async () => {
      const { ZenohTsTransport } = await import("@dimos/topics");
      // discoveryKey "" → no scout; the bench subscribes explicit /bench/* keys.
      return connect({ transport: new ZenohTsTransport(`ws://${host}:10000`, undefined, "") });
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
          rows.push({ scenario: scenario.name, topics: scenario.topics.length, msgs: 0, hz: 0, kbps: 0, latP50: NaN, latP95: NaN, latMax: NaN });
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
    `# dimoscope transport benchmark — in-browser (all 3 transports)\n\n_${stamp} · ${DUR}ms/scenario · end-to-end latency (publish→browser)_\n\n` +
    results.map((r) => formatMarkdown(r.label, r.url, DUR, stamp, r.rows)).join("\n---\n\n");

  return (
    <div style={{ padding: 24, maxWidth: 1100, margin: "0 auto" }}>
      <h2 style={{ fontFamily: "ui-monospace, monospace" }}>
        dimoscope · transport benchmark <span className="muted">(in-browser, all 3 transports)</span>
      </h2>
      <p className="muted small">
        Start the servers + source first: <code>bash bench/serve-bench.sh</code> (3 servers + bench_publisher on both
        buses). {DUR}ms/scenario · latency is end-to-end (publish→browser), comparable across transports.
      </p>
      <div style={{ display: "flex", gap: 8, marginBottom: 16, alignItems: "center" }}>
        <button className="tab" onClick={run} disabled={running}>
          {running ? "running…" : "▶ Run benchmark"}
        </button>
        <button className="tab" disabled={!results.length} onClick={() => navigator.clipboard.writeText(md)}>
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

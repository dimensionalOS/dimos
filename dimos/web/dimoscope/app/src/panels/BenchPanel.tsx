// BenchPanel — drive the `bench-load` blueprint straight from the SDK: Start/Stop the synthetic
// flood via dimos @rpc (BenchLoad.start_bench/stop_bench), and watch the /bench/* topics it emits
// live (hz · kB/s · latency) through the same @dimos/topics observability surface as StatsBar.
// The RPC controls need the gateway's command bridge (Python↔Zenoh); they disable themselves when
// no BenchLoad command is advertised (e.g. Bun↔LCM has no RPC bridge — start it via --option).
import { useState } from "react";
import { useCommands, useRpc, useTopicStats } from "@dimos/react";

const BENCH_TOPICS = ["/bench/p0", "/bench/p1", "/bench/p2", "/bench/p3", "/bench/grid"];

function Row({ topic }: { topic: string }) {
  const s = useTopicStats(topic);
  return (
    <tr>
      <td className="mono">{topic}</td>
      <td>{s?.hz ?? 0}</td>
      <td>{s ? (s.bytesPerSec / 1000).toFixed(1) : "0"}</td>
      <td>{s?.lastLatencyMs != null ? s.lastLatencyMs.toFixed(1) : "–"}</td>
    </tr>
  );
}

export function BenchPanel() {
  const commands = useCommands();
  const { call } = useRpc();
  const [hz, setHz] = useState(100);
  const [busy, setBusy] = useState(false);
  const [last, setLast] = useState<string>();

  const hasRpc = commands.some((c) => c.target === "BenchLoad");

  async function trigger(method: "start_bench" | "stop_bench") {
    setBusy(true);
    try {
      const res = await call<string>("BenchLoad", method, ...(method === "start_bench" ? [hz] : []));
      setLast(String(res));
    } catch (e) {
      setLast(`✗ ${(e as Error).message}`);
    } finally {
      setBusy(false);
    }
  }

  return (
    <div className="panel">
      <div className="panel-title">Bench load · /bench/*</div>
      <div style={{ display: "flex", alignItems: "center", gap: 6, flexWrap: "wrap" }}>
        <label className="muted small">
          Hz{" "}
          <input
            type="number"
            min={1}
            value={hz}
            onChange={(e) => setHz(Number(e.target.value) || 1)}
            style={{ width: 64 }}
          />
        </label>
        <button className="tab" disabled={busy || !hasRpc} onClick={() => trigger("start_bench")}>
          Start
        </button>
        <button className="tab" disabled={busy || !hasRpc} onClick={() => trigger("stop_bench")}>
          Stop
        </button>
      </div>
      {!hasRpc && (
        <div className="muted small" style={{ marginTop: 6 }}>
          RPC controls need the Python↔Zenoh gateway; start the flood with{" "}
          <span className="mono">--option benchload.autostart=true</span>.
        </div>
      )}
      {last && (
        <div className="muted small" style={{ marginTop: 6 }} title={last}>
          {last}
        </div>
      )}
      <table className="stats" style={{ marginTop: 8 }}>
        <thead>
          <tr>
            <th>topic</th>
            <th>hz</th>
            <th>kB/s</th>
            <th>lat ms</th>
          </tr>
        </thead>
        <tbody>
          {BENCH_TOPICS.map((t) => (
            <Row key={t} topic={t} />
          ))}
        </tbody>
      </table>
    </div>
  );
}

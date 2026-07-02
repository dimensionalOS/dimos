// Expanded row detail: the per-lane split (the fair-data-plane evidence — /load/fast staying
// fresh beside an /load/img backlog) and the run's time dimension (ramp, outage recovery,
// backlog drain) as three bucket sparklines.
import type { BenchRow } from "@dimos/web";
import { Sparkline } from "../../widgets/Sparkline";

const f = (n: number) => (Number.isFinite(n) ? String(n) : "–");

function Spark({ label, unit, data, color }: {
  label: string;
  unit: string;
  data: number[];
  color: string;
}) {
  const finite = data.filter(Number.isFinite);
  const min = finite.length ? Math.min(...finite) : NaN;
  const max = finite.length ? Math.max(...finite) : NaN;
  return (
    <div>
      <div className="bench-spark-lbl">{label}</div>
      <Sparkline data={data.map((v) => (Number.isFinite(v) ? v : 0))} width={200} height={32} color={color} />
      <div className="muted small mono">
        {f(min)}–{f(max)} {unit}
      </div>
    </div>
  );
}

export function LaneDetail({ row }: { row: BenchRow }) {
  return (
    <div>
      {row.lanes.length > 1 && (
        <table className="stats" style={{ width: "100%", marginBottom: 8 }}>
          <thead>
            <tr>
              <th>lane</th>
              <th>msgs</th>
              <th>hz</th>
              <th>kB/s</th>
              <th>offered</th>
              <th>deliv%</th>
              <th>p50</th>
              <th>p95</th>
              <th>p99</th>
              <th>loss%</th>
              <th>late%</th>
            </tr>
          </thead>
          <tbody>
            {row.lanes.map((l) => (
              <tr key={l.topic}>
                <td className="mono">
                  {l.topic}
                  {l.seqResets > 0 ? " †" : ""}
                </td>
                <td>{l.msgs}</td>
                <td>{f(l.hz)}</td>
                <td>{f(l.kbps)}</td>
                <td>{f(l.offeredKbps)}</td>
                <td>{f(l.deliveryPct)}</td>
                <td>{f(l.latP50)}</td>
                <td>{f(l.latP95)}</td>
                <td>{f(l.latP99)}</td>
                <td>{f(l.lossPct)}</td>
                <td>{f(l.latePct)}</td>
              </tr>
            ))}
          </tbody>
        </table>
      )}
      {row.buckets.length >= 2
        ? (
          <div className="bench-sparks">
            <Spark label="hz" unit="hz" data={row.buckets.map((b) => b.hz)} color="var(--signal)" />
            <Spark label="kB/s" unit="kB/s" data={row.buckets.map((b) => b.kbps)} color="#9fb4d8" />
            <Spark label="p95" unit="ms" data={row.buckets.map((b) => b.latP95)} color="var(--accent)" />
          </div>
        )
        : (
          <div className="muted small">
            no time-series — persisted runs drop buckets (the JSON export keeps them)
          </div>
        )}
    </div>
  );
}

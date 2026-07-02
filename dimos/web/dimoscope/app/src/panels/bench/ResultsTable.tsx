// The results table — cells append across sweeps and transport switches (side-by-side wires),
// Δ suffixes against the pinned baseline, expandable per-lane/time-series detail, and the
// exports (Markdown, JSON) built from the same RunRecord the history persists.
import { useState } from "react";
import {
  type ClockSample,
  formatMarkdown,
  type RunCell,
  type RunRecord,
  serializeRun,
} from "@dimos/web";
import { type BaselineStat, deltas } from "./model";
import { LaneDetail } from "./LaneDetail";

const f = (n: number) => (Number.isFinite(n) ? String(n) : "–");

function DeltaCell({ v, chip }: { v: number; chip?: { text: string; bad: boolean; faint: boolean } }) {
  return (
    <td>
      {f(v)}
      {chip && (
        <span className={`delta ${chip.faint ? "" : chip.bad ? "delta-bad" : "delta-good"}`}>
          {chip.text}
        </span>
      )}
    </td>
  );
}

export function ResultsTable(
  { cells, clock, baseline, buildRecord, onClear }: {
    cells: RunCell[];
    clock?: ClockSample;
    baseline?: Map<string, BaselineStat>;
    buildRecord: () => RunRecord;
    onClear: () => void;
  },
) {
  const [expanded, setExpanded] = useState<Set<number>>(new Set());
  const [detail, setDetail] = useState(false);
  if (cells.length === 0) return null;
  const toggle = (i: number) =>
    setExpanded((s) => {
      const n = new Set(s);
      if (n.has(i)) n.delete(i);
      else n.add(i);
      return n;
    });
  const downloadJson = () => {
    const rec = buildRecord();
    const blob = new Blob([serializeRun(rec)], { type: "application/json" });
    const a = document.createElement("a");
    a.href = URL.createObjectURL(blob);
    a.download = `dimoscope-bench-${rec.stamp.replace(/[: ]/g, "-")}.json`;
    a.click();
    URL.revokeObjectURL(a.href);
  };
  const cols = 13 + (detail ? 2 : 0);
  return (
    <div className="bench-section">
      <div className="bench-row">
        <div className="bench-label" style={{ margin: 0 }}>Results · end-to-end (publish→browser)</div>
        {clock
          ? (
            <span
              className="badge"
              title="NTP-style probe (lowest-RTT of 5) applied to every latency — gateway ≈ source clock"
            >
              clk {clock.offsetMs >= 0 ? "+" : ""}
              {clock.offsetMs.toFixed(1)}ms · rtt {clock.rttMs.toFixed(1)}ms
            </span>
          )
          : <span className="badge badge-warn" title="clock probe failed — latency is raw recvTs−srcTs">clock raw</span>}
        <button
          className="tab"
          onClick={() => navigator.clipboard.writeText(formatMarkdown(buildRecord()))}
        >
          copy Markdown
        </button>
        <button
          className="tab"
          onClick={() => navigator.clipboard.writeText(serializeRun(buildRecord()))}
        >
          copy JSON
        </button>
        <button className="tab" onClick={downloadJson} title="full fidelity — includes the time-series buckets">
          ⬇ JSON
        </button>
        <button
          className={`tab ${detail ? "tab-active" : ""}`}
          onClick={() => setDetail((d) => !d)}
          title="max + σ columns"
        >
          σ/max
        </button>
        <button className="tab" onClick={onClear}>Clear</button>
      </div>
      <table className="stats" style={{ width: "100%" }}>
        <thead>
          <tr>
            <th></th>
            <th>cfg</th>
            <th>net</th>
            <th>qos</th>
            <th>scenario</th>
            <th>hz</th>
            <th>kB/s</th>
            <th>deliv%</th>
            <th>p50</th>
            <th>p95</th>
            <th>p99</th>
            {detail && <th>max</th>}
            {detail && <th>σ</th>}
            <th>loss%</th>
            <th>late%</th>
          </tr>
        </thead>
        <tbody>
          {cells.map((c, i) => {
            const d = baseline ? deltas(c, baseline) : undefined;
            const wireTag = c.wire && c.wire !== c.transport ? ` (${c.wire})` : "";
            const r = c.row;
            return [
              <tr key={i}>
                <td>
                  <button className="bench-caret-btn" onClick={() => toggle(i)} title="per-lane + time-series">
                    {expanded.has(i) ? "▾" : "▸"}
                  </button>
                </td>
                <td className="mono">{c.transport}{wireTag}</td>
                <td className="mono">{c.netem}</td>
                <td className="mono">{c.maxHz ? `${c.maxHz}Hz` : "∞"}{c.rep > 1 ? ` ·${c.rep}` : ""}</td>
                {c.error
                  ? (
                    <>
                      <td className="mono">{c.scenario}</td>
                      <td colSpan={cols - 5} style={{ color: "var(--err)" }}>✗ {c.error}</td>
                    </>
                  )
                  : (
                    <>
                      <td className="mono">
                        {c.scenario}
                        {r.seqResets > 0 ? " †" : ""}
                      </td>
                      <td>{f(r.hz)}</td>
                      <td>{f(r.kbps)}</td>
                      <td title={Number.isFinite(r.offeredKbps) ? `offered ${r.offeredKbps} kB/s` : undefined}>
                        {f(r.deliveryPct)}
                      </td>
                      <td>{f(r.latP50)}</td>
                      <DeltaCell v={r.latP95} chip={d?.p95} />
                      <td>{f(r.latP99)}</td>
                      {detail && <td>{f(r.latMax)}</td>}
                      {detail && <td>{f(r.latStd)}</td>}
                      <DeltaCell v={r.lossPct} chip={d?.loss} />
                      <td>{f(r.latePct)}</td>
                    </>
                  )}
              </tr>,
              expanded.has(i) && !c.error && (
                <tr key={`${i}-x`} className="bench-sub">
                  <td colSpan={cols}>
                    <LaneDetail row={r} />
                  </td>
                </tr>
              ),
            ];
          })}
        </tbody>
      </table>
      {cells.some((c) => c.row.seqResets > 0) && (
        <div className="muted small">† source restarted mid-run (seq reset) — treat loss/offered with care</div>
      )}
    </div>
  );
}

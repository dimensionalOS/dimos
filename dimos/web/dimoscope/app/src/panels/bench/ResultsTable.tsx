// The results table — cells append across sweeps and transport switches (side-by-side wires),
// Δ suffixes against the pinned baseline, expandable per-lane/time-series detail, and the
// exports (Markdown, JSON) built from the same RunRecord the history persists.
import { useMemo, useState } from "react";
import {
  buildFloodOffIndex,
  type ClockSample,
  fastLane,
  formatMarkdown,
  type RunCell,
  type RunRecord,
  serializeRun,
} from "@dimos/web";
import { type BaselineStat, deltas, interferenceChip } from "./model";
import { LaneDetail } from "./LaneDetail";

const f = (n: number) => (Number.isFinite(n) ? String(n) : "–");

function DeltaCell(
  { v, chip, title }: {
    v: number;
    chip?: { text: string; bad: boolean; faint: boolean };
    title?: string;
  },
) {
  return (
    <td title={title}>
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
  // Flood-off references over the displayed cells — "this table" IS the comparison set.
  const ifIndex = useMemo(() => buildFloodOffIndex(cells), [cells]);
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
  const cols = 14 + (detail ? 3 : 0);
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
            <th title="pooled across lanes (sample-count weighted) — see fast p95 for the small-lane story">
              p95
            </th>
            <th>p99</th>
            <th title="/load/fast lane only — the interference signal beside a flood">fast p95</th>
            {detail && <th title="/load/fast lane deliv%">fast dlv%</th>}
            {detail && <th>max</th>}
            {detail && <th>σ</th>}
            <th>loss%</th>
            <th>late%</th>
          </tr>
        </thead>
        <tbody>
          {cells.map((c, i) => {
            const d = baseline ? deltas(c, baseline) : undefined;
            const ifd = interferenceChip(c, ifIndex);
            const fl = fastLane(c.row);
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
                      <DeltaCell v={fl?.latP95 ?? NaN} chip={ifd?.chip} title={ifd?.title} />
                      {detail && <td>{f(fl?.deliveryPct ?? NaN)}</td>}
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
      {cells.some((c) => interferenceChip(c, ifIndex)) && (
        <div className="muted small">
          × = /load/fast p95 vs this table's flood-off cell at the same transport·net·maxHz
        </div>
      )}
    </div>
  );
}

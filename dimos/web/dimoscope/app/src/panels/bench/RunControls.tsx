// Sweep knobs + execution controls: maxHz axis (server-enforced QoS request), duration,
// repeats, the honest cell-count/ETA line, Run/Stop, and live progress with an hz sparkline
// fed by the in-flight cell's buckets.
import type { BenchBucket } from "@dimos/web";
import { Sparkline } from "../../widgets/Sparkline";
import { estimateMs, fmtDuration, type SweepPlan } from "./model";
import type { BenchUrlConfig } from "./urlParams";

const HZ_PRESETS = [0, 5, 20, 60, 120];
const WARN_MS = 10 * 60_000;

export function RunControls(
  { cfg, patch, plan, running, canRun, progress, note, live, onRun, onStop }: {
    cfg: BenchUrlConfig;
    patch: (p: Partial<BenchUrlConfig>) => void;
    plan: SweepPlan;
    running: boolean;
    canRun: boolean;
    progress: string;
    note?: string;
    live: BenchBucket[];
    onRun: () => void;
    onStop: () => void;
  },
) {
  const toggleHz = (h: number) => {
    const has = cfg.maxHz.includes(h);
    const next = has ? cfg.maxHz.filter((x) => x !== h) : [...cfg.maxHz, h].sort((a, b) => a - b);
    if (next.length) patch({ maxHz: next }); // deselecting the last is a no-op
  };
  const est = estimateMs(plan, cfg.durMs);
  const a = plan.axes;
  return (
    <>
      <div className="bench-section">
        <div className="bench-label">QoS · maxHz request (server-enforced) — multi-select to sweep</div>
        <div className="bench-row">
          <span className="muted small">maxHz</span>
          {HZ_PRESETS.map((h) => (
            <button
              key={h}
              className={`tab ${cfg.maxHz.includes(h) ? "tab-active" : ""}`}
              disabled={running}
              onClick={() => toggleHz(h)}
            >
              {h === 0 ? "∞" : h}
            </button>
          ))}
        </div>
      </div>

      <div className="bench-section">
        <div className="bench-row">
          <span className="muted small">duration</span>
          <input
            type="number"
            min={1000}
            step={1000}
            value={cfg.durMs}
            disabled={running}
            onChange={(e) => patch({ durMs: Number(e.target.value) || 4000 })}
            style={{ width: 80 }}
          />
          <span className="muted small">ms · ×</span>
          <input
            type="number"
            min={1}
            max={20}
            value={cfg.reps}
            disabled={running}
            onChange={(e) => patch({ reps: Math.max(1, Number(e.target.value) || 1) })}
            style={{ width: 48 }}
          />
          <span className="muted small">repeats</span>
        </div>
        <div className="muted small" style={est > WARN_MS ? { color: "var(--warn)" } : undefined}>
          {plan.cells.length} cell{plan.cells.length === 1 ? "" : "s"} ≈ {fmtDuration(est)} —{" "}
          {a.nets} net × {a.workloads} workload{a.workloads === 1 ? "" : "s"} × {a.hzs} maxHz × {a.reps}{" "}
          rep{a.reps === 1 ? "" : "s"}
        </div>
        <div className="bench-row">
          <button className="tab" onClick={onRun} disabled={running || !canRun}>
            {running ? "running…" : "▶ Run sweep"}
          </button>
          <button className="tab" disabled={!running} onClick={onStop}>
            Stop
          </button>
          <span className="muted small">{progress}</span>
          {running && live.length >= 2 && (
            <span title="delivered hz, per second, this cell">
              <Sparkline data={live.map((b) => b.hz)} width={120} height={20} />
            </span>
          )}
        </div>
        {note && <div className="muted small" style={{ color: "var(--warn)" }}>{note}</div>}
      </div>
    </>
  );
}

// Past runs — load back into the table (side-by-side with fresh cells), pin one as the Δ
// baseline, rename inline, export full-fidelity JSON (in-memory copies keep buckets), delete.
import { useState } from "react";
import { type RunRecord, serializeRun } from "@dimos/web";
import type { BenchHistory } from "./history";

export function HistoryPanel({ history, onLoad, loadedIds }: {
  history: BenchHistory;
  onLoad: (rec: RunRecord) => void;
  loadedIds: Set<string>;
}) {
  const [editing, setEditing] = useState<string | null>(null);
  const { records, baselineId, persistOk, remove, rename, pin, get } = history;
  if (records.length === 0) return null;

  const exportJson = (id: string) => {
    const rec = get(id);
    if (!rec) return;
    const blob = new Blob([serializeRun(rec)], { type: "application/json" });
    const a = document.createElement("a");
    a.href = URL.createObjectURL(blob);
    a.download = `dimoscope-bench-${rec.stamp.replace(/[: ]/g, "-")}.json`;
    a.click();
    URL.revokeObjectURL(a.href);
  };

  return (
    <div className="bench-section">
      <div className="bench-label">Runs ({records.length})</div>
      {!persistOk && (
        <div className="muted small" style={{ color: "var(--warn)" }}>
          run too large to persist — export JSON to keep it
        </div>
      )}
      {records.map((r) => (
        <div key={r.id} className="bench-history-row">
          {editing === r.id
            ? (
              <input
                className="bench-name-input"
                autoFocus
                defaultValue={r.name ?? ""}
                placeholder="name this run"
                onBlur={(e) => {
                  rename(r.id, e.target.value.trim());
                  setEditing(null);
                }}
                onKeyDown={(e) => {
                  if (e.key === "Enter") (e.target as HTMLInputElement).blur();
                  if (e.key === "Escape") setEditing(null);
                }}
              />
            )
            : (
              <button
                className="bench-caret-btn"
                style={{ padding: 0 }}
                title="click to rename"
                onClick={() => setEditing(r.id)}
              >
                {r.name ?? r.meta.transportLabel}
              </button>
            )}
          <span className="muted small">
            {r.stamp} · {r.cells.length} cell{r.cells.length === 1 ? "" : "s"}
            {r.meta.gatewayUrl ? ` · ${r.meta.gatewayUrl}` : ""}
            {r.aborted ? " · aborted" : ""}
          </span>
          {r.id === baselineId && <span className="badge">baseline</span>}
          <button
            className={`tab ${loadedIds.has(r.id) ? "tab-active" : ""}`}
            onClick={() => onLoad(r)}
            title="load this run's cells into the table (click again to remove)"
          >
            {loadedIds.has(r.id) ? "loaded" : "load"}
          </button>
          <button
            className={`tab ${r.id === baselineId ? "tab-active" : ""}`}
            title="pin as Δ baseline (match: scenario+netem+maxHz; median over repeats)"
            onClick={() => pin(r.id === baselineId ? null : r.id)}
          >
            ★
          </button>
          <button className="tab" onClick={() => exportJson(r.id)}>json</button>
          <button className="tab" onClick={() => remove(r.id)} title="delete">×</button>
        </div>
      ))}
    </div>
  );
}

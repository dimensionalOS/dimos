// Netem sweep axes — multi-select which profiles the matrix visits. Setting the condition
// NOW lives in the topbar (shared context, so the ● dot here and the topbar select always
// agree). Outages stay here: one-shot failover experiments, legal mid-run.
import { useNetem } from "../../netem";

export function NetemSection({ netSel, onNetSel, netIgnored, running }: {
  netSel: string[];
  onNetSel: (ids: string[]) => void;
  netIgnored: string[];
  running: boolean;
}) {
  const { netem, busy, msg, apply } = useNetem();
  if (!netem?.enabled) {
    return netIgnored.length > 0
      ? (
        <div className="bench-section">
          <div className="muted small">
            <span className="mono">?net</span> ignored — netem not enabled on this gateway
            (Linux + <span className="mono">NETEM_CTL=1</span>); sweeping the live path instead.
          </div>
        </div>
      )
      : null;
  }
  const toggle = (id: string) =>
    onNetSel(netSel.includes(id) ? netSel.filter((x) => x !== id) : [...netSel, id]);
  return (
    <div className="bench-section">
      <div className="bench-label">Network · netem sweep axis (server-side)</div>
      <div className="bench-chips">
        {netem.profiles.map((p) => (
          <button
            key={p.id}
            className={`tab ${netSel.includes(p.id) ? "tab-active" : ""}`}
            title={`${p.desc} — multi-select: the sweep asserts each in turn`}
            disabled={running}
            onClick={() => toggle(p.id)}
          >
            {netem.active === p.id && <span className="net-dot">●</span>}
            {p.label}
          </button>
        ))}
      </div>
      <div className="bench-row">
        <span className="muted small">outage</span>
        {netem.outages.map((o) => (
          <button
            key={o.id}
            className="tab"
            title={`${o.desc} — fires now (also mid-run: failover is data)`}
            disabled={busy}
            onClick={() => apply(o.id)}
          >
            {o.label}
          </button>
        ))}
        {msg && <span className="muted small" title={msg}>{msg.slice(0, 60)}</span>}
      </div>
      {netIgnored.length > 0 && (
        <div className="muted small">
          <span className="mono">?net</span> ids not on this gateway, ignored:{" "}
          <span className="mono">{netIgnored.join(", ")}</span>
        </div>
      )}
      <div className="muted small">
        none selected = measure the path as-is (topbar sets the ambient condition) · asserted per
        group, restored after · self-heals in {Math.round(netem.healsInS / 60)} min · apply loss{" "}
        <em>after</em> the transport is connected — QUIC handshakes fail under loss
      </div>
    </div>
  );
}

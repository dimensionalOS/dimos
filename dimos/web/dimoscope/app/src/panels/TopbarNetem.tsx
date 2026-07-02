// Ambient network condition in the topbar — netem shapes the gateway egress for everything
// (teleop, camera, WorldView), not just bench sweeps, and an active profile used to be
// invisible outside the drawer. Same select idiom as the transport/cam dropdowns beside it;
// renders nothing unless the gateway reports netem enabled. Amber = the path is shaped.
import { useNetem } from "../netem";

export function TopbarNetem() {
  const { netem, busy, apply } = useNetem();
  if (!netem?.enabled) return null;
  const shaped = netem.active !== "clean";
  const active = netem.profiles.find((p) => p.id === netem.active);
  return (
    <select
      className={`server-select${shaped ? " net-warn" : ""}`}
      value={netem.active}
      disabled={busy}
      onChange={(e) => apply(e.target.value)}
      title={`server-side netem — ${active?.desc ?? netem.active} · self-heals in ${
        Math.round(netem.healsInS / 60)
      } min · apply loss after the transport is connected (QUIC handshakes fail under loss)`}
    >
      {netem.profiles.map((p) => (
        <option key={p.id} value={p.id} title={p.desc}>
          net: {p.label}
        </option>
      ))}
    </select>
  );
}

// CommandsPanel — buttons for the dimos @rpc commands the gateway advertises as browser-callable
// (useCommands ← the gateway's `hello` whitelist). Click → client.call(target, method) → show the
// return value. Renders NOTHING when no commands are advertised (e.g. Bun↔LCM has no RPC bridge),
// so the panel only appears where the framework's command registry is actually reachable.
import { useState } from "react";
import { useCommands, useRpc } from "@dimos/react";

export function CommandsPanel() {
  const commands = useCommands();
  const { call } = useRpc();
  const [busy, setBusy] = useState<string | null>(null);
  const [last, setLast] = useState<string>();

  if (!commands.length) return null;

  return (
    <div className="panel">
      <div className="panel-title">Commands · dimos @rpc</div>
      <div style={{ display: "flex", flexWrap: "wrap", gap: 6 }}>
        {commands.map((c) => {
          const key = `${c.target}/${c.method}`;
          return (
            <button
              key={key}
              className="tab"
              disabled={busy !== null}
              title={key}
              onClick={async () => {
                setBusy(key);
                try {
                  const res = await call(c.target, c.method);
                  setLast(`${c.label} → ${JSON.stringify(res)}`);
                } catch (e) {
                  setLast(`${c.label} ✗ ${(e as Error).message}`);
                } finally {
                  setBusy(null);
                }
              }}
            >
              {busy === key ? `${c.label}…` : c.label}
            </button>
          );
        })}
      </div>
      {last && (
        <div className="muted small" style={{ marginTop: 6 }} title={last}>
          {last}
        </div>
      )}
    </div>
  );
}

import { useEffect, useState } from "react";
import { ControlPanel } from "@dimos/cockpit/panels/ControlPanel.tsx";
import { useOptionalSlot } from "@dimos/cockpit/panels/hooks.ts";
import { paramChannel } from "@dimos/cockpit/panels/panelParams.ts";
import type { PanelProps } from "@dimos/cockpit/panels/registry.tsx";
import { txReasonText } from "@dimos/cockpit/panels/txReason.ts";
import { readSnapshot } from "./worldModel.ts";
import styles from "./controls.module.css";

/** Extend the stock strip without copying its policy or mode controls. */
export function WorldControls(props: PanelProps) {
  const { spec, store, teleop } = props;
  const command = paramChannel(spec, "command", 3);
  const policy = useOptionalSlot(store, paramChannel(spec, "policies", 1))?.value;
  const count = policy && typeof policy === "object" && "respawns" in policy &&
      typeof policy.respawns === "number"
    ? policy.respawns
    : null;
  const world = readSnapshot(useOptionalSlot(store, "world_state")?.value);
  const balls = [["football_ball_1", "Ball 1"], ["football_ball_2", "Ball 2"], ["football_ball_3", "Ball 3"], ["ball", "Benchmark ball"]];
  const [dropping, setDropping] = useState<Record<string, { count: number; at: number }>>({});
  useEffect(() => {
    setDropping(old => {
      const entries = Object.entries(old).filter(([ball, pending]) => {
        if ((world?.football?.drops?.[ball] ?? 0) > pending.count) return false;
        return true;
      });
      return entries.length === Object.keys(old).length ? old : Object.fromEntries(entries);
    });
  }, [world]);
  useEffect(() => {
    const entries = Object.values(dropping);
    if (!entries.length) return;
    const timer = setTimeout(() => {
      setError("Ball reset timed out. Try again.");
      setDropping(old => Object.fromEntries(Object.entries(old).filter(([, value]) => Date.now() - value.at < 6000)));
    }, Math.max(1, Math.min(...entries.map(value => value.at + 6000 - Date.now()))));
    return () => clearTimeout(timer);
  }, [dropping]);
  const drop = (ball: string) => {
    if (!teleop || !command) return;
    const result = teleop.tx(command, { name: "drop_ball", args: { ball } });
    if (!result.ok) { setError(txReasonText(result.reason)); return; }
    setError("");
    setDropping(old => ({ ...old, [ball]: { count: world?.football?.drops?.[ball] ?? 0, at: Date.now() } }));
  };
  const [pending, setPending] = useState<number | null>(null);
  const [error, setError] = useState("");

  useEffect(() => {
    if (pending === null) return;
    if (count !== null && count > pending) {
      setPending(null);
      return;
    }
    const timer = setTimeout(() => {
      setPending(null);
      setError("Waiting for a clear midfield spawn or a connection.");
    }, 6000);
    return () => clearTimeout(timer);
  }, [count, pending]);

  const respawn = () => {
    if (!teleop || !command || count === null) return;
    const result = teleop.tx(command, { name: "respawn", args: {} });
    if (!result.ok) {
      setError(txReasonText(result.reason));
      return;
    }
    setError("");
    setPending(count);
  };

  return (
    <div className={styles.controls}>
      <ControlPanel {...props} />
      <div className={styles.recovery}>
        <button
          type="button"
          className={styles.respawn}
          data-testid="duck-respawn"
          title="Return your duck to the midfield touchline when a spawn is clear. Keeps your map and conversation."
          disabled={!teleop || !command || count === null || pending !== null}
          onClick={respawn}
        >
          {pending === null ? "↻ Respawn" : "Respawning…"}
        </button>
        <span className={styles.caption}>Simulation reset</span>
        {error && <span className={styles.error} role="alert">{error}</span>}
      </div>
      <div className={styles.ballDrops} aria-label="Reset footballs">
        <span className={styles.caption}>Drop at midfield · 2 m</span>
        <div>{balls.map(([ball, label]) => <button key={ball} className={styles.respawn} disabled={!teleop || !command || !!dropping[ball]} onClick={() => drop(ball)} title={`Drop ${label.toLowerCase()} from 2 metres above midfield`}>{dropping[ball] ? "Dropping…" : `↧ ${label}`}</button>)}</div>
      </div>
    </div>
  );
}

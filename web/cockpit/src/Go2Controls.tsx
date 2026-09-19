import { useEffect, useRef, useState } from "react";
import type { Session } from "@dimos/sdk";
import { useChannel, useStatus } from "@dimos/sdk/react";
import styles from "./OperatorView.module.css";

const COMMAND = "go2_operator_command";
const ACTIONS = [
  ["StandReady", "Stand / Drive"],
  ["StandDown", "Sit / Lie down"],
  ["Hello", "Wave / Shake hand"],
  ["Stretch", "Stretch"],
  ["RecoveryStand", "Recover stand"],
];
interface RobotState {
  battery: number | null;
  light_requested: number | null;
  last_action: string | null;
}
interface Result {
  id: string;
  ok: boolean;
  message: string;
}

export function Go2Controls({ session }: { session: Session }) {
  const status = useStatus(session);
  const telemetry = useChannel(session, "go2_operator_state");
  const result = useChannel(session, "go2_operator_result");
  const robot = telemetry.slot?.value as RobotState | undefined;
  const reply = result.slot?.value as Result | undefined;
  const [level, setLevel] = useState(0);
  const [message, setMessage] = useState("No command sent");
  const [pending, setPending] = useState<string | null>(null);
  const request = useRef<string | null>(null);
  const available = status.transport.phase === "connected" &&
    status.manifest?.channels.some((channel) => channel.ch === COMMAND);
  const fresh = telemetry.stats.ageMs !== null && telemetry.stats.ageMs < 3500;

  useEffect(() => {
    if (reply && reply.id === request.current) {
      request.current = null;
      setPending(null);
      setMessage(reply.message);
    }
  }, [reply]);
  useEffect(() => {
    if (!pending) return;
    const timer = setTimeout(() => {
      request.current = null;
      setPending(null);
      setMessage("No robot result received; outcome unknown. Not retried.");
    }, 15000);
    return () => clearTimeout(timer);
  }, [pending]);
  useEffect(() => {
    if (!available && request.current) {
      request.current = null;
      setPending(null);
      setMessage("Connection lost; command outcome unknown. Not retried.");
    }
  }, [available]);

  async function send(action: string, light?: number) {
    if (!available || request.current) return;
    const id = crypto.randomUUID();
    request.current = id;
    setPending(action);
    setMessage("Sending to robot…");
    try {
      await session.publish(COMMAND, {
        id,
        action,
        ...(light === undefined ? {} : { level: light }),
      });
      if (request.current === id) setMessage("Delivered to DimOS; waiting for robot API…");
    } catch (error) {
      if (request.current !== id) return;
      request.current = null;
      setPending(null);
      setMessage(
        `Command not confirmed: ${error instanceof Error ? error.message : String(error)}`,
      );
    }
  }

  return (
    <section className={styles.card} aria-label="Go2 actions">
      <h2>Robot</h2>
      <p>
        Battery <strong>{fresh && robot?.battery != null ? `${robot.battery}%` : "—"}</strong>
        {!fresh && " · telemetry unavailable / stale"}
      </p>
      <h2>Posture & actions</h2>
      <div className={styles.actions}>
        {ACTIONS.map(([action, label]) => (
          <button
            key={action}
            disabled={!available || pending !== null}
            onClick={() => void send(action)}
          >
            {label}
          </button>
        ))}
      </div>
      <p>Last accepted action: {fresh ? robot?.last_action ?? "—" : "—"}</p>
      <label>
        Headlight · {level * 10}%
        <input
          aria-label="Headlight brightness"
          type="range"
          min="0"
          max="10"
          step="1"
          value={level}
          onChange={(event) => setLevel(Number(event.target.value))}
        />
      </label>
      <button disabled={!available || pending !== null} onClick={() => void send("light", level)}>
        Apply light
      </button>
      <p>
        Last accepted light request:{" "}
        {fresh && robot?.light_requested != null ? `${robot.light_requested * 10}%` : "—"}
      </p>
      <p role="status">{message}</p>
      <p>Robot API acceptance does not confirm physical pose or action completion.</p>
    </section>
  );
}

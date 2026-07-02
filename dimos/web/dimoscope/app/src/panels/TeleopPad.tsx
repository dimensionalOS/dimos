// TeleopPad — WASD / arrows / buttons → safe velocity commands at 10 Hz. Each carries ttlMs, so the
// gateway deadman-stops the robot if input ceases or the socket drops.
import { useEffect, useRef, useState } from "react";
import { useTeleop } from "../dimos";

const LIN = 0.6; // m/s
const ANG = 1.0; // rad/s
const KEYS = ["w", "a", "s", "d", "ArrowUp", "ArrowDown", "ArrowLeft", "ArrowRight", " "];

export function TeleopPad() {
  const { drive, stop } = useTeleop();
  const held = useRef(new Set<string>());
  const [active, setActive] = useState(false);

  useEffect(() => {
    // Never steal keys from a text field: "localhost" contains a+s, and arrows must move the
    // cursor, not the robot. keyup stays unguarded so a key held before focusing still releases.
    const editing = (e: KeyboardEvent) =>
      e.target instanceof HTMLInputElement ||
      e.target instanceof HTMLTextAreaElement ||
      e.target instanceof HTMLSelectElement ||
      (e.target instanceof HTMLElement && e.target.isContentEditable);
    const down = (e: KeyboardEvent) => {
      if (editing(e)) return;
      if (KEYS.includes(e.key)) {
        held.current.add(e.key);
        e.preventDefault();
      }
    };
    const up = (e: KeyboardEvent) => held.current.delete(e.key);
    window.addEventListener("keydown", down);
    window.addEventListener("keyup", up);
    const id = setInterval(() => {
      const k = held.current;
      let lin = 0,
        ang = 0;
      if (k.has("w") || k.has("ArrowUp")) lin += LIN;
      if (k.has("s") || k.has("ArrowDown")) lin -= LIN;
      if (k.has("a") || k.has("ArrowLeft")) ang += ANG;
      if (k.has("d") || k.has("ArrowRight")) ang -= ANG;
      if (lin || ang) drive(lin, ang, 400);
      setActive(lin !== 0 || ang !== 0); // React bails out when the value is unchanged
    }, 100);
    return () => {
      window.removeEventListener("keydown", down);
      window.removeEventListener("keyup", up);
      clearInterval(id);
      stop();
    };
  }, [drive, stop]);

  const hold = (lin: number, ang: number) => ({
    onMouseDown: () => drive(lin, ang, 500),
    onMouseUp: () => stop(),
    onMouseLeave: () => stop(),
  });

  return (
    <div className="panel">
      <div className="panel-title">Teleop · /cmd_vel {active ? "● live" : ""}</div>
      <div className="teleop-hint">
        WASD / arrows or buttons. Gateway clamps velocity + deadman-stops.
      </div>
      <div className="teleop-grid">
        <div />
        <button {...hold(LIN, 0)}>↑</button>
        <div />
        <button {...hold(0, ANG)}>←</button>
        <button onClick={() => stop()}>■</button>
        <button {...hold(0, -ANG)}>→</button>
        <div />
        <button {...hold(-LIN, 0)}>↓</button>
        <div />
      </div>
    </div>
  );
}

// TeleopPad — closes the loop (Rung 4). Publishes /cmd_vel Twist from WASD /
// arrow keys (and on-screen buttons). Repeats at 10Hz while a key is held,
// because the robot zeroes its velocity after ~0.5s without a command.
import { useEffect, useRef, useState } from "react";
import { useBus } from "../useBus";

const LIN = 0.7; // m/s
const ANG = 1.0; // rad/s

export function TeleopPad() {
  const bus = useBus();
  const keys = useRef(new Set<string>());
  const [active, setActive] = useState(false);

  useEffect(() => {
    const vel = () => {
      const k = keys.current;
      const lin = (k.has("w") || k.has("ArrowUp") ? LIN : 0) + (k.has("s") || k.has("ArrowDown") ? -LIN : 0);
      const ang = (k.has("a") || k.has("ArrowLeft") ? ANG : 0) + (k.has("d") || k.has("ArrowRight") ? -ANG : 0);
      return { lin, ang };
    };
    const down = (e: KeyboardEvent) => {
      if (["w", "a", "s", "d", "ArrowUp", "ArrowDown", "ArrowLeft", "ArrowRight"].includes(e.key)) {
        keys.current.add(e.key);
        setActive(true);
        e.preventDefault();
      }
    };
    const up = (e: KeyboardEvent) => {
      keys.current.delete(e.key);
      if (keys.current.size === 0) setActive(false);
    };
    window.addEventListener("keydown", down);
    window.addEventListener("keyup", up);
    const id = setInterval(() => {
      const { lin, ang } = vel();
      if (lin || ang) bus.publishTwist(lin, ang);
    }, 100);
    return () => {
      window.removeEventListener("keydown", down);
      window.removeEventListener("keyup", up);
      clearInterval(id);
    };
  }, [bus]);

  const tap = (lin: number, ang: number) => bus.publishTwist(lin, ang);

  return (
    <div className="panel">
      <div className="panel-title">Teleop · /cmd_vel {active ? "🟢" : ""}</div>
      <div className="teleop-hint">Focus the page, then use WASD / arrow keys.</div>
      <div className="teleop-grid">
        <span />
        <button onMouseDown={() => tap(LIN, 0)} onMouseUp={() => tap(0, 0)}>▲</button>
        <span />
        <button onMouseDown={() => tap(0, ANG)} onMouseUp={() => tap(0, 0)}>◀</button>
        <button onMouseDown={() => tap(0, 0)}>■</button>
        <button onMouseDown={() => tap(0, -ANG)} onMouseUp={() => tap(0, 0)}>▶</button>
        <span />
        <button onMouseDown={() => tap(-LIN, 0)} onMouseUp={() => tap(0, 0)}>▼</button>
        <span />
      </div>
    </div>
  );
}

import { useRef, useState } from "react";
import type { CSSProperties, PointerEvent } from "react";
import type { MoveCommand } from "@/lib/types";
import { computeDrive } from "@/lib/joystick";

const KNOB = 84; // px, must match the knob element size below

const padStyle: CSSProperties = {
  background:
    "radial-gradient(circle at 50% 42%, #16191f 0%, #0e1014 72%, #0a0c0f 100%)",
  boxShadow:
    "inset 0 2px 6px rgba(0,0,0,.6), inset 0 -2px 10px rgba(255,255,255,.02), 0 0 0 1px var(--color-line)",
};

const knobStyle: CSSProperties = {
  background: "radial-gradient(circle at 50% 34%, #f0cd86, #e3b15e 60%, #c79236)",
  boxShadow:
    "0 8px 18px rgba(0,0,0,.55), inset 0 2px 2px rgba(255,255,255,.5), inset 0 -6px 12px rgba(0,0,0,.25)",
};

function Tick({ className }: { className: string }) {
  return (
    <span
      className={`pointer-events-none absolute h-[11px] w-[3px] rounded bg-faint opacity-50 ${className}`}
    />
  );
}

export default function Joystick({
  onMove,
  onEnd,
}: {
  onMove?: (m: MoveCommand) => void;
  onEnd?: () => void;
}) {
  const padRef = useRef<HTMLDivElement>(null);
  const activeId = useRef<number | null>(null);
  const [knob, setKnob] = useState({ x: 0, y: 0 });

  function update(e: PointerEvent) {
    const el = padRef.current;
    if (!el) return;
    const r = el.getBoundingClientRect();
    const travel = r.width / 2 - KNOB / 2; // keep knob inside the pad
    const d = computeDrive(
      e.clientX - (r.left + r.width / 2),
      e.clientY - (r.top + r.height / 2),
      travel,
    );
    setKnob({ x: d.knobX, y: d.knobY });
    onMove?.({ vx: d.vx, vy: 0, turn: d.turn });
  }

  function handleDown(e: PointerEvent) {
    e.preventDefault();
    activeId.current = e.pointerId;
    padRef.current?.setPointerCapture(e.pointerId);
    update(e);
  }

  function handleMove(e: PointerEvent) {
    if (activeId.current === e.pointerId) update(e);
  }

  function handleUp(e: PointerEvent) {
    if (activeId.current !== e.pointerId) return;
    activeId.current = null;
    setKnob({ x: 0, y: 0 });
    onEnd?.();
  }

  return (
    <div className="flex flex-1 flex-col items-center justify-center gap-3.5">
      <div
        ref={padRef}
        className="relative h-[218px] w-[218px] touch-none rounded-full"
        style={padStyle}
        onPointerDown={handleDown}
        onPointerMove={handleMove}
        onPointerUp={handleUp}
        onPointerCancel={handleUp}
      >
        <Tick className="left-1/2 top-[14px] -translate-x-1/2" />
        <Tick className="bottom-[14px] left-1/2 -translate-x-1/2" />
        <Tick className="left-[14px] top-1/2 -translate-y-1/2 rotate-90" />
        <Tick className="right-[14px] top-1/2 -translate-y-1/2 rotate-90" />
        <div
          className="absolute left-1/2 top-1/2 h-[84px] w-[84px] rounded-full"
          style={{
            ...knobStyle,
            transform: `translate(calc(-50% + ${knob.x}px), calc(-50% + ${knob.y}px))`,
            transition: activeId.current === null ? "transform 0.15s ease-out" : "none",
          }}
        />
      </div>
      <div className="text-[13px] text-faint">Drag to drive · release to stop</div>
    </div>
  );
}

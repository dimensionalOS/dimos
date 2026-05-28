import type { MoveCommand, QuickAction } from "@/lib/types";
import Joystick from "./Joystick";
import QuickActions from "./QuickActions";

export default function ManualPanel({
  drive,
  busy,
  linkConnected,
  linkConfigured,
  onMove,
  onEnd,
  onAction,
}: {
  drive: MoveCommand;
  busy?: boolean;
  linkConnected?: boolean;
  linkConfigured?: boolean;
  onMove: (m: MoveCommand) => void;
  onEnd: () => void;
  onAction: (a: QuickAction) => void;
}) {
  const active = drive.vx !== 0 || drive.turn !== 0;
  // teleop link state: not configured (mock) / configured-but-down / live
  const linkDown = linkConfigured && !linkConnected;
  return (
    <div className="flex flex-1 flex-col">
      <Joystick onMove={onMove} onEnd={onEnd} />
      <div className="rounded-2xl border border-line bg-surface px-4 py-3.5">
        <div className="flex items-baseline justify-between">
          <span className="text-[9.5px] uppercase tracking-[1.5px] text-faint">
            {active ? "Driving" : "Manual"}
          </span>
          {linkDown ? (
            <span className="font-mono text-[12px] text-red">teleop link down</span>
          ) : (
            <span className="font-mono text-[12px] text-[#c4c8d0]">
              vx {drive.vx.toFixed(2)} · turn {drive.turn.toFixed(2)}
            </span>
          )}
        </div>
      </div>
      <QuickActions onAction={onAction} disabled={busy} />
    </div>
  );
}

import type { QuickAction } from "@/lib/types";

const DEFAULT_ACTIONS: QuickAction[] = [
  { label: "Sit", command: "Sit" },
  { label: "Jump", command: "Jump" }, // TBD: not a documented Unitree command yet
  { label: "Lie down", command: "StandDown" },
];

export default function QuickActions({
  onAction,
  disabled,
  actions = DEFAULT_ACTIONS,
}: {
  onAction: (a: QuickAction) => void;
  disabled?: boolean;
  actions?: QuickAction[];
}) {
  return (
    <div className="mt-3 flex gap-2.5">
      {actions.map((a) => (
        <button
          key={a.label}
          type="button"
          disabled={disabled}
          onClick={() => onAction(a)}
          className="flex-1 rounded-[13px] border border-line bg-surface py-3 text-[13px] font-semibold text-[#cfd3da] transition-colors active:bg-surface-2 disabled:opacity-40"
        >
          {a.label}
        </button>
      ))}
    </div>
  );
}

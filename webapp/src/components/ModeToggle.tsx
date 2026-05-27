import type { Mode } from "@/lib/types";

const MODES: Mode[] = ["voice", "manual"];

export default function ModeToggle({
  mode,
  onChange,
}: {
  mode: Mode;
  onChange: (m: Mode) => void;
}) {
  return (
    <div className="mx-5 mt-4 flex rounded-[13px] border border-line bg-[#0c0d11] p-[3px]">
      {MODES.map((m) => {
        const active = mode === m;
        const goldActive = active && m === "voice";
        return (
          <button
            key={m}
            type="button"
            onClick={() => onChange(m)}
            className={`flex-1 rounded-[10px] py-2 text-[13px] font-semibold capitalize transition-colors ${
              goldActive
                ? "bg-gold text-[#1a1500]"
                : active
                  ? "bg-surface-2 text-fg"
                  : "text-muted"
            }`}
          >
            {m}
          </button>
        );
      })}
    </div>
  );
}

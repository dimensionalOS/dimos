import ConnectionChip from "./ConnectionChip";

export default function Header({
  connected,
  ttsEnabled,
  onToggleTts,
}: {
  connected: boolean;
  ttsEnabled: boolean;
  onToggleTts: () => void;
}) {
  return (
    <header className="flex items-center justify-between px-5 pt-7">
      <span className="flex items-center gap-2 text-[17px] font-semibold tracking-tight text-fg">
        <span className="h-[7px] w-[7px] rounded-full bg-gold shadow-[0_0_7px_var(--color-gold)]" />
        Goldie
      </span>
      <div className="flex items-center gap-2">
        <button
          type="button"
          onClick={onToggleTts}
          aria-pressed={ttsEnabled}
          aria-label={
            ttsEnabled ? "Mute spoken responses" : "Unmute spoken responses"
          }
          className="flex items-center gap-1.5 rounded-full border border-line bg-surface px-2.5 py-1 text-[11px] text-muted"
        >
          <span
            className={`h-1.5 w-1.5 rounded-full ${ttsEnabled ? "bg-gold" : "bg-faint"}`}
          />
          speech
        </button>
        <ConnectionChip connected={connected} />
      </div>
    </header>
  );
}

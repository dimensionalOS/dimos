import type { CSSProperties, PointerEvent } from "react";

const idleStyle: CSSProperties = {
  background: "radial-gradient(circle at 50% 36%, #1f232b, #15181e)",
  boxShadow:
    "0 14px 30px rgba(0,0,0,.5), inset 0 1px 1px rgba(255,255,255,.06), inset 0 -10px 20px rgba(0,0,0,.45), 0 0 0 1px var(--color-line-2)",
  color: "#cfd3da",
};

const recStyle: CSSProperties = {
  background: "radial-gradient(circle at 50% 40%, #ff5a4f, #c4271d)",
  boxShadow:
    "0 0 34px rgba(255,69,58,.45), inset 0 1px 2px rgba(255,255,255,.25), 0 0 0 1px #ff6b61",
  color: "#fff",
};

export default function VoiceButton({
  recording,
  disabled,
  transcript,
  error,
  onStart,
  onStop,
}: {
  recording: boolean;
  disabled?: boolean;
  transcript?: string;
  error?: string;
  onStart: () => void;
  onStop: () => void;
}) {
  const start = (e: PointerEvent) => {
    e.preventDefault();
    if (!disabled) onStart();
  };
  const stop = (e: PointerEvent) => {
    e.preventDefault();
    onStop();
  };
  return (
    <div className="flex flex-1 flex-col items-center justify-center gap-3.5">
      <button
        type="button"
        disabled={disabled}
        onPointerDown={start}
        onPointerUp={stop}
        onPointerCancel={stop}
        onPointerLeave={(e) => {
          if (recording) stop(e);
        }}
        onContextMenu={(e) => e.preventDefault()}
        aria-label="Hold to speak a command"
        aria-pressed={recording}
        style={recording ? recStyle : idleStyle}
        className="flex h-[184px] w-[184px] items-center justify-center rounded-full text-center text-sm font-semibold leading-tight transition-transform active:scale-[0.98] disabled:opacity-40"
      >
        {recording ? (
          "Listening…"
        ) : (
          <span>
            Hold
            <br />
            to speak
          </span>
        )}
      </button>
      <div
        className={`min-h-[18px] px-6 text-center text-[13px] italic ${
          error ? "text-red" : transcript ? "text-gold" : "text-faint"
        }`}
      >
        {error || transcript ||" "}
      </div>
    </div>
  );
}

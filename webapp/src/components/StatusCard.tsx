import { useEffect, useRef } from "react";
import type { AgentMessage } from "@/lib/agentMessage";

export default function StatusCard({
  messages,
  active,
}: {
  messages: AgentMessage[];
  active: boolean;
}) {
  const endRef = useRef<HTMLDivElement>(null);
  useEffect(() => {
    endRef.current?.scrollIntoView({ block: "end" });
  }, [messages]);

  const has = messages.length > 0;

  return (
    <div
      className="rounded-2xl border border-line bg-surface px-4 py-3"
      aria-live="polite"
      aria-label="Goldie responses"
    >
      <div className="flex items-center justify-between">
        <span className="text-[9.5px] uppercase tracking-[1.5px] text-faint">
          Goldie
        </span>
        {active && (
          <span className="flex items-center gap-1.5 text-[10px] text-gold">
            <span className="h-1.5 w-1.5 animate-pulse rounded-full bg-gold" />
            thinking…
          </span>
        )}
      </div>

      {has ? (
        <div className="mt-2 flex max-h-32 flex-col gap-1.5 overflow-y-auto">
          {messages.map((m, i) => {
            const latest = i === messages.length - 1;
            const tone =
              m.kind === "spoke"
                ? "text-gold"
                : m.kind === "status"
                  ? "text-[12px] text-faint"
                  : latest
                    ? "text-fg"
                    : "text-muted";
            return (
              <p
                key={i}
                className={`text-[13px] leading-snug ${tone} ${latest ? "" : "opacity-80"}`}
              >
                {m.text}
              </p>
            );
          })}
          <div ref={endRef} />
        </div>
      ) : (
        <p className="mt-1.5 text-[13px] text-muted">
          Ready — hold to give a command
        </p>
      )}
    </div>
  );
}

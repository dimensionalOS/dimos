import { cn } from "@/lib/utils";

// A titled, bordered "console" panel with an optional LIVE pulse in the header.
// Shared by the live-job view and the robot Console.
export function ConsolePanel({
  title,
  live,
  className,
  children,
}: {
  title: string;
  live?: boolean;
  className?: string;
  children: React.ReactNode;
}) {
  return (
    <div className={cn("overflow-hidden rounded-xl border bg-card", className)}>
      <div className="flex items-center justify-between border-b bg-secondary/30 px-4 py-2">
        <span className="font-mono text-muted-foreground text-xs uppercase tracking-wider">
          {title}
        </span>
        {live ? (
          <span className="inline-flex items-center gap-1.5 font-medium font-mono text-[10px] text-signal uppercase tracking-wide">
            <span className="size-1.5 rounded-full bg-signal pulse-dot" /> Live
          </span>
        ) : null}
      </div>
      {children}
    </div>
  );
}

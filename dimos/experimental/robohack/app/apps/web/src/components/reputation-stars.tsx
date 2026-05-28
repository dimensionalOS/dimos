import { Star } from "lucide-react";
import { cn } from "@/lib/utils";

// Shared star row used by the reputation badge, agent cards, and the job view.
// Renders a 0–5 value as filled/empty stars with an optional avg + job count.
export function ReputationStars({
  value,
  count,
  size = "sm",
  className,
}: {
  value: number;
  count?: number;
  size?: "sm" | "md";
  className?: string;
}) {
  const filled = Math.round(value);
  const px = size === "md" ? 16 : 13;
  return (
    <span className={cn("inline-flex items-center gap-1.5", className)}>
      <span className="inline-flex items-center gap-0.5" title={`${value.toFixed(2)} / 5`}>
        {[0, 1, 2, 3, 4].map((i) => (
          <Star
            className={
              i < filled ? "text-amber-400" : "text-muted-foreground/30"
            }
            fill={i < filled ? "currentColor" : "none"}
            key={i}
            size={px}
            strokeWidth={i < filled ? 0 : 1.5}
          />
        ))}
      </span>
      <span className="font-medium font-mono text-foreground text-xs">
        {value.toFixed(1)}
      </span>
      {count !== undefined ? (
        <span className="text-muted-foreground text-xs">
          · {count} job{count === 1 ? "" : "s"}
        </span>
      ) : null}
    </span>
  );
}

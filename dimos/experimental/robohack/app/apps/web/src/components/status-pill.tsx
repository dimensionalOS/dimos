import type { JobStatus } from "@robomoo/shared";
import { cn } from "@/lib/utils";

// Single source of truth for how a job status reads across the app (was
// duplicated inline in jobs-list + agent-jobs). Live/in-flight states carry a
// pulsing dot so a glance tells you the robot is working.
const STATUS: Record<
  JobStatus,
  { label: string; className: string; live?: boolean }
> = {
  booked: { label: "Booked", className: "bg-muted text-muted-foreground" },
  dispatched: {
    label: "Dispatched",
    className: "bg-working/15 text-working",
    live: true,
  },
  scanning: {
    label: "Scanning",
    className: "bg-working/15 text-working",
    live: true,
  },
  reconstructing: {
    label: "Reconstructing",
    className: "bg-working/15 text-working",
    live: true,
  },
  done: { label: "Done", className: "bg-signal/15 text-signal" },
  failed: { label: "Failed", className: "bg-destructive/15 text-destructive" },
  cancelled: {
    label: "Cancelled",
    className: "bg-destructive/15 text-destructive",
  },
};

export function StatusPill({
  status,
  className,
}: {
  status: JobStatus;
  className?: string;
}) {
  const s = STATUS[status] ?? STATUS.booked;
  return (
    <span
      className={cn(
        "inline-flex items-center gap-1.5 rounded-full px-2 py-0.5 font-medium text-[10px] uppercase tracking-wide",
        s.className,
        className,
      )}
    >
      {s.live ? (
        <span className="pulse-dot size-1.5 rounded-full bg-current" />
      ) : null}
      {s.label}
    </span>
  );
}

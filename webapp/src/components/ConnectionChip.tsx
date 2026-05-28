export default function ConnectionChip({ connected }: { connected: boolean }) {
  return (
    <span className="flex items-center gap-1.5 rounded-full border border-line bg-surface px-2.5 py-1 text-[11px] text-muted">
      <span
        className={`h-1.5 w-1.5 rounded-full ${connected ? "bg-green" : "bg-faint"}`}
      />
      {connected ? "connected" : "offline"}
    </span>
  );
}

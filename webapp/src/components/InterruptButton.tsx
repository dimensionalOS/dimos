export default function InterruptButton({
  onInterrupt,
  disabled,
}: {
  onInterrupt: () => void;
  disabled?: boolean;
}) {
  return (
    <button
      type="button"
      disabled={disabled}
      onClick={onInterrupt}
      className="mt-3 flex items-center justify-center gap-2.5 rounded-[14px] border border-red/30 bg-red-soft py-4 text-sm font-bold tracking-wide text-red transition active:scale-[0.99] disabled:opacity-50"
    >
      <span className="inline-block h-[11px] w-[11px] rounded-[2.5px] bg-red" />
      Interrupt
    </button>
  );
}

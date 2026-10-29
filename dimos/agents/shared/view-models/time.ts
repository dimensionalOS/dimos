import type { EpochMs } from "../protocol/time.ts";

/** "14:02", in the viewer's zone, as refusals word times. */
export function clock(at: EpochMs): string {
  return new Date(at).toLocaleTimeString([], { hour12: false, hour: "2-digit", minute: "2-digit" });
}

/** "14:06:12". */
export function clockSeconds(at: EpochMs): string {
  return new Date(at).toLocaleTimeString([], { hour12: false });
}

/** "41s", "3m12s", "6h12m", "2d4h". */
export function duration(ms: number): string {
  const seconds = Math.max(0, Math.floor(ms / 1000));
  if (seconds < 60) return `${seconds}s`;
  const minutes = Math.floor(seconds / 60);
  if (minutes < 60) return `${minutes}m${seconds % 60}s`;
  const hours = Math.floor(minutes / 60);
  if (hours < 24) return `${hours}h${minutes % 60}m`;
  return `${Math.floor(hours / 24)}d${hours % 24}h`;
}

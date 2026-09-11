import type { ChannelStore } from "@dimos/sdk";
import { useStoreChannel } from "@dimos/sdk/react";
import styles from "@dimos/cockpit/layout/PanelFrame.module.css";
/** Sink-side draw diagnostics, mutated in place and sampled by the badge. */
export interface DrawHealth {
  /** Browser ms of the last successful draw, stamped at sink start before the first. */
  lastDrawOkAtMs: number;
  /** Consecutive failed decode-or-draw attempts since the last success. */
  failures: number;
}

/** Hz/staleness readout for a canvas panel's primary channel. Re-rendered on
 * the 500 ms UI tick via useChannel; `health` is mutated by the sink at draw
 * rate and simply sampled here (intended coupling). */
export function Badge({ store, ch, health, staleMs, unit, testId }: {
  store: ChannelStore;
  ch: string;
  health: DrawHealth;
  staleMs: number;
  unit: string;
  testId: string;
}) {
  const { stats } = useStoreChannel(store, ch);
  let text: string;
  let error = false;
  let stale = false;
  if (stats.frames === 0) {
    // Nothing ever arrived; a corrupt first frame is an error, not "waiting".
    text = "waiting";
  } else if (stats.decodeFailing || health.failures > 0) {
    // A single bad frame trips this; the next success clears it.
    text = "decode failing";
    error = true;
  } else if (stats.ageMs !== null && stats.ageMs > staleMs) {
    text = `stale ${(stats.ageMs / 1000).toFixed(1)} s`;
    stale = true;
  } else if (stats.lastFrameAtMs - health.lastDrawOkAtMs > staleMs) {
    // Frames arrive but nothing draws (e.g. a decoder that never settles);
    // both operands are browser milliseconds.
    text = "stalled";
    stale = true;
  } else {
    text = `${stats.hz.toFixed(1)} ${unit}`;
  }
  return (
    <span
      className={error || stale ? styles.badgeStale : styles.badge}
      data-testid={testId}
      data-stale={stale || undefined}
      data-error={error || undefined}
      role="status"
    >
      {text}
    </span>
  );
}

// Shared panel chrome: frame, title bar (title, falling back to the panel
// id), badge slot, and client-side maximize (CSS overlay + Esc restore --
// deliberately view state, not manifest state). The Badge previously lived
// as near-identical copies in VideoPanel/MapPanel; panels now pass their
// primary channel and thresholds instead.

import { type ReactNode, useEffect, useState } from "react";
import type { PanelSpec } from "@dimos/shared";
import { useChannel } from "../session/hooks.ts";
import type { ChannelStore } from "../session/store.ts";
import styles from "./PanelFrame.module.css";

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
  const { stats } = useChannel(store, ch);
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

export function PanelFrame({ spec, badge, children }: {
  spec: PanelSpec;
  badge?: ReactNode;
  children: ReactNode;
}) {
  const [maximized, setMaximized] = useState(false);
  useEffect(() => {
    if (!maximized) return;
    const onKey = (e: KeyboardEvent): void => {
      if (e.key === "Escape") setMaximized(false);
    };
    document.addEventListener("keydown", onKey);
    return () => document.removeEventListener("keydown", onKey);
  }, [maximized]);
  return (
    <section
      className={maximized ? styles.frameMax : styles.frame}
      data-testid={`panel-${spec.id}`}
      data-maximized={maximized || undefined}
    >
      <div className={styles.head}>
        <span className={styles.title}>{spec.title !== "" ? spec.title : spec.id}</span>
        <span className={styles.controls}>
          {badge}
          <button
            type="button"
            className={styles.maxButton}
            aria-label={maximized ? "restore" : "maximize"}
            data-testid={`panel-${spec.id}-max`}
            onClick={() => setMaximized((v) => !v)}
          >
            ⛶
          </button>
        </span>
      </div>
      <div className={styles.body}>{children}</div>
    </section>
  );
}

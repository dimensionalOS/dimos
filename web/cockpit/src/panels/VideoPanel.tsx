// Live JPEG video: canvas drawing driven by the store's direct-subscribe path
// (React is not involved at frame rate; the badge rides the 500 ms UI tick).

import { useEffect, useRef } from "react";
import { useChannel } from "../session/hooks.ts";
import type { ChannelStore } from "../session/store.ts";
import type { PanelProps } from "./registry.ts";
import styles from "./VideoPanel.module.css";

// A frame this far behind source time is flagged stale (frames are useless
// the moment a newer one exists, so this only trips on silence or stalls).
export const VIDEO_STALE_MS = 2000;

/** Test seams: real decode is createImageBitmap; real hidden is the page's. */
export interface VideoSinkDeps {
  decode?: (payload: Uint8Array) => Promise<ImageBitmap>;
  hidden?: () => boolean;
}

/** Sink-side draw diagnostics, mutated in place and sampled by the badge. */
export interface DrawHealth {
  /** Browser ms of the last successful draw, stamped at sink start before the first. */
  lastDrawOkAtMs: number;
  /** Consecutive failed decode-or-draw attempts since the last success. */
  failures: number;
}

/**
 * Drive `canvas` from the channel's slot: at most one decode in flight, and
 * on completion the pump re-checks the slot, so a burst of frames costs one
 * decode of the newest (latest-wins, same shedding rule as everywhere else in
 * the pipeline). Undecodable frames are skipped without spinning. While the
 * document is hidden nothing decodes; visibilitychange catches back up.
 * Returns the cleanup function.
 */
export function startVideoSink(
  store: ChannelStore,
  ch: string,
  canvas: HTMLCanvasElement,
  health: DrawHealth,
  deps: VideoSinkDeps = {},
): () => void {
  const decode = deps.decode ??
    ((payload: Uint8Array) =>
      createImageBitmap(new Blob([payload as BlobPart], { type: "image/jpeg" })));
  const hidden = deps.hidden ?? (() => document.hidden);
  const ctx = canvas.getContext("2d");
  let decoding = false;
  let drawnVersion = -1;
  let stopped = false;
  // A fresh mount is never instantly "stalled".
  health.lastDrawOkAtMs = Date.now();
  health.failures = 0;

  const pump = (): void => {
    if (stopped || decoding || hidden()) return;
    const slot = store.get(ch);
    if (slot === null || slot.version === drawnVersion) return;
    if (!(slot.value instanceof Uint8Array)) return; // undecoded channel: nothing to draw
    const version = slot.version;
    decoding = true;
    decode(slot.value)
      .then((bmp) => {
        try {
          if (!stopped) {
            if (canvas.width !== bmp.width || canvas.height !== bmp.height) {
              canvas.width = bmp.width;
              canvas.height = bmp.height;
            }
            ctx?.drawImage(bmp, 0, 0);
            health.lastDrawOkAtMs = Date.now();
            health.failures = 0;
          }
        } finally {
          bmp.close(); // a throwing draw must not leak the bitmap
        }
      })
      .catch(() => {
        // Decode rejection or draw throw: skip this frame but count it, so
        // the badge can surface a pipeline that never draws.
        health.failures += 1;
      })
      .finally(() => {
        drawnVersion = version;
        decoding = false;
        pump(); // newer frames may have landed during the decode
      });
  };

  const unsubscribe = store.subscribe(ch, pump);
  const onVisibility = (): void => pump();
  document.addEventListener("visibilitychange", onVisibility);
  pump(); // a slot may predate the mount
  return () => {
    stopped = true;
    unsubscribe();
    document.removeEventListener("visibilitychange", onVisibility);
  };
}

function Badge({ store, ch, health }: { store: ChannelStore; ch: string; health: DrawHealth }) {
  // Re-rendered on the 500 ms UI tick via useChannel; `health` is mutated by
  // the sink at draw rate and simply sampled here (intended coupling).
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
  } else if (stats.ageMs !== null && stats.ageMs > VIDEO_STALE_MS) {
    text = `stale ${(stats.ageMs / 1000).toFixed(1)} s`;
    stale = true;
  } else if (stats.lastFrameAtMs - health.lastDrawOkAtMs > VIDEO_STALE_MS) {
    // Frames arrive but nothing draws (e.g. a decoder that never settles);
    // both operands are browser milliseconds.
    text = "stalled";
    stale = true;
  } else {
    text = `${stats.hz.toFixed(1)} fps`;
  }
  return (
    <span
      className={error || stale ? styles.badgeStale : styles.badge}
      data-testid={`video-${ch}-badge`}
      data-stale={stale || undefined}
      data-error={error || undefined}
      role="status"
    >
      {text}
    </span>
  );
}

export function VideoPanel({ spec, store }: PanelProps) {
  const ch = spec.channels[0] as string | undefined;
  if (ch === undefined) {
    // A video panel without a channel is a bridge authoring mistake; render
    // it visibly instead of crashing the grid.
    return <div className={styles.panel}>video panel {spec.id}: no channel bound</div>;
  }
  return <VideoCanvas spec={spec} store={store} ch={ch} />;
}

function VideoCanvas({ spec, store, ch }: PanelProps & { ch: string }) {
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const health = useRef<DrawHealth>({ lastDrawOkAtMs: Date.now(), failures: 0 }).current;
  const { slot } = useChannel(store, ch);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (canvas === null) return;
    return startVideoSink(store, ch, canvas, health);
  }, [store, ch, health]);

  return (
    <div className={styles.panel} data-testid={`panel-${spec.id}`}>
      <div className={styles.head}>
        <span className={styles.title}>{spec.id}</span>
        <Badge store={store} ch={ch} health={health} />
      </div>
      <div className={styles.body}>
        <canvas
          ref={canvasRef}
          className={styles.canvas}
          data-testid={`video-${ch}-canvas`}
          role="img"
          aria-label={spec.id}
        />
        {slot === null && <span className={styles.waiting}>waiting for data...</span>}
      </div>
    </div>
  );
}

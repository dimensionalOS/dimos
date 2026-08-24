// Live JPEG video: canvas drawing driven by the store's direct-subscribe path
// (React is not involved at frame rate; the badge rides the 500 ms UI tick).

import { useEffect, useRef } from "react";
import { Badge, type DrawHealth, PanelFrame } from "../layout/PanelFrame.tsx";
import { useChannel } from "../session/hooks.ts";
import type { ChannelStore } from "../session/store.ts";
import type { PanelProps } from "./registry.tsx";
import styles from "./VideoPanel.module.css";

// A frame this far behind source time is flagged stale (frames are useless
// the moment a newer one exists, so this only trips on silence or stalls).
export const VIDEO_STALE_MS = 2000;

/** Test seams: real decode is createImageBitmap; real hidden is the page's. */
export interface VideoSinkDeps {
  decode?: (payload: Uint8Array) => Promise<ImageBitmap>;
  hidden?: () => boolean;
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

export function VideoPanel({ spec, store }: PanelProps) {
  const ch = spec.channels[0] as string | undefined;
  if (ch === undefined) {
    // A video panel without a channel is a bridge authoring mistake; render
    // it visibly instead of crashing the grid.
    return (
      <PanelFrame spec={spec}>
        <span className={styles.waiting}>video panel {spec.id}: no channel bound</span>
      </PanelFrame>
    );
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
    <PanelFrame
      spec={spec}
      badge={
        <Badge
          store={store}
          ch={ch}
          health={health}
          staleMs={VIDEO_STALE_MS}
          unit="fps"
          testId={`video-${ch}-badge`}
        />
      }
    >
      <canvas
        ref={canvasRef}
        className={styles.canvas}
        data-testid={`video-${ch}-canvas`}
        role="img"
        aria-label={spec.id}
      />
      {slot === null && <span className={styles.waiting}>waiting for data...</span>}
    </PanelFrame>
  );
}

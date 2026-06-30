// useCvOverlay — runs an in-browser object detector on the camera's decoded frames and composites
// boxes onto the SAME canvas useVideo draws to. This is the whole point of the WebCodecs frames path:
// JS gets the actual VideoFrame, so we can detect on it and draw a result registered to THAT frame.
// (Works on any "frames" channel — webcodecs/jpeg — and is inert on webrtc, which has no frames.)
//
// Wiring: pass `onFrame` to useVideo({ onFrame }). useVideo calls it per decoded frame, after drawing
// it, before closing it. So we:
//   • draw the latest boxes synchronously every frame (they ride the live video), and
//   • on a throttle, copy the frame to a reused work-canvas and kick off async inference (single
//     in-flight). We never retain the frame past the call — it's closed right after — so the copy
//     is synchronous and inference reads the copy.
import { useCallback, useRef, useState } from "react";
import type { VideoMeta } from "@dimos/react";
import { type Box, type Detector, loadCocoDetector } from "./detector";

export interface CvStats {
  count: number; // boxes in the latest detection
  infMs: number; // inference latency of the latest run
  loading: boolean; // model still downloading / initialising
}

const MIN_INTERVAL_MS = 100; // cap inference at ~10 Hz; the video still draws every frame
const MIN_SCORE = 0.5; // hide low-confidence boxes

export function useCvOverlay() {
  const [enabled, setEnabled] = useState(false);
  const [stats, setStats] = useState<CvStats>({ count: 0, infMs: 0, loading: false });

  const detectorRef = useRef<Detector | null>(null);
  const boxesRef = useRef<Box[]>([]);
  const busyRef = useRef(false);
  const lastRef = useRef(0);
  const workRef = useRef<HTMLCanvasElement | null>(null);
  const hudRef = useRef<CvStats>({ count: 0, infMs: 0, loading: false });

  const setHud = (s: CvStats) => {
    hudRef.current = s; // drives the canvas HUD (no React render)
    setStats(s); // drives the panel title
  };

  const toggle = useCallback(() => {
    setEnabled((on) => {
      const next = !on;
      if (next && !detectorRef.current) {
        setHud({ ...hudRef.current, loading: true });
        loadCocoDetector()
          .then((d) => {
            detectorRef.current = d;
            setHud({ ...hudRef.current, loading: false });
          })
          .catch(() => setHud({ ...hudRef.current, loading: false }));
      }
      if (!next) boxesRef.current = []; // clear the overlay when turned off
      return next;
    });
  }, []);

  // Stable identity (reads everything from refs) so swapping it never resubscribes useVideo's channel.
  const onFrame = useCallback(
    (frame: VideoFrame | ImageBitmap, meta: VideoMeta, ctx: CanvasRenderingContext2D) => {
      // 1) draw the latest boxes + HUD on THIS frame (registered overlay).
      drawOverlay(ctx, boxesRef.current, meta, hudRef.current);

      // 2) throttled, single-in-flight inference on a synchronous copy of the frame.
      const det = detectorRef.current;
      if (!det || busyRef.current) return;
      const now = performance.now();
      if (now - lastRef.current < MIN_INTERVAL_MS) return;
      lastRef.current = now;

      const w = meta.width || (frame as VideoFrame).displayWidth || (frame as ImageBitmap).width;
      const h = meta.height || (frame as VideoFrame).displayHeight || (frame as ImageBitmap).height;
      if (!w || !h) return;
      const work = workRef.current ?? (workRef.current = document.createElement("canvas"));
      if (work.width !== w || work.height !== h) {
        work.width = w;
        work.height = h;
      }
      const wctx = work.getContext("2d");
      if (!wctx) return;
      wctx.drawImage(frame as CanvasImageSource, 0, 0); // sync copy — safe to close the frame after this

      busyRef.current = true;
      const t0 = performance.now();
      det
        .detect(work)
        .then((boxes) => {
          boxesRef.current = boxes.filter((b) => b.score >= MIN_SCORE);
          setHud({
            count: boxesRef.current.length,
            infMs: Math.round(performance.now() - t0),
            loading: false,
          });
        })
        .catch(() => {})
        .finally(() => {
          busyRef.current = false;
        });
    },
    [],
  );

  return { enabled, toggle, onFrame, stats };
}

// ── overlay drawing (boxes are in frame-pixel coords == canvas size, so 1:1) ──────────────────────
function drawOverlay(ctx: CanvasRenderingContext2D, boxes: Box[], meta: VideoMeta, hud: CvStats) {
  const s = Math.max(1, Math.round((meta.height || ctx.canvas.height) / 240)); // scale to frame size
  ctx.lineWidth = 2 * s;
  ctx.font = `${12 * s}px ui-monospace, monospace`;
  ctx.textBaseline = "top";
  for (const b of boxes) {
    ctx.strokeStyle = "#34d399";
    ctx.strokeRect(b.x, b.y, b.w, b.h);
    const tag = `${b.label} ${Math.round(b.score * 100)}%`;
    const tw = ctx.measureText(tag).width;
    const ty = Math.max(0, b.y - 14 * s);
    ctx.fillStyle = "#34d399";
    ctx.fillRect(b.x, ty, tw + 6 * s, 14 * s);
    ctx.fillStyle = "#062018";
    ctx.fillText(tag, b.x + 3 * s, ty + s);
  }
  // HUD — proves the frame→model→overlay loop is live even when the scene has no COCO objects.
  const txt = hud.loading ? "cv · loading…" : `cv · ${hud.count} obj · ${hud.infMs}ms`;
  ctx.fillStyle = "rgba(0,0,0,0.55)";
  ctx.fillRect(0, 0, ctx.measureText(txt).width + 10 * s, 16 * s);
  ctx.fillStyle = "#34d399";
  ctx.fillText(txt, 4 * s, 2 * s);
}

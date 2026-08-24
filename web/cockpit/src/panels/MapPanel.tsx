// Live 2D costmap: canvas drawing driven by the store's direct-subscribe path
// (React is not involved at grid or pose rate; the badge rides the 500 ms UI
// tick). channels[0] is the costmap, channels[1] (optional) the pose overlay
// - both bindings come from the manifest, never hardcoded stream names.

import { useEffect, useRef } from "react";
import { Badge, type DrawHealth, PanelFrame } from "../layout/PanelFrame.tsx";
import { type CostmapValue, inflateCostmap } from "../session/decoders/costmap.ts";
import { useChannel } from "../session/hooks.ts";
import type { ChannelStore } from "../session/store.ts";
import styles from "./MapPanel.module.css";
import {
  drawPose,
  fitTransform,
  gridBlit,
  type GridPlacement,
  gridToImageData,
  type Pose2d,
} from "./mapRenderer.ts";
import type { PanelProps } from "./registry.tsx";

// The costmap ticks at ~5 Hz; staleness only trips on real silence (mapper
// down, replay ended). New sessions never start stale: the bridge replays
// the last grid on subscribe.
export const MAP_STALE_MS = 5000;

/** Test seams; real inflate is DecompressionStream, real resize an observer. */
export interface MapSinkDeps {
  inflate?: (value: CostmapValue) => Promise<Uint8Array>;
  hidden?: () => boolean;
  /** Calls back on element size changes; returns the disposer. */
  observeResize?: (el: Element, cb: () => void) => () => void;
}

function isCostmapValue(v: unknown): v is CostmapValue {
  return typeof v === "object" && v !== null &&
    (v as CostmapValue).bytes instanceof Uint8Array &&
    typeof (v as CostmapValue).w === "number";
}

function readPose(v: unknown): Pose2d | null {
  if (typeof v !== "object" || v === null) return null;
  const { x, y, yaw } = v as Record<string, unknown>;
  if (typeof x !== "number" || typeof y !== "number" || typeof yaw !== "number") return null;
  return { x, y, yaw };
}

/**
 * Drive `canvas` from the costmap channel's slot: at most one inflate in
 * flight, and on completion the pump re-checks the slot, so a burst of grids
 * costs one inflate of the newest (latest-wins, same shedding rule as
 * everywhere else in the pipeline). The inflated grid lands on an offscreen
 * bitmap at cell resolution; the display canvas redraws (scaled blit + pose
 * triangle) on new grids, pose ingests, resizes, and visibility changes,
 * which is how two consumers share the odom channel without extra
 * subscriptions upstream. While the document is hidden nothing inflates or
 * draws. Returns the cleanup function.
 */
export function startMapSink(
  store: ChannelStore,
  costmapCh: string,
  poseCh: string | undefined,
  canvas: HTMLCanvasElement,
  health: DrawHealth,
  deps: MapSinkDeps = {},
): () => void {
  const inflate = deps.inflate ?? inflateCostmap;
  const hidden = deps.hidden ?? (() => document.hidden);
  const observeResize = deps.observeResize ?? ((el, cb) => {
    const observer = new ResizeObserver(cb);
    observer.observe(el);
    return () => observer.disconnect();
  });
  const ctx = canvas.getContext("2d");
  // Grid bitmap at native cell resolution, rewritten only on new grids; the
  // ImageData buffer is reused while the grid dimensions hold.
  const grid = document.createElement("canvas");
  const gridCtx = grid.getContext("2d");
  let imageData: ImageData | undefined;
  let place: GridPlacement | null = null;
  let inflating = false;
  let drawnVersion = -1;
  let stopped = false;
  // A fresh mount is never instantly "stalled".
  health.lastDrawOkAtMs = Date.now();
  health.failures = 0;

  const draw = (): void => {
    if (stopped || hidden() || ctx === null || place === null) return;
    const cssW = canvas.clientWidth;
    const cssH = canvas.clientHeight;
    if (cssW === 0 || cssH === 0) return;
    const dpr = globalThis.devicePixelRatio || 1;
    const w = Math.round(cssW * dpr);
    const h = Math.round(cssH * dpr);
    if (canvas.width !== w || canvas.height !== h) {
      canvas.width = w;
      canvas.height = h;
    }
    ctx.clearRect(0, 0, w, h);
    const t = fitTransform(place, w, h);
    ctx.imageSmoothingEnabled = false; // crisp cells when zoomed in
    const { ax, ay, rot, dw, dh } = gridBlit(t, place);
    ctx.save();
    ctx.translate(ax, ay);
    ctx.rotate(rot);
    ctx.drawImage(grid, 0, -dh, dw, dh);
    ctx.restore();
    const pose = poseCh === undefined ? null : readPose(store.get(poseCh)?.value);
    if (pose !== null) drawPose(ctx, t, pose, dpr);
  };

  const pump = (): void => {
    if (stopped || inflating || hidden()) return;
    const slot = store.get(costmapCh);
    if (slot === null || slot.version === drawnVersion) return;
    if (!isCostmapValue(slot.value)) return; // undecoded channel: nothing to draw
    const value = slot.value;
    const version = slot.version;
    inflating = true;
    inflate(value)
      .then((cells) => {
        if (stopped) return;
        if (grid.width !== value.w || grid.height !== value.h) {
          // Assigning a canvas dimension resets its backing store even when
          // the value is unchanged, so only touch it on real changes.
          grid.width = value.w;
          grid.height = value.h;
        }
        imageData = gridToImageData(cells, value.w, value.h, imageData);
        gridCtx?.putImageData(imageData, 0, 0);
        place = { w: value.w, h: value.h, res: value.res, origin: value.origin };
        draw();
        health.lastDrawOkAtMs = Date.now();
        health.failures = 0;
      })
      .catch(() => {
        // Inflate rejection or draw throw: skip this grid but count it, so
        // the badge can surface a pipeline that never draws.
        health.failures += 1;
      })
      .finally(() => {
        drawnVersion = version;
        inflating = false;
        pump(); // newer grids may have landed during the inflate
      });
  };

  const unsubscribeGrid = store.subscribe(costmapCh, pump);
  // Pose redraws reuse the cached grid bitmap: no inflate at odom rate.
  const unsubscribePose = poseCh === undefined ? null : store.subscribe(poseCh, draw);
  const disposeResize = observeResize(canvas, draw);
  const onVisibility = (): void => {
    pump();
    draw(); // a resize while hidden must repaint even without a new grid
  };
  document.addEventListener("visibilitychange", onVisibility);
  pump(); // a slot may predate the mount
  return () => {
    stopped = true;
    unsubscribeGrid();
    unsubscribePose?.();
    disposeResize();
    document.removeEventListener("visibilitychange", onVisibility);
  };
}

export function MapPanel({ spec, store }: PanelProps) {
  const costmapCh = spec.channels[0] as string | undefined;
  if (costmapCh === undefined) {
    // A map panel without a costmap channel is a bridge authoring mistake;
    // render it visibly instead of crashing the grid.
    return (
      <PanelFrame spec={spec}>
        <span className={styles.waiting}>map2d panel {spec.id}: no channel bound</span>
      </PanelFrame>
    );
  }
  return (
    <MapCanvas
      spec={spec}
      store={store}
      costmapCh={costmapCh}
      poseCh={spec.channels[1] as string | undefined}
    />
  );
}

function MapCanvas(
  { spec, store, costmapCh, poseCh }: PanelProps & {
    costmapCh: string;
    poseCh: string | undefined;
  },
) {
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const health = useRef<DrawHealth>({ lastDrawOkAtMs: Date.now(), failures: 0 }).current;
  const { slot } = useChannel(store, costmapCh);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (canvas === null) return;
    return startMapSink(store, costmapCh, poseCh, canvas, health);
  }, [store, costmapCh, poseCh, health]);

  return (
    <PanelFrame
      spec={spec}
      badge={
        <Badge
          store={store}
          ch={costmapCh}
          health={health}
          staleMs={MAP_STALE_MS}
          unit="Hz"
          testId={`map2d-${costmapCh}-badge`}
        />
      }
    >
      <canvas
        ref={canvasRef}
        className={styles.canvas}
        data-testid={`map2d-${costmapCh}-canvas`}
        role="img"
        aria-label={spec.id}
      />
      {slot === null && <span className={styles.waiting}>waiting for data...</span>}
    </PanelFrame>
  );
}

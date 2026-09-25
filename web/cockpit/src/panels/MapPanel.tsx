// Canvas updates bypass React; badges and cancellation use the slower UI tick.

import { useEffect, useRef, useState } from "react";
import type { JsonValue, PanelSpec } from "@dimos/shared";
import { Badge, type DrawHealth, PanelFrame } from "../layout/PanelFrame.tsx";
import { type ChannelStore, type CostmapValue, inflateCostmap, type Session } from "@dimos/sdk";
import { useStoreChannel } from "@dimos/sdk/react";
import styles from "./MapPanel.module.css";
import {
  canvasToWorld,
  drawPath,
  drawPose,
  fitTransform,
  gridBlit,
  type GridPlacement,
  gridToImageData,
  type PathPoint,
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

export interface MapSinkOptions {
  costmap: string;
  pose?: string;
  path?: string;
  onClick?: (x: number, y: number) => void;
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

function readPath(v: unknown): PathPoint[] | null {
  if (!Array.isArray(v)) return null;
  const ok = v.every((p) =>
    Array.isArray(p) && p.length === 2 && Number.isFinite(p[0]) && Number.isFinite(p[1])
  );
  return ok ? v as PathPoint[] : null;
}

/**
 * Inflate one grid at a time, skipping to the newest after each completion.
 * Cache the bitmap so overlays and resizes don't repeat decompression.
 * Hidden documents pause both inflation and drawing.
 */
export function startMapSink(
  store: ChannelStore,
  { costmap: costmapCh, pose: poseCh, path: pathCh, onClick }: MapSinkOptions,
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
    const path = pathCh === undefined ? null : readPath(store.get(pathCh)?.value);
    if (path !== null && path.length > 1) drawPath(ctx, t, path, dpr);
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

  // Mouse coordinates are CSS pixels; the fitted map uses backing-store pixels.
  const onCanvasClick = (e: MouseEvent): void => {
    const rect = canvas.getBoundingClientRect();
    if (place === null || rect.width === 0 || rect.height === 0) return;
    const t = fitTransform(place, canvas.width, canvas.height);
    const px = (e.clientX - rect.left) * canvas.width / rect.width;
    const py = (e.clientY - rect.top) * canvas.height / rect.height;
    onClick?.(...canvasToWorld(t, px, py));
  };
  if (onClick !== undefined) canvas.addEventListener("click", onCanvasClick);

  const unsubscribeGrid = store.subscribe(costmapCh, pump);
  const unsubscribePose = poseCh === undefined ? null : store.subscribe(poseCh, draw);
  const unsubscribePath = pathCh === undefined ? null : store.subscribe(pathCh, draw);
  const disposeResize = observeResize(canvas, draw);
  const onVisibility = (): void => {
    pump();
    draw(); // a resize while hidden must repaint even without a new grid
  };
  document.addEventListener("visibilitychange", onVisibility);
  pump(); // a slot may predate the mount
  return () => {
    stopped = true;
    canvas.removeEventListener("click", onCanvasClick);
    unsubscribeGrid();
    unsubscribePose?.();
    unsubscribePath?.();
    disposeResize();
    document.removeEventListener("visibilitychange", onVisibility);
  };
}

function param(spec: PanelSpec, key: string): string | undefined {
  const value = spec.params[key];
  return typeof value === "string" ? value : undefined;
}

function send(
  session: Session,
  ch: string,
  value: JsonValue,
  onError: (message: string | null) => void,
): void {
  session.publish(ch, value).then(
    () => onError(null),
    (err: unknown) => onError(`send failed: ${err instanceof Error ? err.message : String(err)}`),
  );
}

export function MapPanel({ spec, store, session }: PanelProps) {
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
      session={session}
      costmapCh={costmapCh}
      poseCh={spec.channels[1] as string | undefined}
      pathCh={param(spec, "path")}
      clickCh={param(spec, "click")}
      stopCh={param(spec, "stop")}
    />
  );
}

function MapCanvas(
  { spec, store, session, costmapCh, poseCh, pathCh, clickCh, stopCh }: PanelProps & {
    costmapCh: string;
    poseCh: string | undefined;
    pathCh: string | undefined;
    clickCh: string | undefined;
    stopCh: string | undefined;
  },
) {
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const health = useRef<DrawHealth>({ lastDrawOkAtMs: Date.now(), failures: 0 }).current;
  const { slot } = useStoreChannel(store, costmapCh);
  const [error, setError] = useState<string | null>(null);
  const clickable = session !== undefined && clickCh !== undefined;

  useEffect(() => {
    const canvas = canvasRef.current;
    if (canvas === null) return;
    const onClick = session !== undefined && clickCh !== undefined
      ? (x: number, y: number) => send(session, clickCh, { x, y }, setError)
      : undefined;
    const opts = { costmap: costmapCh, pose: poseCh, path: pathCh, onClick };
    return startMapSink(store, opts, canvas, health);
  }, [store, session, costmapCh, poseCh, pathCh, clickCh, health]);

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
        className={clickable ? `${styles.canvas} ${styles.clickable}` : styles.canvas}
        data-testid={`map2d-${costmapCh}-canvas`}
        role="img"
        aria-label={spec.id}
      />
      {slot === null && <span className={styles.waiting}>waiting for data...</span>}
      {error !== null && (
        <span className={styles.error} role="alert">
          {error}
        </span>
      )}
      {session !== undefined && pathCh !== undefined && stopCh !== undefined && (
        <CancelButton
          store={store}
          session={session}
          pathCh={pathCh}
          stopCh={stopCh}
          testId={`map2d-${costmapCh}-cancel`}
          onError={setError}
        />
      )}
    </PanelFrame>
  );
}

function CancelButton({ store, session, pathCh, stopCh, testId, onError }: {
  store: ChannelStore;
  session: Session;
  pathCh: string;
  stopCh: string;
  testId: string;
  onError: (message: string | null) => void;
}) {
  const path = readPath(useStoreChannel(store, pathCh).slot?.value);
  if (path === null || path.length === 0) return null;
  return (
    <button
      type="button"
      className={styles.cancel}
      data-testid={testId}
      onClick={() => send(session, stopCh, true, onError)}
    >
      cancel
    </button>
  );
}

// Canvas updates bypass React; the badge uses the slower UI tick. The
// three.js scene arrives through a lazy import (deps.createScene), so pages
// without a map3d panel never download it.

import { useLayoutEffect, useRef, useState } from "react";
import { Badge, type DrawHealth, PanelFrame } from "../layout/PanelFrame.tsx";
import { type ChannelStore, inflateVoxels, type VoxelsValue } from "@dimos/sdk";
import { useStoreChannel } from "@dimos/sdk/react";
import styles from "./Map3DPanel.module.css";
import { mapChannel, sendMapCommand } from "./mapCommands.ts";
import { readPath } from "./mapPath.ts";
import type { PanelProps } from "./registry.tsx";
import { heightColors, type Pose3d, type VoxelScene } from "./voxelView.ts";

// The mapper republishes the whole map at ~1.5 Hz on hardware and 0.4 Hz in
// simulation; staleness only trips on real silence.
export const MAP3D_STALE_MS = 10_000;
const CLICK_SLOP_PX = 5;

/** Test seams; the real scene is the lazily imported three.js one. */
export interface VoxelSinkDeps {
  inflate?: (value: VoxelsValue) => Promise<Float32Array>;
  hidden?: () => boolean;
  /** Calls back on element size changes; returns the disposer. */
  observeResize?: (el: Element, cb: () => void) => () => void;
  createScene?: (canvas: HTMLCanvasElement) => Promise<VoxelScene>;
  /** The renderer could not start (no WebGL): shown in the panel. */
  onError?: (message: string) => void;
}

export interface VoxelSinkOptions {
  cloud: string;
  pose?: string;
  path?: string;
  onClick?: (x: number, y: number) => void;
}

export interface VoxelSink {
  stop(): void;
  /** Keep the camera behind the robot as it moves (on), or leave it be (off). */
  follow(on: boolean): void;
}

function isVoxelsValue(v: unknown): v is VoxelsValue {
  return typeof v === "object" && v !== null &&
    (v as VoxelsValue).bytes instanceof Uint8Array &&
    typeof (v as VoxelsValue).n === "number";
}

function readPose(v: unknown): Pose3d | null {
  if (typeof v !== "object" || v === null) return null;
  const { x, y, z, yaw } = v as Record<string, unknown>;
  if (
    typeof x !== "number" || typeof y !== "number" || typeof z !== "number" ||
    typeof yaw !== "number"
  ) return null;
  return { x, y, z, yaw };
}

async function loadScene(canvas: HTMLCanvasElement): Promise<VoxelScene> {
  const { createVoxelScene } = await import("./voxelScene.ts");
  return createVoxelScene(canvas);
}

/**
 * Inflate one frame at a time, skipping to the newest after each completion,
 * and hand the voxels to the scene. The first frame with voxels frames the
 * camera unless it is following the robot; later frames leave the viewer's
 * orbit alone, and an empty frame (a cleared or filtered-out map) clears the
 * drawn voxels. Hidden documents pause both inflation and drawing.
 */
export function startVoxelSink(
  store: ChannelStore,
  { cloud: cloudCh, pose: poseCh, path: pathCh, onClick }: VoxelSinkOptions,
  canvas: HTMLCanvasElement,
  health: DrawHealth,
  deps: VoxelSinkDeps = {},
): VoxelSink {
  const inflate = deps.inflate ?? inflateVoxels;
  const hidden = deps.hidden ?? (() => document.hidden);
  const observeResize = deps.observeResize ?? ((el, cb) => {
    const observer = new ResizeObserver(cb);
    observer.observe(el);
    return () => observer.disconnect();
  });
  const createScene = deps.createScene ?? loadScene;
  let scene: VoxelScene | null = null;
  let inflating = false;
  let drawnVersion = -1;
  let pathVersion = -1;
  let fitted = false;
  let following = false;
  let stopped = false;
  // A fresh mount is never instantly "stalled".
  health.lastDrawOkAtMs = Date.now();
  health.failures = 0;

  const resize = (): void => {
    const w = canvas.clientWidth;
    const h = canvas.clientHeight;
    if (scene === null || w === 0 || h === 0) return;
    scene.resize(w, h, globalThis.devicePixelRatio || 1);
  };

  const draw = (): void => {
    if (stopped || hidden() || scene === null) return;
    const pathSlot = pathCh === undefined ? null : store.get(pathCh);
    const version = pathSlot?.version ?? -1;
    if (version !== pathVersion) {
      scene.setPath(readPath(pathSlot?.value));
      pathVersion = version;
    }
    scene.setPose(poseCh === undefined ? null : readPose(store.get(poseCh)?.value));
    scene.render();
  };

  const pump = (): void => {
    if (stopped || inflating || hidden() || scene === null) return;
    const slot = store.get(cloudCh);
    if (slot === null || slot.version === drawnVersion) return;
    if (!isVoxelsValue(slot.value)) return; // undecoded channel: nothing to draw
    const value = slot.value;
    const version = slot.version;
    inflating = true;
    inflate(value)
      .then((positions) => {
        if (stopped || scene === null) return;
        scene.setVoxels(positions, heightColors(positions), value.res);
        if (!fitted && value.n > 0) {
          // An empty frame has nothing to frame and must not use up the fit,
          // and a camera behind the robot is not pulled off it.
          if (!following) scene.fit();
          fitted = true;
        }
        canvas.dataset.voxels = String(value.n);
        draw();
        health.lastDrawOkAtMs = Date.now();
        health.failures = 0;
      })
      .catch(() => {
        // Inflate rejection or draw throw: skip this frame but count it, so
        // the badge can surface a pipeline that never draws.
        health.failures += 1;
      })
      .finally(() => {
        drawnVersion = version;
        inflating = false;
        pump(); // newer frames may have landed during the inflate
      });
  };

  createScene(canvas).then(
    (created) => {
      if (stopped) {
        created.dispose();
        return;
      }
      scene = created;
      if (following) scene.setFollow(true); // toggled while the chunk was loading
      resize();
      pump(); // a slot may predate the scene
      draw();
    },
    (err: unknown) => {
      if (!stopped) {
        deps.onError?.(`3D view unavailable: ${err instanceof Error ? err.message : String(err)}`);
      }
    },
  );

  // OrbitControls captures pointers. Only a primary tap with no drag or
  // modifiers may send a goal; moving away and back is still a drag, and a
  // modifier pressed before the release turns the tap into a modified gesture.
  let pointer: { id: number; x: number; y: number } | null = null;
  const cancelClick = (): void => {
    pointer = null;
  };
  const modified = (e: PointerEvent): boolean => e.ctrlKey || e.metaKey || e.shiftKey || e.altKey;
  const onPointerDown = (e: PointerEvent): void => {
    pointer = e.isPrimary && e.button === 0 && !modified(e)
      ? { id: e.pointerId, x: e.clientX, y: e.clientY }
      : null;
  };
  const onPointerMove = (e: PointerEvent): void => {
    if (
      pointer !== null && pointer.id === e.pointerId &&
      Math.hypot(e.clientX - pointer.x, e.clientY - pointer.y) > CLICK_SLOP_PX
    ) cancelClick();
  };
  const onPointerUp = (e: PointerEvent): void => {
    onPointerMove(e);
    const clicked = pointer !== null && pointer.id === e.pointerId && !modified(e);
    cancelClick();
    if (!clicked || scene === null) return;
    const rect = canvas.getBoundingClientRect();
    if (rect.width === 0 || rect.height === 0) return;
    const x = (e.clientX - rect.left) / rect.width * 2 - 1;
    const y = 1 - (e.clientY - rect.top) / rect.height * 2;
    if (Math.abs(x) > 1 || Math.abs(y) > 1) return;
    const goal = scene.pick(x, y);
    if (goal !== null) onClick?.(...goal);
  };
  if (onClick !== undefined) {
    canvas.addEventListener("pointerdown", onPointerDown);
    canvas.addEventListener("pointermove", onPointerMove);
    canvas.addEventListener("pointerup", onPointerUp);
    canvas.addEventListener("pointercancel", cancelClick);
    canvas.addEventListener("lostpointercapture", cancelClick);
    canvas.addEventListener("wheel", cancelClick);
  }

  const unsubscribeCloud = store.subscribe(cloudCh, pump);
  const unsubscribePose = poseCh === undefined ? null : store.subscribe(poseCh, draw);
  const unsubscribePath = pathCh === undefined ? null : store.subscribe(pathCh, draw);
  const disposeResize = observeResize(canvas, () => {
    resize();
    draw();
  });
  const onVisibility = (): void => {
    pump();
    draw(); // a resize while hidden must repaint even without a new frame
  };
  document.addEventListener("visibilitychange", onVisibility);
  return {
    stop() {
      stopped = true;
      cancelClick();
      canvas.removeEventListener("pointerdown", onPointerDown);
      canvas.removeEventListener("pointermove", onPointerMove);
      canvas.removeEventListener("pointerup", onPointerUp);
      canvas.removeEventListener("pointercancel", cancelClick);
      canvas.removeEventListener("lostpointercapture", cancelClick);
      canvas.removeEventListener("wheel", cancelClick);
      unsubscribeCloud();
      unsubscribePose?.();
      unsubscribePath?.();
      disposeResize();
      document.removeEventListener("visibilitychange", onVisibility);
      scene?.dispose();
      scene = null;
    },
    follow(on) {
      following = on;
      scene?.setFollow(on);
      draw();
    },
  };
}

export function Map3DPanel(
  { spec, store, session, sinkDeps }: PanelProps & { sinkDeps?: VoxelSinkDeps },
) {
  const cloudCh = spec.channels[0] as string | undefined;
  if (cloudCh === undefined) {
    // A map panel without a cloud channel is a bridge authoring mistake;
    // render it visibly instead of crashing the grid.
    return (
      <PanelFrame spec={spec}>
        <span className={styles.waiting}>map3d panel {spec.id}: no channel bound</span>
      </PanelFrame>
    );
  }
  return (
    <VoxelCanvas
      spec={spec}
      store={store}
      session={session}
      cloudCh={cloudCh}
      poseCh={spec.channels[1] as string | undefined}
      pathCh={mapChannel(spec, "path")}
      clickCh={mapChannel(spec, "click")}
      sinkDeps={sinkDeps}
    />
  );
}

function VoxelCanvas(
  { spec, store, session, cloudCh, poseCh, pathCh, clickCh, sinkDeps }: PanelProps & {
    cloudCh: string;
    poseCh: string | undefined;
    pathCh: string | undefined;
    clickCh: string | undefined;
    sinkDeps: VoxelSinkDeps | undefined;
  },
) {
  const canvasHostRef = useRef<HTMLDivElement | null>(null);
  const sinkRef = useRef<VoxelSink | null>(null);
  const health = useRef<DrawHealth>({ lastDrawOkAtMs: Date.now(), failures: 0 }).current;
  const { slot } = useStoreChannel(store, cloudCh);
  const [error, setError] = useState<string | null>(null);
  const [following, setFollowing] = useState(false);
  const clickable = session !== undefined && clickCh !== undefined;

  // A layout effect, not a passive one: React removes a deleted subtree's
  // DOM before its passive cleanups run, and OrbitControls.dispose()
  // unregisters its document keydown listener from the canvas's root node,
  // which for a detached canvas is the detached subtree. The listener (and
  // through it the renderer, the scene and the whole grid) then leaks on
  // every page switch. Layout cleanups run while the canvas is still in the
  // document.
  useLayoutEffect(() => {
    const host = canvasHostRef.current;
    if (host === null) return;
    // Disposal loses the WebGL context. Each effect setup owns a fresh
    // canvas, even when React retains this panel or replays its effects.
    const canvas = document.createElement("canvas");
    canvas.className = clickable ? `${styles.canvas} ${styles.clickable}` : styles.canvas;
    canvas.dataset.testid = `map3d-${cloudCh}-canvas`;
    canvas.setAttribute("role", "img");
    canvas.setAttribute("aria-label", spec.id);
    host.appendChild(canvas);
    setError(null);
    const deps = { ...sinkDeps, onError: setError };
    const onClick = clickable
      ? (x: number, y: number) => sendMapCommand(session, clickCh, { x, y }, setError)
      : undefined;
    const opts = { cloud: cloudCh, pose: poseCh, path: pathCh, onClick };
    const sink = startVoxelSink(store, opts, canvas, health, deps);
    sinkRef.current = sink;
    return () => {
      sinkRef.current = null;
      sink.stop();
      canvas.remove();
    };
  }, [store, session, cloudCh, poseCh, pathCh, clickCh, clickable, health, sinkDeps, spec.id]);

  return (
    <PanelFrame
      spec={spec}
      badge={
        <Badge
          store={store}
          ch={cloudCh}
          health={health}
          staleMs={MAP3D_STALE_MS}
          unit="Hz"
          testId={`map3d-${cloudCh}-badge`}
        />
      }
    >
      <div ref={canvasHostRef} className={styles.canvas} />
      {slot === null && <span className={styles.waiting}>waiting for data...</span>}
      {error !== null && (
        <span className={styles.error} role="alert">
          {error}
        </span>
      )}
      {poseCh !== undefined && (
        <button
          type="button"
          className={styles.follow}
          aria-pressed={following}
          data-testid={`map3d-${cloudCh}-follow`}
          onClick={() => {
            setFollowing(!following);
            sinkRef.current?.follow(!following);
          }}
        >
          <span className={styles.check} aria-hidden="true" />
          follow
        </button>
      )}
    </PanelFrame>
  );
}

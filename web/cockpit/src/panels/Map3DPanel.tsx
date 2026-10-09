// Canvas updates bypass React; the badge uses the slower UI tick. The
// three.js scene arrives through a lazy import (deps.createScene), so pages
// without a map3d panel never download it.

import { useEffect, useRef, useState } from "react";
import type { PanelSpec } from "@dimos/shared";
import { Badge, type DrawHealth, PanelFrame } from "../layout/PanelFrame.tsx";
import { type ChannelStore, inflateVoxels, type VoxelsValue } from "@dimos/sdk";
import { useStoreChannel } from "@dimos/sdk/react";
import styles from "./Map3DPanel.module.css";
import type { PanelProps } from "./registry.tsx";
import { heightColors, type Pose3d, type VoxelScene } from "./voxelView.ts";

// The mapper republishes the whole map at ~1.5 Hz on hardware and 0.4 Hz in
// simulation; staleness only trips on real silence.
export const MAP3D_STALE_MS = 10_000;

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
  { cloud: cloudCh, pose: poseCh }: VoxelSinkOptions,
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
      deps.onError?.(`3D view unavailable: ${err instanceof Error ? err.message : String(err)}`);
    },
  );

  const unsubscribeCloud = store.subscribe(cloudCh, pump);
  const unsubscribePose = poseCh === undefined ? null : store.subscribe(poseCh, draw);
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
      unsubscribeCloud();
      unsubscribePose?.();
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

export function Map3DPanel({ spec, store, sinkDeps }: PanelProps & { sinkDeps?: VoxelSinkDeps }) {
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
      cloudCh={cloudCh}
      poseCh={spec.channels[1] as string | undefined}
      sinkDeps={sinkDeps}
    />
  );
}

function VoxelCanvas({ spec, store, cloudCh, poseCh, sinkDeps }: {
  spec: PanelSpec;
  store: ChannelStore;
  cloudCh: string;
  poseCh: string | undefined;
  sinkDeps: VoxelSinkDeps | undefined;
}) {
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const sinkRef = useRef<VoxelSink | null>(null);
  const health = useRef<DrawHealth>({ lastDrawOkAtMs: Date.now(), failures: 0 }).current;
  const { slot } = useStoreChannel(store, cloudCh);
  const [error, setError] = useState<string | null>(null);
  const [following, setFollowing] = useState(false);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (canvas === null) return;
    const deps = { ...sinkDeps, onError: setError };
    const sink = startVoxelSink(store, { cloud: cloudCh, pose: poseCh }, canvas, health, deps);
    sinkRef.current = sink;
    return () => {
      sinkRef.current = null;
      sink.stop();
    };
  }, [store, cloudCh, poseCh, health, sinkDeps]);

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
      <canvas
        ref={canvasRef}
        className={styles.canvas}
        data-testid={`map3d-${cloudCh}-canvas`}
        role="img"
        aria-label={spec.id}
      />
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

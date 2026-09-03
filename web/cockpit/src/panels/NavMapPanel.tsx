// Navigation map: the live costmap + pose (startMapSink, unchanged) with an
// overlay canvas on top for the semantic layer - rooms with clickable
// labels, landmark objects, tagged places, the planner's path and the
// current goal - and click-to-goal. The overlay learns the base transform
// through MapSinkDeps.onTransform so both canvases agree pixel for pixel;
// before the first costmap it fits a fixed world box so rooms show at once.
// Everything is drawn in device pixels (DPR-aware like MapPanel).
//
// Sends: a click -> pose_goal on the goal channel (a room label sends the
// room's target pose); Esc while the map has focus, or the ✕ chip ->
// ui_command cancel_nav. No document-level key listeners.

import {
  type KeyboardEvent,
  type MouseEvent as ReactMouseEvent,
  useEffect,
  useRef,
  useState,
} from "react";
import { type ChannelStore } from "@dimos/sdk";
import { useStoreChannel } from "@dimos/sdk/react";
import { Badge, type DrawHealth, PanelFrame } from "../layout/PanelFrame.tsx";
import { navCancellable, type NavState, navTone, readNavState } from "./controlPolicy.ts";
import { useOptionalSlot } from "./hooks.ts";
import { type MapSinkDeps, startMapSink } from "./MapPanel.tsx";
import {
  drawPose,
  fitTransform,
  type GridPlacement,
  type MapTransform,
  type Pose2d,
  worldToCanvas,
} from "./mapRenderer.ts";
import {
  clickToWorld,
  fallbackTransform,
  goalPayload,
  hitLabel,
  type LabelBox,
  type PathValue,
  type Places,
  readPath,
  readPlaces,
} from "./navMapGeometry.ts";
import styles from "./NavMapPanel.module.css";
import { paramChannel } from "./panelParams.ts";
import type { PanelProps } from "./registry.tsx";
import { txReasonText } from "./txReason.ts";

export interface NavMapChannels {
  costmap: string;
  pose: string | undefined;
  path: string | undefined;
  places: string | undefined;
  navState: string | undefined;
  goal: string | undefined;
  command: string | undefined;
}

export function navMapChannels(spec: PanelProps["spec"]): NavMapChannels | null {
  const costmap = paramChannel(spec, "costmap", 0);
  if (costmap === undefined) return null;
  return {
    costmap,
    pose: paramChannel(spec, "pose", 1),
    path: paramChannel(spec, "path", 2),
    places: paramChannel(spec, "places", 3),
    navState: paramChannel(spec, "navState", 4),
    goal: paramChannel(spec, "goal", 5),
    command: paramChannel(spec, "command", 6),
  };
}

function readPose(v: unknown): Pose2d | null {
  if (typeof v !== "object" || v === null) return null;
  const { x, y, yaw } = v as Record<string, unknown>;
  if (typeof x !== "number" || typeof y !== "number" || typeof yaw !== "number") return null;
  return { x, y, yaw };
}

/** How long a click's note stays under the map before it clears itself. */
export const NOTE_LINGER_MS = 4000;

/** The costmap is a slow, static-scene channel - about 2 Hz - so MapPanel's
 * 5 s video-shaped threshold leaves only ~8 frames of slack and cries stale
 * on any brief transport hiccup. It is also the wrong thing to panic about:
 * the flat does not move, a grid seconds old is as correct as a fresh one,
 * and the panel still draws rooms, path and the live pose (odom, reliable,
 * never dropped) meanwhile. Flag a genuinely dead mapper instead. */
export const NAVMAP_STALE_MS = 15000;

const NAV_CHIP_CLASS = {
  bad: styles.chipBad,
  active: styles.chipNav,
  neutral: styles.chip,
} as const;

const ROOM_STROKE = "rgba(47, 129, 247, 0.75)";
const LABEL_FILL = "rgba(20, 23, 26, 0.85)";
const TEXT = "#d7dde3";
const MUTED = "#8b949e";
const OBJECT_COLOR = "#d29922";
const TAGGED_COLOR = "#3fb950";
const PATH_COLOR = "#2f81f7";
const GOAL_COLOR = "#f85149";

export interface OverlayScene {
  places: Places | null;
  path: PathValue | null;
  nav: NavState | null;
  /** Drawn only while the base canvas has no grid (it draws the pose itself). */
  pose: Pose2d | null;
}

/** Strictly inside [xmin, xmax] x [ymin, ymax]. */
function roomContains(
  bounds: readonly [number, number, number, number],
  x: number,
  y: number,
): boolean {
  return x > bounds[0] && x < bounds[1] && y > bounds[2] && y < bounds[3];
}

function textWidth(ctx: CanvasRenderingContext2D, text: string, px: number): number {
  const measured = typeof ctx.measureText === "function" ? ctx.measureText(text) : undefined;
  return measured !== undefined && Number.isFinite(measured.width)
    ? measured.width
    : text.length * px * 0.6;
}

/** Paint the semantic layer; returns the clickable room-label boxes. */
export function drawOverlay(
  ctx: CanvasRenderingContext2D,
  t: MapTransform,
  w: number,
  h: number,
  dpr: number,
  scene: OverlayScene,
): LabelBox[] {
  const labels: LabelBox[] = [];
  const fontPx = 11 * dpr;
  ctx.clearRect(0, 0, w, h);
  ctx.font = `${fontPx}px ui-monospace, Menlo, Consolas, monospace`;
  ctx.textBaseline = "middle";
  ctx.textAlign = "left";

  const { places, path, nav, pose } = scene;
  if (places !== null) {
    ctx.lineWidth = 1 * dpr;
    ctx.strokeStyle = ROOM_STROKE;
    for (const room of places.rooms) {
      const [xmin, xmax, ymin, ymax] = room.bounds;
      const [x0, y0] = worldToCanvas(t, xmin, ymax);
      const [x1, y1] = worldToCanvas(t, xmax, ymin);
      ctx.strokeRect(x0, y0, x1 - x0, y1 - y0);
    }
    // The open hub between rooms (a label only, no target of its own): the
    // four-room scene meets at the origin, which no room strictly contains.
    if (places.rooms.length > 0 && !places.rooms.some((r) => roomContains(r.bounds, 0, 0))) {
      const [hx, hy] = worldToCanvas(t, 0, 0);
      ctx.fillStyle = MUTED;
      ctx.textAlign = "center";
      ctx.fillText("hub", hx, hy);
      ctx.textAlign = "left";
    }
    for (const obj of places.objects) {
      const [cx, cy] = worldToCanvas(t, obj.x, obj.y);
      ctx.fillStyle = OBJECT_COLOR;
      ctx.beginPath();
      ctx.arc(cx, cy, 3 * dpr, 0, Math.PI * 2);
      ctx.fill();
      ctx.fillStyle = MUTED;
      ctx.fillText(obj.name, cx + 5 * dpr, cy);
    }
    for (const place of places.tagged) {
      const [cx, cy] = worldToCanvas(t, place.x, place.y);
      const r = 4 * dpr;
      ctx.fillStyle = TAGGED_COLOR;
      ctx.beginPath();
      ctx.moveTo(cx, cy - r);
      ctx.lineTo(cx + r, cy);
      ctx.lineTo(cx, cy + r);
      ctx.lineTo(cx - r, cy);
      ctx.closePath();
      ctx.fill();
      ctx.fillStyle = TEXT;
      ctx.fillText(place.name, cx + 6 * dpr, cy);
    }
    // Labels last so they sit on top of everything they may overlap.
    for (const room of places.rooms) {
      const [xmin, xmax, ymin, ymax] = room.bounds;
      const [cx, cy] = worldToCanvas(t, (xmin + xmax) / 2, (ymin + ymax) / 2);
      const pad = 4 * dpr;
      const tw = textWidth(ctx, room.name, fontPx);
      const box: LabelBox = {
        name: room.name,
        target: room.target,
        x: cx - tw / 2 - pad,
        y: cy - fontPx / 2 - pad,
        w: tw + 2 * pad,
        h: fontPx + 2 * pad,
      };
      ctx.fillStyle = LABEL_FILL;
      ctx.fillRect(box.x, box.y, box.w, box.h);
      ctx.strokeStyle = ROOM_STROKE;
      ctx.strokeRect(box.x, box.y, box.w, box.h);
      ctx.fillStyle = TEXT;
      ctx.fillText(room.name, box.x + pad, cy);
      labels.push(box);
    }
  }

  if (path !== null && path.points.length > 1) {
    ctx.strokeStyle = PATH_COLOR;
    ctx.lineWidth = 2 * dpr;
    ctx.beginPath();
    path.points.forEach(([x, y], i) => {
      const [cx, cy] = worldToCanvas(t, x, y);
      if (i === 0) ctx.moveTo(cx, cy);
      else ctx.lineTo(cx, cy);
    });
    ctx.stroke();
  }

  if (nav !== null && nav.goal !== null) {
    const [gx, gy] = worldToCanvas(t, nav.goal.x, nav.goal.y);
    ctx.strokeStyle = GOAL_COLOR;
    ctx.fillStyle = GOAL_COLOR;
    ctx.lineWidth = 2 * dpr;
    ctx.beginPath();
    ctx.arc(gx, gy, 5 * dpr, 0, Math.PI * 2);
    ctx.stroke();
    ctx.beginPath();
    ctx.arc(gx, gy, 1.5 * dpr, 0, Math.PI * 2);
    ctx.fill();
    const len = 10 * dpr;
    ctx.beginPath();
    ctx.moveTo(gx, gy);
    ctx.lineTo(gx + Math.cos(nav.goal.yaw) * len, gy - Math.sin(nav.goal.yaw) * len);
    ctx.stroke();
  }

  if (pose !== null) drawPose(ctx, t, pose, dpr);
  return labels;
}

export interface NavOverlayHandle {
  stop: () => void;
  /** Transform of the last overlay draw (device px), null before the first. */
  transform: () => MapTransform | null;
  labels: () => readonly LabelBox[];
}

/**
 * Run the base costmap sink on `base` and the semantic overlay on `overlay`
 * (same CSS box). The base draws only the grid; the overlay owns the pose so
 * the robot marker sits above the room labels. It redraws on new grids (via
 * onTransform), on places/path/nav-state/pose frames, on resize and on
 * visibility changes.
 */
/** The rx roles the sink reads (the tx roles belong to the component). */
export type NavOverlayChannels = Pick<
  NavMapChannels,
  "costmap" | "pose" | "path" | "places" | "navState"
>;

export function startNavOverlay(
  store: ChannelStore,
  chans: NavOverlayChannels,
  base: HTMLCanvasElement,
  overlay: HTMLCanvasElement,
  health: DrawHealth,
  deps: MapSinkDeps = {},
): NavOverlayHandle {
  const hidden = deps.hidden ?? (() => document.hidden);
  const observeResize = deps.observeResize ?? ((el, cb) => {
    if (typeof ResizeObserver === "undefined") return () => {};
    const observer = new ResizeObserver(cb);
    observer.observe(el);
    return () => observer.disconnect();
  });
  const ctx = overlay.getContext("2d");
  let place: GridPlacement | null = null;
  let current: MapTransform | null = null;
  let labels: LabelBox[] = [];
  let stopped = false;

  const read = (ch: string | undefined): unknown =>
    ch === undefined ? undefined : store.get(ch)?.value;

  const draw = (): void => {
    if (stopped || hidden() || ctx === null) return;
    const cssW = overlay.clientWidth;
    const cssH = overlay.clientHeight;
    if (cssW === 0 || cssH === 0) return;
    const dpr = globalThis.devicePixelRatio || 1;
    const w = Math.round(cssW * dpr);
    const h = Math.round(cssH * dpr);
    if (overlay.width !== w || overlay.height !== h) {
      overlay.width = w;
      overlay.height = h;
    }
    const t = place === null ? fallbackTransform(w, h) : fitTransform(place, w, h);
    current = t;
    labels = drawOverlay(ctx, t, w, h, dpr, {
      places: readPlaces(read(chans.places)),
      path: readPath(read(chans.path)),
      nav: readNavState(read(chans.navState)),
      // Drawn here, not by the base sink, so the robot stays visible on top
      // of the opaque room labels (it parks right under one on arrival).
      pose: readPose(read(chans.pose)),
    });
  };

  const stopBase = startMapSink(store, chans.costmap, undefined, base, health, {
    ...deps,
    onTransform: (t, p) => {
      deps.onTransform?.(t, p);
      if (p !== place) {
        // A new grid (MapPanel builds a fresh placement per inflate) may
        // move the fit; the overlay follows. Pose-only redraws are skipped.
        place = p;
        draw();
      }
    },
  });

  const unsubscribers: (() => void)[] = [];
  for (const ch of [chans.places, chans.path, chans.navState, chans.pose]) {
    if (ch !== undefined) unsubscribers.push(store.subscribe(ch, draw));
  }
  const disposeResize = observeResize(overlay, draw);
  document.addEventListener("visibilitychange", draw);
  draw();

  return {
    stop: () => {
      stopped = true;
      stopBase();
      for (const unsubscribe of unsubscribers) unsubscribe();
      disposeResize();
      document.removeEventListener("visibilitychange", draw);
    },
    transform: () => current,
    labels: () => labels,
  };
}

export function NavMapPanel({ spec, store, teleop }: PanelProps) {
  const chans = navMapChannels(spec);
  if (chans === null) {
    return (
      <PanelFrame spec={spec}>
        <span className={styles.hint}>navmap panel {spec.id}: no costmap channel bound</span>
      </PanelFrame>
    );
  }
  return <NavMapView spec={spec} store={store} teleop={teleop} chans={chans} />;
}

function NavMapView({ spec, store, teleop, chans }: PanelProps & { chans: NavMapChannels }) {
  const baseRef = useRef<HTMLCanvasElement | null>(null);
  const overlayRef = useRef<HTMLCanvasElement | null>(null);
  const handleRef = useRef<NavOverlayHandle | null>(null);
  const health = useRef<DrawHealth>({ lastDrawOkAtMs: Date.now(), failures: 0 }).current;
  const { slot } = useStoreChannel(store, chans.costmap);
  const nav = readNavState(useOptionalSlot(store, chans.navState)?.value);
  const [note, setNote] = useState<{ text: string; error: boolean } | null>(null);

  // A note reports one click; without this it would sit under the map for the
  // rest of the session, describing a goal several goals ago. Every setNote
  // stores a fresh object, so a repeated identical note restarts the timer.
  useEffect(() => {
    if (note === null) return;
    const id = setTimeout(() => setNote(null), NOTE_LINGER_MS);
    return () => clearTimeout(id);
  }, [note]);

  // Channel names are the effect's identity (chans is rebuilt per render).
  const { costmap, pose, path, places, navState } = chans;
  useEffect(() => {
    const base = baseRef.current;
    const overlay = overlayRef.current;
    if (base === null || overlay === null) return;
    const handle = startNavOverlay(
      store,
      { costmap, pose, path, places, navState },
      base,
      overlay,
      health,
    );
    handleRef.current = handle;
    return () => {
      handleRef.current = null;
      handle.stop();
    };
  }, [store, costmap, pose, path, places, navState, health]);

  const send = (ch: string | undefined, data: Record<string, unknown>, what: string): void => {
    if (teleop === undefined || ch === undefined) {
      setNote({ text: `${what}: no send path bound`, error: true });
      return;
    }
    const result = teleop.tx(ch, data);
    if (!result.ok) {
      setNote({ text: `${what} not sent: ${txReasonText(result.reason)}`, error: true });
    } else setNote({ text: what, error: false });
  };

  const onClick = (e: ReactMouseEvent<HTMLCanvasElement>): void => {
    const handle = handleRef.current;
    const t = handle?.transform() ?? null;
    if (handle === null || t === null) return;
    const canvas = e.currentTarget;
    const rect = canvas.getBoundingClientRect();
    const cssX = e.clientX - rect.left;
    const cssY = e.clientY - rect.top;
    const box = {
      width: canvas.width,
      height: canvas.height,
      clientWidth: canvas.clientWidth || rect.width,
      clientHeight: canvas.clientHeight || rect.height,
    };
    const px = cssX * (box.clientWidth > 0 ? box.width / box.clientWidth : 1);
    const py = cssY * (box.clientHeight > 0 ? box.height / box.clientHeight : 1);
    const hit = hitLabel(handle.labels(), px, py);
    if (hit !== null) {
      const goal = goalPayload(hit.target[0], hit.target[1], hit.target[2]);
      if (goal !== null) send(chans.goal, goal, `goal → ${hit.name}`);
      return;
    }
    const [x, y] = clickToWorld(t, cssX, cssY, box);
    const goal = goalPayload(x, y);
    if (goal === null) {
      setNote({ text: "goal outside the map bounds", error: true });
      return;
    }
    send(
      chans.goal,
      goal,
      `goal → (${(goal.x as number).toFixed(2)}, ${(goal.y as number).toFixed(2)})`,
    );
  };

  const cancelNav = (): void => send(chans.command, { name: "cancel_nav" }, "cancel");
  const onKeyDown = (e: KeyboardEvent<HTMLDivElement>): void => {
    if (e.key === "Escape") cancelNav();
  };

  return (
    <PanelFrame
      spec={spec}
      badge={
        <Badge
          store={store}
          ch={chans.costmap}
          health={health}
          staleMs={NAVMAP_STALE_MS}
          unit="Hz"
          testId={`navmap-${chans.costmap}-badge`}
        />
      }
    >
      <div
        className={styles.wrap}
        data-testid={`navmap-${chans.costmap}`}
        tabIndex={0}
        role="application"
        aria-label={`navigation map ${chans.costmap}`}
        onKeyDown={onKeyDown}
      >
        <canvas
          ref={baseRef}
          className={styles.base}
          data-testid={`navmap-${chans.costmap}-canvas`}
          role="img"
          aria-label={spec.id}
        />
        <canvas
          ref={overlayRef}
          className={styles.overlay}
          data-testid={`navmap-${chans.costmap}-overlay`}
          onClick={onClick}
        />
        {slot === null && <span className={styles.waiting}>waiting for costmap...</span>}
        <div className={styles.chips}>
          {nav !== null && (
            <span
              className={NAV_CHIP_CLASS[navTone(nav)]}
              data-testid="navmap-chip"
              data-nav-state={nav.state}
            >
              {nav.state.replace(/_/g, " ")}
              {nav.goal !== null && ` → (${nav.goal.x.toFixed(2)}, ${nav.goal.y.toFixed(2)})`}
              {navCancellable(nav) && (
                <button
                  type="button"
                  className={styles.cancel}
                  aria-label="cancel navigation"
                  data-testid="navmap-cancel"
                  onMouseDown={(ev) => ev.preventDefault()}
                  onClick={cancelNav}
                >
                  ✕
                </button>
              )}
            </span>
          )}
          {note !== null && (
            <span
              className={note.error ? styles.chipBad : styles.chip}
              data-testid="navmap-note"
              role={note.error ? "alert" : undefined}
            >
              {note.text}
            </span>
          )}
        </div>
      </div>
    </PanelFrame>
  );
}

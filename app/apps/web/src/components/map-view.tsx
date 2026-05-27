"use client";

import type { Frame, MapSnapshot, TrajectoryPoint } from "@robomoo/shared";
import { useCallback, useEffect, useRef, useState } from "react";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { cosine, embedText } from "@/lib/clip-search";
import {
  fitView,
  type MapMeta,
  type View,
  worldToScreen,
  zoomAt,
} from "@/lib/map-transform";
import { rpcClient } from "@/lib/orpc";

const PIN_R = 6; // screen-px pin radius (constant — pins don't scale with zoom)
const TOP_K = 12; // semantic-search results to highlight + list

type Match = { frame: Frame; score: number };

// Recolor the value-encoded grayscale occupancy PNG into a display layer,
// pre-flipped vertically so it sits in map space (y-up). Returns an offscreen
// canvas sized to the grid. Reads pixels back — safe because the source is a
// same-origin data URI, so the canvas is not tainted.
async function buildMapLayer(
  dataUri: string,
  width: number,
  height: number,
): Promise<HTMLCanvasElement> {
  const img = new Image();
  img.src = dataUri;
  await img.decode();

  const off = document.createElement("canvas");
  off.width = width;
  off.height = height;
  const c = off.getContext("2d");
  if (!c) return off;

  c.save();
  c.translate(0, height);
  c.scale(1, -1); // flip: grid row 0 (min world y) → bottom
  c.drawImage(img, 0, 0, width, height);
  c.restore();

  const id = c.getImageData(0, 0, width, height);
  const d = id.data;
  for (let i = 0; i < d.length; i += 4) {
    const v = d[i] ?? 0; // raw cell value lives in the R channel
    if (v === 255) {
      d[i + 3] = 0; // unknown → transparent
    } else if (v === 0) {
      d[i] = 30;
      d[i + 1] = 30;
      d[i + 2] = 34; // free → dim floor
      d[i + 3] = 255;
    } else {
      const g = Math.round(40 + (Math.min(v, 100) / 100) * 190); // occupied → bright wall
      d[i] = g;
      d[i + 1] = g;
      d[i + 2] = Math.min(g + 8, 255);
      d[i + 3] = 255;
    }
  }
  c.putImageData(id, 0, 0);
  return off;
}

export function MapView() {
  const [map, setMap] = useState<MapSnapshot | null>(null);
  const [frames, setFrames] = useState<Frame[]>([]);
  const [traj, setTraj] = useState<TrajectoryPoint[]>([]);
  const [selected, setSelected] = useState<Frame | null>(null);
  const [hovered, setHovered] = useState<{ f: Frame; sx: number; sy: number } | null>(null);

  const [query, setQuery] = useState("");
  const [searching, setSearching] = useState(false);
  const [matches, setMatches] = useState<Match[]>([]);
  const embeddedRef = useRef<Frame[]>([]);
  const matchesRef = useRef<Match[]>([]);

  const canvasRef = useRef<HTMLCanvasElement>(null);
  const containerRef = useRef<HTMLDivElement>(null);
  const viewRef = useRef<View | null>(null);
  const metaRef = useRef<MapMeta | null>(null);
  const layerRef = useRef<HTMLCanvasElement | null>(null);
  const framesRef = useRef<Frame[]>([]);
  const trajRef = useRef<TrajectoryPoint[]>([]);
  const selectedRef = useRef<Frame | null>(null);
  const dragRef = useRef<{ x: number; y: number } | null>(null);
  const interactedRef = useRef(false); // stop auto-fitting once the user pans/zooms
  const rafRef = useRef(0);

  // --- data polling (unchanged cadence) ----------------------------------
  useEffect(() => {
    let active = true;
    const load = async () => {
      try {
        const [m, f, t] = await Promise.all([
          rpcClient.map.latest(),
          rpcClient.frames.list(),
          rpcClient.trajectory.latest(),
        ]);
        if (!active) return;
        setMap(m);
        setFrames(f);
        setTraj(t?.points ?? []);
      } catch {
        /* keep last good state */
      }
    };
    load();
    const t = setInterval(load, 3000);
    return () => {
      active = false;
      clearInterval(t);
    };
  }, []);

  // embedded frames for semantic search — fetched once (heavier payload)
  useEffect(() => {
    let active = true;
    rpcClient.frames
      .embedded()
      .then((f) => {
        if (active) embeddedRef.current = f;
      })
      .catch(() => {});
    return () => {
      active = false;
    };
  }, []);

  // --- draw ---------------------------------------------------------------
  const draw = useCallback(() => {
    const cv = canvasRef.current;
    const view = viewRef.current;
    const meta = metaRef.current;
    if (!cv || !view || !meta) return;
    const ctx = cv.getContext("2d");
    if (!ctx) return;
    const dpr = window.devicePixelRatio || 1;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    const cw = cv.width / dpr;
    const ch = cv.height / dpr;
    ctx.clearRect(0, 0, cw, ch);

    const layer = layerRef.current;
    if (layer) {
      ctx.imageSmoothingEnabled = false;
      ctx.drawImage(
        layer,
        view.tx,
        view.ty,
        meta.width * view.scale,
        meta.height * view.scale,
      );
    }

    // odometry trajectory (driven path) + robot heading marker
    const path = trajRef.current;
    if (path.length > 1) {
      ctx.beginPath();
      path.forEach((p, i) => {
        const { sx, sy } = worldToScreen(meta, view, p.x, p.y);
        if (i === 0) ctx.moveTo(sx, sy);
        else ctx.lineTo(sx, sy);
      });
      ctx.strokeStyle = "#38bdf8";
      ctx.lineWidth = 2;
      ctx.lineJoin = "round";
      ctx.stroke();
    }
    const last = path[path.length - 1];
    if (last) {
      const { sx, sy } = worldToScreen(meta, view, last.x, last.y);
      // heading: world +theta is CCW about +y-up, which is CCW on screen too
      const hx = sx + 14 * Math.cos(last.theta);
      const hy = sy - 14 * Math.sin(last.theta);
      ctx.beginPath();
      ctx.moveTo(sx, sy);
      ctx.lineTo(hx, hy);
      ctx.strokeStyle = "#0ea5e9";
      ctx.lineWidth = 2.5;
      ctx.stroke();
      ctx.beginPath();
      ctx.arc(sx, sy, 5, 0, Math.PI * 2);
      ctx.fillStyle = "#0ea5e9";
      ctx.fill();
      ctx.lineWidth = 2;
      ctx.strokeStyle = "white";
      ctx.stroke();
    }

    // capture pins
    for (const f of framesRef.current) {
      if (f.poseX === null || f.poseY === null) continue;
      const { sx, sy } = worldToScreen(meta, view, f.poseX, f.poseY);
      ctx.beginPath();
      ctx.arc(sx, sy, PIN_R, 0, Math.PI * 2);
      ctx.fillStyle = selectedRef.current?.id === f.id ? "#22c55e" : "#ef4444";
      ctx.fill();
      ctx.lineWidth = 1.5;
      ctx.strokeStyle = "white";
      ctx.stroke();
    }

    // semantic-search matches — ranked highlight rings (best = largest/greenest)
    const top = matchesRef.current;
    top.forEach((m, i) => {
      if (m.frame.poseX === null || m.frame.poseY === null) return;
      const { sx, sy } = worldToScreen(meta, view, m.frame.poseX, m.frame.poseY);
      const t = 1 - i / Math.max(top.length, 1); // rank weight
      ctx.beginPath();
      ctx.arc(sx, sy, PIN_R + 4 + t * 6, 0, Math.PI * 2);
      ctx.lineWidth = 3;
      ctx.strokeStyle = `hsl(${90 + t * 50}, 85%, 55%)`;
      ctx.stroke();
    });
  }, []);

  const scheduleDraw = useCallback(() => {
    cancelAnimationFrame(rafRef.current);
    rafRef.current = requestAnimationFrame(draw);
  }, [draw]);

  const runSearch = useCallback(async () => {
    const q = query.trim();
    if (!q) return;
    setSearching(true);
    try {
      const qv = await embedText(q);
      const scored: Match[] = [];
      for (const f of embeddedRef.current) {
        if (!f.embedding || f.poseX === null || f.poseY === null) continue;
        scored.push({ frame: f, score: cosine(qv, f.embedding) });
      }
      scored.sort((a, b) => b.score - a.score);
      const top = scored.slice(0, TOP_K);
      matchesRef.current = top;
      setMatches(top);
      scheduleDraw();
    } finally {
      setSearching(false);
    }
  }, [query, scheduleDraw]);

  const clearSearch = useCallback(() => {
    setQuery("");
    setMatches([]);
    matchesRef.current = [];
    scheduleDraw();
  }, [scheduleDraw]);

  // rebuild the map layer + (re)fit the view when a new snapshot arrives
  useEffect(() => {
    if (!map) return;
    const meta: MapMeta = {
      resolution: map.resolution,
      originX: map.originX,
      originY: map.originY,
      width: map.width,
      height: map.height,
    };
    metaRef.current = meta;
    let cancelled = false;
    buildMapLayer(map.imageDataUri, map.width, map.height).then((layer) => {
      if (cancelled) return;
      layerRef.current = layer;
      const c = containerRef.current;
      if (c && !interactedRef.current) {
        viewRef.current = fitView(meta, c.clientWidth, c.clientHeight);
      }
      scheduleDraw();
    });
    return () => {
      cancelled = true;
    };
  }, [map, scheduleDraw]);

  useEffect(() => {
    framesRef.current = frames;
    scheduleDraw();
  }, [frames, scheduleDraw]);
  useEffect(() => {
    trajRef.current = traj;
    scheduleDraw();
  }, [traj, scheduleDraw]);
  useEffect(() => {
    selectedRef.current = selected;
    scheduleDraw();
  }, [selected, scheduleDraw]);

  // --- canvas sizing + interaction ---------------------------------------
  useEffect(() => {
    const cv = canvasRef.current;
    const container = containerRef.current;
    if (!cv || !container) return;

    const resize = () => {
      const dpr = window.devicePixelRatio || 1;
      const w = container.clientWidth;
      const h = container.clientHeight;
      cv.width = Math.round(w * dpr);
      cv.height = Math.round(h * dpr);
      cv.style.width = `${w}px`;
      cv.style.height = `${h}px`;
      if (metaRef.current && !interactedRef.current)
        viewRef.current = fitView(metaRef.current, w, h);
      scheduleDraw();
    };
    resize();
    const ro = new ResizeObserver(resize);
    ro.observe(container);

    const pos = (e: PointerEvent) => {
      const r = cv.getBoundingClientRect();
      return { x: e.clientX - r.left, y: e.clientY - r.top };
    };
    const hit = (sx: number, sy: number): Frame | null => {
      const view = viewRef.current;
      const meta = metaRef.current;
      if (!view || !meta) return null;
      for (const f of framesRef.current) {
        if (f.poseX === null || f.poseY === null) continue;
        const p = worldToScreen(meta, view, f.poseX, f.poseY);
        if (Math.hypot(p.sx - sx, p.sy - sy) <= PIN_R + 3) return f;
      }
      return null;
    };

    const onDown = (e: PointerEvent) => {
      const { x, y } = pos(e);
      const f = hit(x, y);
      if (f) {
        setSelected(f);
        return;
      }
      dragRef.current = { x, y };
      cv.setPointerCapture(e.pointerId);
    };
    const onMove = (e: PointerEvent) => {
      const { x, y } = pos(e);
      const view = viewRef.current;
      if (dragRef.current && view) {
        interactedRef.current = true;
        view.tx += x - dragRef.current.x;
        view.ty += y - dragRef.current.y;
        dragRef.current = { x, y };
        setHovered(null);
        scheduleDraw();
        return;
      }
      const f = hit(x, y);
      setHovered(f ? { f, sx: x, sy: y } : null);
    };
    const onUp = (e: PointerEvent) => {
      dragRef.current = null;
      cv.releasePointerCapture?.(e.pointerId);
    };
    const onWheel = (e: WheelEvent) => {
      e.preventDefault();
      const view = viewRef.current;
      if (!view) return;
      interactedRef.current = true;
      const r = cv.getBoundingClientRect();
      const factor = Math.exp(-e.deltaY * 0.0015);
      viewRef.current = zoomAt(view, factor, e.clientX - r.left, e.clientY - r.top);
      scheduleDraw();
    };

    cv.addEventListener("pointerdown", onDown);
    cv.addEventListener("pointermove", onMove);
    cv.addEventListener("pointerup", onUp);
    cv.addEventListener("wheel", onWheel, { passive: false });
    return () => {
      ro.disconnect();
      cv.removeEventListener("pointerdown", onDown);
      cv.removeEventListener("pointermove", onMove);
      cv.removeEventListener("pointerup", onUp);
      cv.removeEventListener("wheel", onWheel);
    };
  }, [scheduleDraw]);

  const pins = frames.filter((f) => f.poseX !== null && f.poseY !== null);

  return (
    <div className="flex flex-col gap-4">
      <form
        className="flex gap-2"
        onSubmit={(e) => {
          e.preventDefault();
          runSearch();
        }}
      >
        <Input
          onChange={(e) => setQuery(e.target.value)}
          placeholder="Search what the robot saw — e.g. “office plant”, “doorway”…"
          value={query}
        />
        <Button disabled={searching || !query.trim()} type="submit">
          {searching ? "Searching…" : "Search"}
        </Button>
        {matches.length > 0 ? (
          <Button onClick={clearSearch} type="button" variant="outline">
            Clear
          </Button>
        ) : null}
      </form>

      <div
        className="relative h-[60vh] w-full overflow-hidden rounded-md border bg-black"
        ref={containerRef}
      >
        <canvas className="block touch-none" ref={canvasRef} />
        {!map ? (
          <div className="pointer-events-none absolute inset-0 grid place-items-center text-muted-foreground text-sm">
            No map snapshot yet. The robot POSTs one to{" "}
            <code className="ml-1">/api/robot/map</code>.
          </div>
        ) : null}
        {hovered ? (
          <div
            className="pointer-events-none absolute z-10 rounded bg-background/90 px-2 py-1 text-xs shadow"
            style={{ left: hovered.sx + 10, top: hovered.sy + 10 }}
          >
            {hovered.f.label ?? hovered.f.note ?? "capture"}
          </div>
        ) : null}
      </div>

      {matches.length > 0 ? (
        <div className="flex flex-col gap-2">
          <span className="text-muted-foreground text-xs">
            Top {matches.length} matches for “{query}” — highlighted on the map.
          </span>
          <div className="grid grid-cols-3 gap-2 sm:grid-cols-6">
            {matches.map((m) => (
              <button
                className="group relative overflow-hidden rounded-md border"
                key={m.frame.id}
                onClick={() => setSelected(m.frame)}
                type="button"
              >
                {/* biome-ignore lint/performance/noImgElement: presigned URL */}
                <img
                  alt={m.frame.label ?? "match"}
                  className="aspect-square w-full object-cover"
                  src={m.frame.imageUrl}
                />
                <span className="absolute right-0 bottom-0 bg-black/70 px-1 text-[10px] text-white">
                  {m.score.toFixed(2)}
                </span>
              </button>
            ))}
          </div>
        </div>
      ) : null}

      {selected ? (
        <div className="flex flex-col gap-1">
          {/* biome-ignore lint/performance/noImgElement: presigned URL */}
          <img
            alt={selected.label ?? "capture"}
            className="max-h-80 rounded-md border object-contain"
            src={selected.imageUrl}
          />
          <span className="text-muted-foreground text-xs">
            {selected.label ?? selected.note ?? "capture"} ·{" "}
            {new Date(selected.createdAt).toLocaleString()}
          </span>
        </div>
      ) : (
        <p className="text-muted-foreground text-sm">
          {pins.length} capture{pins.length === 1 ? "" : "s"} on the map — scroll
          to zoom, drag to pan, click a pin to view the photo.
        </p>
      )}
    </div>
  );
}

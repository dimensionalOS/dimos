// WorldView — fuses spatial streams into one 2D canvas, auto-detected by type:
//   OccupancyGrid → grey cells · PointCloud2 → height-coloured lidar ·
//   LaserScan → cyan points · Path → green line · PoseStamped → yellow robot + trail.
// INTERACT: scroll = zoom (toward cursor) · drag = pan · click = nav goal
//   (→ clicked_point → A* planner → robot drives) · double-click / ⊙ = recenter+follow.
// A view transform (scale + center, in a ref) makes the map zoomable/pannable; the
// canvas is responsive (ResizeObserver + devicePixelRatio). Uses useTopicRef + a
// requestAnimationFrame loop: redraws at display rate, never re-rendering per message.
import { useEffect, useRef, useState } from "react";
import type { TopicInfo } from "@dimos/react";
import { useDimosClient, useTopicRef, useTopics, useTopicStats } from "../dimos";

const LIDAR_CAP = 4000; // max points rendered per frame (stride-decimated)
const DEFAULT_SCALE = 64; // px per metre at rest
const MIN_SCALE = 8;
const MAX_SCALE = 400;

function quatToYaw(q: { x: number; y: number; z: number; w: number }): number {
  return Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
}
function pick(topics: TopicInfo[], type: string, prefer: string[]): string | null {
  for (const p of prefer) if (topics.find((t) => t.topic === p && t.type === type)) return p;
  return topics.find((t) => t.type === type)?.topic ?? null;
}

function fmtBw(bps?: number): string {
  if (!bps) return "";
  const kb = bps / 1000;
  return kb >= 1000 ? `${(kb / 1000).toFixed(1)} MB/s` : kb >= 1 ? `${kb.toFixed(0)} kB/s` : "";
}

/** A layer toggle that ALSO shows the layer's live bandwidth — so turning lidar OFF visibly drops
 *  ~2 MB/s (true on-demand: OFF passes null to useTopicRef → unsubscribes the topic on the wire). */
function LayerChip({ name, topic, color, on, onToggle }: {
  name: string;
  topic: string;
  color: string;
  on: boolean;
  onToggle: () => void;
}) {
  const stats = useTopicStats(topic); // passive read — decays to 0 when the layer is off
  const bw = on ? fmtBw(stats?.bytesPerSec) : "";
  return (
    <button
      className={on ? "tab tab-active" : "tab"}
      onClick={onToggle}
      title={`${on ? "subscribed" : "off — saves bandwidth"} · ${topic}`}
      style={{ padding: "2px 8px", display: "flex", alignItems: "center", gap: 5 }}
    >
      <span
        style={{
          width: 7,
          height: 7,
          borderRadius: "50%",
          background: color,
          opacity: on ? 1 : 0.3,
        }}
      />
      {name}
      {bw && <span className="muted" style={{ fontSize: 10 }}>{bw}</span>}
    </button>
  );
}

export function WorldView() {
  const client = useDimosClient();
  const topics = useTopics();
  const poseTopic = pick(topics, "geometry_msgs.PoseStamped", ["/odom"]);
  const mapTopic = pick(topics, "nav_msgs.OccupancyGrid", ["/map", "/navigation_costmap"]);
  const lidarTopic = pick(topics, "sensor_msgs.PointCloud2", ["/lidar"]);
  const scanTopic = pick(topics, "sensor_msgs.LaserScan", ["/scan"]);
  const pathTopic = pick(topics, "nav_msgs.Path", ["/path"]);

  // per-layer on/off → gates the subscription: a layer that's OFF passes null
  // to useTopicRef, which ref-counts down and actually UNSUBSCRIBES on the wire.
  const [layers, setLayers] = useState({
    pose: true,
    map: true,
    lidar: true,
    scan: true,
    path: true,
  });
  const odom = useTopicRef<any>(layers.pose ? poseTopic : null);
  const map = useTopicRef<any>(layers.map ? mapTopic : null);
  const lidar = useTopicRef<any>(layers.lidar ? lidarTopic : null);
  const scan = useTopicRef<any>(layers.scan ? scanTopic : null);
  const path = useTopicRef<any>(layers.path ? pathTopic : null);

  const canvasRef = useRef<HTMLCanvasElement>(null);
  const trail = useRef<[number, number][]>([]);
  const goal = useRef<[number, number] | null>(null);
  // view transform: scale = px/m, (cx,cy) = world coords shown at canvas centre,
  // follow = keep re-centring on the robot until the user pans/zooms.
  const view = useRef({ scale: DEFAULT_SCALE, cx: 0, cy: 0, follow: true });

  const recenter = () => {
    view.current.follow = true;
    view.current.scale = DEFAULT_SCALE;
  };

  // ── interaction: wheel-zoom (to cursor), drag-pan, click→nav-goal ──────────
  useEffect(() => {
    const cvs = canvasRef.current;
    if (!cvs) return;
    const px = (e: { clientX: number; clientY: number }) => {
      const r = cvs.getBoundingClientRect();
      return {
        mx: e.clientX - r.left,
        my: e.clientY - r.top,
        cw: cvs.clientWidth,
        ch: cvs.clientHeight,
      };
    };
    const onWheel = (e: WheelEvent) => {
      e.preventDefault();
      const { mx, my, cw, ch } = px(e);
      const S = view.current.scale;
      const wx = view.current.cx + (mx - cw / 2) / S;
      const wy = view.current.cy - (my - ch / 2) / S;
      const nS = Math.max(MIN_SCALE, Math.min(MAX_SCALE, S * Math.exp(-e.deltaY * 0.0015)));
      view.current.cx = wx - (mx - cw / 2) / nS;
      view.current.cy = wy + (my - ch / 2) / nS;
      view.current.scale = nS;
      view.current.follow = false;
    };
    let dragging = false;
    let sx = 0, sy = 0, scx = 0, scy = 0, moved = 0;
    const onDown = (e: MouseEvent) => {
      dragging = true;
      sx = e.clientX;
      sy = e.clientY;
      scx = view.current.cx;
      scy = view.current.cy;
      moved = 0;
    };
    const onMove = (e: MouseEvent) => {
      if (!dragging) return;
      const S = view.current.scale;
      const dx = e.clientX - sx, dy = e.clientY - sy;
      moved = Math.max(moved, Math.hypot(dx, dy));
      if (moved > 3) view.current.follow = false;
      view.current.cx = scx - dx / S;
      view.current.cy = scy + dy / S;
    };
    const onUp = (e: MouseEvent) => {
      if (!dragging) return;
      dragging = false;
      if (moved < 4 && client) {
        // a click (not a pan) → unproject at the current view → nav goal
        const { mx, my, cw, ch } = px(e);
        const S = view.current.scale;
        const wx = view.current.cx + (mx - cw / 2) / S;
        const wy = view.current.cy - (my - ch / 2) / S;
        goal.current = [wx, wy];
        client.navigate(wx, wy, 0);
      }
    };
    const onDbl = () => recenter();
    cvs.addEventListener("wheel", onWheel, { passive: false });
    cvs.addEventListener("mousedown", onDown);
    window.addEventListener("mousemove", onMove);
    window.addEventListener("mouseup", onUp);
    cvs.addEventListener("dblclick", onDbl);
    return () => {
      cvs.removeEventListener("wheel", onWheel);
      cvs.removeEventListener("mousedown", onDown);
      window.removeEventListener("mousemove", onMove);
      window.removeEventListener("mouseup", onUp);
      cvs.removeEventListener("dblclick", onDbl);
    };
  }, [client]);

  // ── render loop (responsive + DPR-aware) ──────────────────────────────────
  useEffect(() => {
    const cvs = canvasRef.current;
    if (!cvs) return;
    const ctx = cvs.getContext("2d")!;
    let raf = 0;
    const ro = new ResizeObserver(() => {
      const dpr = window.devicePixelRatio || 1;
      cvs.width = Math.max(1, Math.floor(cvs.clientWidth * dpr));
      cvs.height = Math.max(1, Math.floor(cvs.clientHeight * dpr));
    });
    ro.observe(cvs);

    const draw = () => {
      raf = requestAnimationFrame(draw);
      const cw = cvs.clientWidth, ch = cvs.clientHeight;
      if (cw === 0 || ch === 0) return; // not laid out yet
      const dpr = window.devicePixelRatio || 1;
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0); // draw in CSS px

      const p = odom.current.data?.pose;
      const rx = p?.position?.x ?? 0;
      const ry = p?.position?.y ?? 0;
      const yaw = p ? quatToYaw(p.orientation) : 0;
      if (view.current.follow && p) {
        view.current.cx = rx;
        view.current.cy = ry;
      }
      const S = view.current.scale;
      const W = (wx: number) => cw / 2 + (wx - view.current.cx) * S;
      const H = (wy: number) => ch / 2 - (wy - view.current.cy) * S;

      ctx.fillStyle = "#0b0e14";
      ctx.fillRect(0, 0, cw, ch);

      // adaptive grid — 1 m lines, coarsening to 5 m / 10 m when zoomed out.
      const step = S >= 32 ? 1 : S >= 12 ? 5 : 10;
      const wxMin = view.current.cx - cw / 2 / S, wxMax = view.current.cx + cw / 2 / S;
      const wyMin = view.current.cy - ch / 2 / S, wyMax = view.current.cy + ch / 2 / S;
      ctx.strokeStyle = "#1b2230";
      ctx.lineWidth = 1;
      for (let mx = Math.ceil(wxMin / step) * step; mx <= wxMax; mx += step) {
        ctx.beginPath();
        ctx.moveTo(W(mx), 0);
        ctx.lineTo(W(mx), ch);
        ctx.stroke();
      }
      for (let my = Math.ceil(wyMin / step) * step; my <= wyMax; my += step) {
        ctx.beginPath();
        ctx.moveTo(0, H(my));
        ctx.lineTo(cw, H(my));
        ctx.stroke();
      }

      const m = map.current.data;
      if (m?.info) {
        const { resolution: res, width, height } = m.info;
        const ox = m.info.origin?.position?.x ?? 0;
        const oy = m.info.origin?.position?.y ?? 0;
        const cell = res * S;
        ctx.fillStyle = "#39424f";
        for (let row = 0; row < height; row++) {
          for (let col = 0; col < width; col++) {
            if (m.data[row * width + col] > 50) {
              ctx.fillRect(W(ox + col * res), H(oy + row * res) - cell, cell + 0.5, cell + 0.5);
            }
          }
        }
      }

      // lidar PointCloud2 (world frame) — height-coloured top-down points.
      const lc = lidar.current.data;
      if (lc?.data?.byteLength && lc.point_step && lc.fields?.length) {
        const fx = lc.fields.find((f: any) => f.name === "x");
        const fy = lc.fields.find((f: any) => f.name === "y");
        const fz = lc.fields.find((f: any) => f.name === "z");
        if (fx && fy) {
          const dv = new DataView(lc.data.buffer, lc.data.byteOffset, lc.data.byteLength);
          const le = !lc.is_bigendian;
          const n = Math.floor(lc.data.byteLength / lc.point_step);
          const stride = Math.max(1, Math.ceil(n / LIDAR_CAP));
          for (let i = 0; i < n; i += stride) {
            const b = i * lc.point_step;
            const x = dv.getFloat32(b + fx.offset, le);
            const y = dv.getFloat32(b + fy.offset, le);
            if (!Number.isFinite(x) || !Number.isFinite(y)) continue;
            const z = fz ? dv.getFloat32(b + fz.offset, le) : 0;
            const t = Math.max(0, Math.min(1, (z + 0.2) / 2)); // ~[-0.2,1.8]m → 0..1
            ctx.fillStyle = `hsl(${200 - t * 160}, 85%, ${42 + t * 18}%)`; // blue→amber by height
            ctx.fillRect(W(x), H(y), 1.6, 1.6);
          }
        }
      }

      const s = scan.current.data;
      if (s?.ranges) {
        ctx.fillStyle = "#27d3e0";
        for (let i = 0; i < s.ranges.length; i++) {
          const r = s.ranges[i];
          if (!(r > s.range_min) || r >= s.range_max) continue;
          const a = yaw + s.angle_min + i * s.angle_increment;
          ctx.fillRect(W(rx + r * Math.cos(a)) - 1, H(ry + r * Math.sin(a)) - 1, 2, 2);
        }
      }

      const pa = path.current.data?.poses;
      if (pa?.length) {
        ctx.strokeStyle = "#4ade80";
        ctx.lineWidth = 2;
        ctx.beginPath();
        pa.forEach((ps: any, i: number) => {
          const x = ps.pose.position.x, y = ps.pose.position.y;
          i ? ctx.lineTo(W(x), H(y)) : ctx.moveTo(W(x), H(y));
        });
        ctx.stroke();
      }

      const g = goal.current;
      if (g) {
        ctx.strokeStyle = "#f472b6";
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.arc(W(g[0]), H(g[1]), 7, 0, Math.PI * 2);
        ctx.stroke();
        ctx.beginPath();
        ctx.moveTo(W(g[0]) - 11, H(g[1]));
        ctx.lineTo(W(g[0]) + 11, H(g[1]));
        ctx.moveTo(W(g[0]), H(g[1]) - 11);
        ctx.lineTo(W(g[0]), H(g[1]) + 11);
        ctx.stroke();
      }

      if (p) {
        const t = trail.current;
        const last = t[t.length - 1];
        if (!last || Math.hypot(last[0] - rx, last[1] - ry) > 0.02) t.push([rx, ry]);
        if (t.length > 400) t.shift();
        ctx.strokeStyle = "#3b6ea5";
        ctx.lineWidth = 2;
        ctx.beginPath();
        t.forEach(([wx, wy], i) => (i ? ctx.lineTo(W(wx), H(wy)) : ctx.moveTo(W(wx), H(wy))));
        ctx.stroke();
        ctx.save();
        ctx.translate(W(rx), H(ry));
        ctx.rotate(-yaw);
        ctx.fillStyle = "#ffcb47";
        ctx.beginPath();
        ctx.moveTo(12, 0);
        ctx.lineTo(-7, 7);
        ctx.lineTo(-7, -7);
        ctx.closePath();
        ctx.fill();
        ctx.restore();
      } else {
        ctx.fillStyle = "#5b6676";
        ctx.font = "13px ui-monospace, monospace";
        ctx.fillText("waiting for spatial topics (pose / map / scan / path)…", 16, 26);
      }
    };
    raf = requestAnimationFrame(draw);
    return () => {
      cancelAnimationFrame(raf);
      ro.disconnect();
    };
  }, [odom, map, lidar, scan, path]);

  const tag = (l: string, t: string | null) => (t ? `${l}:${t}` : "");
  const title = [
    tag("pose", poseTopic),
    tag("map", mapTopic),
    tag("lidar", lidarTopic),
    tag("scan", scanTopic),
    tag("path", pathTopic),
  ]
    .filter(Boolean)
    .join("  ") || "auto-detecting…";

  // toggle chips — one per DETECTED layer (absent streams get no chip); dot
  // colour mirrors that layer's draw colour, active chip = currently subscribed.
  const chips: [keyof typeof layers, string | null, string][] = [
    ["pose", poseTopic, "#ffcb47"],
    ["map", mapTopic, "#8b95a5"],
    ["lidar", lidarTopic, "#f0a830"],
    ["scan", scanTopic, "#27d3e0"],
    ["path", pathTopic, "#4ade80"],
  ];

  return (
    <div
      className="panel"
      style={{ display: "flex", flexDirection: "column", flex: 1, minHeight: 0 }}
    >
      <div
        className="panel-title"
        style={{ display: "flex", justifyContent: "space-between", alignItems: "center", gap: 8 }}
      >
        <span>
          WorldView · {title} · <span className="muted">scroll=zoom drag=pan click=goal</span>
        </span>
        <button
          className="tab"
          onClick={recenter}
          title="recenter + follow robot"
          style={{ padding: "2px 8px" }}
        >
          ⊙ recenter
        </button>
      </div>
      {/* per-layer subscribe toggles — only for detected topics */}
      <div style={{ display: "flex", gap: 6, padding: "6px 10px 0", flexWrap: "wrap" }}>
        {chips
          .filter(([, t]) => t)
          .map(([k, t, c]) => (
            <LayerChip
              key={k}
              name={k}
              topic={t!}
              color={c}
              on={layers[k]}
              onToggle={() => setLayers((s) => ({ ...s, [k]: !s[k] }))}
            />
          ))}
      </div>
      <div style={{ flex: 1, minHeight: 0, position: "relative" }}>
        <canvas
          ref={canvasRef}
          className="worldcanvas"
          style={{ width: "100%", height: "100%", display: "block", cursor: "crosshair" }}
        />
      </div>
    </div>
  );
}

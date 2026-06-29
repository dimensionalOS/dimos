// ─────────────────────────────────────────────────────────────────────────
// WorldView — the hero visualization (Rungs 3 & 5, now sim-agnostic)
//
// One 2D canvas fusing whatever spatial streams are on the bus:
//   OccupancyGrid -> grey occupied cells   (e.g. /map or /navigation_costmap)
//   LaserScan     -> cyan lidar points     (e.g. /scan)
//   Path          -> green planned path     (e.g. /path)
//   PoseStamped   -> yellow robot + trail   (e.g. /odom)
//
// It AUTO-DETECTS the topics by message TYPE from discovery, so it works the
// same against simplerobot, fakesensors, and the go2 replay with no config.
// ─────────────────────────────────────────────────────────────────────────
import { useEffect, useRef } from "react";
import { useTopic, useTopics } from "../useBus";

const SIZE = 480;
const SCALE = 64; // pixels per meter

function quatToYaw(q: { x: number; y: number; z: number; w: number }): number {
  return Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
}

// Pick a topic by message type, honoring a preference order.
function pick(topics: Map<string, string>, type: string, prefer: string[]): string | null {
  for (const p of prefer) if (topics.get(p) === type) return p;
  for (const [t, ty] of topics) if (ty === type) return t;
  return null;
}

export function WorldView() {
  const { topics } = useTopics();
  const poseTopic = pick(topics, "geometry_msgs.PoseStamped", ["/odom"]);
  const mapTopic = pick(topics, "nav_msgs.OccupancyGrid", ["/map", "/navigation_costmap", "/global_costmap"]);
  const scanTopic = pick(topics, "sensor_msgs.LaserScan", ["/scan"]);
  const pathTopic = pick(topics, "nav_msgs.Path", ["/path"]);

  const odom = useTopic(poseTopic);
  const map = useTopic(mapTopic);
  const scan = useTopic(scanTopic);
  const path = useTopic(pathTopic);

  const canvasRef = useRef<HTMLCanvasElement>(null);
  const trail = useRef<[number, number][]>([]);

  useEffect(() => {
    const cvs = canvasRef.current;
    if (!cvs) return;
    const ctx = cvs.getContext("2d")!;
    const cx = SIZE / 2, cy = SIZE / 2;
    const W = (wx: number) => cx + wx * SCALE;
    const H = (wy: number) => cy - wy * SCALE;

    ctx.fillStyle = "#0b0e14";
    ctx.fillRect(0, 0, SIZE, SIZE);
    ctx.strokeStyle = "#1b2230";
    ctx.lineWidth = 1;
    for (let m = -3; m <= 3; m++) {
      ctx.beginPath(); ctx.moveTo(W(m), 0); ctx.lineTo(W(m), SIZE); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(0, H(m)); ctx.lineTo(SIZE, H(m)); ctx.stroke();
    }

    // OccupancyGrid
    const m = map?.data;
    if (m && m.info) {
      const { resolution: res, width, height } = m.info;
      const ox = m.info.origin?.position?.x ?? 0;
      const oy = m.info.origin?.position?.y ?? 0;
      const cell = res * SCALE;
      ctx.fillStyle = "#39424f";
      for (let row = 0; row < height; row++) {
        for (let col = 0; col < width; col++) {
          if (m.data[row * width + col] > 50) {
            ctx.fillRect(W(ox + col * res), H(oy + row * res) - cell, cell + 0.5, cell + 0.5);
          }
        }
      }
    }

    const p = odom?.data?.pose;
    const rx = p?.position?.x ?? 0;
    const ry = p?.position?.y ?? 0;
    const yaw = p ? quatToYaw(p.orientation) : 0;

    // LaserScan (angles relative to robot heading)
    const s = scan?.data;
    if (s && s.ranges) {
      ctx.fillStyle = "#27d3e0";
      for (let i = 0; i < s.ranges.length; i++) {
        const r = s.ranges[i];
        if (!(r > s.range_min) || r >= s.range_max) continue;
        const a = yaw + s.angle_min + i * s.angle_increment;
        ctx.fillRect(W(rx + r * Math.cos(a)) - 1, H(ry + r * Math.sin(a)) - 1, 2, 2);
      }
    }

    // Path (planned route): green polyline through pose positions
    const pa = path?.data?.poses;
    if (pa && pa.length) {
      ctx.strokeStyle = "#4ade80";
      ctx.lineWidth = 2;
      ctx.beginPath();
      pa.forEach((ps: any, i: number) => {
        const x = ps.pose.position.x, y = ps.pose.position.y;
        i ? ctx.lineTo(W(x), H(y)) : ctx.moveTo(W(x), H(y));
      });
      ctx.stroke();
    }

    // Robot trail + body
    if (odom) {
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
      ctx.moveTo(12, 0); ctx.lineTo(-7, 7); ctx.lineTo(-7, -7); ctx.closePath();
      ctx.fill();
      ctx.restore();
    }

    if (!odom && !map && !scan && !path) {
      ctx.fillStyle = "#5b6676";
      ctx.font = "13px ui-monospace, monospace";
      ctx.fillText("waiting for spatial topics (pose / map / scan / path) …", 18, 28);
    }
  }, [odom, map, scan, path]);

  const tag = (label: string, topic: string | null) => (topic ? `${label}:${topic}` : "");
  const title = [tag("pose", poseTopic), tag("map", mapTopic), tag("scan", scanTopic), tag("path", pathTopic)]
    .filter(Boolean)
    .join("  ");

  return (
    <div className="panel">
      <div className="panel-title">WorldView · {title || "auto-detecting…"}</div>
      <canvas ref={canvasRef} width={SIZE} height={SIZE} className="worldcanvas" />
    </div>
  );
}

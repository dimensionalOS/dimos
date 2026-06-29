// WorldView — fuses spatial streams into one 2D canvas, auto-detected by type:
//   OccupancyGrid → grey cells · LaserScan → cyan points · Path → green line ·
//   PoseStamped → yellow robot + trail.
// Uses useTopicRef + a requestAnimationFrame loop: the canvas redraws at display
// rate reading the latest sample, never re-rendering React per message.
import { useEffect, useRef } from "react";
import { useTopicRef, useTopics, type TopicInfo } from "@dimos/react";

const SIZE = 460;
const SCALE = 64; // px per meter

function quatToYaw(q: { x: number; y: number; z: number; w: number }): number {
  return Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
}
function pick(topics: TopicInfo[], type: string, prefer: string[]): string | null {
  for (const p of prefer) if (topics.find((t) => t.topic === p && t.type === type)) return p;
  return topics.find((t) => t.type === type)?.topic ?? null;
}

export function WorldView() {
  const topics = useTopics();
  const poseTopic = pick(topics, "geometry_msgs.PoseStamped", ["/odom"]);
  const mapTopic = pick(topics, "nav_msgs.OccupancyGrid", ["/map", "/navigation_costmap"]);
  const scanTopic = pick(topics, "sensor_msgs.LaserScan", ["/scan"]);
  const pathTopic = pick(topics, "nav_msgs.Path", ["/path"]);

  const odom = useTopicRef<any>(poseTopic);
  const map = useTopicRef<any>(mapTopic);
  const scan = useTopicRef<any>(scanTopic);
  const path = useTopicRef<any>(pathTopic);

  const canvasRef = useRef<HTMLCanvasElement>(null);
  const trail = useRef<[number, number][]>([]);

  useEffect(() => {
    const cvs = canvasRef.current;
    if (!cvs) return;
    const ctx = cvs.getContext("2d")!;
    const cx = SIZE / 2,
      cy = SIZE / 2;
    const W = (wx: number) => cx + wx * SCALE;
    const H = (wy: number) => cy - wy * SCALE;
    let raf = 0;

    const draw = () => {
      ctx.fillStyle = "#0b0e14";
      ctx.fillRect(0, 0, SIZE, SIZE);
      ctx.strokeStyle = "#1b2230";
      ctx.lineWidth = 1;
      for (let m = -3; m <= 3; m++) {
        ctx.beginPath(); ctx.moveTo(W(m), 0); ctx.lineTo(W(m), SIZE); ctx.stroke();
        ctx.beginPath(); ctx.moveTo(0, H(m)); ctx.lineTo(SIZE, H(m)); ctx.stroke();
      }

      const m = map.current.data;
      if (m?.info) {
        const { resolution: res, width, height } = m.info;
        const ox = m.info.origin?.position?.x ?? 0;
        const oy = m.info.origin?.position?.y ?? 0;
        const cell = res * SCALE;
        ctx.fillStyle = "#39424f";
        for (let row = 0; row < height; row++)
          for (let col = 0; col < width; col++)
            if (m.data[row * width + col] > 50)
              ctx.fillRect(W(ox + col * res), H(oy + row * res) - cell, cell + 0.5, cell + 0.5);
      }

      const p = odom.current.data?.pose;
      const rx = p?.position?.x ?? 0;
      const ry = p?.position?.y ?? 0;
      const yaw = p ? quatToYaw(p.orientation) : 0;

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
        ctx.moveTo(12, 0); ctx.lineTo(-7, 7); ctx.lineTo(-7, -7); ctx.closePath();
        ctx.fill();
        ctx.restore();
      } else {
        ctx.fillStyle = "#5b6676";
        ctx.font = "13px ui-monospace, monospace";
        ctx.fillText("waiting for spatial topics (pose / map / scan / path)…", 16, 26);
      }
      raf = requestAnimationFrame(draw);
    };
    raf = requestAnimationFrame(draw);
    return () => cancelAnimationFrame(raf);
  }, [odom, map, scan, path]);

  const tag = (l: string, t: string | null) => (t ? `${l}:${t}` : "");
  const title =
    [tag("pose", poseTopic), tag("map", mapTopic), tag("scan", scanTopic), tag("path", pathTopic)]
      .filter(Boolean)
      .join("  ") || "auto-detecting…";

  return (
    <div className="panel">
      <div className="panel-title">WorldView · {title}</div>
      <canvas ref={canvasRef} width={SIZE} height={SIZE} className="worldcanvas" />
    </div>
  );
}

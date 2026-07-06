// CloudCompare — the same live PointCloud2 rendered three ways side-by-side: raw, gateway-downsampled
// (_ds), and Draco-decoded (_draco). All three share ONE auto-fit view (computed from the raw cloud's
// xy bounds) so the comparison is honest — differences are density/fidelity, not zoom. Each cell shows
// live kB/s (useTopicStats) + point count, so you see the bandwidth win beside the visual fidelity.
// The Draco cell decodes the wasm geometry in an async decoding-guarded rAF (mirrors useImageTopic);
// if draco3d isn't installed it falls back to the on-wire stats with a note.
import { useEffect, useMemo, useRef, useState } from "react";
import type { TopicInfo } from "@dimos/react";
import { useTopicRef, useTopics, useTopicStats } from "../../dimos";
import { DRACO_TYPE } from "@dimos/web";
import { decodeDracoGeometry } from "./dracoDecode";
import { type CloudLike, cloudBounds, drawCloud } from "./drawCloud";

const CAP = 8000; // per-cell render cap (higher than WorldView — this is the detail view)

function fmtBw(bps?: number): string {
  if (!bps) return "—";
  const kb = bps / 1000;
  return kb >= 1000 ? `${(kb / 1000).toFixed(1)} MB/s` : `${kb.toFixed(0)} kB/s`;
}

/** Base PointCloud2 topics that have a `<base>_draco` sibling (i.e. the cloud plane is transcoding
 *  them). Prefer /lidar, then /load/cloud. */
function baseClouds(topics: TopicInfo[]): string[] {
  const names = new Set(topics.map((t) => t.topic));
  const bases = topics
    .filter((t) => t.type === "sensor_msgs.PointCloud2" && !t.topic.endsWith("_ds"))
    .map((t) => t.topic)
    .filter((t) => names.has(t + "_draco"));
  return bases.sort((a, b) => {
    const rank = (s: string) => (s === "/lidar" ? 0 : s === "/load/cloud" ? 1 : 2);
    return rank(a) - rank(b) || a.localeCompare(b);
  });
}

/** Build a PointCloud2-shaped object from a Draco-decoded interleaved xyz Float32Array so drawCloud
 *  renders it unchanged (contiguous float32 xyz → point_step 12, little-endian). */
function synthCloud(xyz: Float32Array): CloudLike {
  return {
    data: new Uint8Array(xyz.buffer, xyz.byteOffset, xyz.byteLength),
    point_step: 12,
    fields: [{ name: "x", offset: 0 }, { name: "y", offset: 4 }, { name: "z", offset: 8 }],
    is_bigendian: 0,
  };
}

type View = { scale: number; cx: number; cy: number };

/** Fit the raw cloud's xy bounds into a wxh canvas (px/m + world centre), with a 10% margin. */
function fitView(cloud: CloudLike | null | undefined, w: number, h: number): View {
  const b = cloudBounds(cloud);
  if (!b) return { scale: 40, cx: 0, cy: 0 };
  const spanX = Math.max(0.5, b.maxX - b.minX);
  const spanY = Math.max(0.5, b.maxY - b.minY);
  const scale = Math.min(w / spanX, h / spanY) * 0.9;
  return { scale, cx: (b.minX + b.maxX) / 2, cy: (b.minY + b.maxY) / 2 };
}

function CloudCell(
  { title, topic, note, canvasRef, count }: {
    title: string;
    topic: string | null;
    note?: string;
    canvasRef: React.RefObject<HTMLCanvasElement>;
    count: number;
  },
) {
  const stats = useTopicStats(topic ?? "");
  return (
    <div className="cloud-cell" style={{ flex: 1, display: "flex", flexDirection: "column", minWidth: 0 }}>
      <div className="panel-title" style={{ display: "flex", justifyContent: "space-between", gap: 8 }}>
        <span>{title}</span>
        <span className="muted" style={{ fontSize: 11 }}>
          {fmtBw(stats?.bytesPerSec)} · {count ? `${count.toLocaleString()} pts` : "—"}
        </span>
      </div>
      <div style={{ position: "relative", flex: 1, minHeight: 200, background: "#0b0e14", borderRadius: 6 }}>
        <canvas ref={canvasRef} style={{ width: "100%", height: "100%", display: "block" }} />
        {note && (
          <div
            className="muted"
            style={{ position: "absolute", left: 8, bottom: 6, fontSize: 11, opacity: 0.8 }}
          >
            {note}
          </div>
        )}
      </div>
    </div>
  );
}

export function CloudCompare() {
  const topics = useTopics();
  const bases = useMemo(() => baseClouds(topics), [topics]);
  const [base, setBase] = useState<string | null>(null);
  // default to the first available base (prefers /lidar) until the user picks
  const activeBase = base && bases.includes(base) ? base : bases[0] ?? null;
  const dsTopic = activeBase ? activeBase + "_ds" : null;
  const drTopic = activeBase ? activeBase + "_draco" : null;

  const raw = useTopicRef<any>(activeBase);
  const ds = useTopicRef<any>(dsTopic);
  const dr = useTopicRef<any>(drTopic); // .current.data = {frame_id,n_points,ts,_draco}

  const rawCv = useRef<HTMLCanvasElement>(null);
  const dsCv = useRef<HTMLCanvasElement>(null);
  const drCv = useRef<HTMLCanvasElement>(null);

  // async Draco decode state (mirrors useImageTopic's decoding guard — never await in a subscribe cb)
  const dracoGeom = useRef<CloudLike | null>(null);
  const decoding = useRef(false);
  const lastDecoded = useRef<string | null>(null);
  const [dracoStatus, setDracoStatus] = useState<"idle" | "ok" | "unavailable">("idle");

  const counts = useRef({ raw: 0, ds: 0, draco: 0 });
  const [, force] = useState(0); // low-rate re-render so the pt-count captions update

  const drTypeIsDraco = topics.find((t) => t.topic === drTopic)?.type === DRACO_TYPE;

  useEffect(() => {
    let alive = true;
    const sizeTo = (cv: HTMLCanvasElement | null) => {
      if (!cv) return null;
      const dpr = window.devicePixelRatio || 1;
      const w = cv.clientWidth, h = cv.clientHeight;
      if (cv.width !== Math.round(w * dpr) || cv.height !== Math.round(h * dpr)) {
        cv.width = Math.round(w * dpr);
        cv.height = Math.round(h * dpr);
      }
      const ctx = cv.getContext("2d");
      if (ctx) ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      return ctx ? { ctx, w, h } : null;
    };

    const paint = (cv: HTMLCanvasElement | null, cloud: CloudLike | null | undefined, view: View) => {
      const s = sizeTo(cv);
      if (!s) return 0;
      const { ctx, w, h } = s;
      ctx.fillStyle = "#0b0e14";
      ctx.fillRect(0, 0, w, h);
      const W = (wx: number) => w / 2 + (wx - view.cx) * view.scale;
      const H = (wy: number) => h / 2 - (wy - view.cy) * view.scale;
      return drawCloud(ctx, cloud, W, H, CAP);
    };

    const loop = () => {
      if (!alive) return;
      const rawCloud = raw.current.data as CloudLike | undefined;
      // one shared view from the raw cloud → identical framing in all three cells
      const cw = rawCv.current?.clientWidth ?? 300;
      const ch = rawCv.current?.clientHeight ?? 200;
      const view = fitView(rawCloud, cw, ch);

      counts.current.raw = paint(rawCv.current, rawCloud, view);
      counts.current.ds = paint(dsCv.current, ds.current.data as CloudLike | undefined, view);

      // Draco: kick an async decode when the frame changed and we're free; draw the last decoded.
      const env = dr.current.data as { frame_id?: string; _draco?: Uint8Array } | undefined;
      if (env?._draco && env.frame_id !== lastDecoded.current && !decoding.current) {
        decoding.current = true;
        const fid = env.frame_id ?? "";
        const blob = env._draco;
        decodeDracoGeometry(blob)
          .then((xyz) => {
            if (!alive) return;
            if (xyz) {
              dracoGeom.current = synthCloud(xyz);
              lastDecoded.current = fid;
              setDracoStatus("ok");
            } else {
              setDracoStatus("unavailable");
            }
          })
          .finally(() => {
            decoding.current = false;
          });
      }
      counts.current.draco = paint(drCv.current, dracoGeom.current, view);

      raf = requestAnimationFrame(loop);
    };
    let raf = requestAnimationFrame(loop);
    const tick = setInterval(() => alive && force((n) => n + 1), 500); // refresh captions
    return () => {
      alive = false;
      cancelAnimationFrame(raf);
      clearInterval(tick);
    };
  }, [activeBase]);

  if (!activeBase) {
    return (
      <div className="panel" style={{ margin: 16 }}>
        <div className="panel-title">Cloud compare</div>
        <div className="muted small">
          No PointCloud2 topic with a Draco sibling yet. Start a source (dog sim / replay) and the
          gateway cloud plane will publish <code>…_ds</code> and <code>…_draco</code>.
        </div>
      </div>
    );
  }

  const drNote = !drTypeIsDraco
    ? "no _draco topic"
    : dracoStatus === "unavailable"
    ? "draco3d not installed — showing wire stats only"
    : dracoStatus === "idle"
    ? "decoding…"
    : undefined;

  return (
    <div className="cloud-compare" style={{ height: "100%", display: "flex", flexDirection: "column", padding: 12, gap: 10 }}>
      <div style={{ display: "flex", alignItems: "center", gap: 10 }}>
        <strong>Point-cloud compare</strong>
        <span className="muted small">same cloud, three encodings — raw vs downsampled vs Draco</span>
        <span style={{ flex: 1 }} />
        <label className="muted small">source&nbsp;
          <select value={activeBase} onChange={(e) => setBase(e.target.value)}>
            {bases.map((b) => <option key={b} value={b}>{b}</option>)}
          </select>
        </label>
      </div>
      <div style={{ flex: 1, display: "flex", gap: 10, minHeight: 0 }}>
        <CloudCell title="Raw" topic={activeBase} canvasRef={rawCv} count={counts.current.raw} />
        <CloudCell title="Downsampled" topic={dsTopic} canvasRef={dsCv} count={counts.current.ds} />
        <CloudCell title="Draco" topic={drTopic} note={drNote} canvasRef={drCv} count={counts.current.draco} />
      </div>
    </div>
  );
}

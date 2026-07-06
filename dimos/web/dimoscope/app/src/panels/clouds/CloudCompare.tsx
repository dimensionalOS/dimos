// CloudCompare — the same live PointCloud2 shown three ways side-by-side: raw, gateway-downsampled
// (_ds), and Draco-decoded (_draco), in a shared 3D orbit view (three.js — drag any cell → all rotate
// together). The cells are FRAME-SYNCED by source timestamp so it's literally the same scan (equal
// counts), and Draco decodes per new frame. Colour encodes the compression story: raw = cyan
// (baseline), downsampled = amber (lossy/sparse), Draco = green (best — full density, tiny). Falls
// back to a 2D top-down render if WebGL is unavailable, and to wire-stats-only if draco3d is absent.
import { useEffect, useMemo, useRef, useState } from "react";
import type { TopicInfo } from "@dimos/react";
import { useTopicRef, useTopics, useTopicStats } from "../../dimos";
import { DRACO_TYPE, type TopicStats } from "@dimos/web";
import { decodeDracoGeometry } from "./dracoDecode";
import { type CloudLike, cloudExtent, cloudToXYZ, drawCloud, synthCloud } from "./drawCloud";
import { type CloudTrio, createCloudTrio } from "./cloudThree";

const RING = 12; // frames kept per encoding for timestamp matching
const TS_BUCKET = 10; // ms — round source ts so raw/ds/draco (same source frame) share a key

const TONE = { raw: "var(--signal)", ds: "var(--accent)", draco: "var(--ok)" } as const;

function kbps(bps?: number): { n: string; u: string } {
  if (!bps) return { n: "—", u: "" };
  const kb = bps / 1000;
  return kb >= 1000 ? { n: (kb / 1000).toFixed(1), u: "MB/s" } : { n: kb.toFixed(0), u: "kB/s" };
}

/** Base PointCloud2 topics that have a `<base>_draco` sibling (the cloud plane is transcoding them). */
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

type Frame = { key: number; ts: number; xyz: Float32Array };

function pushFrame(ring: Frame[], ts: number, xyz: Float32Array) {
  const key = Math.round(ts / TS_BUCKET);
  if (ring.length && ring[ring.length - 1].key === key) ring[ring.length - 1] = { key, ts, xyz };
  else {
    ring.push({ key, ts, xyz });
    if (ring.length > RING) ring.shift();
  }
}
const findKey = (ring: Frame[], key: number) => ring.find((f) => f.key === key);

/** Newest ts-key present in ALL THREE rings (the synced scan), or null if none overlap. */
function commonKey(a: Frame[], b: Frame[], c: Frame[]): number | null {
  const bk = new Set(b.map((f) => f.key)), ck = new Set(c.map((f) => f.key));
  for (let i = a.length - 1; i >= 0; i--) if (bk.has(a[i].key) && ck.has(a[i].key)) return a[i].key;
  return null;
}

function CloudCell(
  { name, tone, stats, ratio, count, note, canvasRef }: {
    name: string;
    tone: string;
    stats?: TopicStats | null;
    ratio?: number;
    count: number;
    note?: string;
    canvasRef: React.RefObject<HTMLCanvasElement>;
  },
) {
  const bw = kbps(stats?.bytesPerSec);
  return (
    <div className="cloud-cell">
      <div className="cloud-cell-head">
        <span className="cloud-dot" style={{ background: tone }} />
        <span className="cloud-name">{name}</span>
        {ratio && ratio >= 1.5 && (
          <span className="cloud-ratio" style={{ color: tone, borderColor: tone }}>
            ↓{ratio >= 10 ? ratio.toFixed(0) : ratio.toFixed(1)}×
          </span>
        )}
      </div>
      <div className="cloud-canvas">
        <canvas ref={canvasRef} />
        {note && <div className="cloud-note">{note}</div>}
      </div>
      <div className="cloud-metric">
        <span className="metric"><b>{bw.n}</b><span>{bw.u}</span></span>
        <span className="cloud-pts">{count ? count.toLocaleString() : "—"} pts</span>
      </div>
    </div>
  );
}

export function CloudCompare() {
  const topics = useTopics();
  const bases = useMemo(() => baseClouds(topics), [topics]);
  const [base, setBase] = useState<string | null>(null);
  const activeBase = base && bases.includes(base) ? base : bases[0] ?? null;
  const dsTopic = activeBase ? activeBase + "_ds" : null;
  const drTopic = activeBase ? activeBase + "_draco" : null;

  const raw = useTopicRef<any>(activeBase);
  const ds = useTopicRef<any>(dsTopic);
  const dr = useTopicRef<any>(drTopic);

  const rawStats = useTopicStats(activeBase ?? "");
  const dsStats = useTopicStats(dsTopic ?? "");
  const drStats = useTopicStats(drTopic ?? "");
  const ratio = (s?: TopicStats | null) =>
    rawStats?.bytesPerSec && s?.bytesPerSec ? rawStats.bytesPerSec / s.bytesPerSec : undefined;

  const rowRef = useRef<HTMLDivElement>(null);
  const rawCv = useRef<HTMLCanvasElement>(null);
  const dsCv = useRef<HTMLCanvasElement>(null);
  const drCv = useRef<HTMLCanvasElement>(null);

  const refit = useRef(true);
  const activeBaseRef = useRef(activeBase);
  activeBaseRef.current = activeBase;

  const [dracoStatus, setDracoStatus] = useState<"idle" | "ok" | "unavailable">("idle");
  const counts = useRef({ raw: 0, ds: 0, draco: 0 });
  const [, force] = useState(0);
  const drTypeIsDraco = topics.find((t) => t.topic === drTopic)?.type === DRACO_TYPE;

  useEffect(() => {
    // Wait until a source exists so the canvas cells are mounted (the empty state renders no canvases).
    if (!activeBase || !rawCv.current || !dsCv.current || !drCv.current || !rowRef.current) return;
    let alive = true;
    let trio: CloudTrio | null = null;
    let mode: "pending" | "3d" | "2d" = "pending";
    const cvs = [rawCv.current, dsCv.current, drCv.current];
    createCloudTrio(cvs, rowRef.current!).then((t) => {
      if (!alive) {
        t?.dispose();
        return;
      }
      trio = t;
      mode = t?.ok ? "3d" : "2d";
    });

    const rings: Record<"raw" | "ds" | "draco", Frame[]> = { raw: [], ds: [], draco: [] };
    const lastTs = { raw: -1, ds: -1 };
    let lastBase = activeBaseRef.current;
    let lastBlob: Uint8Array | null = null;
    let decoding = false;
    let zRange: [number, number] = [0, 1];

    const sample = (which: "raw" | "ds", ref: { current: { data?: unknown; meta?: any } }) => {
      const ts = ref.current.meta?.srcTs;
      if (typeof ts !== "number" || ts === lastTs[which]) return;
      const xyz = cloudToXYZ(ref.current.data as CloudLike | undefined);
      if (!xyz) return;
      lastTs[which] = ts;
      pushFrame(rings[which], ts, xyz);
    };

    const paint2D = (cv: HTMLCanvasElement | null, xyz: Float32Array | null) => {
      if (!cv) return;
      const dpr = window.devicePixelRatio || 1;
      const w = Math.round(cv.clientWidth * dpr), h = Math.round(cv.clientHeight * dpr);
      if (cv.width !== w || cv.height !== h) {
        cv.width = w;
        cv.height = h;
      }
      const ctx = cv.getContext("2d");
      if (!ctx) return;
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      ctx.fillStyle = "#0a0d13";
      ctx.fillRect(0, 0, cv.clientWidth, cv.clientHeight);
      if (!xyz) return;
      const e = cloudExtent(xyz), cw = cv.clientWidth, ch = cv.clientHeight;
      const s = Math.min(cw, ch) / Math.max(1, e.radius * 2) * 0.9;
      drawCloud(
        ctx,
        synthCloud(xyz),
        (x) => cw / 2 + (x - e.center[0]) * s,
        (y) => ch / 2 - (y - e.center[1]) * s,
        20000,
      );
    };

    const loop = () => {
      if (!alive) return;
      if (activeBaseRef.current !== lastBase) {
        lastBase = activeBaseRef.current;
        rings.raw = [];
        rings.ds = [];
        rings.draco = [];
        lastTs.raw = lastTs.ds = -1;
        lastBlob = null;
        refit.current = true;
      }

      sample("raw", raw);
      sample("ds", ds);

      const env = dr.current.data as { ts?: number; _draco?: Uint8Array } | undefined;
      if (env?._draco && env._draco !== lastBlob && !decoding) {
        decoding = true;
        lastBlob = env._draco;
        const ts = (env.ts ?? 0) * 1000;
        decodeDracoGeometry(env._draco)
          .then((xyz) => {
            if (!alive) return;
            if (xyz) {
              pushFrame(rings.draco, ts, xyz);
              setDracoStatus("ok");
            } else setDracoStatus("unavailable");
          })
          .finally(() => (decoding = false));
      }

      const key = commonKey(rings.raw, rings.ds, rings.draco);
      const pick = (r: Frame[]) => (key != null ? findKey(r, key) : r[r.length - 1]);
      const fr = pick(rings.raw), fd = pick(rings.ds), fdr = pick(rings.draco);
      const rawXyz = fr?.xyz ?? rings.raw[rings.raw.length - 1]?.xyz ?? null;

      if (rawXyz) {
        const e = cloudExtent(rawXyz);
        zRange = [e.zMin, e.zMax];
        if (refit.current) {
          trio?.setTarget(e.center, e.radius);
          refit.current = false;
        }
      }
      counts.current.raw = (fr?.xyz.length ?? 0) / 3;
      counts.current.ds = (fd?.xyz.length ?? 0) / 3;
      counts.current.draco = (fdr?.xyz.length ?? 0) / 3;

      if (mode === "3d" && trio) {
        trio.resize();
        trio.setFrame(0, fr?.xyz ?? rawXyz, zRange);
        trio.setFrame(1, fd?.xyz ?? null, zRange);
        trio.setFrame(2, fdr?.xyz ?? null, zRange);
        trio.render();
      } else if (mode === "2d") {
        paint2D(rawCv.current, fr?.xyz ?? rawXyz);
        paint2D(dsCv.current, fd?.xyz ?? null);
        paint2D(drCv.current, fdr?.xyz ?? null);
      }
      raf = requestAnimationFrame(loop);
    };
    let raf = requestAnimationFrame(loop);
    const tick = setInterval(() => alive && force((n) => n + 1), 500);

    return () => {
      alive = false;
      cancelAnimationFrame(raf);
      clearInterval(tick);
      trio?.dispose();
    };
    // Re-run once a source first appears (canvases mount). Source *switches* are handled in-loop.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [!!activeBase]);

  const resetView = () => {
    refit.current = true;
  };

  if (!activeBase) {
    return (
      <div className="panel" style={{ margin: 16 }}>
        <div className="panel-title">Point-cloud compare</div>
        <div className="muted small">
          No point-cloud topic with a Draco sibling yet. Start a source (dog sim / replay) and the
          gateway will publish <code>…_ds</code> and <code>…_draco</code> to compare.
        </div>
      </div>
    );
  }

  const drNote = !drTypeIsDraco
    ? "no _draco topic"
    : dracoStatus === "unavailable"
    ? "draco3d not installed"
    : dracoStatus === "idle"
    ? "decoding…"
    : undefined;

  return (
    <div className="cloud-compare">
      <div className="cloud-head">
        <div>
          <div className="cloud-title">Point-cloud compare</div>
          <div className="cloud-sub">same scan, three encodings · drag any view to orbit all</div>
        </div>
        <div className="cloud-controls">
          <label className="cloud-source">
            source
            <select value={activeBase} onChange={(e) => setBase(e.target.value)}>
              {bases.map((b) => <option key={b} value={b}>{b}</option>)}
            </select>
          </label>
          <button className="tab" onClick={resetView} title="re-frame the view">⟳ reset</button>
        </div>
      </div>

      <div className="cloud-row" ref={rowRef}>
        <CloudCell name="Raw" tone={TONE.raw} stats={rawStats} count={counts.current.raw} canvasRef={rawCv} />
        <CloudCell name="Downsampled" tone={TONE.ds} stats={dsStats} ratio={ratio(dsStats)} count={counts.current.ds} canvasRef={dsCv} />
        <CloudCell name="Draco" tone={TONE.draco} stats={drStats} ratio={ratio(drStats)} count={counts.current.draco} note={drNote} canvasRef={drCv} />
      </div>

      <div className="cloud-legend">
        <span className="cloud-legend-label">height</span>
        <span className="cloud-legend-bar" />
        <span className="muted">low → high</span>
      </div>
    </div>
  );
}

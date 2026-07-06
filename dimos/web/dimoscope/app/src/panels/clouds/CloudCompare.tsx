// CloudCompare — the same live PointCloud2 rendered three ways side-by-side: raw, gateway-downsampled
// (_ds), and Draco-decoded (_draco), in a shared 3D orbit view (drag any cell → all rotate together).
// The three cells are FRAME-SYNCED by source timestamp so "same cloud" is literally the same scan
// (equal point counts), and the Draco cell decodes per NEW frame (not per frame_id — real sensors have
// a non-numeric frame_id, which used to freeze Draco on its first frame). Each cell shows live kB/s +
// point count. Falls back to a 2D top-down render if WebGL is unavailable, and to wire-stats-only if
// draco3d isn't installed.
import { useEffect, useMemo, useRef, useState } from "react";
import type { TopicInfo } from "@dimos/react";
import { useTopicRef, useTopics, useTopicStats } from "../../dimos";
import { DRACO_TYPE } from "@dimos/web";
import { decodeDracoGeometry } from "./dracoDecode";
import { type CloudLike, cloudToXYZ, drawCloud, synthCloud } from "./drawCloud";
import { type CloudGL, cloudExtent, createCloudGL, defaultCam, type OrbitCam } from "./cloudGL";

const RING = 12; // frames kept per encoding for timestamp matching
const TS_BUCKET = 10; // ms — round source ts so raw/ds/draco (same source frame) share a key

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

type Frame = { key: number; ts: number; xyz: Float32Array };

/** Push a frame into a ring, deduped by ts bucket (replace same-bucket), capped to RING. */
function pushFrame(ring: Frame[], ts: number, xyz: Float32Array) {
  const key = Math.round(ts / TS_BUCKET);
  if (ring.length && ring[ring.length - 1].key === key) {
    ring[ring.length - 1] = { key, ts, xyz };
  } else {
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
        <canvas
          ref={canvasRef}
          style={{ width: "100%", height: "100%", display: "block", cursor: "grab" }}
        />
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
  const activeBase = base && bases.includes(base) ? base : bases[0] ?? null;
  const dsTopic = activeBase ? activeBase + "_ds" : null;
  const drTopic = activeBase ? activeBase + "_draco" : null;

  const raw = useTopicRef<any>(activeBase);
  const ds = useTopicRef<any>(dsTopic);
  const dr = useTopicRef<any>(drTopic); // .current.data = {frame_id,n_points,ts,_draco}

  const rowRef = useRef<HTMLDivElement>(null);
  const rawCv = useRef<HTMLCanvasElement>(null);
  const dsCv = useRef<HTMLCanvasElement>(null);
  const drCv = useRef<HTMLCanvasElement>(null);

  const cam = useRef<OrbitCam>(defaultCam());
  const refit = useRef(true); // re-frame the orbit cam on next frame (source change / reset view)
  const activeBaseRef = useRef(activeBase);
  activeBaseRef.current = activeBase;

  const [dracoStatus, setDracoStatus] = useState<"idle" | "ok" | "unavailable">("idle");
  const counts = useRef({ raw: 0, ds: 0, draco: 0 });
  const [, force] = useState(0);
  const drTypeIsDraco = topics.find((t) => t.topic === drTopic)?.type === DRACO_TYPE;

  useEffect(() => {
    let alive = true;
    const gls: Record<"raw" | "ds" | "draco", CloudGL | null> = {
      raw: rawCv.current && createCloudGL(rawCv.current),
      ds: dsCv.current && createCloudGL(dsCv.current),
      draco: drCv.current && createCloudGL(drCv.current),
    };
    const use3D = !!gls.raw?.ok;

    const rings: Record<"raw" | "ds" | "draco", Frame[]> = { raw: [], ds: [], draco: [] };
    const lastTs = { raw: -1, ds: -1 };
    let lastBase = activeBaseRef.current;
    let lastBlob: Uint8Array | null = null;
    let decoding = false;
    let zRange: [number, number] = [0, 1];

    // --- orbit interaction (shared cam → all cells rotate together) ---
    let dragging = false, px = 0, py = 0;
    const onDown = (e: PointerEvent) => {
      dragging = true;
      px = e.clientX;
      py = e.clientY;
      (e.target as HTMLElement).setPointerCapture?.(e.pointerId);
    };
    const onMove = (e: PointerEvent) => {
      if (!dragging) return;
      const dx = e.clientX - px, dy = e.clientY - py;
      px = e.clientX;
      py = e.clientY;
      cam.current.azimuth -= dx * 0.008;
      cam.current.elevation = Math.max(-1.4, Math.min(1.4, cam.current.elevation + dy * 0.008));
    };
    const onUp = () => (dragging = false);
    const onWheel = (e: WheelEvent) => {
      e.preventDefault();
      cam.current.distance = Math.max(0.5, Math.min(200, cam.current.distance * Math.exp(e.deltaY * 0.001)));
    };
    const row = rowRef.current;
    row?.addEventListener("pointerdown", onDown);
    row?.addEventListener("pointermove", onMove);
    window.addEventListener("pointerup", onUp);
    row?.addEventListener("wheel", onWheel, { passive: false });

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
      ctx.fillStyle = "#0b0e14";
      ctx.fillRect(0, 0, cv.clientWidth, cv.clientHeight);
      if (!xyz) return;
      const ext = cloudExtent(xyz);
      const cw = cv.clientWidth, ch = cv.clientHeight;
      const spanX = Math.max(0.5, (ext.radius) * 2), s = Math.min(cw, ch) / spanX * 0.9;
      const W = (x: number) => cw / 2 + (x - ext.center[0]) * s;
      const H = (y: number) => ch / 2 - (y - ext.center[1]) * s;
      drawCloud(ctx, synthCloud(xyz), W, H, 20000);
    };

    const loop = () => {
      if (!alive) return;
      // reset on source change
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

      // Draco: decode when a NEW blob arrives (ref identity changes per delivery), not per frame_id.
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

      // pick the synced triple (newest ts common to all three), else each-latest (live, never stall)
      const key = commonKey(rings.raw, rings.ds, rings.draco);
      const pick = (r: Frame[]) => (key != null ? findKey(r, key) : r[r.length - 1]);
      const fr = pick(rings.raw), fd = pick(rings.ds), fdr = pick(rings.draco);

      const rawXyz = fr?.xyz ?? rings.raw[rings.raw.length - 1]?.xyz ?? null;
      if (rawXyz) {
        const e = cloudExtent(rawXyz);
        zRange = [e.zMin, e.zMax];
        if (refit.current) {
          cam.current.target = e.center;
          cam.current.distance = e.radius * 2.4;
          refit.current = false;
        }
      }

      counts.current.raw = (fr?.xyz.length ?? 0) / 3;
      counts.current.ds = (fd?.xyz.length ?? 0) / 3;
      counts.current.draco = (fdr?.xyz.length ?? 0) / 3;

      if (use3D) {
        gls.raw!.setPoints(fr?.xyz ?? rawXyz, zRange);
        gls.raw!.render(cam.current);
        gls.ds?.setPoints(fd?.xyz ?? null, zRange);
        gls.ds?.render(cam.current);
        gls.draco?.setPoints(fdr?.xyz ?? null, zRange);
        gls.draco?.render(cam.current);
      } else {
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
      row?.removeEventListener("pointerdown", onDown);
      row?.removeEventListener("pointermove", onMove);
      window.removeEventListener("pointerup", onUp);
      row?.removeEventListener("wheel", onWheel);
      gls.raw?.dispose();
      gls.ds?.dispose();
      gls.draco?.dispose();
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  const resetView = () => {
    cam.current.azimuth = defaultCam().azimuth;
    cam.current.elevation = defaultCam().elevation;
    refit.current = true; // re-frame target + distance on the next rendered frame
  };

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
        <span className="muted small">same cloud, three encodings — drag to orbit (all together)</span>
        <span style={{ flex: 1 }} />
        <button className="tab" onClick={resetView} style={{ padding: "2px 8px" }}>reset view</button>
        <label className="muted small">source&nbsp;
          <select value={activeBase} onChange={(e) => setBase(e.target.value)}>
            {bases.map((b) => <option key={b} value={b}>{b}</option>)}
          </select>
        </label>
      </div>
      <div ref={rowRef} style={{ flex: 1, display: "flex", gap: 10, minHeight: 0, touchAction: "none" }}>
        <CloudCell title="Raw" topic={activeBase} canvasRef={rawCv} count={counts.current.raw} />
        <CloudCell title="Downsampled" topic={dsTopic} canvasRef={dsCv} count={counts.current.ds} />
        <CloudCell title="Draco" topic={drTopic} note={drNote} canvasRef={drCv} count={counts.current.draco} />
      </div>
    </div>
  );
}

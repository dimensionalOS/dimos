// Capture viewer — a tiny Bun server that browses a go2-full-recorder SQLite
// capture in the browser. Reads the DB directly (no Python), serves camera
// frames as JPEG and the robot trajectory from pose columns.
//
//   bun run tools/capture-viewer/server.ts [db_path]
//   CAPTURE_DB=/path/to.db bun run tools/capture-viewer/server.ts
//
// Demo-grade and disposable; lidar/costmap/agent are shown as counts only
// (their blobs are LCM/pickle and need Python decoders).

import { Database } from "bun:sqlite";
import { existsSync } from "node:fs";

const DB_PATH = process.argv[2] ?? process.env.CAPTURE_DB ?? "recording_go2.db";
const PORT = Number(process.env.PORT ?? 3000);

// Open read-only per request: the DB may be created after the server starts,
// and a fresh handle avoids stale WAL reads. Returns null if the file is absent.
function openDb(): Database | null {
  if (!existsSync(DB_PATH)) return null;
  return new Database(DB_PATH, { readonly: true });
}

// Names of the real streams in this DB. Used both for the UI and to validate
// any identifier we interpolate into SQL (table names can't be bound).
function streamNames(db: Database): string[] {
  try {
    return db
      .query("SELECT name FROM _streams ORDER BY name")
      .all()
      .map((r: any) => r.name as string);
  } catch {
    // Fallback for DBs without the _streams catalog: derive from "<name>_blob".
    return db
      .query(
        "SELECT name FROM sqlite_master WHERE type='table' AND name LIKE '%\\_blob' ESCAPE '\\'",
      )
      .all()
      .map((r: any) => (r.name as string).replace(/_blob$/, ""));
  }
}

// Extract the JPEG payload from an LCM-wrapped Image blob by scanning for the
// JPEG SOI (FF D8) and the last EOI (FF D9). The payload is a contiguous byte
// field inside the LCM struct, so this is reliable here.
function extractJpeg(buf: Uint8Array): Uint8Array | null {
  let soi = -1;
  for (let i = 0; i + 1 < buf.length; i++) {
    if (buf[i] === 0xff && buf[i + 1] === 0xd8) {
      soi = i;
      break;
    }
  }
  if (soi < 0) return null;
  let eoi = -1;
  for (let i = buf.length - 2; i > soi; i--) {
    if (buf[i] === 0xff && buf[i + 1] === 0xd9) {
      eoi = i + 2;
      break;
    }
  }
  if (eoi < 0) return null;
  return buf.subarray(soi, eoi);
}

const json = (data: unknown, status = 200) => Response.json(data, { status });
const noDb = () =>
  json({ error: `No capture DB at ${DB_PATH}. Record one first.` }, 503);

Bun.serve({
  port: PORT,
  routes: {
    "/": () => new Response(HTML, { headers: { "Content-Type": "text/html" } }),

    // [{ name, count }] for every stream, plus dbPath for the UI header.
    "/api/streams": () => {
      const db = openDb();
      if (!db) return noDb();
      try {
        const names = streamNames(db);
        const out = names.map((name) => {
          let count = 0;
          try {
            const row = db.query(`SELECT COUNT(*) AS c FROM "${name}"`).get() as any;
            count = row?.c ?? 0;
          } catch {
            /* listed in _streams but no table yet */
          }
          return { name, count };
        });
        return json({ dbPath: DB_PATH, streams: out });
      } finally {
        db.close();
      }
    },

    // Camera frame index for the scrubber (ordered by time).
    "/api/frames": () => {
      const db = openDb();
      if (!db) return noDb();
      try {
        if (!streamNames(db).includes("color_image")) return json([]);
        const rows = db
          .query(
            `SELECT id, ts, pose_x, pose_y, pose_qz, pose_qw
               FROM "color_image" ORDER BY ts ASC`,
          )
          .all();
        return json(rows);
      } finally {
        db.close();
      }
    },

    // Dense path for the map: prefer odom, fall back to color_image poses.
    "/api/trajectory": () => {
      const db = openDb();
      if (!db) return noDb();
      try {
        const names = streamNames(db);
        const src = names.includes("odom")
          ? "odom"
          : names.includes("color_image")
            ? "color_image"
            : null;
        if (!src) return json([]);
        const rows = db
          .query(
            `SELECT ts, pose_x, pose_y FROM "${src}"
               WHERE pose_x IS NOT NULL ORDER BY ts ASC`,
          )
          .all();
        return json(rows);
      } finally {
        db.close();
      }
    },

    // A single camera frame as image/jpeg.
    "/api/frame/:id": (req) => {
      const db = openDb();
      if (!db) return noDb();
      try {
        const row = db
          .query(`SELECT data FROM "color_image_blob" WHERE id = $id`)
          .get({ $id: Number(req.params.id) }) as any;
        if (!row?.data) return new Response("not found", { status: 404 });
        const jpeg = extractJpeg(row.data as Uint8Array);
        if (!jpeg) return new Response("no jpeg", { status: 404 });
        return new Response(jpeg, { headers: { "Content-Type": "image/jpeg" } });
      } finally {
        db.close();
      }
    },
  },
});

console.log(`capture-viewer: http://localhost:${PORT}  (db: ${DB_PATH})`);

const HTML = /* html */ `<!doctype html>
<html><head><meta charset="utf-8"><title>Go2 Capture Viewer</title>
<style>
  :root { color-scheme: dark; }
  body { margin:0; font:14px/1.4 system-ui,sans-serif; background:#0c0d10; color:#e6e6e6; }
  header { padding:10px 16px; border-bottom:1px solid #222; display:flex; gap:18px; align-items:baseline; }
  header h1 { font-size:15px; margin:0; font-weight:600; }
  header .meta { color:#8a8f98; font-size:12px; }
  main { display:grid; grid-template-columns:1.4fr 1fr; gap:14px; padding:14px; }
  .panel { background:#15171c; border:1px solid #23262d; border-radius:10px; padding:12px; }
  #frame { width:100%; aspect-ratio:16/9; object-fit:contain; background:#000; border-radius:6px; }
  canvas { width:100%; height:auto; background:#0a0a0c; border-radius:6px; display:block; }
  .scrub { display:flex; align-items:center; gap:10px; margin-top:10px; }
  input[type=range] { flex:1; }
  table { width:100%; border-collapse:collapse; font-size:13px; }
  td,th { text-align:left; padding:4px 6px; border-bottom:1px solid #23262d; }
  th { color:#8a8f98; font-weight:500; }
  .num { text-align:right; font-variant-numeric:tabular-nums; }
  .empty { color:#8a8f98; padding:24px; text-align:center; }
</style></head>
<body>
  <header>
    <h1>🐕 Go2 Capture Viewer</h1>
    <span class="meta" id="meta">loading…</span>
  </header>
  <main>
    <section class="panel">
      <img id="frame" alt="camera frame">
      <div class="scrub">
        <button id="play">▶</button>
        <input id="slider" type="range" min="0" max="0" value="0" step="1">
        <span class="meta" id="t">0.0s</span>
      </div>
    </section>
    <section class="panel">
      <canvas id="map" width="600" height="600"></canvas>
      <table id="streams"><thead><tr><th>stream</th><th class="num">count</th></tr></thead><tbody></tbody></table>
    </section>
  </main>
<script>
let frames = [], traj = [], bounds = null, playing = null;
const $ = (id) => document.getElementById(id);

async function load() {
  const s = await fetch('/api/streams').then(r => r.json());
  if (s.error) { $('meta').textContent = s.error; return; }
  const tb = $('streams').querySelector('tbody');
  tb.innerHTML = s.streams.map(x =>
    '<tr><td>' + x.name + '</td><td class="num">' + x.count + '</td></tr>').join('');

  frames = await fetch('/api/frames').then(r => r.json());
  traj = await fetch('/api/trajectory').then(r => r.json());
  computeBounds();
  if (frames.length) {
    const dur = (frames[frames.length-1].ts - frames[0].ts).toFixed(1);
    $('meta').textContent = frames.length + ' frames · ' + dur + 's · ' + s.dbPath;
    $('slider').max = frames.length - 1;
    show(0);
  } else {
    $('meta').textContent = 'no color_image frames yet · ' + s.dbPath;
    drawMap(-1);
  }
}

function computeBounds() {
  const xs = [], ys = [];
  for (const p of traj) { xs.push(p.pose_x); ys.push(p.pose_y); }
  for (const f of frames) if (f.pose_x != null) { xs.push(f.pose_x); ys.push(f.pose_y); }
  if (!xs.length) { bounds = null; return; }
  const pad = 0.5;
  bounds = { minx: Math.min(...xs)-pad, maxx: Math.max(...xs)+pad,
             miny: Math.min(...ys)-pad, maxy: Math.max(...ys)+pad };
}

function toXY(px, py, w, h) {
  const sx = w / (bounds.maxx - bounds.minx), sy = h / (bounds.maxy - bounds.miny);
  const s = Math.min(sx, sy);
  return [ (px - bounds.minx) * s, h - (py - bounds.miny) * s ];  // flip Y
}

function drawMap(i) {
  const c = $('map'), ctx = c.getContext('2d'), w = c.width, h = c.height;
  ctx.clearRect(0,0,w,h);
  if (!bounds) { ctx.fillStyle='#555'; ctx.fillText('no pose data',20,30); return; }
  // path
  ctx.strokeStyle = '#3b82f6'; ctx.lineWidth = 2; ctx.beginPath();
  traj.forEach((p,k) => { const [x,y]=toXY(p.pose_x,p.pose_y,w,h); k?ctx.lineTo(x,y):ctx.moveTo(x,y); });
  ctx.stroke();
  // current pose marker + heading
  const f = frames[i];
  if (f && f.pose_x != null) {
    const [x,y] = toXY(f.pose_x, f.pose_y, w, h);
    const yaw = 2 * Math.atan2(f.pose_qz ?? 0, f.pose_qw ?? 1);
    ctx.strokeStyle = '#22c55e'; ctx.lineWidth = 3; ctx.beginPath();
    ctx.moveTo(x,y); ctx.lineTo(x + 22*Math.cos(-yaw), y + 22*Math.sin(-yaw)); ctx.stroke();
    ctx.fillStyle = '#22c55e'; ctx.beginPath(); ctx.arc(x,y,6,0,7); ctx.fill();
  }
}

function show(i) {
  i = Math.max(0, Math.min(frames.length-1, i|0));
  $('slider').value = i;
  const f = frames[i];
  $('frame').src = '/api/frame/' + f.id;
  $('t').textContent = (f.ts - frames[0].ts).toFixed(1) + 's';
  drawMap(i);
}

$('slider').addEventListener('input', e => show(+e.target.value));
$('play').addEventListener('click', () => {
  if (playing) { clearInterval(playing); playing = null; $('play').textContent='▶'; return; }
  $('play').textContent='⏸';
  playing = setInterval(() => {
    let i = (+$('slider').value + 1);
    if (i > frames.length-1) i = 0;
    show(i);
  }, 100);
});
load();
</script>
</body></html>`;

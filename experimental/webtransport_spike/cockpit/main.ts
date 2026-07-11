// Throwaway spike cockpit: connects to the relay over WebTransport, renders
// video/odom/lidar, sends teleop + ping datagrams, measures rates.
// Bundle: deno bundle --platform browser -o relay/static/main.js cockpit/main.ts

// Minimal WebTransport typings (lib.dom doesn't ship them everywhere; the
// bundler doesn't typecheck, these are just for editing sanity).
type WT = {
  ready: Promise<void>;
  closed: Promise<unknown>;
  close(): void;
  createBidirectionalStream(): Promise<{ readable: ReadableStream<Uint8Array>; writable: WritableStream<Uint8Array> }>;
  incomingUnidirectionalStreams: ReadableStream<ReadableStream<Uint8Array>>;
  datagrams: {
    readable: ReadableStream<Uint8Array>;
    writable: WritableStream<Uint8Array>;
  };
};

const enc = new TextEncoder();
const dec = new TextDecoder();
const $ = (id: string) => document.getElementById(id)!;
const viewerId = Math.random().toString(36).slice(2, 8);

function setStatus(cls: "ok" | "bad" | "", msg: string) {
  const el = $("status");
  el.className = cls;
  el.textContent = msg;
  if (cls === "bad") console.error(msg);
}

function die(msg: string): never {
  setStatus("bad", msg);
  report("failed:" + msg);
  throw new Error(msg);
}

// ---------- stats ----------

interface ChStat {
  frames: number;
  bytes: number;
  windowFrames: number;
  windowBytes: number;
  hz: number;
  kbPerFrame: number;
  minSeq: number;
  maxSeq: number;
  ooo: number; // arrived after a higher seq (expected: streams are unordered)
  lastArr: number;
  arrGapMax: number; // largest inter-arrival gap in the current window (ms)
}
const stats = new Map<string, ChStat>();
let rttMs = -1;
let state = "init";

function bump(ch: string, bytes: number, seq: number) {
  let s = stats.get(ch);
  if (!s) {
    s = {
      frames: 0, bytes: 0, windowFrames: 0, windowBytes: 0, hz: 0, kbPerFrame: 0,
      minSeq: seq, maxSeq: -1, ooo: 0, lastArr: 0, arrGapMax: 0,
    };
    stats.set(ch, s);
  }
  s.frames++;
  s.bytes += bytes;
  s.windowFrames++;
  s.windowBytes += bytes;
  const now = performance.now();
  if (s.lastArr) s.arrGapMax = Math.max(s.arrGapMax, now - s.lastArr);
  s.lastArr = now;
  if (seq < s.maxSeq) s.ooo++;
  else s.maxSeq = seq;
}

// true loss: seqs in [minSeq, maxSeq] that never arrived at all
function lost(s: ChStat): number {
  return Math.max(0, s.maxSeq - s.minSeq + 1 - s.frames);
}

setInterval(() => {
  const tbody = $("stats").querySelector("tbody")!;
  tbody.innerHTML = "";
  for (const [ch, s] of stats) {
    s.hz = 0.6 * s.windowFrames + 0.4 * s.hz; // 1s window EWMA
    s.kbPerFrame = s.windowFrames ? s.windowBytes / s.windowFrames / 1024 : s.kbPerFrame;
    s.windowFrames = 0;
    s.windowBytes = 0;
    const tr = document.createElement("tr");
    tr.innerHTML = `<td>${ch}</td><td>${s.hz.toFixed(1)}</td><td>${s.kbPerFrame.toFixed(1)}</td>` +
      `<td>${s.frames}</td><td>${lost(s)}</td><td>${s.ooo}</td>`;
    tbody.appendChild(tr);
  }
  $("rtt").textContent = rttMs < 0 ? "rtt: –" : `rtt: ${rttMs.toFixed(1)} ms`;
  $("perf").textContent = `ms (ewma): jpeg-decode ${perf.decode.toFixed(1)}, decode-cycle ${perf.cycle.toFixed(1)}, ` +
    `lidar ${perf.lidar.toFixed(1)}, odom ${perf.odom.toFixed(1)}`;
}, 1000);

function report(st?: string) {
  if (st) state = st;
  const channels: Record<string, unknown> = {};
  for (const [ch, s] of stats) {
    channels[ch] = {
      hz: +s.hz.toFixed(1),
      kbPerFrame: +s.kbPerFrame.toFixed(1),
      frames: s.frames,
      lost: lost(s),
      ooo: s.ooo,
      arrGapMax: +s.arrGapMax.toFixed(0), // reset by the stats tick below
    };
    s.arrGapMax = 0;
  }
  fetch("/api/report", {
    method: "POST",
    headers: { "content-type": "application/json" },
    body: JSON.stringify({
      id: viewerId,
      ua: navigator.userAgent,
      state,
      rtt: +rttMs.toFixed(1),
      channels,
      decoded: { odom: lastOdom, lidarPoints, videoSize, jpegFrames, jpegErrors },
      perfMs: {
        decode: +perf.decode.toFixed(1),
        cycle: +perf.cycle.toFixed(1),
        lidar: +perf.lidar.toFixed(1),
        odom: +perf.odom.toFixed(1),
      },
      ts: Date.now(),
    }),
  }).catch(() => {});
}
setInterval(() => report(), 3000);

// ---------- panels ----------

// decoded-content evidence for /api/report
let lastOdom: unknown = null;
let lidarPoints = 0;
let videoSize = "";
let jpegFrames = 0;
let jpegErrors = 0;

// perf instrumentation (EWMA ms), reported + shown on the page
// cycle = time between consecutive decode completions: ~70 ms means
// arrival-limited (good); much higher means the decode chain is being starved
const perf = { decode: 0, lidar: 0, odom: 0, cycle: 0 };
let lastDecodeDone = 0;
function ewma(prev: number, ms: number): number {
  return prev ? 0.8 * prev + 0.2 * ms : ms;
}

const videoCtx = ($("video") as HTMLCanvasElement).getContext("2d")!;
// Latest-wins decode queue: decode back-to-back at whatever rate the decoder
// manages, always on the freshest frame. (The old skip-if-busy variant only
// ATTEMPTED a decode when a frame happened to arrive while idle, so displayed
// fps collapsed to ~1/cycle-time when the main thread was busy.)
let pendingJpeg: Uint8Array | null = null;
let jpegBusy = false;
function drawJpeg(payload: Uint8Array) {
  pendingJpeg = payload;
  if (!jpegBusy) decodeNextJpeg();
}
function decodeNextJpeg() {
  const payload = pendingJpeg;
  pendingJpeg = null;
  if (!payload) {
    jpegBusy = false;
    return;
  }
  jpegBusy = true;
  const t0 = performance.now();
  createImageBitmap(new Blob([payload], { type: "image/jpeg" }))
    .then((bmp) => {
      const c = videoCtx.canvas;
      if (c.width !== bmp.width) {
        c.width = bmp.width;
        c.height = bmp.height;
      }
      videoCtx.drawImage(bmp, 0, 0);
      perf.decode = ewma(perf.decode, performance.now() - t0);
      if (lastDecodeDone) perf.cycle = ewma(perf.cycle, performance.now() - lastDecodeDone);
      lastDecodeDone = performance.now();
      videoSize = `${bmp.width}x${bmp.height}`;
      jpegFrames++;
      bmp.close();
    })
    .catch(() => jpegErrors++) // synthetic payloads aren't valid JPEG
    .finally(() => decodeNextJpeg());
}

const odomCtx = ($("odom") as HTMLCanvasElement).getContext("2d")!;
const trace: [number, number][] = [];
type Odom = { x: number; y: number; z: number; yaw: number; ts: number };

// Arrival must not drive rendering (the page falls behind and stalls its own
// stream credit): dispatch stashes the latest value, a rAF loop draws it.
let pendingOdom: Odom | null = null;
let pendingLidar: Float32Array | null = null;
let lastOdomDraw = 0;
function renderLoop() {
  // odom gated to 10 Hz: the trace polyline redraw is the second-biggest
  // main-thread cost and 19 Hz adds nothing visually
  if (pendingOdom && performance.now() - lastOdomDraw > 100) {
    lastOdomDraw = performance.now();
    drawOdom(pendingOdom);
    pendingOdom = null;
  }
  if (pendingLidar) {
    drawLidar(pendingLidar);
    pendingLidar = null;
  }
  requestAnimationFrame(renderLoop);
}
requestAnimationFrame(renderLoop);

function drawOdom(p: Odom) {
  const t0 = performance.now();
  lastOdom = p;
  const c = odomCtx.canvas;
  odomCtx.clearRect(0, 0, c.width, c.height);
  let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
  for (const [x, y] of trace) {
    minX = Math.min(minX, x);
    maxX = Math.max(maxX, x);
    minY = Math.min(minY, y);
    maxY = Math.max(maxY, y);
  }
  const span = Math.max(maxX - minX, maxY - minY, 1);
  const scale = (c.width - 30) / span;
  const px = (x: number) => 15 + (x - minX) * scale;
  const py = (y: number) => c.height - 15 - (y - minY) * scale;
  odomCtx.strokeStyle = "#4c9be8";
  odomCtx.beginPath();
  const step = Math.max(1, Math.ceil(trace.length / 1000)); // cap polyline cost
  for (let i = 0; i < trace.length; i += step) {
    const [x, y] = trace[i];
    i ? odomCtx.lineTo(px(x), py(y)) : odomCtx.moveTo(px(x), py(y));
  }
  odomCtx.stroke();
  // heading arrow
  odomCtx.strokeStyle = "#e8734c";
  odomCtx.beginPath();
  odomCtx.moveTo(px(p.x), py(p.y));
  odomCtx.lineTo(px(p.x + 0.15 * span * Math.cos(p.yaw)), py(p.y + 0.15 * span * Math.sin(p.yaw)));
  odomCtx.stroke();
  perf.odom = ewma(perf.odom, performance.now() - t0);
  $("odomText").textContent = `x=${p.x.toFixed(2)} y=${p.y.toFixed(2)} z=${p.z.toFixed(2)} yaw=${p.yaw.toFixed(2)}`;
}

const lidarCtx = ($("lidar") as HTMLCanvasElement).getContext("2d")!;
// Pixel-buffer blit, not 25k fillRect+fillStyle ops: the per-point canvas API
// version cost ~100 ms per cloud and starved the whole main thread (this is
// what made video appear to run at ~1 fps).
let lidarImage: ImageData | null = null;
let lidarPix: Uint32Array | null = null;
function drawLidar(pts: Float32Array) {
  const t0 = performance.now();
  const c = lidarCtx.canvas;
  const W = c.width, H = c.height;
  if (!lidarImage) {
    lidarImage = lidarCtx.createImageData(W, H);
    lidarPix = new Uint32Array(lidarImage.data.buffer);
  }
  const pix = lidarPix!;
  pix.fill(0xff0e0c0b); // ABGR: canvas background
  const n = pts.length / 3;
  lidarPoints = n;
  const half = 15; // meters shown from center to edge
  const s = W / (2 * half);
  for (let i = 0; i < n; i++) {
    const cx = (W / 2 + pts[i * 3] * s) | 0;
    const cy = (H / 2 - pts[i * 3 + 1] * s) | 0;
    if (cx < 0 || cy < 0 || cx >= W || cy >= H) continue;
    const z = pts[i * 3 + 2];
    let g = (140 + z * 60) | 0;
    g = g < 60 ? 60 : g > 255 ? 255 : g;
    pix[cy * W + cx] = 0xff000000 | (g << 16) | (g << 8) | g;
  }
  lidarCtx.putImageData(lidarImage, 0, 0);
  perf.lidar = ewma(perf.lidar, performance.now() - t0);
  $("lidarText").textContent = `${n} pts (±${half} m)`;
}

// ---------- connect ----------

async function main() {
  if (!("WebTransport" in globalThis)) {
    if (!isSecureContext) {
      // Safari doesn't treat http://localhost as a secure context, so all
      // SecureContext-gated APIs (WebTransport included) are hidden there.
      die(
        "This page is not a secure context in this browser (Safari doesn't treat " +
          "http://localhost as secure). Run the relay with --cert/--key (mkcert) and open " +
          `https://localhost:8443/ instead. (${navigator.userAgent})`,
      );
    }
    die(
      "No WebTransport API in this browser. Chrome >= 97, Firefox >= 114, or Safari >= 26.4 " +
        `required. (${navigator.userAgent})`,
    );
  }
  setStatus("", "fetching /api/info…");
  const info = await (await fetch("/api/info")).json();
  // deno-lint-ignore no-explicit-any
  const WT_ = (globalThis as any).WebTransport;
  let wt: WT;
  if (!info.certHash) {
    // relay runs an OS-trusted cert (--cert/--key): plain connect
    wt = new WT_(info.wtUrl);
  } else {
    const hash = Uint8Array.from(atob(info.certHash), (ch) => ch.charCodeAt(0));
    try {
      wt = new WT_(info.wtUrl, {
        serverCertificateHashes: [{ algorithm: "sha-256", value: hash }],
      });
    } catch (e) {
      if (e instanceof DOMException && e.name === "NotSupportedError") {
        // Safari: WebKit doesn't implement serverCertificateHashes. A plain
        // connect only succeeds if the relay's cert is OS-trusted (run the
        // relay with --cert/--key from mkcert); otherwise ready will reject.
        setStatus("", "serverCertificateHashes unsupported; retrying via OS trust store…");
        wt = new WT_(info.wtUrl);
      } else {
        throw e;
      }
    }
  }
  wt.closed.then(
    (info) => die("session closed: " + JSON.stringify(info)),
    (e) => die("session died: " + e),
  );
  setStatus("", `connecting WebTransport to ${info.wtUrl}…`);
  await wt.ready.catch((e: unknown) => die(`WebTransport handshake failed: ${e}`));
  setStatus("ok", `connected to ${info.wtUrl} — ${navigator.userAgent.match(/(Chrome|Firefox)\/[\d.]+/)?.[0] ?? ""}`);
  report("connected");

  // control stream: hello -> welcome
  const ctrl = await wt.createBidirectionalStream();
  const cw = ctrl.writable.getWriter();
  const hello = enc.encode(JSON.stringify({ t: "hello", v: 1, role: "viewer", id: viewerId }));
  const frame = new Uint8Array(4 + hello.length);
  new DataView(frame.buffer).setUint32(0, hello.length, true);
  frame.set(hello, 4);
  await cw.write(frame);
  (async () => {
    for await (const chunk of ctrl.readable) {
      // spike: assume one whole frame per chunk on the control stream
      const len = new DataView(chunk.buffer, chunk.byteOffset).getUint32(0, true);
      console.log("control:", dec.decode(chunk.subarray(4, 4 + len)));
    }
  })().catch(() => {});

  // datagrams: teleop 20 Hz + ping 1 Hz out; pong in -> RTT
  const dgw = wt.datagrams.writable.getWriter();
  const keys = new Set<string>();
  addEventListener("keydown", (e) => keys.add(e.key.toLowerCase()));
  addEventListener("keyup", (e) => keys.delete(e.key.toLowerCase()));
  let teleopSeq = 0;
  setInterval(() => {
    const vx = (keys.has("w") ? 1 : 0) + (keys.has("s") ? -1 : 0);
    const wz = (keys.has("a") ? 1 : 0) + (keys.has("d") ? -1 : 0);
    dgw.write(enc.encode(JSON.stringify({ t: "teleop", vx, wz, seq: teleopSeq++ }))).catch(() => {});
    if (vx || wz) $("teleop").textContent = `teleop: sending vx=${vx} wz=${wz} (seq ${teleopSeq})`;
  }, 50);
  const pings = new Map<number, number>();
  let pingId = 0;
  setInterval(() => {
    const id = pingId++;
    pings.set(id, performance.now());
    if (pings.size > 20) pings.delete(id - 20);
    dgw.write(enc.encode(JSON.stringify({ t: "ping", id, from: viewerId }))).catch(() => {});
  }, 1000);
  (async () => {
    for await (const d of wt.datagrams.readable) {
      try {
        const m = JSON.parse(dec.decode(d));
        if (m.t === "pong" && pings.has(m.id)) {
          rttMs = performance.now() - pings.get(m.id)!;
          pings.delete(m.id);
        }
      } catch {
        // ignore non-JSON datagrams
      }
    }
  })().catch(() => {});

  // data plane: one message per incoming uni stream
  for await (const rs of wt.incomingUnidirectionalStreams) {
    readMessage(rs)
      .then((msg) => {
        const dv = new DataView(msg.buffer, msg.byteOffset);
        const hlen = dv.getUint32(0, true);
        const hdr = JSON.parse(dec.decode(msg.subarray(8, 8 + hlen)));
        const payload = msg.subarray(8 + hlen);
        bump(hdr.ch, payload.byteLength, hdr.seq);
        if (hdr.ch === "video") drawJpeg(payload); // has its own busy-skip
        else if (hdr.ch === "odom") {
          const p = JSON.parse(dec.decode(payload)) as Odom;
          trace.push([p.x, p.y]);
          if (trace.length > 3000) trace.shift();
          pendingOdom = p;
        } else if (hdr.ch === "lidar") {
          pendingLidar = new Float32Array(payload.slice().buffer); // slice() realigns
        }
      })
      .catch(() => {});
  }
}

// Read one length-prefixed message (u32 hlen | u32 plen | header | payload)
// WITHOUT waiting for EOF: Deno's writer.close() doesn't send FIN promptly
// (it only goes out when the resource is GC'd, ~1 s later), so an EOF-based
// reader sees all messages arrive in ~1 s clumps.
async function readMessage(rs: ReadableStream<Uint8Array>): Promise<Uint8Array> {
  const reader = rs.getReader();
  const chunks: Uint8Array[] = [];
  let total = 0;
  let expect = -1;
  try {
    while (expect < 0 || total < expect) {
      const { value, done } = await reader.read();
      if (value) {
        chunks.push(value);
        total += value.byteLength;
      }
      if (expect < 0 && total >= 8) {
        const head = new Uint8Array(8);
        let off = 0;
        for (const c of chunks) {
          const n = Math.min(8 - off, c.byteLength);
          head.set(c.subarray(0, n), off);
          off += n;
          if (off === 8) break;
        }
        const dv = new DataView(head.buffer);
        expect = 8 + dv.getUint32(0, true) + dv.getUint32(4, true);
      }
      if (done) break;
    }
  } finally {
    reader.releaseLock(); // don't cancel; the late FIN closes the stream
  }
  const out = new Uint8Array(total);
  let off = 0;
  for (const c of chunks) {
    out.set(c, off);
    off += c.byteLength;
  }
  if (expect > 0 && total > expect) return out.subarray(0, expect);
  return out;
}

main().catch((e) => setStatus("bad", "fatal: " + e));

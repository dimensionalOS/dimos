// Throwaway WebTransport relay spike.
// Run: deno run -A --unstable-net relay/main.ts
//
// - QUIC/WebTransport listener on UDP :4433 (robot connects to /robot,
//   browsers to /viewer).
// - Plain HTTP on TCP :8000: serves static/ and /api/info (WT url + cert hash).
// - Payload-blind fan-out: every robot uni-stream message is copied to every
//   viewer on a fresh uni stream; per (viewer, channel) skip-if-busy
//   (latest-wins, a slow viewer never stalls others).
// - Datagrams: viewer -> robot (teleop, ping), robot -> all viewers (pong).
//   If no robot is connected the relay echoes pings itself so the browser leg
//   is testable alone.
import { makeEphemeralCert } from "./cert.ts";

// deno#28406: WT sessions leak unhandled rejections on disconnect/idle-timeout
// (e.g. datagram writers' internal `closed` promise). Without this guard the
// relay process dies ~30 s after any browser tab closes.
globalThis.addEventListener("unhandledrejection", (e) => {
  console.log("[relay] unhandled rejection (ignored):", (e.reason as Error)?.message ?? e.reason);
  e.preventDefault();
});

const HTTP_PORT = 8000;
const HTTPS_PORT = 8443; // only served in --cert/--key mode (Safari needs it)
const QUIC_PORT = 4433;

const enc = new TextEncoder();
const dec = new TextDecoder();

// ---------- framing ----------

function encodeFrame(obj: unknown): Uint8Array {
  const body = enc.encode(JSON.stringify(obj));
  const out = new Uint8Array(4 + body.length);
  new DataView(out.buffer).setUint32(0, body.length, true);
  out.set(body, 4);
  return out;
}

// Incremental u32-LE length | JSON frame parser for the control stream.
class FrameReader {
  private buf = new Uint8Array(0);
  push(chunk: Uint8Array): unknown[] {
    const merged = new Uint8Array(this.buf.length + chunk.length);
    merged.set(this.buf, 0);
    merged.set(chunk, this.buf.length);
    this.buf = merged;
    const frames: unknown[] = [];
    while (this.buf.length >= 4) {
      const len = new DataView(this.buf.buffer, this.buf.byteOffset).getUint32(0, true);
      if (this.buf.length < 4 + len) break;
      frames.push(JSON.parse(dec.decode(this.buf.subarray(4, 4 + len))));
      this.buf = this.buf.subarray(4 + len);
    }
    return frames;
  }
}

// data message framing: u32 headerLen | u32 payloadLen | header JSON | payload
// (explicit payloadLen because Deno's writer.close() doesn't send FIN promptly;
// receivers count bytes instead of waiting for EOF)
function peekHeader(msg: Uint8Array): { ch: string; seq: number; ts: number } {
  const hlen = new DataView(msg.buffer, msg.byteOffset).getUint32(0, true);
  return JSON.parse(dec.decode(msg.subarray(8, 8 + hlen)));
}

async function readAll(rs: ReadableStream<Uint8Array>): Promise<Uint8Array> {
  // BYOB reader: Deno's WT incoming streams are resource-backed byte streams
  // whose preamble was parsed with a BYOB reader; default readers were
  // observed to never deliver data here (Deno 2.6.10).
  const reader = rs.getReader({ mode: "byob" });
  const chunks: Uint8Array[] = [];
  let total = 0;
  try {
    while (true) {
      const { value, done } = await reader.read(new Uint8Array(64 * 1024));
      if (value && value.byteLength) {
        chunks.push(value);
        total += value.byteLength;
      }
      if (done) break;
    }
  } finally {
    reader.releaseLock();
  }
  const out = new Uint8Array(total);
  let off = 0;
  for (const c of chunks) {
    out.set(c, off);
    off += c.length;
  }
  return out;
}

// ---------- state ----------

interface Viewer {
  id: number;
  wt: WebTransport;
  dg: WritableStreamDefaultWriter<Uint8Array>;
  busy: Map<string, boolean>;
  dropped: number;
  writeMs: number; // EWMA of create-stream+write+close duration
  slowWrites: number; // writes that took > 150 ms (credit stalls)
}

const viewers = new Set<Viewer>();
let robot: { wt: WebTransport; dg: WritableStreamDefaultWriter<Uint8Array> } | null = null;
let nextViewerId = 1;
let sendSeq = 1;

const chStats = new Map<string, { msgs: number; bytes: number }>();
let dgViewerToRobot = 0;
let dgRobotToViewers = 0;

function bumpCh(ch: string, bytes: number) {
  const s = chStats.get(ch) ?? { msgs: 0, bytes: 0 };
  s.msgs++;
  s.bytes += bytes;
  chStats.set(ch, s);
}

// Browser pages POST their measured stats here; `curl /api/reports` shows them.
const reports = new Map<string, unknown>();

// ---------- forwarding ----------

let lastFanout = 0;
function fanout(msg: Uint8Array) {
  let ch = "?";
  try {
    ch = peekHeader(msg).ch;
  } catch {
    // not our framing; forward as channel "?" anyway
  }
  const now = performance.now();
  if (lastFanout && now - lastFanout > 300) {
    console.log(`[relay] robot->relay arrival gap ${(now - lastFanout) | 0}ms (before ${ch})`);
  }
  lastFanout = now;
  bumpCh(ch, msg.byteLength);
  for (const v of viewers) {
    if (v.busy.get(ch)) {
      v.dropped++;
      continue;
    }
    v.busy.set(ch, true);
    const t0 = performance.now();
    (async () => {
      // waitUntilAvailable: a slow page exhausts its stream credit; without
      // this the create call throws and we'd wrongly drop a live viewer.
      // While waiting, the busy flag sheds this channel's newer frames.
      // sendOrder decreasing = FIFO on the wire: without it quinn round-robins
      // all in-flight streams, so message completions arrive in ~0.7 s waves
      // (Chrome netlog showed smooth frames but clumped stream FINs) and the
      // page sees ~1 fps video.
      const s = await v.wt.createUnidirectionalStream({
        waitUntilAvailable: true,
        sendOrder: -(sendSeq++),
      });
      const w = s.getWriter();
      await w.write(msg);
      await w.close();
      const ms = performance.now() - t0;
      v.writeMs = v.writeMs ? 0.8 * v.writeMs + 0.2 * ms : ms;
      if (ms > 150) v.slowWrites++;
    })()
      .catch(() => dropViewer(v))
      .finally(() => v.busy.set(ch, false));
  }
}

function dropViewer(v: Viewer) {
  if (viewers.delete(v)) console.log(`[relay] viewer ${v.id} dropped (${v.dropped} skipped frames)`);
}

// ---------- session handling ----------

// Answer hello/ping on every incoming bidi (control) stream.
async function handleControl(wt: WebTransport, role: string) {
  for await (const bidi of wt.incomingBidirectionalStreams) {
    (async () => {
      const w = bidi.writable.getWriter();
      const fr = new FrameReader();
      for await (const chunk of bidi.readable) {
        for (const m of fr.push(chunk) as { t: string }[]) {
          if (m.t === "hello") {
            console.log(`[relay] hello from ${role}:`, JSON.stringify(m));
            await w.write(encodeFrame({ t: "welcome", v: 1, role }));
          } else if (m.t === "ping") {
            await w.write(encodeFrame({ ...m, t: "pong" }));
          }
        }
      }
    })().catch((e) =>
      console.log(`[relay] ${role} control stream died:`, (e as Error)?.message ?? e)
    );
  }
}

function handleRobot(wt: WebTransport) {
  if (robot) {
    console.log("[relay] robot takeover: closing previous robot session");
    try {
      robot.wt.close();
    } catch {
      // already gone
    }
  }
  const dg = wt.datagrams.writable.getWriter();
  const me = { wt, dg };
  robot = me;
  console.log("[relay] robot connected");
  wt.closed
    .then(
      (info) => console.log("[relay] robot closed cleanly:", JSON.stringify(info)),
      (e) => console.log("[relay] robot closed with error:", (e as Error)?.message ?? e),
    )
    .finally(() => {
      if (robot === me) robot = null;
      console.log("[relay] robot disconnected");
    });
  (async () => {
    // robot datagrams (pongs) -> every viewer; hello also arrives here
    // because the bidi control exchange is unusable with aioquic (see below)
    for await (const d of wt.datagrams.readable) {
      try {
        const m = JSON.parse(dec.decode(d));
        if (m.t === "hello") {
          console.log("[relay] robot hello (datagram):", dec.decode(d));
          continue;
        }
      } catch {
        // binary datagram: fall through and forward
      }
      dgRobotToViewers++;
      for (const v of viewers) v.dg.write(d).catch(() => {});
    }
  })().catch((e) => console.log("[relay] robot datagram loop died:", (e as Error)?.message ?? e));
  (async () => {
    // Data messages arrive on one-shot BIDI streams, not uni streams: Deno
    // 2.6.10 never delivers incoming WT uni-stream payloads to the app (bug,
    // reproduced with Deno's own client), and the bidi path works. We must
    // never write on these streams (aioquic mis-parses replies as H3 frames
    // and kills the connection), so abort our send side to release stream
    // credit (RESET is invisible to aioquic's h3 layer; FIN is not).
    for await (const bidi of wt.incomingBidirectionalStreams) {
      bidi.writable.abort().catch(() => {});
      readAll(bidi.readable).then(fanout).catch((e) =>
        console.log("[relay] robot stream read failed:", (e as Error)?.message ?? e)
      );
    }
  })().catch((e) => console.log("[relay] robot bidi loop died:", (e as Error)?.message ?? e));
}

function handleViewer(wt: WebTransport) {
  const v: Viewer = {
    id: nextViewerId++,
    wt,
    dg: wt.datagrams.writable.getWriter(),
    busy: new Map(),
    dropped: 0,
    writeMs: 0,
    slowWrites: 0,
  };
  viewers.add(v);
  console.log(`[relay] viewer ${v.id} connected (${viewers.size} total)`);
  wt.closed
    .catch(() => {})
    .finally(() => dropViewer(v));
  (async () => {
    // viewer datagrams (teleop, ping) -> robot; relay echoes pings if no robot
    for await (const d of wt.datagrams.readable) {
      dgViewerToRobot++;
      if (robot) {
        robot.dg.write(d).catch(() => {});
      } else {
        try {
          const m = JSON.parse(dec.decode(d));
          if (m.t === "ping") {
            v.dg.write(enc.encode(JSON.stringify({ ...m, t: "pong", echo: "relay" }))).catch(() => {});
          }
        } catch {
          // non-JSON datagram with no robot: drop
        }
      }
    }
  })().catch(() => {});
  handleControl(wt, "viewer").catch(() => {});
}

// ---------- QUIC listener ----------

// --cert/--key: serve an OS-trusted certificate (e.g. from mkcert) instead of
// the ephemeral one. Needed for Safari, which has no serverCertificateHashes
// (WebKit stated they don't intend to implement it). In this mode /api/info
// advertises no hash: every browser verifies via the OS trust store instead
// (Chrome's hash pinning would reject long-lived certs anyway - 14-day cap).
function argVal(name: string): string | null {
  const i = Deno.args.indexOf(name);
  return i >= 0 ? (Deno.args[i + 1] ?? null) : null;
}
const certFile = argVal("--cert");
const keyFile = argVal("--key");
let cert: { certPem: string; keyPem: string; certHashB64: string | null };
if (certFile && keyFile) {
  cert = {
    certPem: await Deno.readTextFile(certFile),
    keyPem: await Deno.readTextFile(keyFile),
    certHashB64: null,
  };
  console.log(`[relay] using cert from ${certFile} (OS trust store mode, no hash pinning)`);
} else {
  cert = await makeEphemeralCert();
}
const endpoint = new Deno.QuicEndpoint({ hostname: "127.0.0.1", port: QUIC_PORT });
const listener = endpoint.listen({
  cert: cert.certPem,
  key: cert.keyPem,
  alpnProtocols: ["h3"],
  maxIdleTimeout: 30_000,
  keepAliveInterval: 4_000,
});

(async () => {
  for await (const incoming of listener) {
    (async () => {
      const conn = await incoming.accept();
      const wt = await Deno.upgradeWebTransport(conn);
      await wt.ready;
      const path = new URL(wt.url).pathname;
      console.log(`[relay] WT session upgraded: ${wt.url}`);
      if (path === "/robot") handleRobot(wt);
      else handleViewer(wt);
    })().catch((e) => console.error("[relay] accept failed:", e?.message ?? e));
  }
})();

// ---------- HTTP listener ----------

const MIME: Record<string, string> = {
  ".html": "text/html",
  ".js": "application/javascript",
  ".css": "text/css",
  ".map": "application/json",
};
const staticDir = new URL("./static/", import.meta.url);

async function handleHttp(req: Request): Promise<Response> {
  const url = new URL(req.url);
  if (url.pathname === "/api/info") {
    return Response.json({
      // 127.0.0.1, not localhost: Chrome resolves localhost to ::1 first and
      // the QUIC endpoint is IPv4-only. Hash pinning replaces hostname checks.
      wtUrl: `https://127.0.0.1:${QUIC_PORT}/viewer`,
      certHash: cert.certHashB64,
      v: 1,
    });
  }
  if (url.pathname === "/api/report" && req.method === "POST") {
    const body = await req.json();
    reports.set(String(body.id ?? "?"), body);
    return Response.json({ ok: true });
  }
  if (url.pathname === "/api/reports") {
    return Response.json(Object.fromEntries(reports));
  }
  const p = url.pathname === "/" ? "index.html" : url.pathname.slice(1);
  if (p.includes("..")) return new Response("no", { status: 400 });
  try {
    const data = await Deno.readFile(new URL(p, staticDir));
    const ext = p.slice(p.lastIndexOf("."));
    return new Response(data, { headers: { "content-type": MIME[ext] ?? "application/octet-stream" } });
  } catch {
    return new Response("not found", { status: 404 });
  }
}

Deno.serve({ hostname: "127.0.0.1", port: HTTP_PORT }, handleHttp);
if (certFile && keyFile) {
  // Safari hides WebTransport on http://localhost (WebKit doesn't treat it as
  // a secure context, unlike Chrome/Firefox), so in trusted-cert mode also
  // serve the cockpit over HTTPS: open https://localhost:8443/ in Safari.
  Deno.serve(
    { hostname: "127.0.0.1", port: HTTPS_PORT, cert: cert.certPem, key: cert.keyPem },
    handleHttp,
  );
}

// ---------- stats ----------

setInterval(() => {
  const parts: string[] = [];
  for (const [ch, s] of chStats) {
    parts.push(`${ch}=${(s.msgs / 2).toFixed(1)}Hz/${(s.bytes / 2048).toFixed(0)}KBs`);
  }
  chStats.clear();
  const drops = [...viewers]
    .map((v) => `v${v.id}:${v.dropped}d/${v.slowWrites}slow/${v.writeMs.toFixed(0)}ms`)
    .join(" ");
  console.log(
    `[stats] robot=${robot ? "yes" : "no"} viewers=${viewers.size} ` +
      `${parts.join(" ") || "(no data)"} dg(v->r)=${dgViewerToRobot} dg(r->v)=${dgRobotToViewers} drops[${drops}]`,
  );
}, 2000);

console.log(JSON.stringify({
  event: "ready",
  httpPort: HTTP_PORT,
  quicPort: QUIC_PORT,
  certHash: cert.certHashB64,
  url: `http://localhost:${HTTP_PORT}/`,
}));

// ─────────────────────────────────────────────────────────────────────────
// dimos-web-gateway  (Bun ↔ LCM ↔ WebSocket)
//
// A thin, transport-agnostic gateway that puts DimOS topics in the browser.
// Reads the LCM UDP-multicast bus, reassembles fragments (LC03→LC02), and
// relays *raw* LCM packets to browser clients over WebSocket — the browser
// decodes with @dimos/msgs (the 8-byte type hash is self-describing).
//
// Beyond the old dimoscope bridge it adds, per the SDK contract:
//   • per-client ON-DEMAND topic filtering (subscribe/unsubscribe)
//   • a JSON control channel (subscribe / unsubscribe / list / teleop / rate)
//   • server-side DOWNSAMPLE (maxHz per client+topic)
//   • TELEOP SAFETY: velocity clamp + TTL/deadman watchdog + stop-on-disconnect
//
// Why a byte-relay: the gateway is I/O-bound (the browser decodes), so it just
// needs native WebSocket + node:dgram multicast. Deno gives both. Deno cannot
// speak Zenoh natively either — the Zenoh path is gateway-zenoh.py.
//
// RUN:  deno run -A servers/gateway.ts     (env: GATEWAY_PORT, DIMOS_LCM_HOST/PORT)
// ─────────────────────────────────────────────────────────────────────────
import dgram from "node:dgram";
import {
  decode,
  decodeChannel,
  encodeChannel,
  encodePacket,
  geometry_msgs,
  std_msgs,
} from "@dimos/msgs";

const MCAST = Deno.env.get("DIMOS_LCM_HOST") ?? "239.255.76.67";
const LPORT = Number(Deno.env.get("DIMOS_LCM_PORT") ?? 7667);
const WS_PORT = Number(Deno.env.get("GATEWAY_PORT") ?? 8090);
const MAX_LIN = Number(Deno.env.get("TELEOP_MAX_LIN") ?? 1.0); // m/s clamp
const MAX_ANG = Number(Deno.env.get("TELEOP_MAX_ANG") ?? 1.5); // rad/s clamp
const DEFAULT_TTL = Number(Deno.env.get("TELEOP_TTL_MS") ?? 400); // deadman

const MAGIC_SHORT = 0x4c433032; // "LC02" single packet
const MAGIC_LONG = 0x4c433033; // "LC03" fragment
const FRAG_HDR = 20;
const td = new TextDecoder();

const clamp = (v: number, lo: number, hi: number) => (v < lo ? lo : v > hi ? hi : v);

// ── per-client state (attached to ws.data) ─────────────────────────────────
interface ClientState {
  id: number;
  subs: Set<string>; // subscribed topics; "*" = all
  rate: Map<string, number>; // topic -> maxHz (0/absent = unlimited)
  lastSent: Map<string, number>; // topic -> last forward time (ms)
  teleopTimer: ReturnType<typeof setTimeout> | null;
  decode?: string; // "server-json" → gateway decodes + sends JSON (the decode-location axis)
  rx: number;
  tx: number;
}
let nextId = 1;
// Deno's WebSocket has no per-connection data slot (Bun's ws.data), so client
// state lives in this Map keyed on the socket.
const clients = new Map<WebSocket, ClientState>();

// ── SSE + HTTP-poll delivery: the SAME bus tap + frame over other transports, so
//    the data-path benchmark can compare WebSocket vs SSE vs HTTP long-poll fairly.
const enc = new TextEncoder();
function toB64(u8: Uint8Array): string {
  let s = "";
  const CH = 0x8000; // chunk so String.fromCharCode stays within arg limits (large frames)
  for (let i = 0; i < u8.length; i += CH) s += String.fromCharCode(...u8.subarray(i, i + CH));
  return btoa(s);
}
const wants = (subs: Set<string>, topic: string) => subs.has("*") || subs.has(topic);

interface SseClient {
  ctrl: ReadableStreamDefaultController<Uint8Array>;
  subs: Set<string>;
}
const sseClients = new Set<SseClient>();

// Ring buffer of recent frames for HTTP long-poll (cursor = monotonic id; a client
// that falls behind the ring sees a seq gap = loss, rather than unbounded buffering).
interface RingEntry {
  id: number;
  topic: string;
  frame: Uint8Array;
}
const ring: RingEntry[] = [];
let ringSeq = 0;
const RING_MAX = Number(Deno.env.get("POLL_RING_MAX") ?? 8192);
const POLL_WAIT_MS = Number(Deno.env.get("POLL_WAIT_MS") ?? 10000);
const pollWaiters = new Set<() => void>();

// ── discovery: every topic ever seen on the bus → its message type ──────────
const topics = new Map<string, string>();
let rxPackets = 0,
  reassembled = 0,
  teleopPubs = 0;

// ── LCM fragment reassembly (keyed by sequence number) → synthetic LC02 ─────
type Assembly = { buf: Uint8Array; channel: string; got: Set<number>; total: number };
const pending = new Map<number, Assembly>();

function reassemble(bytes: Uint8Array): Uint8Array | null {
  const dv = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  if (bytes.length < 8) return null;
  const magic = dv.getUint32(0, false);
  if (magic === MAGIC_SHORT) return bytes; // already a single packet
  if (magic !== MAGIC_LONG) return null;

  const seqno = dv.getUint32(4, false);
  const msgSize = dv.getUint32(8, false);
  const fragOffset = dv.getUint32(12, false);
  const fragNo = dv.getUint16(16, false);
  const fragsInMsg = dv.getUint16(18, false);

  let a = pending.get(seqno);
  if (!a) {
    a = { buf: new Uint8Array(msgSize), channel: "", got: new Set(), total: fragsInMsg };
    pending.set(seqno, a);
  }
  let dataStart = FRAG_HDR;
  if (fragNo === 0) {
    let end = FRAG_HDR;
    while (end < bytes.length && bytes[end] !== 0) end++;
    a.channel = td.decode(bytes.subarray(FRAG_HDR, end));
    dataStart = end + 1;
  }
  a.buf.set(bytes.subarray(dataStart), fragOffset);
  a.got.add(fragNo);
  if (a.got.size === a.total && a.channel) {
    pending.delete(seqno);
    reassembled++;
    return encodeChannel(a.channel, a.buf);
  }
  if (pending.size > 256) pending.clear(); // leak guard
  return null;
}

/** Parse the channel ("<topic>#<pkg>.<Type>") out of an LC02 packet. */
function parseChannel(pkt: Uint8Array): { topic: string; type: string } | null {
  if (pkt.length < 8) return null;
  let end = 8;
  while (end < pkt.length && pkt[end] !== 0) end++;
  if (end >= pkt.length) return null;
  const channel = td.decode(pkt.subarray(8, end));
  const h = channel.indexOf("#");
  return h >= 0
    ? { topic: channel.slice(0, h), type: channel.slice(h + 1) }
    : { topic: channel, type: "?" };
}

// ── LCM → browser ───────────────────────────────────────────────────────────
const udp = dgram.createSocket({ type: "udp4", reuseAddr: true });
udp.on("message", (buf) => {
  rxPackets++;
  const pkt = reassemble(new Uint8Array(buf));
  if (!pkt) return;
  const meta = parseChannel(pkt);
  if (!meta) return;

  if (!topics.has(meta.topic)) {
    topics.set(meta.topic, meta.type);
    broadcastJSON({ op: "topic", topic: meta.topic, type: meta.type }); // live discovery
  }
  const now = Date.now();
  const frame = new Uint8Array(8 + pkt.length); // [f64 BE gateway-send-ms][LC02]
  new DataView(frame.buffer).setFloat64(0, now, false);
  frame.set(pkt, 8);
  let jsonMsg: string | null = null; // server-json: decode once, shared across all json clients
  for (const [ws, st] of clients) {
    if (ws.readyState !== WebSocket.OPEN) continue;
    if (!(st.subs.has("*") || st.subs.has(meta.topic))) continue; // on-demand filter
    const hz = st.rate.get(meta.topic) ?? 0;
    if (hz > 0) {
      const last = st.lastSent.get(meta.topic) ?? 0;
      if (now - last < 1000 / hz) continue; // downsample
      st.lastSent.set(meta.topic, now);
    }
    if (st.decode === "server-json") {
      // rosbridge-style: deserialize here and ship JSON (bigger on the wire, gateway pays the CPU).
      if (jsonMsg === null) {
        try {
          const { payload } = decodeChannel(pkt);
          jsonMsg = JSON.stringify({
            op: "sample",
            topic: meta.topic,
            type: meta.type,
            gatewaySendMs: now,
            data: decode(payload),
          });
        } catch {
          jsonMsg = ""; // undecodable → skip for json clients
        }
      }
      if (jsonMsg) {
        ws.send(jsonMsg);
        st.tx++;
      }
    } else {
      ws.send(frame);
      st.tx++;
    }
  }

  // SSE fan-out — same frame, base64-wrapped in a text event (SSE can't carry binary).
  if (sseClients.size) {
    const line = enc.encode(`data: ${toB64(frame)}\n\n`);
    for (const c of sseClients) {
      if (!wants(c.subs, meta.topic)) continue;
      try {
        c.ctrl.enqueue(line);
      } catch {
        sseClients.delete(c);
      }
    }
  }

  // HTTP-poll ring buffer (+ wake any long-poll waiters).
  ring.push({ id: ++ringSeq, topic: meta.topic, frame });
  if (ring.length > RING_MAX) ring.splice(0, ring.length - RING_MAX);
  if (pollWaiters.size) {
    const waiters = [...pollWaiters];
    pollWaiters.clear();
    for (const w of waiters) w();
  }
});
udp.on("error", (e) => console.error("[lcm] socket error:", e));
udp.bind(LPORT, () => {
  try {
    udp.addMembership(MCAST);
  } catch (e) {
    console.error("[lcm] addMembership failed (multicast route?):", e);
  }
  udp.setMulticastLoopback(true);
  udp.setMulticastTTL(1);
  console.log(`[lcm] listening on ${MCAST}:${LPORT}`);
});

// ── teleop: structured + safe (never raw bytes from the browser) ────────────
function publishTwist(linX: number, angZ: number) {
  const t = new geometry_msgs.Twist({
    linear: new geometry_msgs.Vector3({ x: clamp(linX, -MAX_LIN, MAX_LIN), y: 0, z: 0 }),
    angular: new geometry_msgs.Vector3({ x: 0, y: 0, z: clamp(angZ, -MAX_ANG, MAX_ANG) }),
  });
  udp.send(encodePacket("/cmd_vel#geometry_msgs.Twist", t), LPORT, MCAST);
  teleopPubs++;
}
function armDeadman(st: ClientState, ttlMs: number) {
  if (st.teleopTimer) clearTimeout(st.teleopTimer);
  st.teleopTimer = setTimeout(() => publishTwist(0, 0), ttlMs); // stop if no cmd within ttl
}

// ── nav goal: publish a PointStamped to clicked_point (what the nav stack consumes) ──
function publishGoal(x: number, y: number, z: number) {
  const ps = new geometry_msgs.PointStamped({
    header: new std_msgs.Header({ frame_id: "world" }),
    point: new geometry_msgs.Point({ x, y, z }),
  });
  udp.send(encodePacket("/clicked_point#geometry_msgs.PointStamped", ps), LPORT, MCAST);
}

// ── control channel + fan-out ───────────────────────────────────────────────
function broadcastJSON(obj: unknown) {
  const s = JSON.stringify(obj);
  for (const ws of clients.keys()) if (ws.readyState === WebSocket.OPEN) ws.send(s);
}
function topicList() {
  return [...topics].map(([topic, type]) => ({ topic, type }));
}
const subsFromQuery = (tp: string | null) =>
  new Set(tp == null || tp === "*" ? ["*"] : tp.split(",").filter(Boolean));

// SSE: stream `data: <base64(frame)>` for the requested topics (?topics=csv|*).
function handleSse(req: Request): Response {
  const subs = subsFromQuery(new URL(req.url).searchParams.get("topics"));
  let client: SseClient | undefined;
  const stream = new ReadableStream<Uint8Array>({
    start(ctrl) {
      client = { ctrl, subs };
      sseClients.add(client);
      ctrl.enqueue(
        enc.encode(
          `event: hello\ndata: ${
            JSON.stringify({ topics: topicList(), label: "Deno↔LCM/SSE" })
          }\n\n`,
        ),
      );
    },
    cancel() {
      if (client) sseClients.delete(client);
    },
  });
  return new Response(stream, {
    headers: {
      "content-type": "text/event-stream",
      "cache-control": "no-cache",
      "access-control-allow-origin": "*",
    },
  });
}

// HTTP long-poll: return a binary batch [u32 cursor][u32 count] then count×([u32 len][frame]);
// hold the request until matching frames arrive (or POLL_WAIT_MS) when the ring has nothing new.
async function handlePoll(req: Request): Promise<Response> {
  const u = new URL(req.url);
  const since = Number(u.searchParams.get("since") ?? 0);
  const subs = subsFromQuery(u.searchParams.get("topics"));
  const max = Number(u.searchParams.get("max") ?? 512);
  // since < 0 means "start from head": skip history so a live client only gets new
  // frames (matching WS/SSE, which never replay) — return just the current cursor.
  if (since < 0) {
    const head = ring.length ? ring[ring.length - 1].id : 0;
    const b = new Uint8Array(8);
    new DataView(b.buffer).setUint32(0, head, false);
    return new Response(b, {
      headers: { "content-type": "application/octet-stream", "access-control-allow-origin": "*" },
    });
  }
  const collect = () => {
    const out: RingEntry[] = [];
    for (const e of ring) {
      if (e.id <= since || !wants(subs, e.topic)) continue;
      out.push(e);
      if (out.length >= max) break;
    }
    return out;
  };
  let batch = collect();
  if (batch.length === 0) {
    await new Promise<void>((res) => {
      const w = () => {
        clearTimeout(to);
        res();
      };
      const to = setTimeout(() => {
        pollWaiters.delete(w);
        res();
      }, POLL_WAIT_MS);
      pollWaiters.add(w);
    });
    batch = collect();
  }
  const newCursor = batch.length ? batch[batch.length - 1].id : since;
  let total = 8;
  for (const e of batch) total += 4 + e.frame.length;
  const body = new Uint8Array(total);
  const dv = new DataView(body.buffer);
  dv.setUint32(0, newCursor, false);
  dv.setUint32(4, batch.length, false);
  let off = 8;
  for (const e of batch) {
    dv.setUint32(off, e.frame.length, false);
    off += 4;
    body.set(e.frame, off);
    off += e.frame.length;
  }
  return new Response(body, {
    headers: { "content-type": "application/octet-stream", "access-control-allow-origin": "*" },
  });
}

Deno.serve({ port: WS_PORT }, (req) => {
  if (req.headers.get("upgrade")?.toLowerCase() !== "websocket") {
    const path = new URL(req.url).pathname;
    if (path === "/sse") return handleSse(req);
    if (path === "/poll") return handlePoll(req);
    return new Response("dimos-web-gateway up");
  }
  const { socket: ws, response } = Deno.upgradeWebSocket(req);
  const st: ClientState = {
    id: nextId++,
    subs: new Set<string>(),
    rate: new Map(),
    lastSent: new Map(),
    teleopTimer: null,
    rx: 0,
    tx: 0,
  };

  ws.onopen = () => {
    clients.set(ws, st);
    ws.send(JSON.stringify({ op: "hello", topics: topicList(), label: "Deno↔LCM" }));
    console.log(`[ws] +client#${st.id} (${clients.size} total)`);
  };
  ws.onclose = () => {
    if (st.teleopTimer) clearTimeout(st.teleopTimer);
    publishTwist(0, 0); // safety: stop the robot when an operator disconnects
    clients.delete(ws);
    console.log(`[ws] -client#${st.id} (${clients.size} total)`);
  };
  ws.onerror = (e) => console.error(`[ws] client#${st.id} error:`, (e as ErrorEvent).message ?? e);
  ws.onmessage = (ev) => {
    st.rx++;
    const raw = ev.data;
    if (typeof raw !== "string") return; // browser never sends raw bus bytes
    let m: any;
    try {
      m = JSON.parse(raw);
    } catch {
      return;
    }
    switch (m.op) {
      case "subscribe":
        st.subs.add(m.topic);
        if (m.maxHz) st.rate.set(m.topic, m.maxHz);
        if (m.decode) st.decode = m.decode;
        break;
      case "unsubscribe":
        st.subs.delete(m.topic);
        st.rate.delete(m.topic);
        st.lastSent.delete(m.topic);
        break;
      case "rate":
        st.rate.set(m.topic, m.maxHz || 0);
        break;
      case "list":
        ws.send(JSON.stringify({ op: "topics", topics: topicList() }));
        break;
      case "teleop":
        publishTwist(Number(m.linearX) || 0, Number(m.angularZ) || 0);
        armDeadman(st, Number(m.ttlMs) || DEFAULT_TTL);
        break;
      case "stop":
        if (st.teleopTimer) clearTimeout(st.teleopTimer);
        publishTwist(0, 0);
        break;
      case "goal":
        publishGoal(Number(m.x) || 0, Number(m.y) || 0, Number(m.z) || 0);
        break;
    }
  };
  return response;
});

console.log(`[gateway] ws://localhost:${WS_PORT}  ·  LCM ${MCAST}:${LPORT}`);
setInterval(() => {
  console.log(
    `[stats] rxPackets=${rxPackets} reassembled=${reassembled} topics=${topics.size} clients=${clients.size} teleopPubs=${teleopPubs}`,
  );
}, 3000);

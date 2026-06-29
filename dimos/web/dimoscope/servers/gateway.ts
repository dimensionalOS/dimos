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
// Why Bun: the gateway is a byte-relay (browser decodes), so it's I/O-bound;
// Bun gives native WebSocket + node:dgram + no GIL headroom for media streams.
// Bun cannot speak Zenoh natively — the Zenoh path is gateway-zenoh.py.
//
// RUN:  bun run servers/gateway.ts        (env: GATEWAY_PORT, DIMOS_LCM_HOST/PORT)
// ─────────────────────────────────────────────────────────────────────────
import dgram from "node:dgram";
import { encodeChannel, encodePacket, geometry_msgs, std_msgs } from "@dimos/msgs";

const MCAST = process.env.DIMOS_LCM_HOST ?? "239.255.76.67";
const LPORT = Number(process.env.DIMOS_LCM_PORT ?? 7667);
const WS_PORT = Number(process.env.GATEWAY_PORT ?? 8090);
const MAX_LIN = Number(process.env.TELEOP_MAX_LIN ?? 1.0); // m/s clamp
const MAX_ANG = Number(process.env.TELEOP_MAX_ANG ?? 1.5); // rad/s clamp
const DEFAULT_TTL = Number(process.env.TELEOP_TTL_MS ?? 400); // deadman

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
  rx: number;
  tx: number;
}
let nextId = 1;
const clients = new Set<Bun.ServerWebSocket<ClientState>>();

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
  for (const ws of clients) {
    const st = ws.data;
    if (!(st.subs.has("*") || st.subs.has(meta.topic))) continue; // on-demand filter
    const hz = st.rate.get(meta.topic) ?? 0;
    if (hz > 0) {
      const last = st.lastSent.get(meta.topic) ?? 0;
      if (now - last < 1000 / hz) continue; // downsample
      st.lastSent.set(meta.topic, now);
    }
    ws.send(frame);
    st.tx++;
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
  for (const ws of clients) ws.send(s);
}
function topicList() {
  return [...topics].map(([topic, type]) => ({ topic, type }));
}

Bun.serve<ClientState>({
  port: WS_PORT,
  fetch(req, server) {
    if (
      server.upgrade(req, {
        data: {
          id: nextId++,
          subs: new Set<string>(),
          rate: new Map(),
          lastSent: new Map(),
          teleopTimer: null,
          rx: 0,
          tx: 0,
        },
      })
    )
      return;
    return new Response("dimos-web-gateway up");
  },
  websocket: {
    open(ws) {
      clients.add(ws);
      ws.send(JSON.stringify({ op: "hello", topics: topicList(), label: "Bun↔LCM" }));
      console.log(`[ws] +client#${ws.data.id} (${clients.size} total)`);
    },
    close(ws) {
      const st = ws.data;
      if (st.teleopTimer) clearTimeout(st.teleopTimer);
      publishTwist(0, 0); // safety: stop the robot when an operator disconnects
      clients.delete(ws);
      console.log(`[ws] -client#${st.id} (${clients.size} total)`);
    },
    message(ws, raw) {
      const st = ws.data;
      st.rx++;
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
    },
  },
});

console.log(`[gateway] ws://localhost:${WS_PORT}  ·  LCM ${MCAST}:${LPORT}`);
setInterval(() => {
  console.log(
    `[stats] rxPackets=${rxPackets} reassembled=${reassembled} topics=${topics.size} clients=${clients.size} teleopPubs=${teleopPubs}`,
  );
}, 3000);

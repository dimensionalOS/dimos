// ─────────────────────────────────────────────────────────────────────────
// dimoscope BRIDGE  (Rung 1 + fragment reassembly)
//
// Relay between the dimos LCM bus (UDP multicast) and the browser (WebSocket).
// The browser can't speak UDP multicast, so this is the only piece on the bus.
//
// UDP TRANSPORT: the dimos example uses `@dimos/lcm` (Deno-only), but the bridge
// only receives/sends UDP — Deno's `node:dgram` does multicast UDP. Message
// framing/decoding happens in `@dimos/msgs`.
//
// LCM FRAGMENTATION: dimos fragments any LCM message bigger than ~1.4KB (the UDP
// MTU) into multiple "LC03" datagrams. `@dimos/msgs.decodePacket` only handles
// single "LC02" packets, so we REASSEMBLE fragments here and forward a synthetic
// LC02 packet — the browser stays unchanged. (Small messages like /odom are LC02
// and pass straight through.)
//
// RUN:  deno run -A bridge.ts
// ─────────────────────────────────────────────────────────────────────────
import dgram from "node:dgram";
import { encodeChannel } from "@dimos/msgs";

const MCAST = "239.255.76.67";
const LPORT = 7667;
const WS_PORT = 8080;

const MAGIC_SHORT = 0x4c433032; // "LC02" single packet
const MAGIC_LONG = 0x4c433033; // "LC03" fragment
const FRAG_HDR = 20; // LCM fragment header size

const clients = new Set<WebSocket>();
let rx = 0, tx = 0, reassembled = 0;

// ── Fragment reassembly: key by LCM sequence number ─────────────────────────
type Assembly = { buf: Uint8Array; channel: string; got: Set<number>; total: number };
const pending = new Map<number, Assembly>();
const td = new TextDecoder();

/** Returns a forwardable single (LC02) packet, or null if more fragments needed. */
function process(bytes: Uint8Array): Uint8Array | null {
  const dv = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  if (bytes.length < 8) return null;
  const magic = dv.getUint32(0, false);
  if (magic === MAGIC_SHORT) return bytes; // already a single packet
  if (magic !== MAGIC_LONG) return null; // unknown

  const seqno = dv.getUint32(4, false);
  const msgSize = dv.getUint32(8, false); // total payload size (excludes channel)
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
    // fragment 0 carries the channel name (null-terminated) before its payload
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
    return encodeChannel(a.channel, a.buf); // synthetic LC02 packet
  }
  if (pending.size > 256) pending.clear(); // crude leak guard
  return null;
}

// ── LCM side: UDP multicast socket ──────────────────────────────────────────
const udp = dgram.createSocket({ type: "udp4", reuseAddr: true });
udp.on("message", (buf) => {
  rx++;
  const out = process(new Uint8Array(buf));
  if (out) { for (const ws of clients) if (ws.readyState === WebSocket.OPEN) ws.send(out); }
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

// ── Browser side: WebSocket server (Deno native, zero deps) ─────────────────
Deno.serve({ port: WS_PORT }, (req) => {
  if (req.headers.get("upgrade")?.toLowerCase() !== "websocket") {
    return new Response("dimoscope bridge is up");
  }
  const { socket: ws, response } = Deno.upgradeWebSocket(req);
  ws.binaryType = "arraybuffer";
  ws.onopen = () => {
    clients.add(ws);
    console.log(`[ws] +client (${clients.size} total)`);
  };
  ws.onclose = () => {
    clients.delete(ws);
    console.log(`[ws] -client (${clients.size} total)`);
  };
  ws.onmessage = (ev) => {
    const msg = ev.data;
    const buf = typeof msg === "string"
      ? new TextEncoder().encode(msg)
      : new Uint8Array(msg as ArrayBuffer);
    udp.send(buf, LPORT, MCAST); // browser publish (e.g. teleop) → bus
    tx++;
  };
  return response;
});

console.log(`[ws] serving ws://localhost:${WS_PORT}`);
setInterval(
  () => console.log(`[stats] rx=${rx} reassembled=${reassembled} tx=${tx} clients=${clients.size}`),
  2000,
);

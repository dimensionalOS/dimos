// netsim-udp.ts — the UDP sibling of netsim.ts, for the QUIC/WebTransport mechanism.
//
// netsim.ts is a TCP proxy: it can shape the WS-family mechanisms but it CANNOT carry QUIC (UDP) and
// CANNOT inject real loss (on TCP, loss just becomes retransmit→latency). This relays raw UDP datagrams
// between a single QUIC client (the probe or a browser) and the dimoscope service's QUIC (the gateway),
// and drops a configurable fraction of packets — REAL loss — plus latency/jitter. That's the lever that
// shows WebTransport's win: lost datagrams just don't arrive (no head-of-line blocking), so delivered-
// datagram latency stays flat while a TCP stream would stall. No sudo, no dummynet, cross-platform.
//
//   client ──UDP──► :LISTEN [netsim-udp] :ephemeral ──UDP──► :TARGET (aioquic)   (+ reverse)
//
// Two sockets: `front` faces the client (bound to LISTEN), `back` faces the server (ephemeral). Each
// only ever hears one peer, so no addr disambiguation is needed. QUIC sees stable relay addresses on
// both sides → no connection-migration trip-ups (single client). The server cert SAN is `localhost`,
// independent of port, so a browser pointed at https://localhost:LISTEN still validates.
//
// Needs --unstable-net (Deno.listenDatagram UDP). Profiles mirror netsim.ts + a `plr` (packet-loss rate).
//   NETSIM_PROFILE=lossy NETSIM_UDP_LISTEN=8193 NETSIM_UDP_TARGET=localhost:8093 \
//     deno run -A --unstable-net bench/netsim-udp.ts
const LISTEN = Number(Deno.env.get("NETSIM_UDP_LISTEN") ?? 8193);
const [THOST, TPORT] = (Deno.env.get("NETSIM_UDP_TARGET") ?? "localhost:8443").split(":");

// latency ms, jitter ±ms, plr = packet-loss rate (0..1). Rough but representative.
interface Profile {
  l: number;
  j: number;
  plr: number;
}
const PROFILES: Record<string, Profile> = {
  lan: { l: 0, j: 0, plr: 0 },
  wifi: { l: 8, j: 3, plr: 0 },
  "4g": { l: 50, j: 15, plr: 0.005 },
  "3g": { l: 150, j: 40, plr: 0.01 },
  "2g": { l: 400, j: 120, plr: 0.03 },
  lossy: { l: 80, j: 80, plr: 0.1 }, // the payoff profile: 10% real packet loss
};
const name = Deno.env.get("NETSIM_PROFILE") ?? "lossy";
const p = PROFILES[name] ?? PROFILES.lossy;
const LAT = Number(Deno.env.get("NETSIM_LATENCY_MS") ?? p.l);
const JIT = Number(Deno.env.get("NETSIM_JITTER_MS") ?? p.j);
const PLR = Number(Deno.env.get("NETSIM_PLR") ?? p.plr);

const jitter = () => (JIT ? (Math.random() * 2 - 1) * JIT : 0);
const target: Deno.NetAddr = { transport: "udp", hostname: THOST, port: Number(TPORT) };

const front = Deno.listenDatagram({ port: LISTEN, transport: "udp", hostname: "127.0.0.1" });
const back = Deno.listenDatagram({ port: 0, transport: "udp", hostname: "127.0.0.1" });
let lastClient: Deno.Addr | undefined;
let dropped = 0;
let passed = 0;

// Forward one datagram after the link delay, dropping it with probability PLR. Fire-and-forget (not
// awaited) so independent packets can reorder under jitter — exactly like a real lossy UDP path.
function forward(sock: Deno.DatagramConn, data: Uint8Array, addr: Deno.Addr) {
  if (Math.random() < PLR) {
    dropped++;
    return;
  }
  passed++;
  const wait = Math.max(0, LAT + jitter());
  const send = () => sock.send(data, addr).catch(() => {});
  if (wait > 0) setTimeout(send, wait);
  else send();
}

async function pumpClientToServer() {
  for await (const [data, addr] of front) {
    lastClient = addr; // remember who to send replies back to
    forward(back, data, target);
  }
}
async function pumpServerToClient() {
  for await (const [data, _addr] of back) {
    if (lastClient) forward(front, data, lastClient);
  }
}

console.log(
  `[netsim-udp] :${LISTEN} → ${THOST}:${TPORT}  profile=${name}  latency=${LAT}ms jitter=±${JIT}ms ` +
    `loss=${(PLR * 100).toFixed(1)}%`,
);
// periodic loss report so the bench log shows real drops happening
setInterval(() => {
  const tot = passed + dropped;
  if (tot > 0) {
    console.log(
      `[netsim-udp] forwarded=${passed} dropped=${dropped} (${
        ((dropped / tot) * 100).toFixed(1)
      }%)`,
    );
  }
}, 2000);

await Promise.all([pumpClientToServer(), pumpServerToClient()]);

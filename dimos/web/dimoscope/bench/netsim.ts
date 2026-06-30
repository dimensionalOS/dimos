// netsim.ts — a zero-dependency TCP impairment proxy for the data-path benchmark.
//
// Sits between the client and the gateway/bridge and shapes the byte stream with
// added latency, jitter and a bandwidth cap, per a named cellular profile. One TCP
// proxy impairs ALL the TCP mechanisms uniformly (WebSocket, SSE, HTTP long-poll,
// zenoh-ts-over-WS), so they degrade under the *same* conditions — the whole point.
//
// Why this and not DevTools throttling: DevTools doesn't touch WebSocket/WebRTC.
// Why TCP-level latency/jitter/bw and not "loss %": on a reliable TCP stream, loss
// just becomes retransmit → latency/stall, so latency+jitter+bw (+blackout) is the
// honest, meaningful lever — and the very reason the UDP/QUIC mechanisms (WebRTC,
// WebTransport, Phase 2) are worth comparing. The industry-standard alternative is
// Shopify's toxiproxy (TCP, scriptable) or macOS dummynet for UDP/QUIC — see docs.
//
//   NETSIM_PROFILE=3g NETSIM_LISTEN=8099 NETSIM_TARGET=localhost:8090 \
//     deno run -A bench/netsim.ts
const LISTEN = Number(Deno.env.get("NETSIM_LISTEN") ?? 8099);
const [THOST, TPORT] = (Deno.env.get("NETSIM_TARGET") ?? "localhost:8090").split(":");

// latency ms, jitter ±ms, bandwidth kbps (0 = unlimited). Rough but representative.
interface Profile {
  l: number;
  j: number;
  bw: number;
}
const PROFILES: Record<string, Profile> = {
  lan: { l: 0, j: 0, bw: 0 },
  wifi: { l: 8, j: 3, bw: 0 },
  "4g": { l: 50, j: 15, bw: 12000 },
  "3g": { l: 150, j: 40, bw: 2000 },
  "2g": { l: 400, j: 120, bw: 280 },
  lossy: { l: 80, j: 80, bw: 5000 }, // high jitter stands in for loss→retransmit on TCP
};
const name = Deno.env.get("NETSIM_PROFILE") ?? "lan";
const p = PROFILES[name] ?? PROFILES.lan;
const LAT = Number(Deno.env.get("NETSIM_LATENCY_MS") ?? p.l);
const JIT = Number(Deno.env.get("NETSIM_JITTER_MS") ?? p.j);
const BW = Number(Deno.env.get("NETSIM_BW_KBPS") ?? p.bw); // kbps, 0 = unlimited

const sleep = (ms: number) => new Promise((r) => setTimeout(r, ms));
const jitter = () => (JIT ? (Math.random() * 2 - 1) * JIT : 0);

// Shape one direction: read a chunk, charge it bandwidth time (serialise per stream),
// then deliver after one-way latency+jitter. nextFree models the link being busy.
async function shape(src: Deno.Conn, dst: Deno.Conn) {
  const buf = new Uint8Array(16 * 1024);
  let nextFree = performance.now();
  try {
    for (;;) {
      const n = await src.read(buf);
      if (n === null) break;
      const chunk = buf.slice(0, n);
      const now = performance.now();
      const start = Math.max(now, nextFree);
      const txMs = BW > 0 ? (n * 8) / BW : 0; // ms to clock `n` bytes out at BW kbps
      nextFree = start + txMs;
      const wait = start + txMs + LAT + jitter() - now; // arrival time relative to now
      if (wait > 0) await sleep(wait);
      let off = 0;
      while (off < chunk.length) off += await dst.write(chunk.subarray(off));
    }
  } catch {
    // peer closed / reset
  }
  try {
    dst.close();
  } catch {
    // already closed
  }
}

const listener = Deno.listen({ port: LISTEN });
console.log(
  `[netsim] :${LISTEN} → ${THOST}:${TPORT}  profile=${name}  latency=${LAT}ms jitter=±${JIT}ms bw=${
    BW || "∞"
  }kbps`,
);
for await (const client of listener) {
  (async () => {
    let upstream: Deno.Conn;
    try {
      upstream = await Deno.connect({ hostname: THOST, port: Number(TPORT) });
    } catch {
      try {
        client.close();
      } catch {
        // ignore
      }
      return;
    }
    shape(client, upstream); // client → gateway
    shape(upstream, client); // gateway → client (the data stream that matters)
  })();
}

// netsim.ts — a zero-dependency TCP impairment proxy for the data-path benchmark.
//
// Sits between the client and the gateway/bridge and shapes the byte stream with
// added latency, jitter and a bandwidth cap, per a named cellular profile. One TCP
// proxy impairs ALL the TCP mechanisms uniformly (WebSocket, SSE, HTTP long-poll,
// zenoh-ts-over-WS), so they degrade under the *same* conditions — the whole point.
// Runtime-agnostic: the SAME proxy shapes the headless Deno bench AND a real browser
// (point the app at it, e.g. `…/?gw=localhost:8099`), since it's just TCP under the WS.
//
// Off-the-shelf alternatives (we deliberately don't use one — see below):
//   • Toxiproxy (Shopify) — the direct equivalent: userspace TCP proxy with latency/
//     jitter/bandwidth/packet_loss/slicer toxics, HTTP API + CLI + Node client, no root.
//   • dummynet (macOS, via dnctl+pf) / tc-netem (Linux) / Network Link Conditioner
//     (macOS GUI) / comcast (CLI wrapper) — kernel-level; the ONLY way to impair
//     UDP/WebRTC, system-wide (so they also shape the browser's WebRTC media plane).
//   • mahimahi (MIT) — research-grade reproducible web measurement w/ time-varying traces.
//
// Why hand-rolled anyway: zero-install, no daemon, no root — one ~90-line Deno file that
// runs in CI/dev with nothing extra; transport- AND runtime-agnostic (above); and
// Toxiproxy is *also* TCP-only, so it'd buy nothing for the WebRTC bench (that's what
// dummynet / Network Link Conditioner are for). TCP latency/jitter/bw is the honest lever:
// on a reliable stream loss just becomes retransmit → latency/stall (which is also why the
// UDP/QUIC mechanisms — WebRTC, WebTransport, Phase 2 — are worth comparing).
//
// Note on browser DevTools: it CAN throttle WebSockets (Chrome 99+) and WebRTC (124+), but
// only *manually* — the CDP automation API (Network.emulateNetworkConditions, what
// Playwright/Puppeteer drive) still skips WS/WebRTC, so this proxy stays the reproducible,
// automatable, CDP-independent lever.
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

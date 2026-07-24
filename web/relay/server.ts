// The DimOS relay: QUIC/WebTransport listener (robot + viewer sessions) plus
// a plain-HTTP side (static files, /api/info, /api/stats). Payload-blind:
// all forwarding decisions come from frame headers (see forward.ts).
//
// Leg asymmetry, forced by upstream bugs (see web/README.md):
// - Robot (aioquic): control = datagrams; data = one-shot bidi streams the
//   relay never writes on (send half aborted with RESET, never FIN).
// - Viewer (browser): control = viewer-opened bidi stream; data = relay-
//   opened uni streams.
import {
  ControlFrameReader,
  decodeDatagram,
  encodeControlFrame,
  encodeDatagram,
  type Msg,
  PROTOCOL_VERSION,
} from "@dimos/shared";
import { fileURLToPath } from "node:url";
import { makeEphemeralCert } from "./cert.ts";
import {
  Forwarder,
  readDataFrameBytes,
  readWebTransportPreamble,
  type ViewerSink,
} from "./forward.ts";

export interface RelayOptions {
  /** TCP port for the HTTP side. Default 7780; 0 picks an ephemeral port. */
  port?: number;
  /** Bind host for both listeners. The default is the only secure-context-friendly choice. */
  host?: string;
  /** Directory served over HTTP. Defaults to ./static next to this module. */
  staticDir?: string;
}

export interface RelayHandle {
  httpPort: number;
  quicPort: number;
  /** Base WebTransport URL (no path); clients append /robot or /viewer. */
  wtUrl: string;
  certHash: string;
  shutdown(): Promise<void>;
}

const MIME: Record<string, string> = {
  ".html": "text/html; charset=utf-8",
  ".js": "application/javascript",
  ".css": "text/css",
  ".json": "application/json",
  ".svg": "image/svg+xml",
  ".png": "image/png",
};

export function installUnhandledRejectionGuard(): void {
  // deno#28406: WT sessions leak unhandled rejections on disconnect/idle
  // timeout; without this guard the relay dies ~30 s after a tab closes.
  if ((globalThis as { __dimosRejectionGuard?: boolean }).__dimosRejectionGuard) return;
  (globalThis as { __dimosRejectionGuard?: boolean }).__dimosRejectionGuard = true;
  globalThis.addEventListener("unhandledrejection", (e) => {
    console.log("[relay] unhandled rejection (ignored):", (e.reason as Error)?.message ?? e.reason);
    e.preventDefault();
  });
}

export async function startRelay(options: RelayOptions = {}): Promise<RelayHandle> {
  installUnhandledRejectionGuard();
  const host = options.host ?? "127.0.0.1";
  const cert = await makeEphemeralCert();

  // QUIC always binds an ephemeral port; clients discover it via the ready
  // line or /api/info, so --port stays a single HTTP-facing knob.
  const endpoint = new Deno.QuicEndpoint({ hostname: host, port: 0 });
  const listener = endpoint.listen({
    cert: cert.certPem,
    key: cert.keyPem,
    alpnProtocols: ["h3"],
    maxIdleTimeout: 30_000,
    keepAliveInterval: 4_000,
  });
  const quicPort = endpoint.addr.port;
  // 127.0.0.1 rather than localhost: Chrome resolves localhost to ::1 first
  // and the endpoint binds IPv4. Hash pinning replaces hostname verification.
  const urlHost = host === "0.0.0.0" ? "127.0.0.1" : host;
  const wtUrl = `https://${urlHost}:${quicPort}`;

  const forwarder = new Forwarder();
  const sessions = new Set<WebTransport>();
  let robot: WebTransport | null = null;

  function track(wt: WebTransport): void {
    sessions.add(wt);
    wt.closed.catch(() => {}).finally(() => sessions.delete(wt));
  }

  function sendControl(writer: WritableStreamDefaultWriter<Uint8Array>, msg: Msg): void {
    writer.write(encodeControlFrame(msg)).catch(() => {});
  }

  function closeAfterFlush(wt: WebTransport, reason: string): void {
    // Session close discards queued stream/datagram data, so give a just-sent
    // reply (e.g. the version_mismatch error) a moment to reach the wire.
    setTimeout(() => {
      try {
        wt.close({ closeCode: 1, reason });
      } catch {
        // already gone
      }
    }, 250);
  }

  function sendDatagram(writer: WritableStreamDefaultWriter<Uint8Array>, msg: Msg): void {
    writer.write(encodeDatagram(msg)).catch(() => {});
  }

  /** Replies to hello/ping; returns false if the session must close (bad version). */
  function handleControlMsg(msg: Msg, reply: (msg: Msg) => void): boolean {
    if (msg.t === "hello") {
      if (msg.v !== PROTOCOL_VERSION) {
        reply({
          t: "error",
          code: "version_mismatch",
          message: `protocol v${PROTOCOL_VERSION} required, got v${msg.v}`,
        });
        return false;
      }
      reply({ t: "welcome", v: PROTOCOL_VERSION });
    } else if (msg.t === "ping") {
      reply({ t: "pong", n: msg.n, ts: msg.ts });
    }
    return true;
  }

  function handleRobot(wt: WebTransport, conn: Deno.QuicConn): void {
    if (robot) {
      // Takeover: a restarted robot process must reattach without operator help.
      console.log("[relay] robot takeover: closing previous robot session");
      try {
        robot.close({ closeCode: 0, reason: "replaced by new robot" });
      } catch {
        // already gone
      }
    }
    robot = wt;
    console.log("[relay] robot connected");
    wt.closed
      .catch(() => {})
      .finally(() => {
        if (robot === wt) robot = null;
        console.log("[relay] robot disconnected");
      });

    const dgWriter = wt.datagrams.writable.getWriter();
    (async () => {
      // Robot-leg control rides datagrams: aioquic dies if the relay writes
      // on robot-opened bidi streams, so hello/welcome/ping/pong live here.
      for await (const dg of wt.datagrams.readable) {
        const msg = decodeDatagram(dg);
        if (msg === null) continue;
        if (!handleControlMsg(msg, (m) => sendDatagram(dgWriter, m))) {
          closeAfterFlush(wt, "version mismatch");
          return;
        }
      }
    })().catch(() => {});

    (async () => {
      // Data frames arrive on one-shot bidi streams (Deno never delivers
      // incoming uni payloads), accepted at the QUIC level rather than via
      // wt.incomingBidirectionalStreams: a reset racing that iterator's
      // internal preamble read errors it permanently and would silently end
      // this loop, while the raw accept only fails with the connection.
      // Abort our send half: RESET is invisible to aioquic's h3 layer and
      // releases stream credit; a FIN would kill it.
      for await (const bidi of conn.incomingBidirectionalStreams) {
        bidi.writable.abort().catch(() => {});
        (async () => {
          await readWebTransportPreamble(bidi.readable);
          forwarder.onRobotFrame(await readDataFrameBytes(bidi.readable));
        })().catch(() => {
          // reset before/mid-frame (stale latest-wins write): drop the partial
        });
      }
    })().catch((e) => {
      console.log("[relay] robot stream loop ended:", (e as Error)?.message ?? e);
    });
  }

  function handleViewer(wt: WebTransport): void {
    let sendOrder = 1;
    const sink: ViewerSink = {
      async sendFrame(bytes: Uint8Array): Promise<void> {
        // waitUntilAvailable: a slow page exhausts stream credit; without it
        // this throws and we would drop a live viewer. Decreasing sendOrder
        // keeps stream completions FIFO on the wire (quinn round-robins
        // otherwise and frames complete in ~1 s waves).
        const stream = await wt.createUnidirectionalStream({
          waitUntilAvailable: true,
          sendOrder: -(sendOrder++),
        });
        const writer = stream.getWriter();
        await writer.write(bytes);
        await writer.close();
      },
      kick(reason: string): void {
        console.log(`[relay] kicking viewer: ${reason}`);
        try {
          wt.close({ closeCode: 1, reason });
        } catch {
          // already gone
        }
      },
    };
    const handle = forwarder.addViewer(sink);
    console.log(`[relay] viewer ${handle.id} connected (${forwarder.viewerCount} total)`);
    wt.closed
      .catch(() => {})
      .finally(() => {
        forwarder.removeViewer(handle);
        console.log(`[relay] viewer ${handle.id} disconnected`);
      });

    (async () => {
      // Browser-leg control: viewer-opened bidi stream, replies on the same
      // stream. Deno may write on viewer-initiated streams (browsers are not
      // aioquic). wt.incomingBidirectionalStreams is safe here: unlike the
      // robot leg, viewers never reset a stream racing its acceptance.
      for await (const bidi of wt.incomingBidirectionalStreams) {
        (async () => {
          const writer = bidi.writable.getWriter();
          const frames = new ControlFrameReader();
          for await (const chunk of bidi.readable) {
            for (const msg of frames.push(chunk)) {
              if (!handleControlMsg(msg, (m) => sendControl(writer, m))) {
                closeAfterFlush(wt, "version mismatch");
                return;
              }
            }
          }
          writer.releaseLock();
        })().catch((e) =>
          console.log("[relay] viewer control stream ended:", (e as Error)?.message ?? e)
        );
      }
    })().catch(() => {});

    const dgWriter = wt.datagrams.writable.getWriter();
    (async () => {
      // Datagram control for viewers too: browsers use the bidi stream above,
      // but the Python test viewer cannot receive replies on its own bidi
      // streams (aioquic), so hello/ping work over datagrams on both legs.
      // The relay answers pings itself (RTT works with no robot connected);
      // teleop routing arrives in T6.
      for await (const dg of wt.datagrams.readable) {
        const msg = decodeDatagram(dg);
        if (msg === null) continue;
        if (!handleControlMsg(msg, (m) => sendDatagram(dgWriter, m))) {
          closeAfterFlush(wt, "version mismatch");
          return;
        }
      }
    })().catch(() => {});
  }

  (async () => {
    for await (const incoming of listener) {
      (async () => {
        const conn = await incoming.accept();
        const wt = await Deno.upgradeWebTransport(conn);
        await wt.ready;
        track(wt);
        const path = new URL(wt.url).pathname;
        if (path === "/robot") handleRobot(wt, conn);
        else handleViewer(wt);
      })().catch((e) => console.log("[relay] accept failed:", (e as Error)?.message ?? e));
    }
  })().catch(() => {
    // listener stopped (shutdown)
  });

  const staticRoot = options.staticDir
    ? new URL(
      options.staticDir.endsWith("/") ? options.staticDir : options.staticDir + "/",
      `file://${Deno.cwd()}/`,
    )
    : new URL("./static/", import.meta.url);
  // Resolved filesystem prefix every served path must stay under (href ends
  // with "/", so the path does too).
  const staticRootPath = fileURLToPath(staticRoot);

  async function handleHttp(req: Request): Promise<Response> {
    const url = new URL(req.url);
    if (url.pathname === "/api/info") {
      return Response.json({
        wtUrl: `${wtUrl}/viewer`,
        certHash: cert.certHashB64,
        v: PROTOCOL_VERSION,
      });
    }
    if (url.pathname === "/api/stats") {
      return Response.json({ robot: robot !== null, ...(forwarder.stats() as object) });
    }
    const name = url.pathname === "/" ? "debug.html" : url.pathname.slice(1);
    // Resolve the request to a real path and confirm it stays under the static
    // root. A leading "/" or "\" makes `new URL(name, root)` jump to the
    // filesystem root; fileURLToPath additionally throws on encoded slashes.
    let filePath: string;
    try {
      filePath = fileURLToPath(new URL(name, staticRoot));
    } catch {
      return new Response("bad path", { status: 400 });
    }
    if (!filePath.startsWith(staticRootPath)) return new Response("bad path", { status: 400 });
    try {
      const data = await Deno.readFile(filePath);
      const ext = name.slice(name.lastIndexOf("."));
      return new Response(data, {
        headers: { "content-type": MIME[ext] ?? "application/octet-stream" },
      });
    } catch {
      return new Response("not found", { status: 404 });
    }
  }

  const httpServer = Deno.serve(
    { hostname: host, port: options.port ?? 7780, onListen: () => {} },
    handleHttp,
  );
  const httpPort = (httpServer.addr as Deno.NetAddr).port;

  return {
    httpPort,
    quicPort,
    wtUrl,
    certHash: cert.certHashB64,
    async shutdown(): Promise<void> {
      for (const wt of sessions) {
        try {
          wt.close({ closeCode: 0, reason: "relay shutdown" });
        } catch {
          // already gone
        }
      }
      listener.stop();
      endpoint.close({ closeCode: 0, reason: "relay shutdown" });
      await httpServer.shutdown();
    },
  };
}

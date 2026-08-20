// The DimOS relay: QUIC/WebTransport listener (robot + viewer sessions) plus
// a plain-HTTP side (static files, /api/info, /api/stats). Payload-blind:
// all forwarding decisions come from frame headers and robot manifests.
// Session/transport handling lives in session.ts, registration + routing in
// registry.ts; this file owns the listeners and process-level wiring.
import { PROTOCOL_VERSION } from "@dimos/shared";
import { fileURLToPath, pathToFileURL } from "node:url";
import { makeEphemeralCert } from "./cert.ts";
import { LATEST_STALE_MS } from "./forward.ts";
import { Registry } from "./registry.ts";
import { RobotSession, ViewerSession } from "./session.ts";

// Subs snapshots ride lossy datagrams; this resend interval is the loss- and
// reorder-healing mechanism (bridges ignore stale `n`).
const SNAPSHOT_RESEND_MS = 2_000;

export interface RelayOptions {
  /** TCP port for the HTTP side. Default 7780; 0 picks an ephemeral port. */
  port?: number;
  /** Bind host for both listeners. The default is the only secure-context-friendly choice. */
  host?: string;
  /**
   * Built Cockpit app (web/cockpit/dist): / serves its index.html. Without it
   * the relay has no UI (/ answers 404 with a build hint); only /api/* works.
   */
  cockpitDir?: string;
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

function resolveDirUrl(dir: string, label: string): URL {
  // Canonical (realPath) so serveFrom compares symlink-free paths (macOS /tmp
  // is itself a symlink); href must end with "/" so new URL(name, root)
  // resolves under it. Fail with a clear labeled error on a bad path: the
  // raw NotFound is cryptic, and a plain file would "start" fine and then
  // 404 every request.
  let real: string;
  try {
    real = Deno.realPathSync(dir);
  } catch {
    throw new Error(`${label} does not exist: ${dir}`);
  }
  if (!Deno.statSync(real).isDirectory) {
    throw new Error(`${label} is not a directory: ${dir}`);
  }
  return pathToFileURL(real.endsWith("/") ? real : real + "/");
}

/**
 * Serve `name` from under `root` (canonical, via resolveDirUrl): a 400 for
 * path traversal or symlink escape, null when the file does not exist
 * (callers fall through to the next root or a 404).
 */
async function serveFrom(root: URL, name: string): Promise<Response | null> {
  // Resolve the request to a real path and confirm it stays under the root. A
  // leading "/" or "\" makes `new URL(name, root)` jump to the filesystem
  // root; fileURLToPath additionally throws on encoded slashes.
  let filePath: string;
  try {
    filePath = fileURLToPath(new URL(name, root));
  } catch {
    return new Response("bad path", { status: 400 });
  }
  const rootPath = fileURLToPath(root);
  if (!filePath.startsWith(rootPath)) return new Response("bad path", { status: 400 });
  // The lexical check cannot see symlinks: canonicalize (realPath follows
  // them) and require the target to still be under the root, so a link
  // inside a served tree cannot expose files outside it.
  let realPath: string;
  try {
    realPath = await Deno.realPath(filePath);
  } catch {
    return null; // absent (or a dangling link)
  }
  if (!realPath.startsWith(rootPath)) return new Response("bad path", { status: 400 });
  try {
    const data = await Deno.readFile(realPath);
    const ext = name.slice(name.lastIndexOf("."));
    return new Response(data, {
      headers: { "content-type": MIME[ext] ?? "application/octet-stream" },
    });
  } catch {
    return null;
  }
}

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

  // Resolve the served root before binding anything so a bad path fails
  // fast, without a QUIC endpoint or timer left behind.
  const cockpitRoot = options.cockpitDir ? resolveDirUrl(options.cockpitDir, "cockpitDir") : null;

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

  const registry = new Registry();
  const sessions = new Set<WebTransport>();
  let nextViewerId = 1;

  function track(wt: WebTransport): void {
    sessions.add(wt);
    wt.closed.catch(() => {}).finally(() => sessions.delete(wt));
  }

  const resendTimer = setInterval(() => registry.resendSnapshots(), SNAPSHOT_RESEND_MS);
  // A pending resend must not keep the Deno process alive after shutdown().
  Deno.unrefTimer(resendTimer);
  // Offers reap stale latest streams opportunistically, but an idle input
  // stops offering; this interval bounds an idle stream's lifetime to just
  // under 2x the stale window.
  const reapTimer = setInterval(() => registry.reapAll(Date.now()), LATEST_STALE_MS);
  Deno.unrefTimer(reapTimer);

  (async () => {
    for await (const incoming of listener) {
      (async () => {
        const conn = await incoming.accept();
        const wt = await Deno.upgradeWebTransport(conn);
        await wt.ready;
        track(wt);
        const path = new URL(wt.url).pathname;
        if (path === "/robot") new RobotSession(wt, conn, registry).start();
        else if (path === "/viewer") new ViewerSession(wt, nextViewerId++, registry).start();
        else {
          console.log(`[relay] rejecting unknown WebTransport endpoint ${path}`);
          wt.close({ closeCode: 1, reason: "unknown WebTransport endpoint" });
        }
      })().catch((e) => console.log("[relay] accept failed:", (e as Error)?.message ?? e));
    }
  })().catch(() => {
    // listener stopped (shutdown)
  });

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
      return Response.json(registry.stats());
    }
    if (cockpitRoot === null) {
      return new Response(
        "cockpit dist not built (dimos run --local-relay builds it; " +
          "or run `deno task build` in web/cockpit)",
        { status: 404 },
      );
    }
    const name = url.pathname === "/" ? "index.html" : url.pathname.slice(1);
    return await serveFrom(cockpitRoot, name) ?? new Response("not found", { status: 404 });
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
      clearInterval(resendTimer);
      clearInterval(reapTimer);
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

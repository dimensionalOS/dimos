import { parseArgs } from "@std/cli";
import { PROTOCOL_VERSION } from "../vendor/dimos/web/shared/protocol.ts";
import { startRelay } from "../vendor/dimos/web/relay/server.ts";
import { Lobby, NameError, ROBOTS } from "./lobby.ts";
import { startPublicBridge } from "./public.ts";

const args = parseArgs(Deno.args, {
  default: { port: 7780, host: "127.0.0.1" },
});
const lobby = new Lobby();
const robotKey = crypto.randomUUID() + crypto.randomUUID();
const json = (data: unknown, status = 200) =>
  Response.json(data, {
    status,
    headers: { "cache-control": "no-store" },
  });
const relay = await startRelay({
  port: Number(args.port),
  host: "127.0.0.1",
  registry: lobby,
  authorizeSession: (url) =>
    url.pathname === "/robot"
      ? url.searchParams.get("key") === robotKey
      : !!lobby.find(url.searchParams.get("ticket")),
  viewerConnected: (viewer, url, close) => {
    const p = lobby.find(url.searchParams.get("ticket"));
    if (p) lobby.attach(viewer, p, close);
    else close();
  },
  handleHttp: async (req, info) => {
    const url = new URL(req.url);
    // These endpoints are loopback only: the project HTTPS gateway rejects /internal/.
    if (url.pathname === "/internal/robot-info") {
      return json({
        ...info,
        wtUrl: info.wtUrl.replace("/viewer", `/robot?key=${robotKey}`),
      });
    }
    if (url.pathname === "/internal/scorers") return json(lobby.scorerIdentities());
    if (url.pathname === "/internal/assignments") {
      return json(lobby.assignments());
    }
    if (url.pathname === "/api/lobby") {
      if (req.method === "GET") return json(lobby.state());
      if (req.method !== "POST") {
        return json({ error: "Method not allowed" }, 405);
      }
      const chunks: Uint8Array[] = [];
      let size = 0;
      for await (const chunk of req.body ?? []) {
        size += chunk.byteLength;
        if (size > 1024) return json({ error: "Request too large" }, 413);
        chunks.push(chunk);
      }
      let body;
      try {
        body = JSON.parse(new TextDecoder().decode(
          Uint8Array.from(chunks.flatMap((c) => [...c])),
        ));
      } catch {
        return json({ error: "Invalid request" }, 400);
      }
      if (!body || typeof body !== "object") {
        return json({ error: "Invalid request" }, 400);
      }
      let p = lobby.find(typeof body.token === "string" ? body.token : null);
      if (!p && body.action !== "create") {
        return json({ error: "Session expired" }, 401);
      }
      if (!p) p = lobby.create() ?? undefined;
      if (!p) {
        return json({ error: "The world is busy. Try again shortly." }, 503);
      }
      if (["observe", "join", "host"].includes(body.action)) {
        if (
          body.robot !== undefined &&
          (typeof body.robot !== "string" || !ROBOTS.includes(body.robot))
        ) {
          return json({ error: "Choose a valid duck." }, 400);
        }
        try {
          if (!lobby.change(p, body.action, body.robot, body.displayName)) {
            return json({
              error: "That duck is unavailable. Choose another duck or watch the world.",
            }, 409);
          }
        } catch (error) {
          if (error instanceof NameError) {
            return json({ error: error.message }, 422);
          }
          throw error;
        }
      } else if (!["create", "status"].includes(body.action)) {
        return json({ error: "Invalid action" }, 400);
      }
      return json({ ...lobby.state(p), token: p.token });
    }
    const ticket = url.pathname.match(/^\/sessions\/([^/]+)\/api\/info$/)?.[1];
    if (ticket) {
      const p = lobby.find(ticket);
      if (!p) return json({ error: "Session expired" }, 401);
      return json({ ...info, wtUrl: `${info.wtUrl}?ticket=${p.token}` });
    }
    if (url.pathname === "/api/info") {
      return json({ error: "Join the lobby first." }, 401);
    }
    return null;
  },
});
console.log(
  JSON.stringify({
    event: "ready",
    httpPort: relay.httpPort,
    wtUrl: relay.wtUrl,
    certHash: relay.certHash,
    v: PROTOCOL_VERSION,
  }),
);
let closePublic = () => {};
try {
  const config = JSON.parse(await Deno.readTextFile(new URL("./.public.json", import.meta.url)));
  closePublic = startPublicBridge(lobby, config.url, config.secret);
} catch (error) {
  if (!(error instanceof Deno.errors.NotFound)) throw error;
}
for (const signal of ["SIGINT", "SIGTERM"] as const) {
  Deno.addSignalListener(signal, async () => {
    closePublic();
    await relay.shutdown();
    Deno.exit(0);
  });
}

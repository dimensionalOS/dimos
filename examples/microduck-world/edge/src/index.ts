import {
  admin,
  authRoute,
  equalSecret,
  json,
  sameOrigin,
  userFor,
} from "./auth.ts";
export { MatchRelay } from "./room.ts";

class RequestError extends Error {
  constructor(message: string, readonly status: number) {
    super(message);
  }
}

async function boundedJson(request: Request, limit = 4096): Promise<unknown> {
  if (!request.body) return null;
  const reader = request.body.getReader();
  const chunks: Uint8Array[] = [];
  let length = 0;
  for (;;) {
    const { value, done } = await reader.read();
    if (done) break;
    length += value.length;
    if (length > limit) {
      await reader.cancel();
      throw new RequestError("Request too large", 413);
    }
    chunks.push(value);
  }
  const bytes = new Uint8Array(length);
  let offset = 0;
  for (const chunk of chunks) {
    bytes.set(chunk, offset);
    offset += chunk.length;
  }
  try {
    return JSON.parse(new TextDecoder().decode(bytes));
  } catch {
    throw new RequestError("Invalid JSON", 400);
  }
}

async function handle(request: Request, env: Env): Promise<Response> {
  const url = new URL(request.url);
  const room = env.MATCH.getByName(env.MATCH_ID);
  let authenticated: Awaited<ReturnType<typeof userFor>> = null;
  try {
    if (url.pathname === "/bridge") {
      const protocols =
        request.headers.get("sec-websocket-protocol")?.split(",").map((s) =>
          s.trim()
        ) ?? [];
      if (
        !env.HOST_SECRET || protocols[0] !== "microduck-origin" ||
        !(await equalSecret(protocols[1] ?? "", env.HOST_SECRET))
      ) return json({ error: "Forbidden" }, 403);
      return room.fetch(new Request("https://match/host", request));
    }
    if (
      url.pathname.startsWith("/auth/") || url.pathname.startsWith("/api/") ||
      url.pathname === "/connect" || url.pathname.startsWith("/sessions/")
    ) {
      const ip = request.headers.get("cf-connecting-ip") ?? "local";
      authenticated = await userFor(request, env);
      const rate = await env.REQUEST_LIMITER.limit({
        key: authenticated ? `user:${authenticated.id}` : `ip:${ip}`,
      });
      if (!rate.success) {
        return json({ error: "Too many requests. Please wait a minute." }, 429);
      }
    }
    const auth = await authRoute(request, env);
    if (auth) return auth;
    if (
      request.method === "GET" &&
      (url.pathname === "/api/lobby" || url.pathname === "/api/preview")
    ) {
      const result = await room.http(null, url.pathname);
      return json(JSON.parse(result.body), result.status);
    }
    if (url.pathname === "/healthz") {
      const result = await room.http(null, "/healthz");
      return json(JSON.parse(result.body), result.status);
    }
    if (
      url.pathname.startsWith("/api/") ||
      url.pathname.startsWith("/sessions/") || url.pathname === "/connect"
    ) {
      const user = authenticated;
      if (!user) {
        return json({
          error: "Sign in with GitHub to join.",
          login: "/auth/login",
        }, 401);
      }
      if (
        (request.method !== "GET" || url.pathname === "/connect") &&
        !sameOrigin(request, env)
      ) return json({ error: "Forbidden origin" }, 403);
      if (url.pathname === "/connect") {
        const headers = new Headers(request.headers);
        headers.set("x-user-id", user.id);
        headers.set("x-user-login", user.login);
        headers.set("x-session-expires", String(user.expires_at));
        return room.fetch(
          new Request(
            `https://match/viewer?ticket=${
              encodeURIComponent(url.searchParams.get("ticket") ?? "")
            }`,
            { headers },
          ),
        );
      }
      if (/^\/sessions\/[a-zA-Z0-9-]{1,100}\/api\/info$/.test(url.pathname)) {
        const ticket = url.pathname.split("/")[2];
        const result = await room.http(
          user,
          "/api/session",
          JSON.stringify({ token: ticket }),
        );
        if (result.status !== 200) {
          return json(JSON.parse(result.body), result.status);
        }
        const body = JSON.parse(result.body) as { v: number };
        return json({
          wtUrl: `${env.PUBLIC_ORIGIN.replace(/^http/, "ws")}/connect?ticket=${
            encodeURIComponent(ticket)
          }`,
          certHash: "",
          v: body.v,
        });
      }
      if (url.pathname === "/api/lobby" && request.method === "POST") {
        const body = await boundedJson(request);
        const result = await room.http(
          user,
          "/api/lobby",
          JSON.stringify(body),
        );
        if (result.status === 200) {
          const name =
            (JSON.parse(result.body) as { displayName?: string }).displayName;
          if (
            name && body && typeof body === "object" && "action" in body &&
            body.action === "join"
          ) {
            await env.DB.prepare("UPDATE users SET preferred_name=? WHERE id=?")
              .bind(name, user.id).run();
          }
        }
        return json(JSON.parse(result.body), result.status);
      }
      if (
        url.pathname === "/api/admin/ban" && request.method === "POST" &&
        admin(user, env)
      ) {
        const body = await boundedJson(request) as {
          userId?: string;
          banned?: boolean;
        };
        if (
          !body || typeof body.userId !== "string" ||
          !/^\d+$/.test(body.userId) || typeof body.banned !== "boolean" ||
          body.userId === user.id
        ) return json({ error: "Invalid account" }, 400);
        await env.DB.prepare("UPDATE users SET banned=? WHERE id=?").bind(
          body.banned ? 1 : 0,
          body.userId,
        ).run();
        if (body.banned) {
          await env.DB.prepare("DELETE FROM sessions WHERE user_id=?").bind(
            body.userId,
          ).run();
          await room.revoke(body.userId);
        }
        return json({ ok: true });
      }
      return json({ error: "Not found" }, 404);
    }
    if (request.method !== "GET" && request.method !== "HEAD") {
      return new Response("Method not allowed", { status: 405 });
    }
    return env.ASSETS.fetch(request);
  } catch (error) {
    if (error instanceof RequestError) {
      return json({ error: error.message }, error.status);
    }
    // Do not log callback codes, session cookies, bridge protocols or secrets.
    console.error(
      JSON.stringify({
        event: "request_failed",
        path: url.pathname.startsWith("/sessions/")
          ? "/sessions/…"
          : url.pathname,
        kind: error instanceof Error ? error.name : "Error",
      }),
    );
    return json({
      error: "The request could not be completed. Please try again.",
    }, 503);
  }
}

export default {
  async fetch(request, env): Promise<Response> {
    const response = await handle(request, env);
    if (response.status === 101) return response;
    const headers = new Headers(response.headers);
    headers.set("x-content-type-options", "nosniff");
    headers.set("referrer-policy", "no-referrer");
    headers.set(
      "permissions-policy",
      "camera=(), microphone=(), geolocation=()",
    );
    headers.set(
      "content-security-policy",
      "default-src 'self'; script-src 'self' 'wasm-unsafe-eval'; style-src 'self' 'unsafe-inline'; img-src 'self' data: blob:; connect-src 'self' wss://sim.tule.world; worker-src 'self' blob:; frame-ancestors 'none'; base-uri 'none'; form-action 'self'",
    );
    return new Response(response.body, {
      status: response.status,
      statusText: response.statusText,
      headers,
    });
  },
} satisfies ExportedHandler<Env>;

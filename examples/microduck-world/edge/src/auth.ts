import { timingSafeEqual } from "node:crypto";
import type { Identity } from "../../shared/publicWire.ts";

export type User = Identity & {
  preferred_name: string;
  banned: number;
  expires_at: number;
};
const SESSION_COOKIE = "__Host-microduck";
const STATE_COOKIE = "__Host-microduck-oauth";
export const SESSION_SECONDS = 7 * 86400;
const encoder = new TextEncoder();

export function randomToken(): string {
  return Array.from(
    crypto.getRandomValues(new Uint8Array(32)),
    (b) => b.toString(16).padStart(2, "0"),
  ).join("");
}
export async function digest(value: string): Promise<string> {
  return Array.from(
    new Uint8Array(
      await crypto.subtle.digest("SHA-256", encoder.encode(value)),
    ),
    (b) => b.toString(16).padStart(2, "0"),
  ).join("");
}
export async function equalSecret(a: string, b: string): Promise<boolean> {
  const [x, y] = await Promise.all([digest(a), digest(b)]);
  return timingSafeEqual(encoder.encode(x), encoder.encode(y));
}
export function cookie(req: Request, name: string): string {
  return req.headers.get("cookie")?.split(";").map((s) => s.trim()).find((s) =>
    s.startsWith(`${name}=`)
  )?.slice(name.length + 1) ?? "";
}
function setCookie(name: string, value: string, age: number): string {
  return `${name}=${value}; Path=/; HttpOnly; Secure; SameSite=Lax; Max-Age=${age}`;
}
export function json(value: unknown, status = 200): Response {
  return Response.json(value, {
    status,
    headers: { "cache-control": "no-store" },
  });
}
export function sameOrigin(req: Request, env: Env): boolean {
  return req.headers.get("origin") === env.PUBLIC_ORIGIN;
}
export async function userFor(req: Request, env: Env): Promise<User | null> {
  const token = cookie(req, SESSION_COOKIE);
  if (!/^[a-f0-9]{64}$/.test(token)) return null;
  return env.DB.prepare(
    "SELECT u.id, u.login, u.preferred_name, u.banned, s.expires_at FROM sessions s JOIN users u ON u.id=s.user_id WHERE s.hash=? AND s.expires_at>? AND u.banned=0",
  ).bind(await digest(token), Date.now()).first<User>();
}
export function admin(user: Identity, env: Env): boolean {
  return env.ADMIN_GITHUB_IDS.split(",").map((s) => s.trim()).includes(user.id);
}

export async function authRoute(
  req: Request,
  env: Env,
): Promise<Response | null> {
  const url = new URL(req.url);
  if (
    ["/api/auth", "/auth/login", "/auth/callback"].includes(url.pathname) &&
    req.method !== "GET"
  ) return json({ error: "Method not allowed" }, 405);
  if (url.pathname === "/api/auth") {
    const user = await userFor(req, env);
    return json({
      required: true,
      provider: "github",
      user: user
        ? {
          id: user.id,
          login: user.login,
          preferredName: user.preferred_name,
          admin: admin(user, env),
        }
        : null,
      configured: !!env.GITHUB_CLIENT_ID && !!env.GITHUB_CLIENT_SECRET,
    });
  }
  if (url.pathname === "/auth/login") {
    if (!env.GITHUB_CLIENT_ID || !env.GITHUB_CLIENT_SECRET) {
      return json({ error: "GitHub sign-in is being configured." }, 503);
    }
    const state = randomToken();
    const verifier = randomToken();
    const raw = new Uint8Array(
      await crypto.subtle.digest("SHA-256", encoder.encode(verifier)),
    );
    const challenge = btoa(String.fromCharCode(...raw)).replaceAll("+", "-")
      .replaceAll("/", "_").replaceAll("=", "");
    await env.DB.batch([
      env.DB.prepare("DELETE FROM oauth_states WHERE expires_at<?").bind(
        Date.now(),
      ),
      env.DB.prepare("DELETE FROM sessions WHERE expires_at<?").bind(
        Date.now(),
      ),
      env.DB.prepare(
        "INSERT INTO oauth_states(hash,verifier,expires_at) VALUES(?,?,?)",
      ).bind(await digest(state), verifier, Date.now() + 300_000),
    ]);
    const auth = new URL("https://github.com/login/oauth/authorize");
    auth.search = new URLSearchParams({
      client_id: env.GITHUB_CLIENT_ID,
      redirect_uri: `${env.PUBLIC_ORIGIN}/auth/callback`,
      scope: "",
      prompt: "select_account",
      state,
      code_challenge: challenge,
      code_challenge_method: "S256",
    }).toString();
    return new Response(null, {
      status: 302,
      headers: {
        location: auth.href,
        "set-cookie": setCookie(STATE_COOKIE, state, 300),
        "cache-control": "no-store",
      },
    });
  }
  if (url.pathname === "/auth/callback") {
    const state = url.searchParams.get("state") ?? "";
    const code = url.searchParams.get("code") ?? "";
    if (
      !/^[a-f0-9]{64}$/.test(state) || !code || code.length > 512 ||
      !(await equalSecret(state, cookie(req, STATE_COOKIE)))
    ) {
      return json({
        error: "Sign-in expired. Return to the homepage and try again.",
      }, 400);
    }
    const saved = await env.DB.prepare(
      "DELETE FROM oauth_states WHERE hash=? AND expires_at>? RETURNING verifier",
    ).bind(await digest(state), Date.now()).first<{ verifier: string }>();
    if (!saved) {
      return json({ error: "Sign-in expired. Please try again." }, 400);
    }
    const tokenResponse = await fetch(
      "https://github.com/login/oauth/access_token",
      {
        method: "POST",
        headers: {
          accept: "application/json",
          "content-type": "application/json",
        },
        body: JSON.stringify({
          client_id: env.GITHUB_CLIENT_ID,
          client_secret: env.GITHUB_CLIENT_SECRET,
          code,
          code_verifier: saved.verifier,
          redirect_uri: `${env.PUBLIC_ORIGIN}/auth/callback`,
        }),
        signal: AbortSignal.timeout(10_000),
      },
    );
    const token = await tokenResponse.json<{ access_token?: string }>();
    if (!tokenResponse.ok || !token.access_token) {
      return json({ error: "GitHub could not complete sign-in." }, 502);
    }
    const profileResponse = await fetch("https://api.github.com/user", {
      headers: {
        authorization: `Bearer ${token.access_token}`,
        accept: "application/vnd.github+json",
        "user-agent": "Microduck-DimOS",
        "x-github-api-version": "2022-11-28",
      },
      signal: AbortSignal.timeout(10_000),
    });
    const profile = await profileResponse.json<
      { id?: number; login?: string }
    >();
    if (
      !profileResponse.ok || !Number.isSafeInteger(profile.id) ||
      !profile.login || !/^[a-zA-Z0-9-]{1,39}$/.test(profile.login)
    ) {
      return json({ error: "Could not verify your GitHub identity." }, 502);
    }
    const id = String(profile.id);
    await env.DB.prepare(
      "INSERT INTO users(id,login,created_at) VALUES(?,?,?) ON CONFLICT(id) DO UPDATE SET login=excluded.login",
    ).bind(id, profile.login, Date.now()).run();
    if (
      (await env.DB.prepare("SELECT banned FROM users WHERE id=?").bind(id)
        .first<{ banned: number }>())?.banned
    ) return json({ error: "This account cannot join the demo." }, 403);
    const session = randomToken();
    await env.DB.prepare(
      "INSERT INTO sessions(hash,user_id,expires_at) VALUES(?,?,?)",
    ).bind(await digest(session), id, Date.now() + SESSION_SECONDS * 1000)
      .run();
    const headers = new Headers({
      location: env.PUBLIC_ORIGIN,
      "cache-control": "no-store",
    });
    headers.append(
      "set-cookie",
      setCookie(SESSION_COOKIE, session, SESSION_SECONDS),
    );
    headers.append("set-cookie", setCookie(STATE_COOKIE, "", 0));
    return new Response(null, { status: 302, headers });
  }
  if (url.pathname === "/auth/logout") {
    if (req.method !== "POST" || !sameOrigin(req, env)) {
      return json({ error: "Forbidden" }, 403);
    }
    const user = await userFor(req, env);
    if (user) {
      await env.DB.prepare("DELETE FROM sessions WHERE hash=?").bind(
        await digest(cookie(req, SESSION_COOKIE)),
      ).run();
      await env.MATCH.getByName(env.MATCH_ID).revoke(user.id);
    }
    const pendingState = cookie(req, STATE_COOKIE);
    if (pendingState) await env.DB.prepare("DELETE FROM oauth_states WHERE hash=?")
      .bind(await digest(pendingState)).run();
    const headers = new Headers({ "cache-control": "no-store" });
    headers.append("set-cookie", setCookie(SESSION_COOKIE, "", 0));
    headers.append("set-cookie", setCookie(STATE_COOKIE, "", 0));
    return new Response(null, { status: 204, headers });
  }
  return null;
}

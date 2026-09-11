import { env, exports } from "cloudflare:workers";
import {
  afterEach,
  beforeAll,
  beforeEach,
  describe,
  expect,
  it,
  vi,
} from "vitest";
import { digest } from "../src/auth.ts";
import migration from "../migrations/0001_identity.sql?raw";

const origin = "https://sim.tule.world";
const request = (path: string, init?: RequestInit) =>
  exports.default.fetch(origin + path, { redirect: "manual", ...init });
const token = "b".repeat(64);
async function session(id = "100", expires = Date.now() + 60_000) {
  await env.DB.prepare(
    "INSERT OR REPLACE INTO users(id,login,created_at) VALUES(?,?,?)",
  ).bind(id, "player", Date.now()).run();
  await env.DB.prepare(
    "INSERT OR REPLACE INTO sessions(hash,user_id,expires_at) VALUES(?,?,?)",
  ).bind(await digest(token), id, expires).run();
  return `__Host-microduck=${token}`;
}
beforeAll(async () => {
  for (
    const statement of migration.split(";").filter((s: string) => s.trim())
  ) await env.DB.prepare(statement).run();
});
beforeEach(async () => {
  await env.DB.batch([
    env.DB.prepare("DELETE FROM sessions"),
    env.DB.prepare("DELETE FROM oauth_states"),
    env.DB.prepare("DELETE FROM users"),
  ]);
});
afterEach(() => vi.restoreAllMocks());

describe("GitHub identity boundary", () => {
  it("keeps anonymous clients out of control and session discovery", async () => {
    for (const path of ["/connect", "/sessions/fake/api/info"]) {
      expect((await request(path)).status).toBe(401);
    }
    expect(
      (await request("/api/lobby", {
        method: "POST",
        body: '{"action":"create"}',
      })).status,
    ).toBe(401);
    const response = await request("/api/auth");
    expect(await response.json()).toMatchObject({
      required: true,
      provider: "github",
      user: null,
    });
  });
  it("does not trust forged identity headers or cross-origin requests", async () => {
    expect(
      (await request("/api/lobby", {
        method: "POST",
        headers: { "x-user-id": "100", origin },
        body: "{}",
      })).status,
    ).toBe(401);
    const cookie = await session();
    expect(
      (await request("/api/lobby", {
        method: "POST",
        headers: { cookie, origin: "https://attacker.example" },
        body: "{}",
      })).status,
    ).toBe(403);
    expect(
      (await request("/connect", {
        headers: { cookie, origin: "https://attacker.example" },
      })).status,
    ).toBe(403);
  });
  it("rejects expired and banned sessions", async () => {
    const cookie = await session("100", Date.now() - 1);
    expect((await request("/connect", { headers: { cookie, origin } })).status)
      .toBe(401);
    await session();
    await env.DB.prepare("UPDATE users SET banned=1 WHERE id='100'").run();
    expect((await request("/connect", { headers: { cookie, origin } })).status)
      .toBe(401);
  });
  it("requires the configured server credential for the outbound origin", async () => {
    expect(
      (await request("/bridge", {
        headers: {
          upgrade: "websocket",
          "sec-websocket-protocol": "microduck-origin,wrong",
        },
      })).status,
    ).toBe(403);
  });
  it("rejects malformed and oversized JSON before forwarding it to Omarchy", async () => {
    const cookie = await session();
    const headers = { cookie, origin };
    expect(
      (await request("/api/lobby", { method: "POST", headers, body: "{" }))
        .status,
    ).toBe(400);
    expect(
      (await request("/api/lobby", {
        method: "POST",
        headers,
        body: JSON.stringify({ name: "x".repeat(5000) }),
      })).status,
    ).toBe(413);
  });
  it("uses single-use OAuth state, PKCE and a secure HttpOnly site session", async () => {
    const login = await request("/auth/login");
    expect(login.status).toBe(302);
    const authorize = new URL(login.headers.get("location")!);
    expect(authorize.origin).toBe("https://github.com");
    expect(authorize.searchParams.get("scope")).toBe("");
    expect(authorize.searchParams.get("prompt")).toBe("select_account");
    expect(authorize.searchParams.get("code_challenge_method")).toBe("S256");
    const state = authorize.searchParams.get("state")!;
    const stateCookie = login.headers.get("set-cookie")!.split(";")[0];
    const callback = `/auth/callback?code=test-code&state=${state}`;
    expect((await request(callback)).status).toBe(400);
    const mocked = vi.spyOn(globalThis, "fetch").mockImplementation(
      async (input, init) => {
        if (String(input) === "https://github.com/login/oauth/access_token") {
          const body = JSON.parse(String(init?.body));
          expect(body.code_verifier).toHaveLength(64);
          expect(body.redirect_uri).toBe(`${origin}/auth/callback`);
          return Response.json({ access_token: "github-test-access" });
        }
        if (String(input) === "https://api.github.com/user") {
          return Response.json({ id: 100, login: "player" });
        }
        throw new Error("Unexpected outbound request");
      },
    );
    const response = await request(callback, {
      headers: { cookie: stateCookie },
    });
    expect(response.status).toBe(302);
    const cookies = response.headers.get("set-cookie")!;
    expect(cookies).toContain("HttpOnly");
    expect(cookies).toContain("Secure");
    expect(cookies).toContain("SameSite=Lax");
    expect(cookies).not.toContain("github-test-access");
    expect(
      await env.DB.prepare("SELECT COUNT(*) AS count FROM sessions").first(),
    ).toEqual({ count: 1 });
    expect(
      (await request(callback, { headers: { cookie: stateCookie } })).status,
    ).toBe(400);
    expect(mocked).toHaveBeenCalledTimes(2);
  });
  it("sign-out invalidates the session and pending OAuth state and clears both cookies", async () => {
    const sessionCookie = await session();
    const login = await request("/auth/login");
    const oauthCookie = login.headers.get("set-cookie")!.split(";")[0];
    const cookies = `${sessionCookie}; ${oauthCookie}`;
    const response = await request("/auth/logout", { method: "POST", headers: { origin, cookie: cookies } });
    expect(response.status).toBe(204);
    const cleared = response.headers.getSetCookie();
    expect(cleared).toHaveLength(2);
    expect(cleared.some(c => c.startsWith("__Host-microduck="))).toBe(true);
    expect(cleared.some(c => c.startsWith("__Host-microduck-oauth="))).toBe(true);
    for (const c of cleared) expect(c).toContain("Max-Age=0");
    expect(await env.DB.prepare("SELECT COUNT(*) AS count FROM sessions").first()).toEqual({ count: 0 });
    expect(await env.DB.prepare("SELECT COUNT(*) AS count FROM oauth_states").first()).toEqual({ count: 0 });
    expect(await (await request("/api/auth", { headers: { cookie: cookies } })).json()).toMatchObject({ user: null });
  });
  it("checks admin identity and revokes a banned account's sessions", async () => {
    let cookie = await session();
    expect(
      (await request("/api/admin/ban", {
        method: "POST",
        headers: { cookie, origin },
        body: JSON.stringify({ userId: "200", banned: true }),
      })).status,
    ).toBe(404);
    await env.DB.prepare(
      "INSERT INTO users(id,login,created_at) VALUES('200','target',0)",
    ).run();
    await env.DB.prepare(
      "INSERT INTO sessions(hash,user_id,expires_at) VALUES('target','200',?)",
    ).bind(Date.now() + 1000).run();
    cookie = await session("6902572");
    expect(
      (await request("/api/admin/ban", {
        method: "POST",
        headers: { cookie, origin },
        body: JSON.stringify({ userId: "200", banned: true }),
      })).status,
    ).toBe(200);
    expect(
      await env.DB.prepare("SELECT banned FROM users WHERE id='200'").first(),
    ).toEqual({ banned: 1 });
    expect(
      await env.DB.prepare("SELECT hash FROM sessions WHERE user_id='200'")
        .first(),
    ).toBeNull();
  });
});

// CloudflareClient over a fake fetch. No network.
import {
  assert,
  assertEquals,
  assertRejects,
  assertStringIncludes,
  assertThrows,
} from "@std/assert";
import {
  CF_API_BASE,
  CloudflareClient,
  CloudflareError,
  type FetchFn,
  parseRtcFile,
  SessionGoneError,
  STUN_ONLY,
  TURN_TTL_S,
} from "./cloudflare.ts";

const CONFIG = { appId: "app-1", appSecret: "secret-0123456789abcdef" };
const TURN = { ...CONFIG, turnKeyId: "turn-key", turnToken: "turn-token-0123456789" };

interface Call {
  url: string;
  method: string;
  headers: Record<string, string>;
  body: unknown;
}

/** A fake fetch answering every call with `respond(call)`; records calls. */
function fakeFetch(respond: (call: Call) => Response): { fetchFn: FetchFn; calls: Call[] } {
  const calls: Call[] = [];
  const fetchFn = ((input: string | URL | Request, init?: RequestInit) => {
    const headers: Record<string, string> = {};
    new Headers(init?.headers).forEach((v, k) => {
      headers[k] = v;
    });
    const call: Call = {
      url: String(input),
      method: init?.method ?? "GET",
      headers,
      body: typeof init?.body === "string" ? JSON.parse(init.body) : null,
    };
    calls.push(call);
    return Promise.resolve(respond(call));
  }) as FetchFn;
  return { fetchFn, calls };
}

function json(body: unknown, status = 200): Response {
  return new Response(JSON.stringify(body), { status });
}

Deno.test("newSession posts the offer with the app secret and returns id + answer", async () => {
  const { fetchFn, calls } = fakeFetch(() =>
    json({ sessionId: "sess-1", sessionDescription: { type: "answer", sdp: "v=0 answer" } }, 201)
  );
  const client = new CloudflareClient(CONFIG, fetchFn);
  assertEquals(await client.newSession("v=0 offer"), {
    sessionId: "sess-1",
    answerSdp: "v=0 answer",
  });
  assertEquals(calls[0].url, `${CF_API_BASE}/apps/app-1/sessions/new`);
  assertEquals(calls[0].method, "POST");
  assertEquals(calls[0].headers.authorization, `Bearer ${CONFIG.appSecret}`);
  assertEquals(calls[0].body, { sessionDescription: { type: "offer", sdp: "v=0 offer" } });
});

Deno.test("errors: non-2xx keeps the status and never echoes the body", async () => {
  const { fetchFn } = fakeFetch(() => new Response("v=0 secret-ice-ufrag", { status: 502 }));
  const client = new CloudflareClient(CONFIG, fetchFn);
  const err = await assertRejects(() => client.newSession("v=0"), CloudflareError);
  assertEquals(err.status, 502);
  assertEquals(err.message.includes("secret-ice-ufrag"), false);
});

Deno.test("errors: a 2xx body with errorCode is an error; disconnected or 410 is session gone", async () => {
  let body: unknown = { errorCode: "invalid_sdp", errorDescription: "bad offer" };
  let status = 200;
  const { fetchFn } = fakeFetch(() => json(body, status));
  const client = new CloudflareClient(CONFIG, fetchFn);
  const err = await assertRejects(() => client.renegotiate("s", "v=0"), CloudflareError);
  assertEquals(err.code, "invalid_sdp");
  assertEquals(err.sessionId, "s");
  assertStringIncludes(err.message, "bad offer");
  assert(!(err instanceof SessionGoneError));

  body = { errorCode: "session_error", errorDescription: "Session is disconnected" };
  const gone = await assertRejects(() => client.renegotiate("s", "v=0"), SessionGoneError);
  assertEquals(gone.sessionId, "s");

  body = {};
  status = 410;
  await assertRejects(() => client.renegotiate("s", "v=0"), SessionGoneError);
});

Deno.test("errors: a non-JSON or non-object 2xx body is an error", async () => {
  let text = "<html>";
  const { fetchFn } = fakeFetch(() => new Response(text, { status: 200 }));
  const client = new CloudflareClient(CONFIG, fetchFn);
  await assertRejects(() => client.renegotiate("s", "v=0"), CloudflareError);
  text = "[1]";
  await assertRejects(() => client.renegotiate("s", "v=0"), CloudflareError);
  text = JSON.stringify({ sessionId: 5 });
  await assertRejects(() => client.newSession("v=0"), CloudflareError);
});

Deno.test("addTracks posts local and remote refs and parses per-track results", async () => {
  const { fetchFn, calls } = fakeFetch((call) => {
    const tracks = (call.body as { tracks: { location: string }[] }).tracks;
    if (tracks[0].location === "local") {
      return json({
        requiresImmediateRenegotiation: false,
        tracks: [{ mid: "0", trackName: "cam" }],
      });
    }
    return json({
      requiresImmediateRenegotiation: true,
      tracks: [
        { sessionId: "robot", trackName: "cam", mid: "7" },
        {
          sessionId: "robot",
          trackName: "rear",
          errorCode: "not_found_track_error",
          errorDescription: "no",
        },
      ],
      sessionDescription: { type: "offer", sdp: "v=0 pull-offer" },
    });
  });
  const client = new CloudflareClient(CONFIG, fetchFn);
  const published = await client.addTracks("robot", [{
    location: "local",
    mid: "0",
    trackName: "cam",
  }]);
  assertEquals(published, { tracks: [{ mid: "0", trackName: "cam" }], sdp: null });
  assertEquals(calls[0].url, `${CF_API_BASE}/apps/app-1/sessions/robot/tracks/new`);
  assertEquals(calls[0].body, { tracks: [{ location: "local", mid: "0", trackName: "cam" }] });

  const pulled = await client.addTracks("viewer", [
    { location: "remote", sessionId: "robot", trackName: "cam" },
    { location: "remote", sessionId: "robot", trackName: "rear" },
  ]);
  assertEquals(pulled.sdp, "v=0 pull-offer");
  assertEquals(pulled.tracks, [
    { mid: "7", trackName: "cam" },
    { trackName: "rear", errorCode: "not_found_track_error", errorDescription: "no" },
  ]);
});

Deno.test("closeTracks forces the stop and returns per-track results; renegotiate submits the answer", async () => {
  let body: unknown = {};
  const { fetchFn, calls } = fakeFetch(() => json(body));
  const client = new CloudflareClient(CONFIG, fetchFn);
  assertEquals(await client.closeTracks("viewer", ["7", "8"]), []);
  // An HTTP 200 whose body fails one mid: reported, not thrown.
  body = { tracks: [{ mid: "7" }, { mid: "8", errorCode: "track_error", errorDescription: "x" }] };
  assertEquals(await client.closeTracks("viewer", ["7", "8"]), [
    { mid: "7" },
    { mid: "8", errorCode: "track_error", errorDescription: "x" },
  ]);
  await client.renegotiate("viewer", "v=0 answer");
  const close = ["PUT", `${CF_API_BASE}/apps/app-1/sessions/viewer/tracks/close`, {
    tracks: [{ mid: "7" }, { mid: "8" }],
    force: true,
  }];
  assertEquals(calls.map((c) => [c.method, c.url, c.body]), [
    close,
    close,
    ["PUT", `${CF_API_BASE}/apps/app-1/sessions/viewer/renegotiate`, {
      sessionDescription: { type: "answer", sdp: "v=0 answer" },
    }],
  ]);
});

Deno.test("iceServers is STUN only without a TURN key and never calls out", async () => {
  const { fetchFn, calls } = fakeFetch(() => json({}));
  const client = new CloudflareClient(CONFIG, fetchFn);
  assertEquals(await client.iceServers(), STUN_ONLY);
  assertEquals(calls.length, 0);
});

Deno.test("iceServers mints TURN credentials with the TURN token and ttl", async () => {
  const servers = [
    { urls: ["stun:stun.cloudflare.com:3478"] },
    { urls: ["turn:turn.cloudflare.com:3478?transport=udp"], username: "u", credential: "c" },
  ];
  let body: unknown = { iceServers: servers };
  const { fetchFn, calls } = fakeFetch(() => json(body, 201));
  const client = new CloudflareClient(TURN, fetchFn);
  assertEquals(await client.iceServers(), servers);
  assertEquals(calls[0].url, `${CF_API_BASE}/turn/keys/turn-key/credentials/generate-ice-servers`);
  assertEquals(calls[0].headers.authorization, `Bearer ${TURN.turnToken}`);
  assertEquals(calls[0].body, { ttl: TURN_TTL_S });
  body = {};
  await assertRejects(() => client.iceServers(), CloudflareError);
});

Deno.test("parseRtcFile accepts the two shapes and names keys, never values, in errors", () => {
  assertEquals(parseRtcFile(JSON.stringify(CONFIG)), CONFIG);
  assertEquals(parseRtcFile(JSON.stringify(TURN)), TURN);
  const bad: [string, string][] = [
    ["not json", "not valid JSON"],
    ["[1]", "top level"],
    [JSON.stringify({ appId: "a" }), '"appSecret"'],
    [JSON.stringify({ appId: "", appSecret: "s" }), '"appId"'],
    [JSON.stringify({ ...CONFIG, app_secret: "x" }), 'unknown key "app_secret"'],
    [JSON.stringify({ ...CONFIG, turnKeyId: "k" }), "together"],
    [JSON.stringify({ ...CONFIG, turnKeyId: "k", turnToken: "" }), '"turnToken"'],
  ];
  for (const [text, expected] of bad) {
    const err = assertThrows(() => parseRtcFile(text), Error, undefined, text);
    assertStringIncludes(err.message, expected);
    assertEquals(err.message.includes(CONFIG.appSecret), false);
  }
});

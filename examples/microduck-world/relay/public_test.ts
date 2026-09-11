import { assertEquals } from "@std/assert";
import { Lobby } from "./lobby.ts";
import { publicRequest, startPublicBridge } from "./public.ts";
import { pack, unpack } from "../shared/publicWire.ts";
import {
  ControlFrameReader,
  encodeControlFrame,
  PROTOCOL_VERSION,
} from "../vendor/dimos/web/shared/protocol.ts";
const a = { id: "100", login: "one" }, b = { id: "200", login: "two" };
Deno.test("GitHub identity owns exactly one participant and cannot use another account's ticket", () => {
  const lobby = new Lobby();
  const first = publicRequest(lobby, a, "/api/lobby", { action: "create" }, null).body as any;
  const again = publicRequest(lobby, a, "/api/lobby", { action: "create" }, null).body as any;
  assertEquals(first.token, again.token);
  assertEquals(publicRequest(lobby, b, "/api/session", { token: first.token }, null).status, 401);
  assertEquals(
    publicRequest(lobby, null, "/api/session", { token: first.token }, null).status,
    401,
  );
  publicRequest(
    lobby,
    a,
    "/api/lobby",
    { action: "join", robot: "duck1", displayName: "One" },
    null,
  );
  assertEquals(
    publicRequest(
      lobby,
      b,
      "/api/lobby",
      { action: "join", token: first.token, robot: "duck1" },
      null,
    ).status,
    409,
  );
  assertEquals(lobby.find(first.token)?.userId, a.id);
  assertEquals(lobby.participants.size, 2);
});
Deno.test("public entry only permits normal join and observe actions", () => {
  const lobby = new Lobby();
  assertEquals(publicRequest(lobby, a, "/api/lobby", { action: "host" }, null).status, 400);
  assertEquals(publicRequest(lobby, null, "/api/lobby", { action: "join" }, null).status, 401);
  assertEquals(publicRequest(lobby, a, "/internal/robot-info", {}, null).status, 404);
});
Deno.test("revoking GitHub identity releases its duck and invalidates its ticket", () => {
  const lobby = new Lobby();
  const p = lobby.forUser(a.id)!;
  lobby.change(p, "join", "duck2");
  lobby.revokeUser(a.id);
  assertEquals(lobby.find(p.token), undefined);
  assertEquals(lobby.assignments(), {});
});
Deno.test("the public bridge rejects nonlocal plaintext origins", async () => {
  const { startPublicBridge } = await import("./public.ts");
  let rejected = false;
  try {
    startPublicBridge(new Lobby(), "ws://example.com/bridge", "a".repeat(64));
  } catch {
    rejected = true;
  }
  assertEquals(rejected, true);
});

Deno.test("a corrupt viewer stream leaves the origin and other viewers connected", async () => {
  const original = globalThis.WebSocket;
  let socket: FakeSocket;
  class FakeSocket extends EventTarget {
    static OPEN = 1;
    readyState = 1;
    bufferedAmount = 0;
    binaryType = "arraybuffer";
    sent: (string | Uint8Array)[] = [];
    constructor() {
      super();
      socket = this;
    }
    send(data: string | Uint8Array) {
      this.sent.push(data);
    }
    close() {
      this.readyState = 3;
      this.dispatchEvent(new Event("close"));
    }
    receive(data: string | Uint8Array) {
      this.dispatchEvent(
        new MessageEvent("message", { data: typeof data === "string" ? data : data.buffer }),
      );
    }
  }
  globalThis.WebSocket = FakeSocket as unknown as typeof WebSocket;
  const lobby = new Lobby();
  const stop = startPublicBridge(lobby, "wss://test.example/bridge", "a".repeat(64));
  try {
    for (const user of [a, b]) {
      const participant = lobby.forUser(user.id)!;
      socket!.receive(JSON.stringify({ t: "open", id: user.id, user, ticket: participant.token }));
    }
    socket!.receive(
      pack({ to: [a.id], kind: 0, sentAt: Date.now() }, new Uint8Array([255, 255, 255, 255])),
    );
    socket!.receive(
      pack(
        { to: [b.id], kind: 0, sentAt: Date.now() },
        encodeControlFrame({ t: "hello", role: "viewer", v: PROTOCOL_VERSION }),
      ),
    );
    await Promise.resolve();
    assertEquals(socket!.readyState, 1);
    const events = socket!.sent.filter((s): s is string => typeof s === "string").map((s) =>
      JSON.parse(s)
    );
    assertEquals(events.some((s) => s.t === "close" && s.id === a.id), true);
    assertEquals(events.some((s) => s.t === "close" && s.id === b.id), false);
    const replies = socket!.sent.filter((s): s is Uint8Array => typeof s !== "string").map(unpack);
    assertEquals(
      replies.some(({ header, payload }) =>
        header.to.includes(b.id) &&
        new ControlFrameReader().push(payload).some((m) => m.t === "welcome")
      ),
      true,
    );
  } finally {
    stop();
    globalThis.WebSocket = original;
  }
});

Deno.test("scorer identities use authenticated handles and generation, never display names", () => {
  const lobby = new Lobby();
  publicRequest(lobby, a, "/api/lobby", { action: "join", robot: "duck1", displayName: "NotGitHub", userLogin: "spoof" }, null);
  const identity = lobby.scorerIdentities().duck1;
  assertEquals(identity.userId, a.id);
  assertEquals(identity.handle, a.login);
  assertEquals(identity.generation, lobby.assignments().duck1);
  publicRequest(lobby, a, "/api/lobby", { action: "observe" }, null);
  assertEquals(lobby.scorerIdentities(), {});
});

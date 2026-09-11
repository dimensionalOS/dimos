import {
  ControlFrameReader,
  decodeDataFrame,
  decodeDatagram,
  encodeControlFrame,
  type Msg,
  PROTOCOL_VERSION,
} from "../vendor/dimos/web/shared/protocol.ts";
import { Registry, type ViewerPeer } from "../vendor/dimos/web/relay/registry.ts";
import type { ViewerSink } from "../vendor/dimos/web/relay/forward.ts";
import {
  type Identity,
  MAX_CONTROL_BYTES,
  MAX_FRAME_BYTES,
  pack,
  unpack,
} from "../shared/publicWire.ts";
import { Lobby, NameError, ROBOTS } from "./lobby.ts";

type Result = { status: number; body: unknown };
const result = (body: unknown, status = 200): Result => ({ status, body });

/** Identity is supplied by our authenticated edge, never by a browser message. */
export function publicRequest(
  lobby: Lobby,
  user: Identity | null,
  path: string,
  body: any,
  preview: unknown,
): Result {
  if (path === "/healthz") {
    return result({ online: true, players: Object.keys(lobby.assignments()).length });
  }
  if (path === "/api/preview") {
    return preview ? result(preview) : result({ error: "World starting" }, 503);
  }
  if (path === "/api/lobby" && body === null) return result(lobby.state());
  if (!user || !/^\d+$/.test(user.id)) return result({ error: "Sign in with GitHub." }, 401);
  if (path === "/api/session") {
    const p = lobby.find(typeof body?.token === "string" ? body.token : null);
    return p?.userId === user.id
      ? result({ v: PROTOCOL_VERSION })
      : result({ error: "Session expired" }, 401);
  }
  if (path !== "/api/lobby") return result({ error: "Not found" }, 404);
  if (
    !body || typeof body !== "object" ||
    !["create", "status", "join", "observe"].includes(body.action)
  ) return result({ error: "Invalid action" }, 400);
  const p = lobby.forUser(user.id);
  if (!p) return result({ error: "The world is busy. Try again shortly." }, 503);
  p.userLogin = user.login;
  // Tokens only identify transport sessions; an account cannot claim another
  // account's participant by putting its token in a lobby request.
  if (body.action === "join" || body.action === "observe") {
    if (
      body.action === "observe" && !p.connections.size &&
      [...lobby.participants.values()].filter((other) =>
          other !== p && other.role === "observe" && other.connections.size > 0
        ).length >= 20
    ) {
      return result(
        { error: "All 20 spectator places are in use. Please try again shortly." },
        503,
      );
    }
    if (
      body.robot !== undefined && (typeof body.robot !== "string" || !ROBOTS.includes(body.robot))
    ) return result({ error: "Choose a valid duck." }, 400);
    try {
      if (!lobby.change(p, body.action, body.robot, body.displayName)) {
        return result({
          error: "That duck is unavailable. Choose another duck or watch the world.",
        }, 409);
      }
    } catch (error) {
      if (error instanceof NameError) return result({ error: error.message }, 422);
      throw error;
    }
  }
  return result({ ...lobby.state(p), token: p.token });
}

function frameSink(write: (bytes: Uint8Array) => void, kick: (reason: string) => void): ViewerSink {
  return {
    kick,
    openStream: async () => ({ write: async (bytes) => write(bytes), abort: async () => {} }),
    sendFrame(bytes) {
      let aborted = false;
      let started = false;
      let value = bytes;
      const done = Promise.resolve().then(() => {
        started = true;
        if (aborted) throw new Error("Frame aborted");
        write(value);
      });
      return {
        done,
        get aborted() {
          return aborted;
        },
        abort() {
          aborted = true;
        },
        supersede(next) {
          if (started || aborted) return false;
          value = next;
          return true;
        },
      };
    },
  };
}

/** An internal read-only subscriber supplies the public landing preview. */
export function createPreview(lobby: Lobby): { read: () => unknown; close: () => void } {
  let snapshot: unknown = null;
  let stopped = false;
  const receive = (bytes: Uint8Array) => {
    const frame = decodeDataFrame(bytes);
    if (frame.header.ch === "world_state") {
      snapshot = JSON.parse(new TextDecoder().decode(frame.payload));
    }
  };
  const peer: ViewerPeer = {
    id: -1,
    watched: null,
    subs: new Set(),
    policies: new Map(),
    greeted: false,
    sink: frameSink(receive, () => {}),
    sendMsg: () => {},
  };
  lobby.addViewer(peer);
  const send = (msg: Msg) => Registry.prototype.onViewerMsg.call(lobby, peer, msg, () => {});
  send({ t: "hello", v: PROTOCOL_VERSION, role: "viewer" });
  const timer = setInterval(() => {
    if (stopped || peer.watched === "world") return;
    send({ t: "watch", robotId: "world" });
    if (peer.watched === "world") send({ t: "sub", ch: "world_state" });
  }, 1000);
  return {
    read: () => snapshot,
    close() {
      stopped = true;
      clearInterval(timer);
      lobby.viewerClosed(peer);
    },
  };
}

/** Outbound-only public connection; MuJoCo, ownership and DimOS remain on Omarchy. */
export function startPublicBridge(lobby: Lobby, url: string, secret: string): () => void {
  if (!url.startsWith("wss://") || !/^[a-f0-9]{64}$/.test(secret)) {
    throw new Error("Invalid public bridge configuration");
  }
  const preview = createPreview(lobby);
  let stopped = false;
  let socket: WebSocket | null = null;
  let timer: ReturnType<typeof setTimeout> | undefined;
  let nextId = -2;
  let retryMs = 1000;
  const connect = () => {
    if (stopped) return;
    const ws = new WebSocket(url, ["microduck-origin", secret]);
    socket = ws;
    ws.binaryType = "arraybuffer";
    const peers = new Map<string, { peer: ViewerPeer; reader: ControlFrameReader }>();
    // The registry offers the same immutable bytes to all subscribers. Batch
    // recipient IDs so the Omarchy uplink sends each camera/world frame once.
    const frames = new Map<Uint8Array, { kind: 0 | 1; to: Set<string> }>();
    let pendingBytes = 0;
    let flushQueued = false;
    let seen = Date.now();
    const send = (data: string | Uint8Array) => {
      if (ws.readyState !== WebSocket.OPEN || ws.bufferedAmount + data.length > MAX_FRAME_BYTES) {
        ws.close(1011, "Origin backpressure");
        throw new Error("Origin backpressure");
      }
      ws.send(data);
    };
    const drop = (id: string, notify = true) => {
      const p = peers.get(id);
      if (!p) return;
      peers.delete(id);
      lobby.viewerClosed(p.peer);
      if (notify && ws.readyState === WebSocket.OPEN) send(JSON.stringify({ t: "close", id }));
    };
    const queue = (id: string, kind: 0 | 1, bytes: Uint8Array) => {
      if (!peers.has(id)) throw new Error("Viewer disconnected");
      let frame = frames.get(bytes);
      if (!frame) {
        frame = { kind, to: new Set() };
        frames.set(bytes, frame);
        pendingBytes += bytes.length;
      }
      frame.to.add(id);
      if (pendingBytes > MAX_FRAME_BYTES) {
        ws.close(1011, "Origin queue full");
        throw new Error("Origin queue full");
      }
      if (flushQueued) return;
      flushQueued = true;
      queueMicrotask(() => {
        flushQueued = false;
        try {
          for (const [bytes, frame] of frames) {
            const to = [...frame.to].filter((id) => peers.has(id));
            if (to.length) send(pack({ kind: frame.kind, to }, bytes));
          }
        } catch {
          ws.close(1011, "Origin write failed");
        } finally {
          frames.clear();
          pendingBytes = 0;
        }
      });
    };
    const heartbeat = setInterval(() => {
      if (Date.now() - seen > 15_000) ws.close(1011, "Edge timeout");
      else if (ws.readyState === WebSocket.OPEN) {
        try {
          send(JSON.stringify({ t: "ping" }));
        } catch { /* reconnect */ }
      }
    }, 5000);
    ws.addEventListener("open", () => {
      retryMs = 1000;
      console.log("[public] edge connected");
    });
    ws.addEventListener("message", (event) => {
      seen = Date.now();
      try {
        if (typeof event.data === "string") {
          if (event.data.length > MAX_CONTROL_BYTES) throw new Error("Edge control too large");
          const msg = JSON.parse(event.data);
          if (msg.t === "open") {
            const p = lobby.find(msg.ticket);
            if (
              !p || p.userId !== msg.user?.id || peers.has(msg.id) || peers.size >= 26 ||
              (p.role === "observe" &&
                [...peers.values()].filter(({ peer }) =>
                    lobby.peers.get(peer.id)?.role === "observe"
                  ).length >= 20)
            ) {
              send(JSON.stringify({ t: "close", id: msg.id }));
              return;
            }
            const peer: ViewerPeer = {
              id: nextId--,
              watched: null,
              subs: new Set(),
              policies: new Map(),
              greeted: false,
              sendMsg: (reply) => queue(msg.id, 0, encodeControlFrame(reply)),
              sink: frameSink((bytes) => queue(msg.id, 1, bytes), () => drop(msg.id)),
            };
            peers.set(msg.id, { peer, reader: new ControlFrameReader() });
            lobby.attach(peer, p, () => drop(msg.id));
            lobby.addViewer(peer);
          } else if (msg.t === "close") drop(msg.id, false);
          else if (msg.t === "revoke") lobby.revokeUser(msg.userId);
          else if (msg.t === "http") {
            const answer = publicRequest(lobby, msg.user, msg.path, msg.body, preview.read());
            send(JSON.stringify({ t: "reply", id: msg.id, ...answer }));
          }
          return;
        }
        const { header, payload } = unpack(new Uint8Array(event.data));
        const id = header.to[0];
        const p = peers.get(id);
        if (!p) return;
        if (
          header.to.length !== 1 || !header.sentAt || Math.abs(Date.now() - header.sentAt) > 1500 ||
          payload.length > MAX_CONTROL_BYTES
        ) {
          drop(id);
          return;
        }
        // Browser bytes remain untrusted after authentication. A malformed
        // client's protocol stream must not disconnect the shared origin.
        try {
          const messages = header.kind === 0
            ? p.reader.push(payload)
            : header.kind === 2
            ? [decodeDatagram(payload)].filter((m): m is Msg => m !== null)
            : [];
          for (const msg of messages) {
            if (!lobby.onViewerMsg(p.peer, msg, p.peer.sendMsg)) {
              drop(id);
              break;
            }
          }
        } catch {
          drop(id);
        }
        send(JSON.stringify({ t: "received", id }));
      } catch {
        ws.close(1011, "Invalid edge message");
      }
    });
    ws.addEventListener("error", () => {
      try {
        ws.close();
      } catch { /* closed */ }
    });
    ws.addEventListener("close", () => {
      clearInterval(heartbeat);
      for (const id of peers.keys()) drop(id, false);
      frames.clear();
      if (!stopped) {
        timer = setTimeout(connect, retryMs);
        retryMs = Math.min(retryMs * 2, 10_000);
      }
    }, { once: true });
  };
  connect();
  return () => {
    stopped = true;
    clearTimeout(timer);
    preview.close();
    socket?.close(1000, "World stopped");
  };
}

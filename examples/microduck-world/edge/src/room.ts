import { DurableObject } from "cloudflare:workers";
import {
  type Identity,
  MAX_CLIENTS,
  MAX_CONTROL_BYTES,
  MAX_FRAME_BYTES,
  MAX_PENDING_DOWNLOADS,
  MAX_PENDING_UPLOADS,
  pack,
  serverFrame,
  unpack,
} from "../../shared/publicWire.ts";

type Client = {
  socket: WebSocket;
  user: Identity;
  pending: Map<number, { bytes: number; at: number }>;
  bytes: number;
  seq: number;
  count: number;
  window: number;
  expires: number;
  clockOffset: number | null;
  uploads: { bytes: number; at: number }[];
};
type Reply = { status: number; body: string };

/** Transport only: the stock DimOS registry on Omarchy authorizes every command. */
export class MatchRelay extends DurableObject<Env> {
  private host: WebSocket | null = null;
  private hostSeen = 0;
  private uploadBytes = 0;
  private clients = new Map<string, Client>();
  private pending = new Map<
    string,
    { resolve: (r: Reply) => void; timer: ReturnType<typeof setTimeout> }
  >();

  constructor(ctx: DurableObjectState, env: Env) {
    super(ctx, env);
    // Continuous simulation traffic already keeps this object active. Close peers
    // on host/credit timeout; no stale command is retained for a later reconnect.
    setInterval(() => this.sweep(), 5000);
  }

  async fetch(request: Request): Promise<Response> {
    const url = new URL(request.url);
    if (request.headers.get("upgrade")?.toLowerCase() !== "websocket") {
      return new Response("WebSocket required", { status: 426 });
    }
    if (url.pathname === "/host") {
      if (this.host) {
        return new Response("Origin already connected", { status: 409 });
      }
      const pair = new WebSocketPair();
      this.host = pair[1];
      this.hostSeen = Date.now();
      pair[1].binaryType = "arraybuffer";
      pair[1].accept();
      pair[1].addEventListener(
        "message",
        (e) => this.fromHost(pair[1], e.data),
      );
      pair[1].addEventListener("close", () => this.hostClosed(pair[1]));
      pair[1].addEventListener("error", () => this.hostClosed(pair[1]));
      return new Response(null, {
        status: 101,
        webSocket: pair[0],
        headers: { "sec-websocket-protocol": "microduck-origin" },
      });
    }
    if (url.pathname !== "/viewer") {
      return new Response("Not found", { status: 404 });
    }
    if (!this.host || Date.now() - this.hostSeen > 15_000) {
      return new Response("World offline", { status: 503 });
    }
    const connectingUser = request.headers.get("x-user-id");
    for (const [oldId, old] of this.clients) {
      if (old.user.id === connectingUser) {
        this.closeClient(oldId, "Replaced by your new connection");
      }
    }
    if (this.clients.size >= MAX_CLIENTS) {
      return new Response("The world is full", { status: 503 });
    }
    const user = {
      id: request.headers.get("x-user-id") ?? "",
      login: request.headers.get("x-user-login") ?? "",
    };
    const expires = Number(request.headers.get("x-session-expires"));
    if (
      !/^\d+$/.test(user.id) || !Number.isFinite(expires) ||
      expires <= Date.now()
    ) return new Response("Sign in again", { status: 401 });
    const id = crypto.randomUUID();
    const pair = new WebSocketPair();
    pair[1].binaryType = "arraybuffer";
    pair[1].accept();
    const client: Client = {
      socket: pair[1],
      user,
      pending: new Map(),
      bytes: 0,
      seq: 0,
      count: 0,
      window: Date.now(),
      expires,
      clockOffset: null,
      uploads: [],
    };
    this.clients.set(id, client);
    pair[1].addEventListener("message", (e) => this.fromClient(id, e.data));
    pair[1].addEventListener("close", () => this.closeClient(id));
    pair[1].addEventListener("error", () => this.closeClient(id));
    this.host.send(
      JSON.stringify({
        t: "open",
        id,
        user,
        ticket: url.searchParams.get("ticket") ?? "",
      }),
    );
    return new Response(null, { status: 101, webSocket: pair[0] });
  }

  async http(
    user: Identity | null,
    path: string,
    bodyJson = "null",
  ): Promise<Reply> {
    if (!this.host || Date.now() - this.hostSeen > 15_000) {
      return {
        status: 503,
        body: JSON.stringify({
          error: "The world is offline. Please try again shortly.",
        }),
      };
    }
    if (this.pending.size >= 64) {
      return {
        status: 503,
        body: JSON.stringify({ error: "The world is busy." }),
      };
    }
    const id = crypto.randomUUID();
    const message = JSON.stringify({
      t: "http",
      id,
      user,
      path,
      body: JSON.parse(bodyJson),
    });
    if (message.length > MAX_CONTROL_BYTES) {
      return {
        status: 413,
        body: JSON.stringify({ error: "Request too large" }),
      };
    }
    return new Promise((resolve) => {
      const timer = setTimeout(() => {
        this.pending.delete(id);
        resolve({
          status: 504,
          body: JSON.stringify({ error: "The world did not respond." }),
        });
      }, 7000);
      this.pending.set(id, { resolve, timer });
      try {
        this.host!.send(message);
      } catch {
        this.hostClosed(this.host!);
      }
    });
  }

  async revoke(userId: string): Promise<void> {
    for (const [id, client] of this.clients) {
      if (client.user.id === userId) this.closeClient(id, "Session revoked");
    }
    this.host?.send(JSON.stringify({ t: "revoke", userId }));
  }

  private fromHost(socket: WebSocket, data: string | ArrayBuffer): void {
    if (socket !== this.host) return;
    this.hostSeen = Date.now();
    try {
      if (typeof data === "string") {
        if (data.length > MAX_FRAME_BYTES) {
          throw new Error("Origin message too large");
        }
        const msg = JSON.parse(data);
        if (msg.t === "ping") socket.send(JSON.stringify({ t: "pong" }));
        else if (msg.t === "received") {
          const item = this.clients.get(msg.id)?.uploads.shift();
          if (item) this.uploadBytes -= item.bytes;
        } else if (msg.t === "reply") {
          const p = this.pending.get(msg.id);
          if (
            p && Number.isInteger(msg.status) && msg.status >= 200 &&
            msg.status <= 599
          ) {
            clearTimeout(p.timer);
            this.pending.delete(msg.id);
            p.resolve({ status: msg.status, body: JSON.stringify(msg.body) });
          }
        } else if (msg.t === "close") {
          this.closeClient(msg.id, "Session changed. Reconnecting.");
        }
        return;
      }
      const { header, payload } = unpack(new Uint8Array(data));
      if (header.kind !== 0 && header.kind !== 1) {
        throw new Error("Invalid origin frame kind");
      }
      for (const id of header.to) {
        const client = this.clients.get(id);
        if (!client) continue;
        // Credit is released only when the browser receives a message. Large or
        // suspended viewers cannot create an unbounded Cloudflare send queue.
        if (
          client.pending.size >= MAX_PENDING_DOWNLOADS ||
          client.bytes + payload.length > MAX_FRAME_BYTES
        ) {
          this.closeClient(id, "Connection too slow. Reconnect to resume.");
          continue;
        }
        const seq = ++client.seq;
        client.pending.set(seq, { bytes: payload.length, at: Date.now() });
        client.bytes += payload.length;
        client.socket.send(serverFrame(header.kind, seq, payload));
      }
    } catch {
      this.hostClosed(socket);
    }
  }

  private fromClient(id: string, data: string | ArrayBuffer): void {
    const c = this.clients.get(id);
    if (!c) return;
    try {
      if (Date.now() - c.window > 1000) {
        c.window = Date.now();
        c.count = 0;
      }
      if (++c.count > 300) throw new Error("Message rate exceeded");
      if (typeof data === "string") {
        if (data.length > 100) throw new Error("Invalid acknowledgment");
        const ack = JSON.parse(data);
        if (ack.t !== "ack" || !Number.isInteger(ack.n)) {
          throw new Error("Invalid acknowledgment");
        }
        const pending = c.pending.get(ack.n);
        if (pending) {
          c.bytes -= pending.bytes;
          c.pending.delete(ack.n);
        }
        return;
      }
      if (!this.host || c.expires <= Date.now()) {
        throw new Error("Session expired");
      }
      const bytes = new Uint8Array(data);
      if (
        bytes.length < 9 || bytes.length > MAX_CONTROL_BYTES + 9 ||
        ![0, 2].includes(bytes[0])
      ) throw new Error("Invalid control");
      const sentAt = new DataView(
        bytes.buffer,
        bytes.byteOffset,
        bytes.byteLength,
      ).getFloat64(1);
      if (!Number.isFinite(sentAt)) throw new Error("Invalid client clock");
      c.clockOffset ??= Date.now() - sentAt;
      if (Math.abs(Date.now() - sentAt - c.clockOffset) > 1500) {
        throw new Error("Stale control");
      }
      if (
        c.uploads.length >= MAX_PENDING_UPLOADS ||
        this.uploadBytes + bytes.length > MAX_FRAME_BYTES
      ) throw new Error("Origin backpressure");
      c.uploads.push({ bytes: bytes.length, at: Date.now() });
      this.uploadBytes += bytes.length;
      this.host.send(
        pack(
          { to: [id], kind: bytes[0] as 0 | 2, sentAt: Date.now() },
          bytes.subarray(9),
        ),
      );
    } catch (error) {
      console.log("viewer_rejected", { reason: error instanceof Error ? error.message : "Invalid client message" });
      this.closeClient(
        id,
        "Session expired or connection delayed. Please reconnect.",
      );
    }
  }

  private closeClient(id: string, reason = "Disconnected"): void {
    const c = this.clients.get(id);
    if (!c) return;
    console.log("viewer_closed", { reason, pendingFrames: c.pending.size, pendingBytes: c.bytes, uploads: c.uploads.length, messagesThisSecond: c.count });
    this.clients.delete(id);
    this.uploadBytes -= c.uploads.reduce((sum, item) => sum + item.bytes, 0);
    try {
      c.socket.close(1000, reason);
    } catch { /* already closed */ }
    try {
      this.host?.send(JSON.stringify({ t: "close", id }));
    } catch { /* origin closed */ }
  }
  private hostClosed(host: WebSocket): void {
    if (host !== this.host) return;
    this.host = null;
    try {
      host.close(1011, "Origin disconnected");
    } catch { /* already closed */ }
    for (const id of this.clients.keys()) {
      this.closeClient(id, "World reconnecting");
    }
    for (const p of this.pending.values()) {
      clearTimeout(p.timer);
      p.resolve({
        status: 503,
        body: JSON.stringify({ error: "World reconnecting" }),
      });
    }
    this.pending.clear();
  }
  private sweep(): void {
    if (this.host && Date.now() - this.hostSeen > 15_000) {
      this.hostClosed(this.host);
    }
    for (const [id, c] of this.clients) {
      if (
        c.expires <= Date.now() || c.uploads.some((p) =>
          Date.now() - p.at > 1500
        ) || [...c.pending.values()].some((p) => Date.now() - p.at > 5000)
      ) this.closeClient(id, "Session expired or viewer stalled");
    }
  }
}

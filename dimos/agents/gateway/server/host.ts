import type { ObjectDeps, ServeObject } from "./objects/base.ts";
import type { Host, Link } from "@earendil-works/chord";
import type { Principal } from "../auth/principal.ts";
import type { Server } from "node:http";
import { DatabaseSync } from "node:sqlite";
import { WebSocketServer } from "ws";
import { chmod, lstat, mkdir, unlink } from "node:fs/promises";
import { createHost, webSocketLink } from "@earendil-works/chord";
import { createServer } from "node:http";
import { dirname } from "node:path";
import { principalKey, principalOf } from "../auth/principal.ts";
import { reachable } from "../../shared/socket.ts";
import { serveConversation } from "./objects/conversation.ts";
import { serveGateway } from "./objects/gateway.ts";
import { serveInventory } from "./objects/inventory.ts";
import { serveLifecycle } from "./objects/lifecycle.ts";
import { serveRoom } from "./objects/room.ts";
import { serveStore } from "./objects/store.ts";

/** The family's one list: every served Chord type, the shared local ones first. */
const OBJECTS: readonly ServeObject[] = [
  serveStore,
  serveRoom,
  serveLifecycle,
  serveGateway,
  serveConversation,
  serveInventory,
];

type Serving = {
  readonly host: Host<Principal>;
  readonly sockets: WebSocketServer;
  /** Every accepted client link, so shutdown can close each with the reason `stopping`, which stops its redialing. */
  readonly links: Set<Link>;
};

export class GatewayRunning extends Error {
  readonly socketPath: string;

  constructor(socketPath: string) {
    super(`A gateway already answers on ${socketPath}`);
    this.socketPath = socketPath;
  }
}

const SQLITE_BUSY = 5;

/** An exclusive SQLite lock on `path`, which the OS releases when this process dies; null while another holds it. */
function takeLock(path: string): DatabaseSync | null {
  const db = new DatabaseSync(path);
  try {
    db.exec("PRAGMA locking_mode = EXCLUSIVE; BEGIN EXCLUSIVE");
    return db;
  } catch (error) {
    db.close();
    if (error instanceof Error && "errcode" in error && error.errcode === SQLITE_BUSY) return null;
    throw error;
  }
}

function listen(server: Server, socketPath: string): Promise<void> {
  return new Promise((resolve, reject) => {
    const failed = (error: Error) => {
      server.off("listening", ready);
      reject(error);
    };
    const ready = () => {
      server.off("error", failed);
      resolve();
    };
    server.once("error", failed);
    server.once("listening", ready);
    server.listen(socketPath);
  });
}

/**
 * The socket clients connect to and the Chord host behind it. Binding is still the first thing the gateway does (a
 * second gateway finds the lock held or the first answering and exits before it opens the store); an upgrade before
 * `serve` is refused with 503, so a client retries with its backoff.
 */
export class GatewayServer {
  private readonly http: Server;
  private readonly socketPath: string;
  /** `<socket>.lock`, held from bind until the socket file is removed, so one gateway at a time owns the path. */
  private readonly lock: DatabaseSync;
  private serving: Serving | null = null;
  private closing = false;

  private constructor(http: Server, socketPath: string, lock: DatabaseSync) {
    this.http = http;
    this.socketPath = socketPath;
    this.lock = lock;
  }

  /**
   * Takes the lock, then binds, before anything else opens: directory 0700, socket 0600. A held lock or a socket that
   * answers throws GatewayRunning; a socket file nobody answers on is removed.
   */
  static async bind(socketPath: string): Promise<GatewayServer> {
    await mkdir(dirname(socketPath), { recursive: true, mode: 0o700 });
    const lock = takeLock(`${socketPath}.lock`);
    if (lock === null) throw new GatewayRunning(socketPath);
    const http = createServer((_request, response) => {
      response.writeHead(404, { "content-type": "text/plain" });
      response.end("This socket speaks WebSocket only.\n");
    });
    const gateway = new GatewayServer(http, socketPath, lock);
    http.on("upgrade", (_request, socket) => {
      if (gateway.serving !== null) return;
      socket.write("HTTP/1.1 503 Service Unavailable\r\nConnection: close\r\n\r\n");
      socket.destroy();
    });
    try {
      await listen(http, socketPath).catch(async (error: unknown) => {
        if (!(error instanceof Error && "code" in error && error.code === "EADDRINUSE")) throw error;
        if (!(await lstat(socketPath)).isSocket()) throw error;
        if (await reachable(socketPath, 500)) throw new GatewayRunning(socketPath);
        await unlink(socketPath);
        await listen(http, socketPath);
      });
      await chmod(socketPath, 0o600);
    } catch (error) {
      lock.close();
      throw error;
    }
    return gateway;
  }

  /** Registers the object implementations, waits for their resources, then accepts client connections. */
  async serve(deps: ObjectDeps): Promise<void> {
    const host = createHost<Principal>({
      name: "gateway",
      self: { kind: "gateway" },
      principalKey,
      onError: deps.report,
    });
    try {
      for (const serve of OBJECTS) serve(host, deps);
      await host.ready();
      if (this.closing) throw new Error("The gateway closed while its objects were starting");
    } catch (error) {
      await host.close();
      throw error;
    }
    const sockets = new WebSocketServer({ server: this.http });
    const links = new Set<Link>();
    sockets.on("connection", (socket, request) => {
      // As Chord's acceptWebSockets: wrapped at once so frames sent meanwhile queue; clients may not send packed frames.
      const link = webSocketLink(socket, { compress: true, acceptPacked: false });
      const principal = principalOf(request, "socket", new Map());
      if (principal === undefined) {
        link.close("unauthorized");
        return;
      }
      links.add(link);
      link.onClose(() => links.delete(link));
      host.accept(link, { principal, kind: "client" });
    });
    this.serving = { host, sockets, links };
  }

  /**
   * Stops accepting, tells every client `stopping`, disposes every object and its resources, removes the socket file,
   * then releases the lock.
   */
  async close(): Promise<void> {
    if (this.closing) return;
    this.closing = true;
    const serving = this.serving;
    this.serving = null;
    if (serving !== null) {
      for (const link of serving.links) link.close("stopping");
      await serving.host.close();
      for (const client of serving.sockets.clients) client.terminate();
      serving.sockets.close();
    }
    this.http.closeAllConnections();
    await new Promise<void>((resolve) => this.http.close(() => resolve()));
    await unlink(this.socketPath).catch(() => {});
    this.lock.close();
  }
}

import type {
  RelayInfo,
  TransportDeps,
  WebTransportLike,
} from "../../vendor/dimos/web/sdk/src/transport.ts";
import { clientFrame, MAX_CONTROL_BYTES, MAX_FRAME_BYTES } from "../../shared/publicWire.ts";

/** Adapt the stock DimOS Session to WSS; retain its codecs, stores and teleop state machine. */
export class PublicTransport implements WebTransportLike {
  readonly ready: Promise<void>;
  readonly closed: Promise<{ reason: string }>;
  readonly incomingUnidirectionalStreams: ReadableStream<ReadableStream<Uint8Array>>;
  readonly datagrams: { writable: WritableStream<Uint8Array> };
  private socket: WebSocket;
  private control!: ReadableStreamDefaultController<Uint8Array>;
  private incoming!: ReadableStreamDefaultController<ReadableStream<Uint8Array>>;
  private stream: ReadableStream<Uint8Array>;
  private ended = false;
  private finish!: (value: { reason: string }) => void;

  constructor(info: RelayInfo, onReplaced: () => void = () => {}) {
    this.socket = new WebSocket(info.wtUrl);
    this.socket.binaryType = "arraybuffer";
    this.stream = new ReadableStream({
      start: (c) => {
        this.control = c;
      },
    });
    this.incomingUnidirectionalStreams = new ReadableStream({
      start: (c) => {
        this.incoming = c;
      },
    });
    this.datagrams = { writable: new WritableStream({ write: (bytes) => this.send(2, bytes) }) };
    this.closed = new Promise((resolve) => {
      this.finish = resolve;
    });
    this.ready = new Promise((resolve, reject) => {
      this.socket.addEventListener("open", () => resolve(), { once: true });
      this.socket.addEventListener("error", () => reject(new Error("Public connection failed")), {
        once: true,
      });
      this.socket.addEventListener("close", () => reject(new Error("Public connection closed")), {
        once: true,
      });
    });
    this.socket.addEventListener("close", (e) => {
      if (e.reason === "Replaced by your new connection") onReplaced();
      this.end(e.reason || "Connection interrupted");
    });
    this.socket.addEventListener("message", (e) => {
      try {
        if (!(e.data instanceof ArrayBuffer)) throw new Error("Invalid server frame");
        const bytes = new Uint8Array(e.data);
        if (bytes.length < 5 || bytes.length > MAX_FRAME_BYTES + 5) {
          throw new Error("Invalid frame size");
        }
        const seq = new DataView(bytes.buffer).getUint32(1);
        const data = bytes.slice(5);
        if (bytes[0] === 0) {
          if ((this.control.desiredSize ?? 0) < -64) throw new Error("Control reader stalled");
          this.control.enqueue(data);
        } else if (bytes[0] === 1) {
          if ((this.incoming.desiredSize ?? 0) < -16) throw new Error("Data reader stalled");
          this.incoming.enqueue(
            new ReadableStream({
              start(c) {
                c.enqueue(data);
                c.close();
              },
            }),
          );
        } else throw new Error("Invalid frame kind");
        this.socket.send(JSON.stringify({ t: "ack", n: seq }));
      } catch {
        this.close();
      }
    });
  }

  private send(kind: 0 | 2, bytes: Uint8Array): void {
    if (
      this.socket.readyState !== WebSocket.OPEN || this.socket.bufferedAmount > MAX_CONTROL_BYTES
    ) {
      this.close();
      throw new Error("Public connection is backpressured");
    }
    this.socket.send(clientFrame(kind, bytes));
  }

  async createBidirectionalStream() {
    await this.ready;
    return {
      readable: this.stream,
      writable: new WritableStream<Uint8Array>({ write: (b) => this.send(0, b) }),
    };
  }

  close(): void {
    this.socket.close(1000, "Session ended");
    this.end("Session ended");
  }
  private end(reason: string): void {
    if (this.ended) return;
    this.ended = true;
    try {
      this.control.close();
    } catch { /* SDK canceled its reader */ }
    try {
      this.incoming.close();
    } catch { /* SDK canceled its reader */ }
    this.finish({ reason });
  }
}

export const transportForSession = (onReplaced: () => void): TransportDeps => ({
  createWebTransport: (info) =>
    info.wtUrl.startsWith("wss:") || info.wtUrl.startsWith("ws:")
      ? new PublicTransport(info, onReplaced)
      : new WebTransport(info.wtUrl, {
        serverCertificateHashes: [{
          algorithm: "sha-256",
          value: Uint8Array.from(atob(info.certHash), (c) => c.charCodeAt(0)),
        }],
      }),
});

export const publicTransport = transportForSession(() => {});

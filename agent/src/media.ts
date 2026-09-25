import {
  connect,
  type ConnectOptions,
  type Session,
  type Slot,
  type TransportDeps,
} from "@dimos/sdk";

export async function nodeTransport(): Promise<TransportDeps> {
  const { WebTransport } = await import("rwebtransport");
  return {
    createWebTransport: (info) =>
      new WebTransport(new URL("/viewer", info.wtUrl).href, {
        serverCertificateHashes: info.certHash
          ? [
              {
                algorithm: "sha-256",
                value: Uint8Array.from(Buffer.from(info.certHash, "base64"))
                  .buffer,
              },
            ]
          : [],
      }),
  };
}
export type MediaSelection = Pick<ConnectOptions, "url" | "robot">;
export type Decoder<T> = (slot: Slot) => T;
export interface MediaLease<T> {
  current(): T | undefined;
  close(): void;
}

/** Only render consumers open connections; the last consumer closes the SDK session. */
export class MediaPool {
  private readonly connections = new Map<
    string,
    { session: Session; consumers: number }
  >();
  private pending: Promise<TransportDeps> | undefined;
  private closed = false;
  constructor(
    private readonly transport: () => Promise<TransportDeps> = nodeTransport,
    private readonly options: Omit<ConnectOptions, "url" | "robot"> = {},
    private readonly create: typeof connect = connect,
  ) {}
  async acquire<T>(
    selection: MediaSelection,
    channel: string,
    decode: Decoder<T>,
    changed: () => void,
  ): Promise<MediaLease<T>> {
    const key = JSON.stringify([selection.url, selection.robot]);
    this.pending ??= this.transport();
    const deps = await this.pending;
    if (this.closed) throw new Error("Media pool closed");
    let entry = this.connections.get(key);
    if (!entry) {
      entry = {
        session: this.create({ ...this.options, ...selection }, deps),
        consumers: 0,
      };
      this.connections.set(key, entry);
    }
    entry.consumers++;
    const held = entry;
    const interest = held.session.subscribe(channel, changed);
    const frames = held.session.store.subscribe(channel, changed);
    let closed = false;
    return {
      current: () => {
        const slot = held.session.store.get(channel);
        return slot ? decode(slot) : undefined;
      },
      close: () => {
        if (closed) return;
        closed = true;
        frames();
        interest();
        if (--held.consumers === 0) {
          held.session.close();
          this.connections.delete(key);
        }
      },
    };
  }
  close(): void {
    this.closed = true;
    for (const { session } of this.connections.values()) session.close();
    this.connections.clear();
  }
}

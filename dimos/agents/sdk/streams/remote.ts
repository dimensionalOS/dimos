import type { Session } from "@dimos/sdk";
import type { Channel, StreamRef } from "#dimos/agents/shared/protocol/streams.ts";
import type { FrameRecord, FrameSource } from "#dimos/agents/sdk/streams/base.ts";
import type { RelayEndpoint } from "#dimos/agents/sdk/relay.ts";
import { epochMs } from "#dimos/agents/shared/protocol/time.ts";
import { openRelay, watchFrames } from "#dimos/agents/sdk/relay.ts";

export class RemoteFrames implements FrameSource {
  private endpoint: RelayEndpoint | null = null;
  private session: Session | null = null;
  private streams: readonly StreamRef[] = [];
  private wanted: ReadonlySet<Channel> = new Set();
  private readonly watches = new Map<
    Channel,
    { stream: StreamRef; stop: () => void; count: number; last: number | null }
  >();
  private readonly listeners = new Set<(record: FrameRecord) => void>();
  private readonly timer: NodeJS.Timeout;
  private closed = false;

  private readonly report: (error: unknown) => void;

  constructor(report: (error: unknown) => void = () => {}) {
    this.report = report;
    this.timer = setInterval(() => {
      const now = Date.now();
      const counts = [...this.watches].map(([channel, watch]) => {
        const hz = watch.count;
        watch.count = 0;
        return { channel, hz, ageS: watch.last === null ? null : (now - watch.last) / 1000 };
      });
      this.emit({ kind: "rates", sample: { at: epochMs(now), counts } });
    }, 1_000).unref();
  }

  configure(endpoint: RelayEndpoint | null, streams: readonly StreamRef[] = []): void {
    if (this.closed) return;
    this.streams = streams;
    if (this.endpoint?.url === endpoint?.url && this.endpoint?.source === endpoint?.source) {
      this.sync();
      return;
    }
    this.endpoint = endpoint === null ? null : { ...endpoint };
    for (const watch of this.watches.values()) watch.stop();
    this.watches.clear();
    this.session?.close();
    this.session = null;
    if (this.endpoint !== null) void this.open(this.endpoint);
  }

  want(channels: ReadonlySet<Channel>): void {
    this.wanted = new Set(channels);
    this.sync();
  }

  onRecord(listener: (record: FrameRecord) => void): () => void {
    this.listeners.add(listener);
    return () => {
      this.listeners.delete(listener);
    };
  }

  close(): void {
    if (this.closed) return;
    this.configure(null);
    this.closed = true;
    clearInterval(this.timer);
    this.listeners.clear();
  }

  private async open(endpoint: RelayEndpoint): Promise<void> {
    try {
      const session = await openRelay(endpoint);
      if (this.closed || this.endpoint !== endpoint) {
        session.close();
        return;
      }
      this.session = session;
      this.sync();
    } catch (error) {
      if (!this.closed && this.endpoint === endpoint) this.report(error);
    }
  }

  private sync(): void {
    const session = this.session;
    if (session === null || this.closed) return;
    const desired = new Map(this.streams.filter((s) => this.wanted.has(s.channel)).map((s) => [s.channel, s]));
    for (const [channel, watch] of this.watches) {
      if (JSON.stringify(desired.get(channel)) === JSON.stringify(watch.stream)) continue;
      this.watches.delete(channel);
      watch.stop();
    }
    for (const [channel, stream] of desired) {
      if (this.watches.has(channel)) continue;
      const watch = { stream, stop: () => {}, count: 0, last: null as number | null };
      this.watches.set(channel, watch);
      watch.stop = watchFrames(session, stream, (frame) => {
        if (this.watches.get(channel) !== watch) return;
        if (frame.kind !== "error") {
          watch.count++;
          watch.last = Date.now();
        }
        this.emit(frame);
      });
    }
  }

  private emit(record: FrameRecord): void {
    for (const listener of this.listeners) {
      try {
        listener(record);
      } catch (error) {
        this.report(error);
      }
    }
  }
}

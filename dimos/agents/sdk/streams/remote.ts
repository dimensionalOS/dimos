import type { ChannelSpec, CostmapValue, Session, Slot, VoxelsValue } from "@dimos/sdk";
import type { Channel } from "#dimos/agents/shared/protocol/streams.ts";
import type { FrameRecord, FrameSource } from "#dimos/agents/sdk/streams/base.ts";
import type { EpochMs } from "#dimos/agents/shared/protocol/time.ts";
import type { RelayEndpoint } from "#dimos/agents/sdk/relay.ts";
import { inflateCostmap, inflateVoxels } from "@dimos/sdk";
import sharp from "sharp";
import { epochMs } from "#dimos/agents/shared/protocol/time.ts";
import { channelFor, openRelay } from "#dimos/agents/sdk/relay.ts";

type Watching = { spec: ChannelSpec; epoch: number; stop: () => void; converting: boolean; version: number };

export async function toFrame(channel: Channel, spec: ChannelSpec, slot: Slot, at: EpochMs): Promise<FrameRecord> {
  try {
    switch (spec.encoding) {
      case "jpeg.v1": {
        if (!(slot.value instanceof Uint8Array)) throw new Error("jpeg slot has no bytes");
        const { width, height } = await sharp(slot.value).metadata();
        return { kind: "image", channel, at, width, height, jpeg: slot.value };
      }
      case "costmap.zlib.v1": {
        const value = slot.value as CostmapValue;
        const cells = await inflateCostmap(value);
        return {
          kind: "grid",
          channel,
          at,
          width: value.w,
          height: value.h,
          resolution: value.res,
          origin: value.origin,
          cells: new Int8Array(cells.buffer, cells.byteOffset, cells.byteLength),
        };
      }
      case "voxels.zlib.v1":
        return { kind: "cloud", channel, at, xyz: await inflateVoxels(slot.value as VoxelsValue) };
      default:
        if (slot.value === undefined) throw new Error(`unsupported preview encoding ${spec.encoding}`);
        return { kind: "value", channel, at, text: (slot.preview ?? "").slice(0, 200) };
    }
  } catch (error) {
    return { kind: "error", channel, text: String(error) };
  }
}

export class RemoteFrames implements FrameSource {
  private endpoint: RelayEndpoint | null = null;
  private session: Session | null = null;
  private stopStatus = () => {};
  private wanted: ReadonlySet<Channel> = new Set();
  private readonly watches = new Map<Channel, Watching>();
  private readonly listeners = new Set<(record: FrameRecord) => void>();
  private readonly timer: NodeJS.Timeout;
  private closed = false;
  private readonly report: (error: unknown) => void;

  constructor(report: (error: unknown) => void = () => {}) {
    this.report = report;
    this.timer = setInterval(() => {
      const session = this.session;
      const counts =
        session === null
          ? []
          : [...this.watches].map(([channel, watch]) => {
              const stats = session.store.getUiSnapshot(watch.spec.ch).stats;
              return { channel, hz: stats.hz, ageS: stats.ageMs === null ? null : stats.ageMs / 1000 };
            });
      this.emit({ kind: "rates", sample: { at: epochMs(Date.now()), counts } });
    }, 1_000).unref();
  }

  configure(endpoint: RelayEndpoint | null): void {
    if (this.closed || (this.endpoint?.url === endpoint?.url && this.endpoint?.robot === endpoint?.robot)) return;
    this.endpoint = endpoint === null ? null : { ...endpoint };
    this.stopStatus();
    this.clearWatches();
    this.session?.close();
    this.session = null;
    if (this.endpoint !== null) void this.open(this.endpoint);
  }

  want(channels: ReadonlySet<Channel>): void {
    if (this.closed) return;
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
      this.stopStatus = session.status.subscribe(() => this.sync());
      this.sync();
    } catch (error) {
      if (!this.closed && this.endpoint === endpoint) this.report(error);
    }
  }

  private clearWatches(): void {
    for (const watch of this.watches.values()) watch.stop();
    this.watches.clear();
  }

  private sync(): void {
    const session = this.session;
    if (session === null) return;
    const status = session.status.get();
    for (const [channel, watch] of this.watches) {
      if (this.wanted.has(channel) && status.manifest !== null && watch.epoch === status.epoch) continue;
      this.watches.delete(channel);
      watch.stop();
    }
    for (const channel of this.wanted) {
      if (this.watches.has(channel)) continue;
      try {
        const spec = channelFor(session, channel);
        if (spec === undefined) continue;
        const watch: Watching = { spec, epoch: status.epoch, stop: () => {}, converting: false, version: 0 };
        this.watches.set(channel, watch);
        watch.stop = session.subscribe(spec.ch, ({ slot, stats }) => {
          if (stats.decodeFailing) {
            this.emit({ kind: "error", channel, text: `failed to decode ${spec.encoding}` });
            return;
          }
          if (slot === null) {
            watch.version = 0;
            return;
          }
          void this.drain(session, channel, watch);
        });
        void this.drain(session, channel, watch);
      } catch (error) {
        this.emit({ kind: "error", channel, text: String(error) });
      }
    }
  }

  private async drain(session: Session, channel: Channel, watch: Watching): Promise<void> {
    if (watch.converting) return;
    watch.converting = true;
    try {
      while (this.watches.get(channel) === watch) {
        const slot = session.store.get(watch.spec.ch);
        if (slot === null || slot.version === watch.version) break;
        watch.version = slot.version;
        const record = await toFrame(channel, watch.spec, slot, epochMs(Date.now()));
        if (this.watches.get(channel) === watch && session.store.get(watch.spec.ch) !== null) this.emit(record);
      }
    } finally {
      watch.converting = false;
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

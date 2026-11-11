import type { Session, TopicSample, TopicMedia, TopicSubscription } from "@dimos/sdk";
import type { Inventory } from "#dimos/agents/shared/protocol/runtime/inventory.ts";
import type { StreamRef } from "#dimos/agents/shared/protocol/streams.ts";
import { StreamRefSchema } from "#dimos/agents/shared/protocol/streams.ts";
import { connect, decodeTopicMedia, TopicMediaDecoder, projectCbor, isLcmSchema, lcmDecoder } from "@dimos/sdk";
import { WebTransport, quicheLoaded } from "@fails-components/webtransport";
import { Value } from "typebox/value";
import { Type } from "typebox";
import { inspect } from "node:util";
import type { FrameRecord } from "#dimos/agents/sdk/streams/base.ts";
import { until } from "#dimos/agents/sdk/until.ts";
import { epochMs } from "#dimos/agents/shared/protocol/time.ts";

export type RelayEndpoint = NonNullable<Inventory["environment"]["relay"]>;

export async function openRelay(endpoint: RelayEndpoint): Promise<Session> {
  await quicheLoaded;
  return connect(
    { url: endpoint.url, topicSource: endpoint.source },
    {
      createWebTransport(info) {
        const transport = new WebTransport(`${info.wtUrl}/viewer`, {
          requireUnreliable: true,
          serverCertificateHashes: info.certHash
            ? [{ algorithm: "sha-256", value: Buffer.from(info.certHash, "base64") }]
            : [],
        });
        let closed = false;
        return {
          ready: transport.ready,
          closed: transport.closed,
          close() {
            if (closed) return;
            closed = true;
            transport.close();
            // webtransport 1.6.11 leaves its UDP client alive on a local close.
            const hook = transport.sessionint.closeHook;
            transport.sessionint.closeHook = null;
            hook?.();
          },
          createBidirectionalStream: () => transport.createBidirectionalStream(),
          incomingUnidirectionalStreams: transport.incomingUnidirectionalStreams,
          datagrams: { writable: transport.datagrams.createWritable() },
        };
      },
    },
  );
}

/** Race a request against its caller without retaining abort listeners. */
export async function abortable<T>(promise: Promise<T>, signal: AbortSignal): Promise<T> {
  signal.throwIfAborted();
  let abort = () => {};
  try {
    return await Promise.race([
      promise,
      new Promise<never>((_, reject) => {
        abort = () => reject(signal.reason);
        signal.addEventListener("abort", abort, { once: true });
      }),
    ]);
  } finally {
    signal.removeEventListener("abort", abort);
  }
}

const samples = new WeakMap<Session, Map<string, TopicSample>>();

export async function topicStreams(
  session: Session,
  inventory: Inventory,
  signal: AbortSignal,
): Promise<readonly StreamRef[]> {
  const source = inventory.environment.relay?.source;
  if (source === undefined) return [];
  const owner = await until(
    {
      get value() {
        return session.topics.sources();
      },
      subscribe: (cb) => session.topics.onSources(cb),
    },
    (sources) => sources.find((candidate) => candidate.source === source),
    signal,
  );
  const streams: StreamRef[] = [];
  const wired = new Map(
    inventory.runtime.streams
      .filter((s) => s.direction === "out" || s.direction === "inout")
      .map((s) => [s.channel, s]),
  );
  let cursor = "";
  do {
    const page = await abortable(session.topics.list({ source, cursor }), signal);
    for (const descriptor of page.entries) {
      const port = wired.get(descriptor.key);
      if (port === undefined) continue;
      const preferred =
        descriptor.type === "sensor_msgs.Image"
          ? "image.jpeg.balanced.v1"
          : descriptor.type === "sensor_msgs.PointCloud2"
            ? "cloud.q16.balanced.v1"
            : "raw";
      streams.push(
        Value.Parse(StreamRefSchema, {
          channel: descriptor.key,
          encoding: descriptor.encoding,
          profile: owner.profiles.includes(preferred) ? preferred : "raw",
          lcm: port.lcm,
          source: { module: port.module, name: port.name, direction: port.direction, type: port.type },
        }),
      );
    }
    cursor = page.cursor;
  } while (cursor);
  if (session.topics.sources().find((s) => s.source === source)?.gen !== owner.gen)
    throw new Error("Topic source changed during discovery");
  const live = new Set<string>(streams.map((s) => s.channel));
  for (const key of samples.get(session)?.keys() ?? []) if (!live.has(key)) samples.get(session)?.delete(key);
  return streams;
}

const Grid = Type.Object({
  info: Type.Object({
    width: Type.Number(),
    height: Type.Number(),
    resolution: Type.Number(),
    origin: Type.Object({
      position: Type.Object({ x: Type.Number(), y: Type.Number() }),
      orientation: Type.Object({ x: Type.Number(), y: Type.Number(), z: Type.Number(), w: Type.Number() }),
    }),
  }),
  data: Type.Unknown(),
});

function mediaFrame(stream: StreamRef, sample: TopicSample, media: TopicMedia): FrameRecord {
  const base = { channel: stream.channel, at: epochMs(sample.ingressTs * 1000) };
  if (media.kind === "image")
    return { ...base, kind: "image", width: media.width, height: media.height, jpeg: media.data };
  if (media.kind === "cloud") return { ...base, kind: "cloud", xyz: media.positions };
  throw new Error("Unsupported depth preview");
}

export function topicValue(stream: StreamRef, sample: TopicSample): { value: unknown; preview: string } {
  if (stream.encoding === "application/cbor") {
    const projected = projectCbor(sample.payload);
    if (projected.kind === "opaque") throw new Error(projected.reason);
    return { value: projected.value, preview: inspect(projected.value, { depth: 3, maxArrayLength: 8 }).slice(0, 512) };
  }
  if (stream.encoding === "application/x-lcm" && isLcmSchema(stream.lcm)) {
    const decoded = lcmDecoder(stream.lcm)(sample.payload, {
      ch: sample.key,
      seq: sample.sequence,
      ts: sample.ingressTs,
      delivery: "latest",
    });
    return { value: decoded.value, preview: decoded.preview ?? "" };
  }
  throw new Error(`Unsupported topic encoding ${stream.encoding}`);
}

export async function toFrame(stream: StreamRef, sample: TopicSample, maxChars = 200): Promise<FrameRecord> {
  if ((stream.profile ?? "raw") !== "raw") return mediaFrame(stream, sample, await decodeTopicMedia(sample));
  const { value, preview } = topicValue(stream, sample);
  const base = { channel: stream.channel, at: epochMs(sample.ingressTs * 1000) };
  if (stream.source?.type === "OccupancyGrid") {
    const { info, data } = Value.Parse(Grid, value);
    if (!(data instanceof Int8Array) && !(data instanceof Uint8Array) && !Array.isArray(data))
      throw new Error("Invalid grid cells");
    const q = info.origin.orientation;
    return {
      ...base,
      kind: "grid",
      width: info.width,
      height: info.height,
      resolution: info.resolution,
      origin: [
        info.origin.position.x,
        info.origin.position.y,
        Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)),
      ],
      cells: Int8Array.from(data),
    };
  }
  return { ...base, kind: "value", text: preview.slice(0, maxChars) };
}

export function watchFrames(session: Session, stream: StreamRef, emit: (record: FrameRecord) => void): () => void {
  const error = (error: unknown) => emit({ kind: "error", channel: stream.channel, text: String(error) });
  const media = new TopicMediaDecoder((value, sample) => emit(mediaFrame(stream, sample, value)), error);
  const sub = session.topics.subscribe(stream.channel, { profile: stream.profile ?? "raw" }, (sample) => {
    if ((stream.profile ?? "raw") !== "raw") media.push(sample);
    else
      void toFrame(stream, sample).then((frame) => {
        if (sub.status().phase === "ready") emit(frame);
      }, error);
  });
  const stopStatus = sub.onStatus((status) => {
    if (status.phase !== "ready") media.reset();
    if (status.phase === "closed" || status.phase === "rejected") error(status.error ?? status.phase);
  });
  return () => {
    stopStatus();
    media.close();
    sub.close();
  };
}

export const LATEST_REFRESH_MS = 2_000;

export async function readTopic(
  session: Session,
  stream: StreamRef,
  mode: "latest" | "next",
  signal: AbortSignal,
): Promise<{ sample: TopicSample; stale: boolean }> {
  const cache = samples.get(session) ?? new Map<string, TopicSample>();
  samples.set(session, cache);
  const previous = cache.get(stream.channel);
  const refresh = AbortSignal.timeout(LATEST_REFRESH_MS);
  let stopStatus = () => {};
  let sub: TopicSubscription | undefined;
  try {
    const sample = await abortable(
      new Promise<TopicSample>((resolve, reject) => {
        sub = session.topics.subscribe(stream.channel, { profile: stream.profile ?? "raw" }, resolve);
        stopStatus = sub.onStatus((status) => {
          if (status.phase === "closed" || status.phase === "rejected") reject(status.error ?? new Error(status.phase));
        });
      }),
      mode === "latest" && previous !== undefined ? AbortSignal.any([signal, refresh]) : signal,
    );
    cache.set(stream.channel, sample);
    return { sample, stale: false };
  } catch (error) {
    signal.throwIfAborted();
    if (
      mode === "latest" &&
      refresh.aborted &&
      previous !== undefined &&
      session.topics.sources().some((source) => source.source === previous.source && source.gen === previous.generation)
    )
      return { sample: previous, stale: true };
    throw error;
  } finally {
    stopStatus();
    sub?.close();
  }
}

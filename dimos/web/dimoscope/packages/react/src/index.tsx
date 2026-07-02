// @dimos/react — thin React bindings over @dimos/web. For high-rate topics prefer
// useTopicRef (no re-render; read in a rAF loop) over useTopicLatest.
import {
  createContext,
  type MutableRefObject,
  type ReactNode,
  useContext,
  useEffect,
  useMemo,
  useRef,
  useState,
  useSyncExternalStore,
} from "react";
import { createDimosClient, type DimosClient, selectMediaChannel } from "@dimos/web";
import type {
  CommandInfo,
  MediaChannel,
  MediaKind,
  MessageMeta,
  Status,
  TopicInfo,
  TopicStats,
  TransportCaps,
  VideoMeta,
} from "@dimos/web";

/** One selectable transport/server: a label + a thunk that builds a connected client. */
export interface ServerOpt {
  id: string;
  label: string;
  connect: () => Promise<DimosClient>;
  /** Gateway WS URL, when this server is a plain gateway transport (lets a panel open a sibling
   *  client to the same gateway). Absent for webrtc. */
  url?: string;
  /** Media-plane config: WebRTC gateway + which kinds it serves. Absent → the Image-topic floor. */
  media?: { gatewayUrl?: string; kinds?: readonly MediaKind[] };
}

interface DimosCtx {
  client: DimosClient | null;
  status: Status;
  servers: ServerOpt[];
  activeId: string | null;
  setActiveId: (id: string) => void;
}
const Ctx = createContext<DimosCtx>({
  client: null,
  status: "connecting",
  servers: [],
  activeId: null,
  setActiveId: () => {},
});

/** Provides a DimosClient to the tree. Pass `servers` to expose a switcher (the client is rebuilt on
 *  change); a bare `url` is a one-server shorthand. */
export function DimosProvider({
  servers,
  url,
  children,
}: {
  servers?: ServerOpt[];
  url?: string;
  children: ReactNode;
}) {
  const list = useMemo<ServerOpt[]>(
    () =>
      servers ?? (url
        ? [{
          id: url,
          label: url,
          connect: () => {
            const c = createDimosClient();
            return c.connect(url).then(() => c);
          },
        }]
        : []),
    [servers, url],
  );
  const [activeId, setActiveId] = useState<string | null>(list[0]?.id ?? null);
  const [client, setClient] = useState<DimosClient | null>(null);
  const [status, setStatus] = useState<Status>("connecting");

  useEffect(() => {
    const opt = list.find((s) => s.id === activeId);
    if (!opt) return;
    let alive = true;
    let c: DimosClient | undefined;
    let unsub: (() => void) | undefined;
    setClient(null);
    setStatus("connecting");
    opt
      .connect()
      .then((cl) => {
        if (!alive) {
          cl.close();
          return;
        }
        c = cl;
        setClient(cl);
        setStatus(cl.status);
        unsub = cl.onStatus(setStatus);
      })
      .catch((e) => {
        if (alive) {
          console.error("[dimos] connect failed:", e);
          setStatus("closed");
        }
      });
    // Detach the old status listener BEFORE closing, so the outgoing client's
    // "closed" can't clobber the incoming client's status on a server switch.
    return () => {
      alive = false;
      unsub?.();
      c?.close();
    };
  }, [list, activeId]);

  return (
    <Ctx.Provider value={{ client, status, servers: list, activeId, setActiveId }}>
      {children}
    </Ctx.Provider>
  );
}

export const useDimos = () => useContext(Ctx);
export const useDimosClient = () => useContext(Ctx).client;
export const useStatus = () => useContext(Ctx).status;
/** The transport switcher: list of servers + the active id + a setter. */
export const useServers = () => {
  const { servers, activeId, setActiveId } = useContext(Ctx);
  return { servers, activeId, setActiveId };
};

/** The active transport's capabilities (on-demand, discovery, qos) — for QoS-aware UI. */
export function useCaps(): TransportCaps | null {
  return useDimosClient()?.caps ?? null;
}

// Stable empty snapshots for the disconnected case — useSyncExternalStore compares snapshots by
// reference, so returning a fresh `[]` each call would loop.
const NO_TOPICS: TopicInfo[] = [];
const NO_COMMANDS: CommandInfo[] = [];

/**
 * Bridge a client-level push store (an `onX` subscribe + a snapshot getter) into React via
 * useSyncExternalStore. Null-safe (client is null before connect / mid server-switch). The snapshot is
 * cached in a closure updated on each notification (stable ref between changes) and re-read at subscribe
 * time to catch anything that changed between render and subscription.
 */
function useClientStore<T>(
  read: (c: DimosClient) => T,
  listen: (c: DimosClient, onChange: () => void) => () => void,
  empty: T,
): T {
  const client = useDimosClient();
  const [subscribe, getSnapshot] = useMemo(() => {
    if (!client) return [(_: () => void) => () => {}, () => empty] as const;
    let snap = read(client);
    return [
      (onChange: () => void) => {
        const unsub = listen(client, () => {
          snap = read(client);
          onChange();
        });
        snap = read(client); // catch anything that changed before we subscribed
        return unsub;
      },
      () => snap,
    ] as const;
    // read/listen/empty are stable across renders — only re-key on the client.
  }, [client]);
  return useSyncExternalStore(subscribe, getSnapshot);
}

/** Live list of discovered topics. */
export function useTopics(): TopicInfo[] {
  return useClientStore((c) => c.listTopics(), (c, cb) => c.onTopics(cb), NO_TOPICS);
}

/** Latest message for a topic; re-renders on each delivery (use maxHz for high-rate). */
export function useTopicLatest<T = unknown>(
  topic: string | null,
  opts?: { maxHz?: number },
): { data?: T; meta?: MessageMeta } {
  const client = useDimosClient();
  const [state, setState] = useState<{ data?: T; meta?: MessageMeta }>({});
  const maxHz = opts?.maxHz;
  useEffect(() => {
    setState({}); // clear stale data from the previous topic on switch
    if (!client || !topic) return;
    const t = client.topic(topic);
    if (maxHz) t.setQos({ maxHz });
    const sub = t.subscribeLatest((m) => setState({ data: m.data as T, meta: m.meta }));
    return () => sub.unsubscribe();
  }, [client, topic, maxHz]);
  return state;
}

/** A ref updated on every message WITHOUT re-rendering — read it in a rAF loop (canvas). */
export function useTopicRef<T = unknown>(
  topic: string | null,
): MutableRefObject<{ data?: T; meta?: MessageMeta }> {
  const client = useDimosClient();
  const ref = useRef<{ data?: T; meta?: MessageMeta }>({});
  useEffect(() => {
    ref.current = {}; // clear stale data from the previous topic on switch
    if (!client || !topic) return;
    const sub = client.topic(topic).subscribe((m) => {
      ref.current = { data: m.data as T, meta: m.meta };
    });
    return () => sub.unsubscribe();
  }, [client, topic]);
  return ref;
}

/** Live stats for a topic, polled + PASSIVE: `Topic.stats()` reads a rolling window without
 *  subscribing, so it never forces the gateway to stream a topic just to measure it. */
export function useTopicStats(topic: string | null, pollMs = 500): TopicStats | null {
  const client = useDimosClient();
  const [stats, setStats] = useState<TopicStats | null>(null);
  useEffect(() => {
    if (!client || !topic) return;
    const t = client.topic(topic);
    const id = setInterval(() => setStats(t.stats()), pollMs);
    return () => clearInterval(id);
  }, [client, topic, pollMs]);
  return stats;
}

// Live message feed: keep only the latest message in a closure and flush one compact row at
// `displayHz`, so a high-rate stream never re-renders React per message.

const _replacer = (_k: string, v: unknown) =>
  typeof v === "bigint"
    ? v.toString()
    : ArrayBuffer.isView(v)
    ? `<${(v as ArrayBufferView).byteLength} bytes>` // any TypedArray/DataView → never stringify pixels
    : v;
/** A short one-line preview of a message value (Uint8Array/bigint-safe — never serializes pixels). */
export function jsonPreview(v: unknown, max = 120): string {
  return JSON.stringify(v, _replacer)?.slice(0, max) ?? "";
}
/** A pretty, size-capped JSON dump of a message value (Uint8Array/bigint-safe). */
export function jsonPretty(v: unknown, max = 4000): string {
  return JSON.stringify(v, _replacer, 2)?.slice(0, max) ?? "";
}

/** One compact feed row — enough to render a line without retaining the (possibly MB-sized) payload. */
export interface FeedRow {
  /** Stable, monotonic per-subscription id — use as the React key so the list doesn't remount each flush. */
  id: number;
  recvTs: number;
  sizeBytes: number;
  seq?: number;
  preview: string;
}
export interface TopicFeed {
  /** Oldest→newest sampled rows (≤ maxRows). */
  rows: FeedRow[];
  /** The most recent full message (for an expand-to-JSON view). */
  latest?: { data: unknown; meta: MessageMeta };
  /** Source-sequence gap over the delivered stream, %: rises when the gateway sheds (or a client
   *  rate-limit drops) messages. null when the topic carries no numeric seq. */
  lossPct: number | null;
}

/** Subscribe to a topic and expose a rolling, display-throttled feed of its recent messages. */
export function useTopicFeed(
  topic: string | null,
  opts?: { maxRows?: number; displayHz?: number },
): TopicFeed {
  const client = useDimosClient();
  const maxRows = opts?.maxRows ?? 50;
  const displayHz = opts?.displayHz ?? 12;
  const [feed, setFeed] = useState<TopicFeed>({ rows: [], lossPct: null });
  useEffect(() => {
    setFeed({ rows: [], lossPct: null }); // clear on topic switch
    if (!client || !topic) return;
    const ring: FeedRow[] = [];
    let nextId = 0; // stable, monotonic row id → a steady React key (no per-flush remount)
    let latest: { data: unknown; meta: MessageMeta } | undefined;
    let pending = false; // a new message arrived since the last flush
    let seqMin: number | undefined;
    let seqMax: number | undefined;
    let seqCount = 0;
    let lastSeq: number | undefined;
    const sub = client.topic(topic).subscribe((m) => {
      latest = { data: m.data, meta: m.meta };
      pending = true;
      const s = m.meta.seq;
      if (s != null) {
        if (lastSeq !== undefined && s < lastSeq) {
          // seq went backwards → the publisher restarted; start a fresh gap window
          // (WS is ordered, so a decrease is a restart, not a reorder).
          seqMin = s;
          seqMax = s;
          seqCount = 1;
        } else {
          if (seqMin === undefined || s < seqMin) seqMin = s;
          if (seqMax === undefined || s > seqMax) seqMax = s;
          seqCount++;
        }
        lastSeq = s;
      }
    });
    const id = setInterval(() => {
      if (!pending || !latest) return; // nothing new → no re-render (silent topics stay quiet)
      ring.push({
        id: nextId++,
        recvTs: latest.meta.recvTs,
        sizeBytes: latest.meta.sizeBytes,
        seq: latest.meta.seq,
        preview: jsonPreview(latest.data, 140),
      });
      while (ring.length > maxRows) ring.shift();
      pending = false;
      const span = seqMin != null && seqMax != null ? seqMax - seqMin + 1 : 0;
      const lossPct = span > 1 ? Math.max(0, (1 - seqCount / span) * 100) : null;
      setFeed({ rows: ring.slice(), latest, lossPct });
    }, Math.max(1, Math.round(1000 / displayHz)));
    return () => {
      sub.unsubscribe();
      clearInterval(id);
    };
  }, [client, topic, maxRows, displayHz]);
  return feed;
}

interface ImageMsg {
  width: number;
  height: number;
  encoding: string;
  is_bigendian: number;
  step: number;
  data: Uint8Array;
}

/** Convert a raw (uncompressed) Image to RGBA ImageData. Returns null for jpeg/unknown. */
function rawToRGBA(img: ImageMsg): ImageData | null {
  const { width: w, height: h, data } = img;
  const enc = (img.encoding || "").toLowerCase();
  if (!w || !h || !data?.length) return null;
  const out = new Uint8ClampedArray(w * h * 4);
  const ch = enc === "mono8" || enc === "8uc1" ? 1 : enc === "rgba8" || enc === "bgra8" ? 4 : 3;
  const step = img.step && img.step >= w * ch ? img.step : w * ch;
  const bgr = enc === "bgr8" || enc === "bgra8";
  for (let y = 0; y < h; y++) {
    for (let x = 0; x < w; x++) {
      const i = y * step + x * ch;
      const o = (y * w + x) * 4;
      if (ch === 1) {
        const v = data[i];
        out[o] = v;
        out[o + 1] = v;
        out[o + 2] = v;
        out[o + 3] = 255;
      } else {
        out[o] = data[i + (bgr ? 2 : 0)];
        out[o + 1] = data[i + 1];
        out[o + 2] = data[i + (bgr ? 0 : 2)];
        out[o + 3] = ch === 4 ? data[i + 3] : 255;
      }
    }
  }
  return new ImageData(out, w, h);
}

export interface ImageInfo {
  width: number;
  height: number;
  encoding: string;
  fps: number;
}

/**
 * Subscribe a sensor_msgs.Image topic and paint it into a canvas (jpeg + raw
 * rgb8/bgr8/mono8/rgba8/bgra8). Returns a ref to attach to a <canvas>, plus live
 * info (dims/encoding/fps). Decodes off the React render path (rAF + on-message).
 */
export function useImageTopic(topic: string | null, opts?: { maxFps?: number }) {
  const client = useDimosClient();
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const [info, setInfo] = useState<ImageInfo | null>(null);
  const maxFps = opts?.maxFps;
  useEffect(() => {
    setInfo(null);
    const cvs = canvasRef.current;
    if (!client || !topic || !cvs) return;
    const ctx = cvs.getContext("2d");
    if (!ctx) return;
    let latest: ImageMsg | null = null;
    let drawn: ImageMsg | null = null;
    let decoding = false;
    let raf = 0;
    let lastDraw = 0;
    let frames = 0;
    let fpsT = 0;
    const minMs = maxFps ? 1000 / maxFps : 0;

    const sub = client.topic(topic).subscribe((m) => {
      latest = m.data as ImageMsg;
    });
    const fit = (w: number, h: number) => {
      if (cvs.width !== w || cvs.height !== h) {
        cvs.width = w;
        cvs.height = h;
      }
    };
    const tick = (img: ImageMsg, ts: number) => {
      if (fpsT === 0) fpsT = ts; // start the fps window at the first frame, not page load
      frames++;
      if (ts - fpsT > 500) {
        setInfo({
          width: img.width,
          height: img.height,
          encoding: img.encoding,
          fps: Math.round((frames * 1000) / (ts - fpsT)),
        });
        frames = 0;
        fpsT = ts;
      }
    };
    const loop = (ts: number) => {
      raf = requestAnimationFrame(loop);
      const img = latest;
      if (!img || img === drawn) return;
      if (minMs && ts - lastDraw < minMs) return;
      const enc = (img.encoding || "").toLowerCase();
      if (enc === "jpeg" || enc === "jpg") {
        if (decoding) return;
        decoding = true;
        drawn = img;
        lastDraw = ts;
        createImageBitmap(new Blob([img.data as BlobPart], { type: "image/jpeg" }))
          .then((bmp) => {
            fit(bmp.width, bmp.height);
            ctx.drawImage(bmp, 0, 0);
            bmp.close?.();
            tick(img, ts);
          })
          .catch(() => {})
          .finally(() => {
            decoding = false;
          });
      } else {
        const id = rawToRGBA(img);
        if (!id) return;
        drawn = img;
        lastDraw = ts;
        fit(img.width, img.height);
        ctx.putImageData(id, 0, 0);
        tick(img, ts);
      }
    };
    raf = requestAnimationFrame(loop);
    return () => {
      cancelAnimationFrame(raf);
      sub.unsubscribe();
    };
  }, [client, topic, maxFps]);
  return { canvasRef, info };
}

/** Forced media mode (the topbar toggle). "auto" = best available, falling back to jpeg. */
export type MediaMode = "auto" | "jpeg" | "webrtc" | "webcodecs";
const MODE_PREFER: Record<MediaMode, MediaKind[]> = {
  auto: ["webrtc", "webcodecs", "jpeg"], // proven default first; webcodecs/jpeg as fallback
  jpeg: ["jpeg"],
  webrtc: ["webrtc", "jpeg"],
  webcodecs: ["webcodecs", "jpeg"],
};

/**
 * Subscribe a camera topic via the negotiated media plane: a <video> fed by a WebRTC MediaStream
 * when available, else the JPEG Image-topic floor on a <canvas>. Caller attaches both refs and shows
 * the one matching `kind`. One MediaChannel per hook instance.
 */
export function useVideo(
  topic: string | null,
  opts?: {
    mode?: MediaMode;
    /** Per-frame tap (frames channels only — webcodecs/jpeg, not webrtc): called after the canvas
     *  draw, before the frame is closed. The seam for in-browser CV/overlays. Must not retain the
     *  frame past the call (it's closed right after). */
    onFrame?: (
      frame: VideoFrame | ImageBitmap,
      meta: VideoMeta,
      ctx: CanvasRenderingContext2D,
    ) => void;
  },
) {
  const { client, servers, activeId } = useContext(Ctx);
  const mode = opts?.mode ?? "auto";
  const videoRef = useRef<HTMLVideoElement>(null);
  const canvasRef = useRef<HTMLCanvasElement>(null);
  // Keep the tap in a ref so swapping it never resubscribes the media channel (the heavy effect
  // below keys on transport/topic/mode only).
  const onFrameRef = useRef(opts?.onFrame);
  onFrameRef.current = opts?.onFrame;
  const [kind, setKind] = useState<"stream" | "frames">("frames");
  const [label, setLabel] = useState<string>();
  const [active, setActive] = useState<MediaKind>("jpeg"); // the kind actually negotiated

  const mediaCfg = servers.find((s) => s.id === activeId)?.media;
  const gatewayUrl = mediaCfg?.gatewayUrl;
  const serverMedia = mediaCfg?.kinds;

  useEffect(() => {
    if (!client || !topic) return;
    let alive = true;
    let current: MediaChannel | null = null;
    const kindOf = (ch: MediaChannel): MediaKind =>
      ch.caps.codec === "jpeg" ? "jpeg" : ch.caps.output === "stream" ? "webrtc" : "webcodecs";

    const wire = (ch: MediaChannel): Promise<void> => {
      current = ch;
      setKind(ch.caps.output);
      setLabel(ch.label);
      setActive(kindOf(ch));
      if (ch.caps.output === "stream") {
        ch.onStream((id, stream) => {
          if (alive && id === topic && videoRef.current) videoRef.current.srcObject = stream;
        });
      } else {
        ch.onFrame((id, frame, m) => {
          if (!alive || id !== topic) return;
          const cvs = canvasRef.current;
          const ctx = cvs?.getContext("2d");
          if (!cvs || !ctx) return;
          // ImageBitmap exposes .width/.height; VideoFrame (WebCodecs) exposes displayWidth/Height
          // (its .width is undefined → would zero the canvas). Handle both.
          const vf = frame as VideoFrame;
          const fw = vf.displayWidth || (frame as ImageBitmap).width;
          const fh = vf.displayHeight || (frame as ImageBitmap).height;
          if (fw && fh && (cvs.width !== fw || cvs.height !== fh)) {
            cvs.width = fw;
            cvs.height = fh;
          }
          ctx.drawImage(frame as CanvasImageSource, 0, 0);
          onFrameRef.current?.(frame, m, ctx); // CV/overlay seam — draw on ctx / copy frame, then we close it
          (frame as ImageBitmap).close?.();
        });
      }
      return ch.connect().then(() => {
        if (alive) ch.subscribe(topic);
      });
    };

    // Try the preferred channel; on connect failure drop to the jpeg floor at runtime so the
    // camera never goes dark just because the media gateway hiccuped.
    const primary = selectMediaChannel({
      client,
      gatewayUrl,
      serverMedia,
      prefer: MODE_PREFER[mode],
    });
    wire(primary).catch(() => {
      if (!alive || primary.caps.codec === "jpeg") return;
      primary.close();
      wire(selectMediaChannel({ client, prefer: ["jpeg"] })).catch(() => {});
    });

    return () => {
      alive = false;
      current?.unsubscribe(topic);
      current?.close();
    };
  }, [client, topic, mode, gatewayUrl, serverMedia]);

  return { kind, videoRef, canvasRef, label, active, requested: mode, offered: serverMedia };
}

/**
 * Subscribe to any topic by name from the UI — exercises the on-demand machinery (topic.ts is
 * ref-counted: a pin = a subscribe, an unpin = an unsubscribe). Works for not-yet-discovered topics.
 */
export function SubscribeBar() {
  const topics = useTopics();
  const [pins, setPins] = useState<string[]>([]);
  const [text, setText] = useState("");
  const add = (name: string) => {
    const t = name.trim();
    if (t && !pins.includes(t)) setPins((p) => [...p, t]);
    setText("");
  };
  const inputStyle: React.CSSProperties = {
    flex: 1,
    background: "var(--bg)",
    color: "var(--text)",
    border: "1px solid var(--line)",
    borderRadius: 6,
    padding: "4px 8px",
    font: "12px ui-monospace, monospace",
  };
  return (
    <div className="panel">
      <div className="panel-title">Subscribe · any topic</div>
      <form
        onSubmit={(e) => {
          e.preventDefault();
          add(text);
        }}
        style={{ display: "flex", gap: 6 }}
      >
        <input
          list="dimos-topic-names"
          placeholder="/dimos/…  subscribe by name"
          value={text}
          onChange={(e) => setText(e.target.value)}
          style={inputStyle}
        />
        <datalist id="dimos-topic-names">
          {topics.map((t) => <option key={t.topic} value={t.topic} />)}
        </datalist>
        <button type="submit" className="tab">+</button>
      </form>
      <div style={{ marginTop: 8, display: "flex", flexDirection: "column", gap: 4 }}>
        {pins.map((p) => (
          <PinnedTopic
            key={p}
            topic={p}
            onRemove={() => setPins((ps) => ps.filter((x) => x !== p))}
          />
        ))}
      </div>
    </div>
  );
}

function PinnedTopic({ topic, onRemove }: { topic: string; onRemove: () => void }) {
  const { data, meta } = useTopicLatest<unknown>(topic, { maxHz: 4 });
  const stats = useTopicStats(topic);
  const live = !!meta;
  // short one-line preview of the latest value.
  const preview = data == null
    ? ""
    : JSON.stringify(data, (_k, v) =>
      typeof v === "bigint" ? v.toString() : v instanceof Uint8Array ? `<${v.length}B>` : v)?.slice(
        0,
        120,
      );
  return (
    <div
      style={{
        display: "flex",
        flexDirection: "column",
        gap: 2,
        font: "12px ui-monospace, monospace",
        padding: "5px 7px",
        border: "1px solid var(--line)",
        borderRadius: 6,
        background: "var(--bg)",
      }}
    >
      <div style={{ display: "flex", alignItems: "center", gap: 8 }}>
        <span
          style={{ color: live ? "#4ade80" : "var(--muted)" }}
          title={live ? "receiving messages" : "subscribed — waiting for the first message"}
        >
          ●
        </span>
        <span
          style={{ flex: 1, color: "var(--text)", overflow: "hidden", textOverflow: "ellipsis" }}
        >
          {topic}
        </span>
        <span className="muted small" title="message rate · size of the last message">
          {live ? `${stats?.hz ?? 0} Hz · ${meta!.sizeBytes} B` : "waiting…"}
        </span>
        <button
          type="button"
          className="tab"
          onClick={onRemove}
          title="unsubscribe"
          style={{ padding: "0 7px" }}
        >
          ×
        </button>
      </div>
      {live && preview && (
        <div
          className="muted"
          style={{
            paddingLeft: 16,
            whiteSpace: "nowrap",
            overflow: "hidden",
            textOverflow: "ellipsis",
            opacity: 0.85,
          }}
          title={preview}
        >
          {preview}
        </div>
      )}
    </div>
  );
}

/** Safe teleop helpers — gateway clamps + runs a TTL/deadman watchdog. */
export function useTeleop() {
  const client = useDimosClient();
  return useMemo(
    () => ({
      drive: (linearX: number, angularZ: number, ttlMs = 400) =>
        client?.teleop(linearX, angularZ, ttlMs),
      stop: () => client?.stop(),
    }),
    [client],
  );
}

/** Invoke a whitelisted dimos `@rpc` command, e.g. `useRpc().call("GO2Connection", "standup")`. */
export function useRpc() {
  const client = useDimosClient();
  return useMemo(
    () => ({
      call: <T = unknown>(target: string, method: string, ...args: unknown[]): Promise<T> =>
        client
          ? client.call<T>(target, method, ...args)
          : Promise.reject(new Error("not connected")),
    }),
    [client],
  );
}

/** Commands the connected gateway advertises as browser-callable (empty if none / unsupported). */
export function useCommands(): CommandInfo[] {
  return useClientStore((c) => c.commands, (c, cb) => c.onCommands(cb), NO_COMMANDS);
}

/**
 * Typed hooks bound to a generated topic + command map (`DimosTopics`/`DimosCommands` from
 * packages/web/scripts/gen_types.py). Every topic-name hook autocompletes the name + infers the
 * message type, `useDimosClient()` returns a typed `DimosClient<TMap, TCmds>`, and
 * `useModules().Target.method(...)` is a typed RPC callable. Pure type-level wrapper (zero runtime
 * cost); map-agnostic hooks (useTeleop/useRpc/useCommands/…) don't need the map. Runtime-discovered
 * names still work (the `string & {}` fallback → `unknown`).
 */
// Topic-name key: a known key of TMap (autocompletes) OR any string (→ unknown). `string & Record<never,
// never>` keeps literal autocomplete alive while accepting arbitrary strings (and dodges the `{}` lint).
type NameKey<TMap> = (keyof TMap & string) | (string & Record<never, never>);
type MsgFor<TMap, K> = K extends keyof TMap ? TMap[K] : unknown;

export function createDimosHooks<TMap, TCmds = Record<never, never>>() {
  return {
    // Typed client: `subscribe`/`topic`/`peek`/`latest` autocomplete over TMap and
    // `modules.Target.method()` is typed over TCmds — we just cast the untyped context value to shape.
    useDimosClient: useDimosClient as unknown as () => DimosClient<TMap, TCmds> | null,
    // Typed RPC surface: `useModules().ScopeNav.navigate_to(goal)`. Null until connected. The `modules`
    // proxy is runtime-generic, so this is safe; the type comes straight off the typed client.
    useModules: (() => useDimosClient()?.modules ?? null) as unknown as () =>
      | DimosClient<TMap, TCmds>["modules"]
      | null,
    useTopicLatest: useTopicLatest as unknown as <K extends NameKey<TMap>>(
      topic: K | null,
      opts?: { maxHz?: number },
    ) => { data?: MsgFor<TMap, K>; meta?: MessageMeta },
    useTopicRef: useTopicRef as unknown as <K extends NameKey<TMap>>(
      topic: K | null,
    ) => MutableRefObject<{ data?: MsgFor<TMap, K>; meta?: MessageMeta }>,
    useTopicStats: useTopicStats as unknown as <K extends NameKey<TMap>>(
      topic: K | null,
      pollMs?: number,
    ) => TopicStats | null,
    useImageTopic: useImageTopic as unknown as <K extends NameKey<TMap>>(
      topic: K | null,
      opts?: { maxFps?: number },
    ) => ReturnType<typeof useImageTopic>,
  };
}

export type {
  CommandInfo,
  MessageMeta,
  Qos,
  Status,
  TopicInfo,
  TopicStats,
  TransportCaps,
  VideoMeta,
} from "@dimos/web";

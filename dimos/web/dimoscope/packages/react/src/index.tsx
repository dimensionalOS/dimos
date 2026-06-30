// @dimos/react — thin React bindings over @dimos/topics.
// High-rate topics: prefer useTopicRef (no re-render; read in a rAF loop) over
// useTopicLatest (re-renders per message).
import {
  createContext,
  type MutableRefObject,
  type ReactNode,
  useContext,
  useEffect,
  useMemo,
  useRef,
  useState,
} from "react";
import { connect, type DimosClient, selectMediaChannel } from "@dimos/topics";
import type {
  CommandInfo,
  MediaChannel,
  MediaKind,
  MessageMeta,
  Status,
  TopicInfo,
  TopicStats,
  VideoMeta,
} from "@dimos/topics";

/** One selectable transport/server: a label + a thunk that builds a connected client. */
export interface ServerOpt {
  id: string;
  label: string;
  connect: () => Promise<DimosClient>;
  /** Media-plane config for this server: where the camera can come from (WebRTC gateway +
   *  which kinds it can serve). Absent/jpeg-only → the camera uses the Image-topic floor. */
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

/**
 * Provides a DimosClient to the tree. Pass `servers` (a list of selectable
 * transports) to expose a switcher — the client is rebuilt (old one closed) when
 * the active server changes. A bare `url` is accepted as a one-server shorthand.
 */
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
    () => servers ?? (url ? [{ id: url, label: url, connect: () => connect({ url }) }] : []),
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

/** Live list of discovered topics. */
export function useTopics(): TopicInfo[] {
  const client = useDimosClient();
  const [topics, setTopics] = useState<TopicInfo[]>(() => client?.listTopics() ?? []);
  useEffect(() => {
    if (!client) return;
    setTopics(client.listTopics());
    const unsub = client.onTopics(setTopics);
    return () => {
      unsub();
    };
  }, [client]);
  return topics;
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
    const t = client.topic<T>(topic);
    if (maxHz) t.setRateLimit(maxHz);
    const sub = t.subscribeLatest((data, meta) => setState({ data, meta }));
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
    const sub = client.topic<T>(topic).subscribe((data, meta) => {
      ref.current = { data, meta };
    });
    return () => sub.unsubscribe();
  }, [client, topic]);
  return ref;
}

/**
 * Live stats for a topic (polled, PASSIVE). `Topic.stats()` is a pure read of a
 * rolling 1s window, so this does NOT subscribe — it never forces the gateway to
 * stream a topic just to measure it. It reports the rate of whatever is already
 * flowing (subscribed by some panel); topics nobody subscribes read 0. This is what
 * keeps the StatsBar from pulling every topic (incl. the ~11 MB/s camera) at once.
 */
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

// ── camera: decode sensor_msgs.Image → canvas ──────────────────────────────
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

    const sub = client.topic<ImageMsg>(topic).subscribe((data) => {
      latest = data;
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

// ── media plane: the camera as a negotiated, pluggable stream ───────────────
/** Forced media mode (the topbar toggle). "auto" = best available, falling back to jpeg. */
export type MediaMode = "auto" | "jpeg" | "webrtc" | "webcodecs";
const MODE_PREFER: Record<MediaMode, MediaKind[]> = {
  auto: ["webrtc", "webcodecs", "jpeg"], // proven default first; webcodecs/jpeg as fallback
  jpeg: ["jpeg"],
  webrtc: ["webrtc", "jpeg"],
  webcodecs: ["webcodecs", "jpeg"],
};

/**
 * Subscribe a camera topic via the negotiated media plane and render it — a <video> fed by a
 * WebRTC MediaStream when available, else the JPEG Image-topic floor painted to a <canvas>. The
 * caller attaches BOTH refs and shows the one matching `kind`. Negotiation uses the active
 * server's `media` config, so the data plane (topics) is untouched when the mode changes.
 *
 * Slice note: builds its own MediaChannel per hook instance (fine for one camera). A multi-cam
 * grid would hoist the channel to a provider to share one PeerConnection.
 */
export function useVideo(
  topic: string | null,
  opts?: {
    mode?: MediaMode;
    /** Per-frame tap (frames channels only — webcodecs/jpeg, NOT webrtc): called after the canvas
     *  draw, before the frame is closed. The seam for in-browser CV/overlays. Must NOT retain the
     *  frame past the call (it's closed right after) — copy it synchronously if you need it async. */
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
 * Subscribe to ANY topic by name from the UI — proves the on-demand machinery (topic.ts is
 * ref-counted, so a pin = a subscribe, an unpin = an unsubscribe). Works for not-yet-discovered
 * topics (the gateway forwards on first publish) and on every transport.
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
  // a short, human one-liner of the latest value so "subscribed" obviously means "data flowing".
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
  const client = useDimosClient();
  const [cmds, setCmds] = useState<CommandInfo[]>(() => client?.commands ?? []);
  useEffect(() => {
    if (!client) {
      setCmds([]);
      return;
    }
    setCmds(client.commands);
    const unsub = client.onCommands(setCmds);
    return () => {
      unsub();
    };
  }, [client]);
  return cmds;
}

export type {
  CommandInfo,
  MessageMeta,
  Status,
  TopicInfo,
  TopicStats,
  VideoMeta,
} from "@dimos/topics";

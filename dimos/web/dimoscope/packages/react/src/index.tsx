// @dimos/react — thin React bindings over @dimos/topics.
// High-rate topics: prefer useTopicRef (no re-render; read in a rAF loop) over
// useTopicLatest (re-renders per message).
import {
  createContext,
  useContext,
  useEffect,
  useMemo,
  useRef,
  useState,
  type MutableRefObject,
  type ReactNode,
} from "react";
import { connect, selectMediaChannel, type DimosClient } from "@dimos/topics";
import type { MediaKind, MessageMeta, Status, TopicInfo, TopicStats } from "@dimos/topics";

/** One selectable transport/server: a label + a thunk that builds a connected client. */
export interface ServerOpt {
  id: string;
  label: string;
  connect: () => Promise<DimosClient>;
  /** Media-plane config for this server: where the camera can come from (WebRTC gateway +
   *  which kinds it can serve). Absent/jpeg-only → the camera uses the Image-topic floor. */
  media?: { gatewayUrl?: string; kinds?: MediaKind[] };
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
    return client.onTopics(setTopics);
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
        out[o] = v; out[o + 1] = v; out[o + 2] = v; out[o + 3] = 255;
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
        createImageBitmap(new Blob([img.data], { type: "image/jpeg" }))
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
export type MediaMode = "auto" | "jpeg" | "webrtc";
const MODE_PREFER: Record<MediaMode, MediaKind[]> = {
  auto: ["webrtc", "jpeg"],
  jpeg: ["jpeg"],
  webrtc: ["webrtc", "jpeg"],
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
export function useVideo(topic: string | null, opts?: { mode?: MediaMode }) {
  const { client, servers, activeId } = useContext(Ctx);
  const mode = opts?.mode ?? "auto";
  const videoRef = useRef<HTMLVideoElement>(null);
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const [kind, setKind] = useState<"stream" | "frames">("frames");
  const [label, setLabel] = useState<string>();

  const mediaCfg = servers.find((s) => s.id === activeId)?.media;
  const gatewayUrl = mediaCfg?.gatewayUrl;
  const serverMedia = mediaCfg?.kinds;

  useEffect(() => {
    if (!client || !topic) return;
    const ch = selectMediaChannel({ client, gatewayUrl, serverMedia, prefer: MODE_PREFER[mode] });
    setKind(ch.caps.output);
    setLabel(ch.label);
    let alive = true;
    if (ch.caps.output === "stream") {
      ch.onStream((id, stream) => {
        if (alive && id === topic && videoRef.current) videoRef.current.srcObject = stream;
      });
    } else {
      ch.onFrame((id, frame) => {
        if (!alive || id !== topic) return;
        const cvs = canvasRef.current;
        const ctx = cvs?.getContext("2d");
        if (!cvs || !ctx) return;
        const f = frame as ImageBitmap; // ImageBitmap | VideoFrame are both CanvasImageSource
        if (cvs.width !== f.width || cvs.height !== f.height) {
          cvs.width = f.width;
          cvs.height = f.height;
        }
        ctx.drawImage(frame as CanvasImageSource, 0, 0);
        (frame as ImageBitmap).close?.();
      });
    }
    ch.connect()
      .then(() => alive && ch.subscribe(topic))
      .catch(() => {}); // TODO: runtime fallback to jpeg on connect failure
    return () => {
      alive = false;
      ch.unsubscribe(topic);
      ch.close();
    };
  }, [client, topic, mode, gatewayUrl, serverMedia]);

  return { kind, videoRef, canvasRef, label };
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
      <form onSubmit={(e) => { e.preventDefault(); add(text); }} style={{ display: "flex", gap: 6 }}>
        <input
          list="dimos-topic-names"
          placeholder="/dimos/…  subscribe by name"
          value={text}
          onChange={(e) => setText(e.target.value)}
          style={inputStyle}
        />
        <datalist id="dimos-topic-names">
          {topics.map((t) => (
            <option key={t.topic} value={t.topic} />
          ))}
        </datalist>
        <button type="submit" className="tab">+</button>
      </form>
      <div style={{ marginTop: 8, display: "flex", flexDirection: "column", gap: 4 }}>
        {pins.map((p) => (
          <PinnedTopic key={p} topic={p} onRemove={() => setPins((ps) => ps.filter((x) => x !== p))} />
        ))}
      </div>
    </div>
  );
}

function PinnedTopic({ topic, onRemove }: { topic: string; onRemove: () => void }) {
  const { meta } = useTopicLatest<unknown>(topic, { maxHz: 4 });
  return (
    <div style={{ display: "flex", alignItems: "center", gap: 8, font: "12px ui-monospace, monospace" }}>
      <span style={{ color: meta ? "#4ade80" : "var(--muted)" }}>●</span>
      <span style={{ flex: 1, color: "var(--text)" }}>{topic}</span>
      <span className="muted small">{meta ? `${meta.sizeBytes}B` : "waiting…"}</span>
      <button type="button" className="tab" onClick={onRemove} title="unsubscribe">
        ×
      </button>
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

export type { MessageMeta, Status, TopicInfo, TopicStats } from "@dimos/topics";

// jpeg is the universal media floor — the Image-topic path repackaged as a MediaChannel so the app
// consumes video uniformly; needs no media gateway; works on all browsers/transports.
import type { MediaCaps, MediaChannel, VideoMeta } from "../media.ts";
import type { Status } from "../transport.ts";
import type { Subscription } from "../types.ts";
import type { DimosClient } from "../client.ts";

export interface JpegTopicMediaDeps {
  client: DimosClient; // for the jpeg-topic floor (subscribes via client.topic)
}

// sensor_msgs.Image → ImageBitmap: jpeg → native decode; raw rgb8/bgr8/mono8/rgba8/bgra8 → RGBA.
interface ImageMsg {
  width: number;
  height: number;
  encoding: string;
  is_bigendian?: number;
  step?: number;
  data: Uint8Array;
}

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

/** Decode a sensor_msgs.Image (jpeg or raw) to an ImageBitmap. Rejects on unsupported/empty. */
function decodeImageToBitmap(img: ImageMsg): Promise<ImageBitmap> {
  const enc = (img.encoding || "").toLowerCase();
  if (enc === "jpeg" || enc === "jpg") {
    return createImageBitmap(new Blob([img.data as BlobPart], { type: "image/jpeg" }));
  }
  const id = rawToRGBA(img);
  if (!id) return Promise.reject(new Error(`unsupported image encoding: ${img.encoding}`));
  return createImageBitmap(id);
}

export const createJpegTopicMedia = (deps: JpegTopicMediaDeps): MediaChannel => {
  const { client } = deps;
  const caps: MediaCaps = {
    output: "frames",
    codec: "jpeg",
    onDemand: true,
    multiStream: true,
    hardwareDecode: false,
  };
  const subs = new Map<string, Subscription>();
  let frameCb: ((id: string, f: ImageBitmap, m: VideoMeta) => void) | undefined;
  let statusCb: ((s: Status) => void) | undefined;

  function connect(): Promise<void> {
    statusCb?.("open"); // rides the existing client connection — nothing to open
    return Promise.resolve();
  }

  function subscribe(streamId: string): void {
    if (subs.has(streamId)) return;
    const sub = client.topic(streamId).subscribeLatest((raw) => {
      const img = raw.data as ImageMsg;
      decodeImageToBitmap(img)
        .then((bmp) =>
          frameCb?.(streamId, bmp, {
            width: img.width,
            height: img.height,
            fps: 0,
            codec: img.encoding,
          })
        )
        .catch(() => {});
    });
    subs.set(streamId, sub);
  }

  function unsubscribe(streamId: string): void {
    subs.get(streamId)?.unsubscribe();
    subs.delete(streamId);
  }

  function close(): void {
    for (const s of subs.values()) s.unsubscribe();
    subs.clear();
    statusCb?.("closed");
  }

  return {
    caps,
    label: "jpeg (Image topic)",
    connect,
    subscribe,
    unsubscribe,
    onStream() {}, // n/a — this channel is "frames"
    onFrame(cb: (id: string, f: ImageBitmap, m: VideoMeta) => void): void {
      frameCb = cb;
    },
    onStatus(cb: (s: Status) => void): void {
      statusCb = cb;
    },
    close,
  };
};

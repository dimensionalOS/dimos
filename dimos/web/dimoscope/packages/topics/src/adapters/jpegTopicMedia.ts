// createJpegTopicMedia — the universal media FLOOR. The existing Image-topic path repackaged as a
// MediaChannel, so the app consumes video uniformly (and "jpeg is just another MediaChannel").
// Works on EVERY browser + EVERY transport: it just subscribes the camera topic on the normal
// data plane and decodes each frame to an ImageBitmap. No gateway media support required.
import type { MediaCaps, MediaChannel, VideoMeta } from "../media";
import type { Status } from "../transport";
import type { Subscription } from "../types";
import type { DimosClient } from "../client";
import { decodeImageToBitmap, type ImageMsg } from "../image";

export interface JpegTopicMediaDeps {
  client: DimosClient; // for the jpeg-topic floor (subscribes via client.topic)
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

  async function connect(): Promise<void> {
    statusCb?.("open"); // rides the existing client connection — nothing to open
  }

  function subscribe(streamId: string): void {
    if (subs.has(streamId)) return;
    const sub = client.topic<ImageMsg>(streamId).subscribeLatest((img) => {
      decodeImageToBitmap(img)
        .then((bmp) =>
          frameCb?.(streamId, bmp, {
            width: img.width,
            height: img.height,
            fps: 0,
            codec: img.encoding,
          }),
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

// JpegTopicMedia — the universal media FLOOR. The existing Image-topic path repackaged as a
// MediaChannel, so the app consumes video uniformly (and "jpeg is just another MediaChannel").
// Works on EVERY browser + EVERY transport: it just subscribes the camera topic on the normal
// data plane and decodes each frame to an ImageBitmap. No gateway media support required.
import type { MediaCaps, MediaChannel, VideoMeta } from "../media";
import type { Status } from "../transport";
import type { Subscription } from "../types";
import type { DimosClient } from "../client";
import { decodeImageToBitmap, type ImageMsg } from "../image";

export class JpegTopicMedia implements MediaChannel {
  readonly caps: MediaCaps = {
    output: "frames",
    codec: "jpeg",
    onDemand: true,
    multiStream: true,
    hardwareDecode: false,
  };
  label = "jpeg (Image topic)";
  private subs = new Map<string, Subscription>();
  private frameCb?: (id: string, f: ImageBitmap, m: VideoMeta) => void;
  private statusCb?: (s: Status) => void;

  constructor(private client: DimosClient) {}

  async connect(): Promise<void> {
    this.statusCb?.("open"); // rides the existing client connection — nothing to open
  }

  subscribe(streamId: string): void {
    if (this.subs.has(streamId)) return;
    const sub = this.client.topic<ImageMsg>(streamId).subscribeLatest((img) => {
      decodeImageToBitmap(img)
        .then((bmp) =>
          this.frameCb?.(streamId, bmp, {
            width: img.width,
            height: img.height,
            fps: 0,
            codec: img.encoding,
          }),
        )
        .catch(() => {});
    });
    this.subs.set(streamId, sub);
  }

  unsubscribe(streamId: string): void {
    this.subs.get(streamId)?.unsubscribe();
    this.subs.delete(streamId);
  }

  onStream(): void {} // n/a — this channel is "frames"
  onFrame(cb: (id: string, f: ImageBitmap, m: VideoMeta) => void): void {
    this.frameCb = cb;
  }
  onStatus(cb: (s: Status) => void): void {
    this.statusCb = cb;
  }

  close(): void {
    for (const s of this.subs.values()) s.unsubscribe();
    this.subs.clear();
    this.statusCb?.("closed");
  }
}

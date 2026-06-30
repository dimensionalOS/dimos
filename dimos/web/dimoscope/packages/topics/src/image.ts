// sensor_msgs.Image → ImageBitmap, for the JpegTopicMedia floor.
// (A small copy of the decode in @dimos/react's useImageTopic, kept here so the media plane
// doesn't depend on the React layer. jpeg → native decode; raw rgb8/bgr8/mono8/rgba8/bgra8 →
// RGBA. Both end as an ImageBitmap the canvas can blit.)

export interface ImageMsg {
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
export function decodeImageToBitmap(img: ImageMsg): Promise<ImageBitmap> {
  const enc = (img.encoding || "").toLowerCase();
  if (enc === "jpeg" || enc === "jpg") {
    return createImageBitmap(new Blob([img.data as BlobPart], { type: "image/jpeg" }));
  }
  const id = rawToRGBA(img);
  if (!id) return Promise.reject(new Error(`unsupported image encoding: ${img.encoding}`));
  return createImageBitmap(id);
}

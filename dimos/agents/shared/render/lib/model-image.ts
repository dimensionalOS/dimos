import sharp from "sharp";
import type { ImageContent } from "@earendil-works/pi-ai";
import type { Metadata } from "sharp";

/** The largest image the model receives, in pixels and base64 bytes; a larger one is scaled down to fit. */
export type ImageBox = {
  readonly width: number;
  readonly height: number;
  readonly maxBytes: number;
};

/** 4.5 MiB of base64 stays under Anthropic's 5 MB per image, as pi-durable's own default does. */
export const MODEL_IMAGE_BOX: ImageBox = { width: 1600, height: 1200, maxBytes: 4.5 * 2 ** 20 };

/** A photo is encoded as JPEG; a graphic (line art, text, transparency) as PNG. */
export type ImageKind = "photo" | "graphic";

/** Formats every provider takes inline, by sharp's name for them. */
const INLINE_TYPES: { readonly [format: string]: string } = {
  jpeg: "image/jpeg",
  png: "image/png",
  webp: "image/webp",
  gif: "image/gif",
};

/** Lossy photo formats, by sharp's name: re-encoding one as PNG multiplies its size for no detail. */
const PHOTO_FORMATS = new Set(["jpeg", "webp", "heif", "jp2"]);

export type ModelImage = {
  readonly content: ImageContent;
  readonly width: number;
  readonly height: number;
  /** The source's format and size when the bytes sent are not the source's own. */
  readonly from: { readonly format: string; readonly width: number; readonly height: number } | null;
};

function base64Length(bytes: number): number {
  return Math.ceil(bytes / 3) * 4;
}

function kindOfMetadata(meta: Metadata): ImageKind {
  return PHOTO_FORMATS.has(meta.format) && !meta.hasAlpha ? "photo" : "graphic";
}

/** A file's kind by its format: a photo for JPEG, WebP, HEIF and JPEG 2000 without transparency. */
export async function kindOf(bytes: Buffer): Promise<ImageKind> {
  return kindOfMetadata(await sharp(bytes, { density: 144 }).metadata());
}

/**
 * The image as the model receives it. A file in an inline format, upright, one frame, and within `box` is sent as it
 * is. Anything else is turned upright, fitted into `box` and encoded by its kind: JPEG for a photo without
 * transparency, PNG otherwise, smaller by a quarter until the base64 fits. `composed` is the kind of an image this
 * package drew, which has no original to keep, so it is sent as it is only when already in its kind's format.
 * Throws when sharp cannot read the bytes.
 */
export async function modelImage(
  bytes: Buffer,
  composed: ImageKind | null = null,
  box: ImageBox = MODEL_IMAGE_BOX,
): Promise<ModelImage> {
  const meta = await sharp(bytes, { density: 144 }).metadata();
  const kind = composed ?? kindOfMetadata(meta);
  const mimeType = INLINE_TYPES[meta.format];
  const asIs =
    mimeType !== undefined &&
    (composed === null || meta.format === (kind === "photo" ? "jpeg" : "png")) &&
    (meta.orientation ?? 1) === 1 &&
    (meta.pages ?? 1) === 1 &&
    meta.width <= box.width &&
    meta.height <= box.height &&
    base64Length(bytes.length) <= box.maxBytes;
  if (asIs) {
    return {
      content: { type: "image", data: bytes.toString("base64"), mimeType },
      width: meta.width,
      height: meta.height,
      from: null,
    };
  }
  for (let scale = 1; scale >= 0.1; scale *= 0.75) {
    const fitted = sharp(bytes, { density: 144 })
      .rotate()
      .resize({
        width: Math.round(box.width * scale),
        height: Math.round(box.height * scale),
        fit: "inside",
        withoutEnlargement: true,
      });
    const { data, info } = await (kind === "photo" ? fitted.jpeg() : fitted.png()).toBuffer({
      resolveWithObject: true,
    });
    if (base64Length(data.length) > box.maxBytes) continue;
    return {
      content: {
        type: "image",
        data: data.toString("base64"),
        mimeType: kind === "photo" ? "image/jpeg" : "image/png",
      },
      width: info.width,
      height: info.height,
      from: { format: meta.format, width: meta.autoOrient.width, height: meta.autoOrient.height },
    };
  }
  throw new Error(`cannot fit into ${box.maxBytes} bytes of base64`);
}

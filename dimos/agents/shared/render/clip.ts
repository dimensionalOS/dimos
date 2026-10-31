import sharp from "sharp";
import type { Cloud } from "./points.ts";
import type { FileRef, RenderedView, Sha256 } from "../protocol/media.ts";
import type { ImageKind } from "./lib/model-image.ts";
import type { Renderer } from "./base.ts";
import { Compile } from "typebox/compile";
import { Sha256Schema } from "../protocol/media.ts";
import { Type } from "typebox";
import { dirname, resolve } from "node:path";
import { firstProblem } from "./lib/schema.ts";
import { fitCamera, paintCloud, parseCloud } from "./points.ts";
import { kindOf, modelImage } from "./lib/model-image.ts";
import { readSource } from "./lib/artifacts.ts";

/** A frame index: `{type: "points" | "image", timeOrigin?, source?, frames: [{path, timestamp, sha256?}]}`, paths relative to the index. */
type FrameIndex = {
  readonly type: "points" | "image";
  readonly timeOrigin: number | null;
  readonly source: string | null;
  readonly frames: readonly { readonly path: string; readonly timestamp: number; readonly sha256: Sha256 | null }[];
};

const FrameIndexJson = Compile(
  Type.Object({
    type: Type.Union([Type.Literal("points"), Type.Literal("image")]),
    timeOrigin: Type.Optional(Type.Number()),
    source: Type.Optional(Type.String()),
    frames: Type.Array(
      Type.Object({ path: Type.String(), timestamp: Type.Number(), sha256: Type.Optional(Sha256Schema) }),
      { minItems: 1, maxItems: 10_000 },
    ),
  }),
);

const FrameIndexShape = Compile(
  Type.Object({
    type: Type.Union([Type.Literal("points"), Type.Literal("image")]),
    frames: Type.Array(Type.Unknown()),
  }),
);

/** Throws with the first problem when the JSON is not a valid frame index. Moved from dimcode clips.ts without `frameIndex`, which only `dimcode_render` used. */
function parseFrameIndex(json: unknown): FrameIndex {
  if (!FrameIndexJson.Check(json)) throw new Error(`not a frame index: ${firstProblem(FrameIndexJson, json)}`);
  const frames = json.frames.map(({ path, timestamp, sha256 = null }) => ({ path, timestamp, sha256 }));
  const backwards = frames.findIndex((frame, i) => i > 0 && frame.timestamp <= frames[i - 1].timestamp);
  if (backwards !== -1) throw new Error(`frame timestamps must be strictly increasing; frame ${backwards} is not`);
  return { type: json.type, timeOrigin: json.timeOrigin ?? null, source: json.source ?? null, frames };
}

/** Frames at least 0.1 s apart and at most 120, the first and last always among them. */
function previewIndexes(times: readonly number[]): number[] {
  const selected = [0];
  const step = Math.max(0.1, (times[times.length - 1] - times[0]) / 119);
  for (let i = 1; i < times.length - 1; i++) {
    // The tolerance keeps every frame of a 10 Hz export, whose float timestamps can differ by a hair under 0.1.
    if (times[i] - times[selected[selected.length - 1]] >= step - 1e-9) selected.push(i);
  }
  if (times.length > 1) selected.push(times.length - 1);
  return selected;
}

/** GIF delays in milliseconds at source timing; the last frame holds 0.1 s. */
function frameDelays(times: readonly number[]): number[] {
  return times.map((time, i) => Math.max(10, Math.round(((times[i + 1] ?? time + 0.1) - time) * 1000)));
}

function timestampStrip(seconds: number): Buffer {
  return Buffer.from(
    `<svg xmlns="http://www.w3.org/2000/svg" width="900" height="28"><rect width="900" height="28" fill="#0c1216"/>` +
      `<text x="16" y="19" fill="#8ba4a8" font-family="monospace" font-size="14">t = ${seconds.toFixed(3)} s</text></svg>`,
  );
}

type ClipFrame = Extract<RenderedView, { readonly kind: "clip" }>["clip"]["frames"][number];

type LoadedFrame = { readonly index: number; readonly source: FileRef; readonly bytes: Buffer; readonly json: unknown };

function frameError(frame: number, path: string, error: unknown): Error {
  return new Error(`frame ${frame} (${path}): ${error instanceof Error ? error.message : String(error)}`);
}

/** The selected frames' files, hashed and checked against the index's hashes. */
async function readFrames(
  index: FrameIndex,
  indexPath: string,
  selected: readonly number[],
  maxSourceBytes: number,
): Promise<LoadedFrame[]> {
  const frames: LoadedFrame[] = [];
  let total = 0;
  for (const i of selected) {
    const entry = index.frames[i];
    const file = await readSource(resolve(dirname(indexPath), entry.path), maxSourceBytes, entry.sha256).catch(
      (error: unknown) => {
        throw frameError(i, entry.path, error);
      },
    );
    total += file.bytes.length;
    if (total > 128 * 1024 * 1024) throw new Error("the clip's frames exceed 128 MiB; export a shorter interval");
    frames.push({ index: i, source: { path: file.path, sha256: file.sha256 }, bytes: file.bytes, json: file.json });
  }
  return frames;
}

/** Paints each cloud frame with one camera fitted to all of them, so scale and height brightness hold across the clip. */
function cloudPainter(index: FrameIndex, frames: readonly LoadedFrame[]): (position: number) => Promise<Buffer> {
  let total = 0;
  const clouds = frames.map((frame): Cloud => {
    const entry = index.frames[frame.index];
    try {
      const cloud = parseCloud(frame.json);
      total += cloud.points.length;
      if (total > 2_000_000) throw new Error("the clip's frames exceed two million points; export fewer");
      if (cloud.timestamp !== null && Math.abs(cloud.timestamp - entry.timestamp) > 0.000001) {
        throw new Error(`cloud timestamp ${cloud.timestamp} differs from the index's ${entry.timestamp}`);
      }
      return cloud;
    } catch (error) {
      throw frameError(frame.index, entry.path, error);
    }
  });
  if (new Set(clouds.map((cloud) => cloud.frame)).size > 1) {
    throw new Error("frames are in mixed coordinate frames; align them in DimOS before exporting");
  }
  const camera = fitCamera(...clouds);
  return (position) => paintCloud(clouds[position], camera, { width: 900, height: 448 });
}

/** A sheet of photos is a photo; one of painted clouds or of any graphic frame is a graphic. */
async function sheetKind(index: FrameIndex, frames: readonly LoadedFrame[]): Promise<ImageKind> {
  if (index.type === "points") return "graphic";
  const kinds = await Promise.all(frames.map((frame) => kindOf(frame.bytes)));
  return kinds.every((kind) => kind === "photo") ? "photo" : "graphic";
}

export const clipRenderer: Renderer<"clip"> = {
  noun: "clip",
  accepts: async (file) => FrameIndexShape.Check(file.json),
  measure: async (file) => `${parseFrameIndex(file.json).frames.length} frames`,
  render: async (file, label, context) => {
    const index = parseFrameIndex(file.json);
    const times = index.frames.map((frame) => frame.timestamp);
    const origin = index.timeOrigin ?? times[0];
    const span = times[times.length - 1] - times[0];
    if (span > 60) throw new Error(`spans ${span.toFixed(3)} s, over 60 seconds; export a shorter interval`);
    const frames = await readFrames(index, file.path, previewIndexes(times), context.maxSourceBytes);
    const pixels =
      index.type === "points"
        ? cloudPainter(index, frames)
        : (position: number) => Promise.resolve(frames[position].bytes);

    const rendered: ClipFrame[] = [];
    const pngs: Buffer[] = [];
    let outputBytes = 0;
    let reported = 0;
    for (const [position, frame] of frames.entries()) {
      const timestamp = times[frame.index];
      const png = await sharp(await pixels(position), { density: 144 })
        .resize(900, 448, { fit: "contain", background: "#101b21" })
        .flatten({ background: "#101b21" })
        .extend({ bottom: 28, background: "#0c1216" })
        .composite([{ input: timestampStrip(timestamp - origin), top: 448, left: 0 }])
        .removeAlpha()
        .png()
        .toBuffer();
      outputBytes += png.length;
      if (outputBytes > 64 * 1024 * 1024)
        throw new Error("the clip's frame previews exceed 64 MiB; export fewer frames");
      const ref = await context.cache.put(png, "png");
      rendered.push({ index: frame.index, timestamp, source: frame.source, png: ref });
      pngs.push(png);
      const now = Date.now();
      if (pngs.length === 1 || pngs.length === frames.length || now - reported >= 100) {
        reported = now;
        context.onProgress?.({ done: pngs.length, total: frames.length, latest: ref });
      }
    }

    let gif: FileRef | null = null;
    if (pngs.length > 1) {
      const bytes = await sharp(pngs, { join: { animated: true } })
        .gif({ loop: 0, delay: frameDelays(rendered.map((frame) => frame.timestamp)), effort: 3 })
        .toBuffer();
      if (bytes.length > 32 * 1024 * 1024) throw new Error("the clip's GIF exceeds 32 MiB; export fewer frames");
      gif = await context.cache.put(bytes, "gif");
    }

    const tiles = Math.min(6, pngs.length);
    const shown = [
      ...new Set(Array.from({ length: tiles }, (_, i) => Math.round((i * (pngs.length - 1)) / Math.max(1, tiles - 1)))),
    ];
    const sheet =
      pngs.length === 1
        ? pngs[0]
        : await sharp({
            create: {
              width: Math.min(3, shown.length) * 360,
              height: Math.ceil(shown.length / 3) * 190,
              channels: 3,
              background: "#101b21",
            },
          })
            .composite(
              await Promise.all(
                shown.map(async (frame, i) => ({
                  input: await sharp(pngs[frame]).resize(360, 190).png().toBuffer(),
                  left: (i % 3) * 360,
                  top: Math.floor(i / 3) * 190,
                })),
              ),
            )
            .png()
            .toBuffer();

    const seconds = (time: number): string => (time - origin).toFixed(3);
    const caption = [
      `${rendered.length} of ${times.length} ${index.type} frames, t = ${seconds(times[0])}-${seconds(times[times.length - 1])} s` +
        `${index.type === "points" ? ", one camera and height scale" : ""}${index.source === null ? "" : `, from ${index.source}`}` +
        `; the image is a contact sheet of ${shown.length}`,
      ...shown.map((frame) => `  t = ${seconds(rendered[frame].timestamp)} s: ${rendered[frame].png.path}`),
      ...(gif === null ? [] : [`  GIF: ${gif.path}`]),
    ].join("\n");
    return {
      view: {
        kind: "clip",
        label,
        source: { path: file.path, sha256: file.sha256 },
        sheet: await context.cache.put(sheet, "png"),
        clip: { frames: rendered, gif },
      },
      image: await modelImage(sheet, await sheetKind(index, frames)),
      caption,
    };
  },
};

import { dirname, resolve } from "node:path";
import sharp from "sharp";
import { z } from "zod";
import { cacheImage, readArtifact } from "./artifacts.js";
import type { Paths } from "./config.js";
import { cloudSchema, fitClouds, paintCloud, type Cloud } from "./points.js";

const timestamp = z.number().finite();
export const clipSourceSchema = z
  .object({
    type: z.enum(["points", "image"]),
    timeOrigin: timestamp.optional(),
    source: z.string().optional(),
    frames: z
      .array(
        z.object({
          path: z.string(),
          timestamp,
          sha256: z.string().optional(),
        }),
      )
      .min(1)
      .max(10_000),
  })
  .refine(
    (clip) =>
      clip.frames.every(
        (frame, i) => !i || frame.timestamp > clip.frames[i - 1].timestamp,
      ),
    "Frame timestamps must be strictly increasing",
  );
export const clipSchema = z.object({
  type: z.enum(["points", "image"]),
  timeOrigin: timestamp,
  sourceCount: z.number().int().positive(),
  frames: z
    .array(
      z.object({
        path: z.string(),
        timestamp,
        sha256: z.string(),
        source: z.string(),
        sourceSha256: z.string(),
        index: z.number().int().nonnegative(),
      }),
    )
    .min(1)
    .max(120),
  gif: z.string().optional(),
});
export type Clip = z.infer<typeof clipSchema>;

/** Preview selection only. All selected times and original indexes remain explicit. */
export function previewIndexes(times: readonly number[]): number[] {
  const selected = [0],
    step = Math.max(0.1, (times.at(-1)! - times[0]) / 119);
  for (let i = 1; i < times.length - 1; i++)
    if (times[i] - times[selected.at(-1)!] >= step) selected.push(i);
  if (times.length > 1) selected.push(times.length - 1);
  return selected;
}
export function frameDelays(times: readonly number[]): number[] {
  return times.map((time, i) =>
    Math.max(10, Math.round(((times[i + 1] ?? time + 0.1) - time) * 1000)),
  );
}
function timestampLabel(time: number): Buffer {
  return Buffer.from(
    `<svg xmlns="http://www.w3.org/2000/svg" width="900" height="28"><rect width="900" height="28" fill="#0c1216"/><text x="16" y="19" fill="#8ba4a8" font-family="monospace" font-size="14">t = ${time.toFixed(3)} s</text></svg>`,
  );
}

/** Local presentation of an already exported, finite result. No DimOS query executes here. */
export async function renderClip(
  paths: Paths,
  source: string,
  data: unknown,
  signal?: AbortSignal,
  frameIndex?: number,
  progress?: (done: number, total: number) => void,
): Promise<{ clip: Clip; png: Buffer; summary: string }> {
  const input = clipSourceSchema.parse(data),
    times = input.frames.map((frame) => frame.timestamp);
  if (times.at(-1)! - times[0] > 60)
    throw new Error(
      "Clip exceeds 60 seconds; select a smaller interval in DimOS.",
    );
  if (
    frameIndex !== undefined &&
    (!Number.isInteger(frameIndex) ||
      frameIndex < 0 ||
      frameIndex >= times.length)
  )
    throw new Error(
      `Choose a source frame index from 0 to ${times.length - 1}.`,
    );
  const indexes = previewIndexes(times);
  const required = [
    ...new Set([...indexes, ...(frameIndex === undefined ? [] : [frameIndex])]),
  ];
  const loaded = new Map<
    number,
    { bytes: Buffer; sha256: string; source: string; cloud?: Cloud }
  >();
  let totalBytes = 0,
    totalPoints = 0;
  for (const i of required) {
    signal?.throwIfAborted();
    const frame = input.frames[i],
      file = resolve(dirname(source), frame.path);
    const value = await readArtifact(file, frame.sha256);
    totalBytes += value.bytes.length;
    if (totalBytes > 128 * 1024 * 1024)
      throw new Error(
        "Clip source exceeds 128 MiB; select a smaller interval in DimOS.",
      );
    const cloud =
      input.type === "points"
        ? cloudSchema.parse(JSON.parse(value.bytes.toString("utf8")))
        : undefined;
    if (cloud) {
      totalPoints += cloud.points.length;
      if (totalPoints > 2_000_000)
        throw new Error(
          "Clip exceeds two million source points; export a smaller preview from DimOS.",
        );
      if (
        cloud.timestamp !== undefined &&
        Math.abs(cloud.timestamp - frame.timestamp) > 0.000001
      )
        throw new Error("Cloud timestamp differs from its frame index.");
    }
    loaded.set(i, { ...value, source: file, cloud });
  }
  const clouds = [...loaded.values()].flatMap((value) =>
    value.cloud ? [value.cloud] : [],
  );
  if (new Set(clouds.map((cloud) => cloud.frame)).size > 1)
    throw new Error(
      "Mixed coordinate frames; align the data in DimOS before rendering.",
    );
  const camera = clouds.length ? fitClouds(clouds) : undefined;
  const selected = frameIndex === undefined ? indexes : [frameIndex];
  const clip: Clip = {
    type: input.type,
    timeOrigin: input.timeOrigin ?? times[0],
    sourceCount: times.length,
    frames: [],
  };
  const images: Buffer[] = [];
  let outputBytes = 0;
  for (const index of selected) {
    signal?.throwIfAborted();
    const value = loaded.get(index)!;
    const pixels = value.cloud
      ? await paintCloud(value.cloud, camera)
      : value.bytes;
    const png = await sharp(pixels, { density: 144 })
      .resize(900, 448, { fit: "contain", background: "#101b21" })
      .flatten({ background: "#101b21" })
      .extend({ bottom: 28, background: "#0c1216" })
      .composite([
        {
          input: timestampLabel(times[index] - clip.timeOrigin),
          top: 448,
          left: 0,
        },
      ])
      .removeAlpha()
      .png()
      .toBuffer();
    signal?.throwIfAborted();
    outputBytes += png.length;
    if (outputBytes > 64 * 1024 * 1024)
      throw new Error(
        "Clip preview exceeds 64 MiB; export a smaller preview from DimOS.",
      );
    const path = await cacheImage(paths, png);
    const sha256 = path.split("/").at(-1)!.split(".")[0];
    clip.frames.push({
      path,
      sha256,
      timestamp: times[index],
      source: value.source,
      sourceSha256: value.sha256,
      index,
    });
    images.push(png);
    progress?.(images.length, selected.length);
  }
  if (images.length > 1) {
    const gif = await sharp(images, { join: { animated: true } })
      .gif({
        loop: 0,
        delay: frameDelays(clip.frames.map((frame) => frame.timestamp)),
        effort: 3,
      })
      .toBuffer();
    signal?.throwIfAborted();
    if (gif.length > 32 * 1024 * 1024)
      throw new Error(
        "GIF exceeds 32 MiB; export a smaller preview from DimOS.",
      );
    clip.gif = await cacheImage(paths, gif, "gif");
  }
  const chosen = [
    ...new Set(
      Array.from({ length: Math.min(6, images.length) }, (_, i) =>
        Math.round(
          (i * (images.length - 1)) /
            Math.max(1, Math.min(6, images.length) - 1),
        ),
      ),
    ),
  ];
  const tiles = await Promise.all(
    chosen.map(async (index, i) => ({
      input: await sharp(images[index]).resize(360, 190).png().toBuffer(),
      left: (i % 3) * 360,
      top: Math.floor(i / 3) * 190,
    })),
  );
  const png =
    images.length === 1
      ? images[0]
      : await sharp({
          create: {
            width: Math.min(3, tiles.length) * 360,
            height: Math.ceil(tiles.length / 3) * 190,
            channels: 3,
            background: "#101b21",
          },
        })
          .composite(tiles)
          .png()
          .toBuffer();
  const summary = `${selected.length}/${times.length} source frames · ${input.type} · ${(times[0] - clip.timeOrigin).toFixed(3)}–${(times.at(-1)! - clip.timeOrigin).toFixed(3)} s; ${camera ? "fixed camera and height scale; " : ""}model image shows ${chosen.length} timestamped frames; ${input.source ?? "source index"}`;
  signal?.throwIfAborted();
  return { clip, png, summary };
}

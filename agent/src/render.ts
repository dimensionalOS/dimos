import { createHash } from "node:crypto";
import {
  mkdir,
  readFile,
  readdir,
  stat,
  unlink,
  writeFile,
} from "node:fs/promises";
import { basename, join, resolve } from "node:path";
import { setTimeout as delay } from "node:timers/promises";
import type { ExtensionFactory } from "@earendil-works/pi-coding-agent";
import type { ImageContent, TextContent } from "@earendil-works/pi-ai";
import { Container, Image, Text, type Component } from "@earendil-works/pi-tui";
import { Type } from "typebox";
import { z } from "zod";
import sharp from "sharp";
import type { Paths } from "./config.js";
import { MediaPool } from "./media.js";

const point = z.tuple([
  z.number().finite(),
  z.number().finite(),
  z.number().finite(),
]);
const sample = z.tuple([z.number().finite(), z.number().finite()]);
const cloud = z
  .object({
    points: z.array(point).max(1_000_000),
    frame: z.string().optional(),
    timestamp: z.number().optional(),
  })
  .passthrough();
const series = z
  .object({
    series: z
      .array(
        z.object({ name: z.string(), values: z.array(sample).max(100_000) }),
      )
      .max(20),
  })
  .passthrough();
export const renderDetails = z.object({
  source: z.string(),
  sha256: z.string().optional(),
  summary: z.string(),
  title: z.string().optional(),
  views: z
    .array(
      z.object({ label: z.string(), source: z.string(), sha256: z.string() }),
    )
    .optional(),
  live: z
    .object({
      url: z.string(),
      robot: z.string().optional(),
      channel: z.string(),
    })
    .optional(),
});
export type RenderDetails = z.infer<typeof renderDetails>;

export async function plot(
  data: unknown,
  kind: "points" | "series",
): Promise<{ png: Buffer; summary: string }> {
  let groups: Array<{ name: string; values: Array<[number, number, number?]> }>;
  let metadata: Record<string, unknown>;
  if (kind === "points") {
    const { points, ...rest } = cloud.parse(data);
    groups = [{ name: "points", values: points }];
    metadata = rest;
  } else {
    const { series: measured, ...rest } = series.parse(data);
    groups = measured;
    metadata = rest;
  }
  const values = groups.flatMap((group) => group.values);
  if (!values.length) throw new Error("No points to render");
  const projected = values.map(([x, y, z = 0]) =>
    kind === "points" ? [x - y * 0.35, z + y * 0.35] : [x, y],
  );
  let xmin = Infinity,
    xmax = -Infinity,
    ymin = Infinity,
    ymax = -Infinity;
  for (const [x, y] of projected) {
    xmin = Math.min(xmin, x);
    xmax = Math.max(xmax, x);
    ymin = Math.min(ymin, y);
    ymax = Math.max(ymax, y);
  }
  const project = ([x, y]: number[]) => [
    24 + ((x - xmin) / (xmax - xmin || 1)) * 752,
    424 - ((y - ymin) / (ymax - ymin || 1)) * 400,
  ];
  const colors = ["#5eead4", "#fbbf24", "#f472b6", "#60a5fa"];
  const step = Math.max(1, Math.ceil(values.length / 20_000));
  let offset = 0;
  const shapes = groups
    .map((group, index) => {
      const coords = projected
        .slice(offset, offset + group.values.length)
        .filter((_, i) => i % step === 0)
        .map(project);
      offset += group.values.length;
      const color = colors[index % colors.length];
      return kind === "points"
        ? coords
            .map(
              ([x, y]) =>
                '<circle cx="' +
                x +
                '" cy="' +
                y +
                '" r="1.2" fill="' +
                color +
                '"/>',
            )
            .join("")
        : '<polyline points="' +
            coords.map((pair) => pair.join(",")).join(" ") +
            '" fill="none" stroke="' +
            color +
            '" stroke-width="2"/>';
    })
    .join("");
  const png = await sharp(
    Buffer.from(
      '<svg xmlns="http://www.w3.org/2000/svg" width="800" height="448"><rect width="800" height="448" fill="#111827"/>' +
        shapes +
        "</svg>",
    ),
  )
    .png()
    .toBuffer();
  return {
    png,
    summary:
      values.length +
      " source samples; " +
      (kind === "points"
        ? "XYZ oblique projection"
        : groups.map((group) => group.name).join(", ")) +
      "; display every " +
      step +
      " sample(s); axes " +
      JSON.stringify({ xmin, xmax, ymin, ymax }) +
      "; source metadata " +
      JSON.stringify(metadata),
  };
}
async function cacheImage(paths: Paths, png: Buffer): Promise<string> {
  await mkdir(paths.cache, { recursive: true, mode: 0o700 });
  const path = join(
    paths.cache,
    createHash("sha256").update(png).digest("hex") + ".png",
  );
  await writeFile(path, png, { mode: 0o600 });
  const entries = (
    await Promise.allSettled(
      (await readdir(paths.cache))
        .filter((name) => /^[a-f0-9]{64}\.png$/.test(name))
        .map(async (name) => ({
          name,
          info: await stat(join(paths.cache, name)),
        })),
    )
  ).flatMap((result) => (result.status === "fulfilled" ? [result.value] : []));
  let bytes = 0;
  for (const entry of entries.sort((a, b) => b.info.mtimeMs - a.info.mtimeMs)) {
    bytes += entry.info.size;
    if (bytes > 128 * 1024 * 1024 && entry.name !== path.split("/").pop())
      await unlink(join(paths.cache, entry.name)).catch(() => {});
  }
  return path;
}
export function imageComponent(
  data: string,
  mimeType: string,
  summary = "",
): Component {
  const block = new Container();
  if (summary) block.addChild(new Text(summary, 0, 0));
  block.addChild(
    new Image(
      data,
      mimeType,
      { fallbackColor: (text) => text },
      { maxHeightCells: 18, maxWidthCells: 80 },
    ),
  );
  return block;
}

/** Rasterize DimOS's own exports; never evaluate or reinterpret a memory query. */
export async function renderFiles(
  paths: Paths,
  cwd: string,
  views: Array<{ path: string; label: string }>,
  title?: string,
  signal?: AbortSignal,
): Promise<{
  content: Array<TextContent | ImageContent>;
  details: RenderDetails;
}> {
  if (!views.length || views.length > 6)
    throw new Error("Choose 1–6 saved views from the same operation.");
  const content: Array<TextContent | ImageContent> = [];
  const sources: NonNullable<RenderDetails["views"]> = [];
  for (const view of views) {
    signal?.throwIfAborted();
    const source = resolve(cwd, view.path);
    if ((await stat(source)).size > 32 * 1024 * 1024)
      throw new Error(
        "Export exceeds 32 MiB; select a smaller view while retaining the original source.",
      );
    const bytes = await readFile(source);
    const sha256 = createHash("sha256").update(bytes).digest("hex");
    const png = await sharp(bytes, { density: 144 })
      .resize({
        width: 1600,
        height: 1200,
        fit: "inside",
        withoutEnlargement: true,
      })
      .flatten({ background: "#101b21" })
      .png()
      .toBuffer();
    signal?.throwIfAborted();
    const preview = await cacheImage(paths, png);
    sources.push({ label: view.label, source, sha256 });
    content.push(
      {
        type: "text",
        text: `${view.label}\nSource: ${source}\nSHA-256: ${sha256}\nPreview: ${preview}`,
      },
      { type: "image", data: png.toString("base64"), mimeType: "image/png" },
    );
  }
  return {
    content,
    details: {
      source: sources[0].source,
      sha256: sources[0].sha256,
      title,
      views: sources,
      summary: `${sources.length} saved views · original exports retained`,
    },
  };
}
export const renderExtension =
  (paths: Paths): ExtensionFactory =>
  (pi) => {
    const pool = new MediaPool();
    pi.on("session_shutdown", () => pool.close());
    pi.registerTool({
      name: "dimcode_render",
      label: "Inspect sensor result",
      description:
        "Display an existing operation's results in its terminal tool card. image: path or views [{path,label}] for up to 6 related SVG/PNG exports, with an optional title. For memory queries prefer DimOS Space/Plot SVGs and selected frames from the same materialized query; all views are returned to the model together. points: JSON {points:[[x,y,z]], frame?, timestamp?, selectedIndices?}. series: JSON {series:[{name,values:[[time,value]]}]}. Never rerun or reimplement memory analysis just to render it. live: explicit existing relay URL, robot and channel; terminal frames stream directly and only one final snapshot enters model context.",
      parameters: Type.Object({
        kind: Type.Union([
          Type.Literal("image"),
          Type.Literal("points"),
          Type.Literal("series"),
          Type.Literal("live"),
        ]),
        path: Type.Optional(Type.String()),
        title: Type.Optional(Type.String()),
        views: Type.Optional(
          Type.Array(
            Type.Object({ path: Type.String(), label: Type.String() }),
            { minItems: 1, maxItems: 6 },
          ),
        ),
        url: Type.Optional(Type.String()),
        robot: Type.Optional(Type.String()),
        channel: Type.Optional(Type.String()),
        seconds: Type.Optional(Type.Number({ minimum: 1, maximum: 30 })),
      }),
      execute: async (_id, args, signal, update, ctx) => {
        if (args.kind === "image")
          return renderFiles(
            paths,
            ctx.cwd,
            args.views ??
              (args.path
                ? [{ path: args.path, label: basename(args.path) }]
                : []),
            args.title,
            signal,
          );
        let png: Buffer,
          source: string,
          summary: string,
          digest: string | undefined;
        if (args.kind === "live") {
          if (!args.url || !args.channel)
            throw new Error(
              "Live rendering requires an existing relay URL and channel.",
            );
          source =
            args.url +
            " / " +
            (args.robot ?? "selected robot") +
            " / " +
            args.channel;
          const live = {
            url: args.url,
            robot: args.robot,
            channel: args.channel,
          };
          update?.({
            content: [{ type: "text", text: "Inspecting live channel" }],
            details: {
              source,
              summary: "Live context; timestamps may be send time.",
              live,
            } satisfies RenderDetails,
          });
          const lease = await pool.acquire(
            live,
            args.channel,
            (slot) => slot,
            () => {},
          );
          try {
            const end = Date.now() + (args.seconds ?? 5) * 1000;
            while (Date.now() < end)
              await delay(Math.min(100, end - Date.now()), undefined, {
                signal,
              });
            const slot = lease.current();
            if (!slot)
              throw new Error("No frame received from selected channel");
            if (!(slot.value instanceof Uint8Array))
              throw new Error(
                "This channel needs its domain decoder/renderer; use an exported result for this encoding.",
              );
            png = await sharp(slot.value)
              .resize({
                width: 1280,
                height: 960,
                fit: "inside",
                withoutEnlargement: true,
              })
              .png()
              .toBuffer();
            summary =
              "Selected live snapshot, sequence " +
              slot.seq +
              ", header timestamp " +
              slot.ts +
              ". Header time is not necessarily capture time.";
          } finally {
            lease.close();
          }
        } else {
          if (!args.path) throw new Error("Saved rendering requires a path");
          source = resolve(ctx.cwd, args.path);
          const info = await stat(source);
          if (info.size > 32 * 1024 * 1024)
            throw new Error(
              "Export exceeds 32 MiB; select a smaller view while retaining the original source.",
            );
          const bytes = await readFile(source);
          signal?.throwIfAborted();
          digest = createHash("sha256").update(bytes).digest("hex");
          ({ png, summary } = await plot(
            JSON.parse(bytes.toString("utf8")),
            args.kind,
          ));
        }
        const preview = await cacheImage(paths, png);
        return {
          content: [
            {
              type: "text",
              text:
                summary +
                "\nSource: " +
                source +
                (digest ? "\nSHA-256: " + digest : "") +
                "\nPreview: " +
                preview,
            },
            {
              type: "image",
              data: png.toString("base64"),
              mimeType: "image/png",
            },
          ],
          details: { source, sha256: digest, summary } satisfies RenderDetails,
        };
      },
      renderResult: (result) => {
        const block = new Container();
        for (const item of result.content)
          block.addChild(
            item.type === "image"
              ? imageComponent(item.data, item.mimeType)
              : new Text(item.text, 0, 0),
          );
        return block;
      },
    });
  };

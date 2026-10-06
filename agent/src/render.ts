import { TerminalImage as Image } from "./terminal-image.js";
import { basename, extname, resolve } from "node:path";
import { setTimeout as delay } from "node:timers/promises";
import type { ExtensionFactory } from "@earendil-works/pi-coding-agent";
import type { ImageContent, TextContent } from "@earendil-works/pi-ai";
import { Container, Text, type Component } from "@earendil-works/pi-tui";
import { Type } from "typebox";
import { z } from "zod";
import sharp from "sharp";
import type { Paths } from "./config.js";
import { MediaPool } from "./media.js";
import { cacheImage, readArtifact } from "./artifacts.js";
import { cloudSchema, paintCloud } from "./points.js";
import { clipSchema, renderClip } from "./clips.js";

export const renderDetails = z.object({
  source: z.string(),
  kind: z.enum(["image", "points", "sequence"]).optional(),
  clip: clipSchema.optional(),
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

export function imageComponent(
  data: string,
  mimeType: string,
  summary = "",
  changed: () => void = () => {},
): Component {
  const block = new Container();
  if (summary) block.addChild(new Text(summary, 0, 0));
  block.addChild(
    new Image(
      data,
      mimeType,
      { fallbackColor: (text) => text },
      { maxHeightCells: 18, maxWidthCells: 80 },
      changed,
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
    const { bytes, sha256 } = await readArtifact(source);
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
/** Inspect saved results without coupling presentation to a DimOS runtime or transport. */
export async function renderSaved(
  paths: Paths,
  cwd: string,
  path: string,
  options: {
    kind?: "auto" | "image" | "points" | "sequence";
    title?: string;
    frame?: number;
  } = {},
  signal?: AbortSignal,
  progress?: (done: number, total: number) => void,
): Promise<{
  content: Array<TextContent | ImageContent>;
  details: RenderDetails;
}> {
  const source = resolve(cwd, path);
  let kind = options.kind ?? "auto";
  if (
    (kind === "image" || kind === "auto") &&
    extname(source).toLowerCase() !== ".json"
  )
    return renderFiles(
      paths,
      cwd,
      [{ path, label: basename(path) }],
      options.title,
      signal,
    );
  const { bytes, sha256 } = await readArtifact(source);
  signal?.throwIfAborted();
  const data: unknown = JSON.parse(bytes.toString("utf8"));
  // A tagged frame index is authoritative, including when a caller supplies a still-view hint.
  if (
    z
      .object({
        type: z.enum(["points", "image"]),
        frames: z.array(z.unknown()),
      })
      .safeParse(data).success
  )
    kind = "sequence";
  else if (kind === "auto") {
    if (cloudSchema.safeParse(data).success) kind = "points";
    else
      throw new Error(
        "Unsupported JSON view. Export a plot/graph with DimOS or Python as SVG, then render that file.",
      );
  }
  let png: Buffer, summary: string, clip: RenderDetails["clip"];
  if (kind === "sequence")
    ({ png, summary, clip } = await renderClip(
      paths,
      source,
      data,
      signal,
      options.frame,
      progress,
    ));
  else {
    if (options.frame !== undefined)
      throw new Error(
        "frame selects an original sequence frame; provide a frame index file.",
      );
    const cloud = cloudSchema.parse(data);
    png = await paintCloud(cloud);
    summary = `${cloud.points.length} source points; frame ${cloud.frame ?? "unspecified"}; timestamp ${cloud.timestamp ?? "unspecified"}; terminal projection, source retained`;
  }
  signal?.throwIfAborted();
  const preview = await cacheImage(paths, png);
  return {
    content: [
      {
        type: "text",
        text: `${summary}\nSource: ${source}\nSHA-256: ${sha256}\nPreview: ${preview}${clip?.gif ? "\nGIF: " + clip.gif : ""}${clip ? "\nOriginal frame indexes in preview: " + clip.frames.map((frame) => frame.index).join(", ") + "\nUse frame: INDEX to inspect any original frame without repeating the query." : ""}`,
      },
      { type: "image", data: png.toString("base64"), mimeType: "image/png" },
    ],
    details: { source, sha256, summary, kind, clip, title: options.title },
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
        "Display existing results in their terminal tool card. Auto-detect PNG/SVG images, XYZ JSON {points:[[x,y,z]],colors?:[[r,g,b]],frame?,timestamp?,selectedIndices?}, or a finite sequence index {type:'points'|'image',timeOrigin?,source?,frames:[{path,timestamp,sha256?}]}. Frame paths are relative to the index; timestamps are seconds, strictly increasing. Sequence previews use a fixed camera, source timing and a GIF; model receives a timestamped contact sheet. frame: INDEX returns one original source frame for closer inspection. views [{path,label}] groups up to 6 related SVG/PNG exports. Memory selection, filtering and analysis stay in DimOS. For any unsupported plot/graph, use DimOS Space/Plot or inline Python to export SVG and display it here. Never repeat a memory query to render it. live: explicit existing relay URL, robot and channel; only the final snapshot enters model context.",
      parameters: Type.Object({
        kind: Type.Optional(
          Type.Union([
            Type.Literal("auto"),
            Type.Literal("image"),
            Type.Literal("points"),
            Type.Literal("sequence"),
            Type.Literal("live"),
          ]),
        ),
        frame: Type.Optional(Type.Integer({ minimum: 0 })),
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
        if (args.views)
          return renderFiles(paths, ctx.cwd, args.views, args.title, signal);
        if (args.kind !== "live") {
          if (!args.path) throw new Error("Saved rendering requires a path");
          return renderSaved(
            paths,
            ctx.cwd,
            args.path,
            { kind: args.kind, title: args.title, frame: args.frame },
            signal,
            (done, total) =>
              update?.({
                content: [
                  {
                    type: "text",
                    text: `Rendering saved frames ${done}/${total}`,
                  },
                ],
                details: {
                  source: resolve(ctx.cwd, args.path!),
                  summary: `Rendering saved frames ${done}/${total}`,
                  title: args.title,
                },
              }),
          );
        }
        let png: Buffer, source: string, summary: string;
        {
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
        }
        const preview = await cacheImage(paths, png);
        return {
          content: [
            {
              type: "text",
              text: summary + "\nSource: " + source + "\nPreview: " + preview,
            },
            {
              type: "image",
              data: png.toString("base64"),
              mimeType: "image/png",
            },
          ],
          details: { source, summary } satisfies RenderDetails,
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

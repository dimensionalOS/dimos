import sharp from "sharp";
import type { ImageContent } from "@earendil-works/pi-ai";
import type { ImageProcessor } from "@earendil-works/pi-durable/tools";
import type { RenderDetails, Sha256 } from "#dimos/agents/shared/protocol/media.ts";
import type { RenderProgress, Rendering } from "#dimos/agents/shared/render/types.ts";
import type { SourceFile } from "#dimos/agents/shared/render/lib/artifacts.ts";
import { MAX_VIEWS } from "#dimos/agents/shared/protocol/media.ts";
import { MODEL_IMAGE_BOX, modelImage } from "#dimos/agents/shared/render/lib/model-image.ts";
import { RenderCache, readSource } from "#dimos/agents/shared/render/lib/artifacts.ts";
import { basename } from "node:path";
import { clipRenderer } from "#dimos/agents/shared/render/clip.ts";
import { imageRenderer } from "#dimos/agents/shared/render/image.ts";
import { pointsRenderer } from "#dimos/agents/shared/render/points.ts";

/** Tried in this order; the first that accepts a file renders it. A frame index wins over a cloud, as in dimcode. */
const RENDERERS = [clipRenderer, pointsRenderer, imageRenderer] as const;

async function rendererFor(source: SourceFile): Promise<(typeof RENDERERS)[number] | null> {
  for (const renderer of RENDERERS) if (await renderer.accepts(source)) return renderer;
  return null;
}

type FileToRender = {
  /** Absolute; the path the views and the model's text name. */
  readonly path: string;
  /** Set when the producer hashed the file, as HarnessRuntime does for its exports. */
  readonly sha256: Sha256 | null;
};

type FileRendererOptions = {
  /** `~/.cache/dimos/agent/render/` for the gateway; the same directory for `/inspect`. */
  readonly cacheDir: string;
  /** The cache is pruned oldest first past this size, sparing what an unfinished render has put. */
  readonly maxCacheBytes: number;
  /** A larger source is reported as failed instead of read. */
  readonly maxSourceBytes: number;
};

/**
 * Renders whatever of a call's or an event's files is renderable, as one group: images, point clouds and frame
 * indexes. MAX_VIEWS views at most; each file past them is a notice naming its path, which `read` brings back.
 */
export class FileRenderer {
  private readonly cache: RenderCache;
  private readonly maxSourceBytes: number;

  constructor(options: FileRendererOptions) {
    this.cache = new RenderCache(options.cacheDir, options.maxCacheBytes);
    this.maxSourceBytes = options.maxSourceBytes;
  }

  /** Null when no file is renderable and none failed, so callers add nothing. */
  async render(
    files: readonly FileToRender[],
    title: string,
    onProgress: ((progress: RenderProgress) => void) | null = null,
  ): Promise<Rendering | null> {
    return this.cache.pinning(async (cache) => {
      const context = { cache, maxSourceBytes: this.maxSourceBytes, onProgress };
      const details: RenderDetails = { title, views: [], listed: [], failed: [] };
      const images: ImageContent[] = [];
      const lines: string[] = [];
      const notices: string[] = [];
      for (const file of files) {
        try {
          const source = await readSource(file.path, this.maxSourceBytes, file.sha256);
          const renderer = await rendererFor(source);
          if (renderer === null) continue;
          if (details.views.length === MAX_VIEWS) {
            const size = await renderer.measure(source);
            details.listed.push(file.path);
            notices.push(`[${renderer.noun} not shown: ${file.path} (${size}); read the path to see it]`);
            continue;
          }
          const part = await renderer.render(source, basename(file.path), context);
          details.views.push(part.view);
          images.push(part.image.content);
          lines.push(`${part.view.label} · ${source.path} · sha256 ${source.sha256} · ${part.caption}`);
        } catch (error) {
          details.failed.push({ path: file.path, error: error instanceof Error ? error.message : String(error) });
        }
      }
      if (details.views.length + details.listed.length + details.failed.length === 0) return null;
      lines.push(...notices);
      lines.push(...details.failed.map(({ path, error }) => `Not rendered: ${path}: ${error}`));
      return { text: lines.join("\n"), images, details };
    });
  }
}

/** Whether sharp can decode the bytes; a format it cannot, such as BMP, goes to the processor `modelImages` wraps. */
async function decodable(bytes: Buffer): Promise<boolean> {
  return sharp(bytes)
    .metadata()
    .then(
      () => true,
      () => false,
    );
}

/**
 * The image processor for pi-durable's `read`, with the policy the renderers use, inside the model's own limits.
 * `undecodable` prepares what sharp cannot read, within the same limits.
 */
export function modelImages(undecodable: ImageProcessor): ImageProcessor {
  return {
    prepare: async (bytes, mimeType, limits) => {
      const box = {
        width: Math.min(MODEL_IMAGE_BOX.width, limits.maxWidth),
        height: Math.min(MODEL_IMAGE_BOX.height, limits.maxHeight),
        maxBytes: Math.min(MODEL_IMAGE_BOX.maxBytes, limits.maxBytes),
      };
      const source = Buffer.from(bytes);
      if (!(await decodable(source)))
        return undecodable.prepare(bytes, mimeType, {
          maxWidth: box.width,
          maxHeight: box.height,
          maxBytes: box.maxBytes,
        });
      const image = await modelImage(source, null, box).catch(() => null);
      if (image === null) return undefined;
      const { content, from, width, height } = image;
      const resized = from !== null && (from.width !== width || from.height !== height);
      return {
        data: content.data,
        mimeType: content.mimeType,
        ...(resized ? { resized: { from: { width: from.width, height: from.height }, to: { width, height } } } : {}),
        ...(content.mimeType === mimeType ? {} : { convertedFrom: mimeType }),
      };
    },
  };
}

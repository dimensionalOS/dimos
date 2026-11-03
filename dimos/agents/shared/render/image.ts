import sharp from "sharp";
import type { Renderer } from "#dimos/agents/shared/render/base.ts";
import { modelImage } from "#dimos/agents/shared/render/lib/model-image.ts";

/**
 * PNG, JPEG, SVG and the other formats sharp reads. The model gets the file itself when it can take it; the terminal
 * gets a PNG fitted into 1600x1200 on #101b21 (dimcode render.ts renderFiles).
 */
export const imageRenderer: Renderer<"image"> = {
  noun: "image",
  accepts: (file) =>
    sharp(file.bytes, { density: 144 })
      .metadata()
      .then(
        () => true,
        () => false,
      ),
  measure: async (file) => {
    const { autoOrient } = await sharp(file.bytes, { density: 144 }).metadata();
    return `${autoOrient.width}x${autoOrient.height}`;
  },
  render: async (file, label, context) => {
    const image = sharp(file.bytes, { density: 144 });
    const { format, width, height } = await image.metadata();
    const preview = await image
      .rotate()
      .resize({ width: 1600, height: 1200, fit: "inside", withoutEnlargement: true })
      .flatten({ background: "#101b21" })
      .png()
      .toBuffer();
    const sent = await modelImage(file.bytes);
    const type = sent.content.mimeType.replace("image/", "");
    const source = sent.from ?? { format, width, height };
    const changed = type !== source.format || sent.width !== source.width || sent.height !== source.height;
    return {
      view: {
        kind: "image",
        label,
        source: { path: file.path, sha256: file.sha256 },
        preview: await context.cache.put(preview, "png"),
      },
      image: sent,
      caption: `${format} ${source.width}x${source.height}${changed ? `, sent as ${type} ${sent.width}x${sent.height}` : ""}`,
    };
  },
};

import sharp from "sharp";

/** `image`, in any format sharp reads, as a PNG: kitty-protocol terminals take only PNG. */
export function pngOf(image: Buffer): Promise<Buffer> {
  return sharp(image).png().toBuffer();
}

/** RGB samples for a cell picture, with two vertical samples per terminal row. */
export async function cellPixels(image: Buffer, columns: number, rows: number): Promise<Buffer> {
  return sharp(image)
    .resize(columns, rows * 2, { fit: "fill" })
    .flatten({ background: "#101b21" })
    .removeAlpha()
    .toColourspace("srgb")
    .raw()
    .toBuffer();
}

type SliceSize = {
  /** The width of every slice, in pixels. */
  readonly width: number;
  /** One terminal cell row, in pixels. */
  readonly rowHeight: number;
  readonly rows: number;
};

/** `png` fitted into `width` × `rows`·`rowHeight` pixels on #101b21, cut into `rows` PNGs one row tall, top first. */
export async function rowSlices(png: Buffer, { width, rowHeight, rows }: SliceSize): Promise<Buffer[]> {
  const pixels = await sharp(png)
    .resize(width, rows * rowHeight, { fit: "contain", background: "#101b21" })
    .flatten({ background: "#101b21" })
    .removeAlpha()
    .toColourspace("srgb")
    .raw()
    .toBuffer();
  const bytes = width * rowHeight * 3;
  return Promise.all(
    Array.from({ length: rows }, (_, row) =>
      sharp(pixels.subarray(row * bytes, (row + 1) * bytes), { raw: { width, height: rowHeight, channels: 3 } })
        .png()
        .toBuffer(),
    ),
  );
}

/**
 * The overview of a rendering's view previews: one preview as it is; more as dimcode's 1000-pixel-wide mosaic, where
 * with three or more the first spans the top and the rest sit two to a row.
 */
export async function overviewOf(previews: readonly Buffer[]): Promise<Buffer> {
  if (previews.length === 1) return previews[0];
  const wide = previews.length >= 3;
  const gridTop = wide ? 230 : 0;
  const tiles = await Promise.all(
    previews.map(async (preview, index) => {
      const first = wide && index === 0;
      const slot = index - Number(wide);
      const width = first ? 1000 : 500;
      const height = first ? 220 : 260;
      return {
        input: await sharp(preview)
          .resize(width - 12, height - 12, { fit: "contain", background: "#101b21" })
          .png()
          .toBuffer(),
        left: first ? 6 : (slot % 2) * 500 + 6,
        top: first ? 6 : gridTop + Math.floor(slot / 2) * 270 + 6,
      };
    }),
  );
  const height = gridTop + Math.ceil((previews.length - Number(wide)) / 2) * 270;
  return sharp({ create: { width: 1000, height, channels: 3, background: "#101b21" } })
    .composite(tiles)
    .png()
    .toBuffer();
}

import {
  Image,
  getCapabilities,
  getCellDimensions,
  getImageDimensions,
  truncateToWidth,
  type ImageDimensions,
  type Component,
  type ImageOptions,
  type ImageTheme,
} from "@earendil-works/pi-tui";
import { calculateImageCellSize } from "@earendil-works/pi-tui/dist/terminal-image.js";
import sharp from "sharp";

export function needsImageSlices(env = process.env): boolean {
  return (
    env.TERM_PROGRAM?.toLowerCase() === "warpterminal" ||
    !!(env.WARP_SESSION_ID || env.WARP_TERMINAL_SESSION_UUID)
  );
}

/** Warp ignores source-rectangle cropping. Whole cell-row images let Pi scroll by clipping rows. */
export class TerminalImage implements Component {
  private readonly fallback: Image;
  private readonly dimensions: ImageDimensions | null;
  private slices?: Image[];
  private key?: string;
  private disposed = false;
  private failed = false;
  constructor(
    private readonly data: string,
    mimeType: string,
    private readonly theme: ImageTheme,
    private readonly options: ImageOptions,
    private readonly changed: () => void,
  ) {
    this.fallback = new Image(data, mimeType, theme, options);
    this.dimensions = getImageDimensions(data, mimeType);
  }
  close(): void {
    this.disposed = true;
    this.slices = undefined;
  }
  invalidate(): void {
    this.fallback.invalidate();
    for (const slice of this.slices ?? []) slice.invalidate();
  }
  private async prepare(
    key: string,
    columns: number,
    rows: number,
  ): Promise<void> {
    const cell = getCellDimensions();
    const width = Math.round(columns * cell.widthPx),
      rowHeight = Math.round(cell.heightPx);
    const pixels = await sharp(Buffer.from(this.data, "base64"))
      .resize(width, rows * rowHeight, {
        fit: "contain",
        background: "#101b21",
      })
      .flatten({ background: "#101b21" })
      .removeAlpha()
      .toColourspace("srgb")
      .raw()
      .toBuffer();
    const slices = await Promise.all(
      Array.from({ length: rows }, async (_, i) => {
        const start = i * width * rowHeight * 3;
        const png = await sharp(
          pixels.subarray(start, start + width * rowHeight * 3),
          {
            raw: { width, height: rowHeight, channels: 3 },
          },
        )
          .png()
          .toBuffer();
        return new Image(png.toString("base64"), "image/png", this.theme, {
          maxWidthCells: columns,
          maxHeightCells: 1,
        });
      }),
    );
    if (this.disposed || key !== this.key) return;
    this.slices = slices;
    this.changed();
  }
  render(width: number): string[] {
    if (!needsImageSlices() || getCapabilities().images !== "kitty")
      return this.fallback.render(width);
    const dimensions = this.dimensions;
    if (!dimensions) return this.fallback.render(width);
    const cell = getCellDimensions();
    const maxWidth = Math.max(
      1,
      Math.min(width - 2, this.options.maxWidthCells ?? 60),
    );
    const size = calculateImageCellSize(
      dimensions,
      maxWidth,
      this.options.maxHeightCells ??
        Math.max(1, Math.ceil((maxWidth * cell.widthPx) / cell.heightPx)),
      cell,
    );
    const key = `${size.columns}:${size.rows}:${cell.widthPx}:${cell.heightPx}`;
    if (key !== this.key && !this.disposed) {
      this.key = key;
      this.slices = undefined;
      this.failed = false;
      void this.prepare(key, size.columns, size.rows).catch(() => {
        if (!this.disposed && key === this.key) {
          this.failed = true;
          this.changed();
        }
      });
    }
    return (
      this.slices?.map((slice) => slice.render(size.columns + 2)[0]) ??
      Array.from({ length: size.rows }, (_, i) =>
        i === 0 && this.failed
          ? truncateToWidth(
              this.theme.fallbackColor("Image preview unavailable"),
              width,
            )
          : "",
      )
    );
  }
}

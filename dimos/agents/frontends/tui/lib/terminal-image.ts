import type { Component, ImageDimensions } from "@earendil-works/pi-tui";
import {
  Image,
  backgroundAnsi,
  foregroundAnsi,
  getCapabilities,
  getCellDimensions,
  getPngDimensions,
  rgbColor,
  truncateToWidth,
} from "@earendil-works/pi-tui";
import { cellPixels, pngOf, rowSlices } from "../../../shared/render/raster.ts";
import { muted } from "../style.ts";

type ImageSize = { readonly maxWidthCells: number; readonly maxHeightCells: number };

type CellMode = "rgb" | "256" | "basic" | "ascii";

type CellPicture =
  | {
      readonly kind: "pending";
      readonly key: string;
      readonly revision: number;
      readonly lines: string[] | null;
      readonly error: boolean;
    }
  | { readonly kind: "ready"; readonly key: string; readonly revision: number; readonly lines: string[] }
  | { readonly kind: "failed"; readonly key: string; readonly revision: number; readonly lines: string[] | null };

const ASCII = " .:-=+*#%@";

const BASIC_COLORS = [
  [0, 0, 0],
  [205, 0, 0],
  [0, 205, 0],
  [205, 205, 0],
  [0, 0, 238],
  [205, 0, 205],
  [0, 205, 205],
  [229, 229, 229],
] as const;

function cellMode(): CellMode {
  const term = process.env.TERM?.toLowerCase() ?? "";
  if (process.env.NO_COLOR !== undefined || term === "dumb") return "ascii";
  if (term === "linux") return "basic";
  if (getCapabilities().trueColor) return "rgb";
  if (term.includes("256color")) return "256";
  if (/^(?:xterm|rxvt|screen|tmux|ansi)(?:-|$)/.test(term)) return "basic";
  return "ascii";
}

function basicColor(red: number, green: number, blue: number): number {
  let nearest = 0;
  let distance = Infinity;
  for (let index = 0; index < BASIC_COLORS.length; index++) {
    const [r, g, b] = BASIC_COLORS[index]!;
    const error = (red - r) ** 2 + (green - g) ** 2 + (blue - b) ** 2;
    if (error < distance) {
      distance = error;
      nearest = index;
    }
  }
  return nearest;
}

function cellLines(pixels: Buffer, columns: number, rows: number, mode: CellMode): string[] {
  const color = (red: number, green: number, blue: number, background: boolean): string => {
    if (mode === "rgb" || mode === "256") {
      const rgb = rgbColor(red, green, blue);
      const colorMode = mode === "rgb" ? "truecolor" : "256color";
      return background ? backgroundAnsi(rgb, colorMode) : foregroundAnsi(rgb, colorMode);
    }
    return `\x1b[${(background ? 40 : 30) + basicColor(red, green, blue)}m`;
  };
  return Array.from({ length: rows }, (_, row) => {
    let line = "";
    for (let column = 0; column < columns; column++) {
      const top = (row * 2 * columns + column) * 3;
      const bottom = top + columns * 3;
      const tr = pixels[top]!,
        tg = pixels[top + 1]!,
        tb = pixels[top + 2]!;
      const br = pixels[bottom]!,
        bg = pixels[bottom + 1]!,
        bb = pixels[bottom + 2]!;
      if (mode === "ascii") {
        const luminance = (tr * 0.2126 + tg * 0.7152 + tb * 0.0722 + br * 0.2126 + bg * 0.7152 + bb * 0.0722) / 2;
        line += ASCII[Math.min(ASCII.length - 1, Math.floor((luminance / 256) * ASCII.length))];
      } else {
        line += `${color(tr, tg, tb, false)}${color(br, bg, bb, true)}▀`;
      }
    }
    return mode === "ascii" ? line : `${line}\x1b[0m`;
  });
}

/** Whether the terminal needs images sliced into rows (Warp with kitty graphics). */
export function needsImageSlices(env: NodeJS.ProcessEnv = process.env): boolean {
  return (
    env.TERM_PROGRAM?.toLowerCase() === "warpterminal" || Boolean(env.WARP_SESSION_ID || env.WARP_TERMINAL_SESSION_UUID)
  );
}

/**
 * pi-tui's Image, sliced into one image per cell row on Warp with kitty graphics: Warp ignores source-rectangle
 * cropping, and whole-row images let a scroll view clip rows instead of squeezing the picture. Moved from dimcode
 * terminal-image.ts.
 */
export class TerminalImage implements Component {
  private png: Buffer;
  /** Null while bytes that are not a PNG are converted to one. */
  private source: ImageDimensions | null;
  private readonly size: ImageSize;
  private readonly changed: () => void;
  private fallback: Image;
  private slices: Image[] | null = null;
  private cellPicture: CellPicture | null = null;
  private key: string | null = null;
  private failed = false;
  private disposed = false;
  private revision = 0;

  /** `image` is a PNG or any format sharp reads, such as the JPEG the model was sent; the terminal gets a PNG. */
  constructor(image: Buffer, size: ImageSize, changed: () => void, imageId?: number) {
    this.size = size;
    this.changed = changed;
    this.png = image;
    this.source = getPngDimensions(image.toString("base64"));
    this.fallback = new Image(image.toString("base64"), "image/png", { fallbackColor: muted }, { ...size, imageId });
    if (this.source !== null) return;
    const revision = this.revision;
    pngOf(image).then(
      (png) => {
        if (this.disposed || revision !== this.revision) return;
        this.png = png;
        this.source = getPngDimensions(png.toString("base64"));
        this.fallback = new Image(png.toString("base64"), "image/png", { fallbackColor: muted }, { ...size, imageId });
        changed();
      },
      () => {
        if (this.disposed || revision !== this.revision) return;
        this.failed = true;
        changed();
      },
    );
  }

  setPng(png: Buffer): void {
    const source = getPngDimensions(png.toString("base64"));
    if (source === null) throw new Error("Expected a PNG image");
    if (this.disposed) return;
    this.revision++;
    this.png = png;
    this.source = source;
    const imageId = this.fallback.getImageId();
    this.fallback = new Image(png.toString("base64"), "image/png", { fallbackColor: muted }, { ...this.size, imageId });
    this.slices = null;
    this.key = null;
    this.failed = false;
  }

  close(): void {
    this.disposed = true;
    this.slices = null;
    this.cellPicture = null;
  }

  invalidate(): void {
    this.fallback.invalidate();
    for (const slice of this.slices ?? []) slice.invalidate();
  }

  render(width: number): string[] {
    const source = this.source;
    if (source === null) return this.failed ? [truncateToWidth(muted("Image preview unavailable"), width)] : [];
    if (getCapabilities().images === null) return this.renderCells(width, source);
    if (!needsImageSlices() || getCapabilities().images !== "kitty") return this.fallback.render(width);
    const cell = getCellDimensions();
    const maxColumns = Math.max(1, Math.min(width - 2, this.size.maxWidthCells));
    const scale = Math.min(
      (maxColumns * cell.widthPx) / source.widthPx,
      (this.size.maxHeightCells * cell.heightPx) / source.heightPx,
    );
    const columns = Math.max(1, Math.min(maxColumns, Math.ceil((source.widthPx * scale) / cell.widthPx)));
    const rows = Math.max(1, Math.min(this.size.maxHeightCells, Math.ceil((source.heightPx * scale) / cell.heightPx)));
    const key = `${columns}:${rows}:${cell.widthPx}:${cell.heightPx}`;
    if (key !== this.key && !this.disposed) {
      this.key = key;
      this.slices = null;
      this.failed = false;
      void this.prepare(key, columns, rows, this.revision);
    }
    if (this.slices !== null) return this.slices.map((slice) => slice.render(columns + 2)[0] ?? "");
    return Array.from({ length: rows }, (_, row) =>
      row === 0 && this.failed ? truncateToWidth(muted("Image preview unavailable"), width) : "",
    );
  }

  private renderCells(width: number, source: ImageDimensions): string[] {
    const cell = getCellDimensions();
    const maxColumns = Math.max(1, Math.min(width - 2, this.size.maxWidthCells));
    const scale = Math.min(
      (maxColumns * cell.widthPx) / source.widthPx,
      (this.size.maxHeightCells * cell.heightPx) / source.heightPx,
    );
    const columns = Math.max(1, Math.min(maxColumns, Math.ceil((source.widthPx * scale) / cell.widthPx)));
    const rows = Math.max(1, Math.min(this.size.maxHeightCells, Math.ceil((source.heightPx * scale) / cell.heightPx)));
    const mode = cellMode();
    const key = `${maxColumns}:${cell.widthPx}:${cell.heightPx}:${mode}`;
    const current = this.cellPicture;
    if (
      !this.disposed &&
      (current?.key !== key || (current.kind !== "pending" && current.revision !== this.revision))
    ) {
      const lines = current?.key === key ? current.lines : null;
      const pending: CellPicture = {
        kind: "pending",
        key,
        revision: this.revision,
        lines,
        error: current?.key === key && current.kind === "failed",
      };
      this.cellPicture = pending;
      void cellPixels(this.png, columns, rows).then(
        (pixels) => {
          if (this.disposed || this.cellPicture !== pending) return;
          this.cellPicture = {
            kind: "ready",
            key,
            revision: pending.revision,
            lines: cellLines(pixels, columns, rows, mode),
          };
          this.changed();
        },
        () => {
          if (this.disposed || this.cellPicture !== pending) return;
          this.cellPicture = { kind: "failed", key, revision: pending.revision, lines: pending.lines };
          this.changed();
        },
      );
    }
    const picture = this.cellPicture;
    if (picture?.key === key) {
      if (picture.kind === "failed" || (picture.kind === "pending" && picture.error)) {
        const error = truncateToWidth(muted("Image preview unavailable"), width);
        return picture.lines === null ? [error] : [...picture.lines.slice(0, -1), error];
      }
      if (picture.lines !== null) return picture.lines;
    }
    return Array.from({ length: rows }, () => "");
  }

  private async prepare(key: string, columns: number, rows: number, revision: number): Promise<void> {
    const cell = getCellDimensions();
    const slices = await rowSlices(this.png, {
      width: Math.round(columns * cell.widthPx),
      rowHeight: Math.round(cell.heightPx),
      rows,
    }).then(
      (pngs) =>
        pngs.map(
          (png) =>
            new Image(
              png.toString("base64"),
              "image/png",
              { fallbackColor: muted },
              { maxWidthCells: columns, maxHeightCells: 1 },
            ),
        ),
      () => null,
    );
    if (this.disposed || key !== this.key || revision !== this.revision) return;
    this.slices = slices;
    this.failed = slices === null;
    this.changed();
  }
}

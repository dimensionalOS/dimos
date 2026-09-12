import {
  Image,
  Text,
  truncateToWidth,
  visibleWidth,
  type Component,
  type TuiMouseEvent,
} from "@earendil-works/pi-tui";
import sharp from "sharp";
import { accent, clean, muted } from "./terminal-style.js";

export interface ResultImage {
  data: string;
  mimeType: string;
  label: string;
}

/** Presentation only: preserves the images produced by the owning memory operation. */
export class ResultImages implements Component {
  private selected = 0;
  private panels: Image[] = [];
  private overview?: Image;
  private disposed = false;
  private error?: string;
  private tabs: Array<{ start: number; end: number; index: number }> = [];
  constructor(
    readonly images: ResultImage[],
    private readonly changed: () => void,
  ) {
    void this.prepare().catch((error) => {
      if (!this.disposed) {
        this.error = clean(String(error));
        this.changed();
      }
    });
  }
  close(): void {
    this.disposed = true;
  }
  invalidate(): void {
    this.overview?.invalidate();
    for (const panel of this.panels) panel.invalidate();
  }
  select(index: number): void {
    this.selected = Math.max(0, Math.min(this.images.length, index));
    this.changed();
  }
  private component(png: Buffer): Image {
    return new Image(
      png.toString("base64"),
      "image/png",
      { fallbackColor: muted },
      { maxWidthCells: 120, maxHeightCells: 26 },
    );
  }
  private async prepare(): Promise<void> {
    const bytes = await Promise.all(
      this.images.map(async (image) =>
        sharp(Buffer.from(image.data, "base64"), { density: 144 })
          .resize({
            width: 1600,
            height: 1200,
            fit: "inside",
            withoutEnlargement: true,
          })
          .flatten({ background: "#101b21" })
          .png()
          .toBuffer(),
      ),
    );
    if (this.disposed) return;
    this.panels = bytes.map((png) => this.component(png));
    if (bytes.length === 1) this.overview = this.panels[0];
    else {
      const shown = bytes.slice(0, 6);
      const columns = 2;
      const top = shown.length >= 3;
      const height =
        (top ? 230 : 0) +
        Math.ceil((shown.length - Number(top)) / columns) * 270;
      const tiles = await Promise.all(
        shown.map(async (png, i) => {
          const first = top && i === 0,
            n = i - Number(top);
          const width = first ? 1000 : 500;
          const imageHeight = first ? 220 : 260;
          const tile = await sharp(png)
            .resize(width - 12, imageHeight - 12, {
              fit: "contain",
              background: "#101b21",
            })
            .png()
            .toBuffer();
          return {
            input: tile,
            left: first ? 6 : (n % columns) * 500 + 6,
            top: first
              ? 6
              : (top ? 230 : 0) + Math.floor(n / columns) * 270 + 6,
          };
        }),
      );
      const png = await sharp({
        create: { width: 1000, height, channels: 3, background: "#101b21" },
      })
        .composite(tiles)
        .png()
        .toBuffer();
      if (this.disposed) return;
      this.overview = this.component(png);
    }
    this.changed();
  }
  handleMouse(
    event: TuiMouseEvent,
  ): { handled: boolean; render: boolean } | undefined {
    if (event.type !== "press" || event.y !== 0) return;
    const tab = this.tabs.find(
      (tab) => event.x >= tab.start && event.x < tab.end,
    );
    if (!tab) return;
    this.select(tab.index);
    return { handled: true, render: true };
  }
  render(width: number): string[] {
    const labels = ["Overview", ...this.images.map((image) => image.label)];
    let line = " ";
    this.tabs = [];
    for (const [index, label] of labels.entries()) {
      const text = `${index} ${clean(label).slice(0, 16)}  `;
      const start = visibleWidth(line),
        end = start + visibleWidth(text);
      if (end > width) break;
      line += (index === this.selected ? accent : muted)(text);
      this.tabs.push({ start, end, index });
    }
    const panel = this.selected
      ? this.panels[this.selected - 1]
      : this.overview;
    return [
      truncateToWidth(line, width),
      ...(panel
        ? panel.render(width)
        : new Text(
            muted(
              this.error
                ? ` Preview unavailable: ${this.error}`
                : " Preparing result views…",
            ),
            0,
            0,
          ).render(width)),
      ...new Text(
        muted(
          ` /panel 0–${this.images.length} selects a view · original tool output${this.images.length > 6 && !this.selected ? " · overview shows first 6" : ""}`,
        ),
        0,
        0,
      ).render(width),
    ];
  }
}

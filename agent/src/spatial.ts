import { basename } from "node:path";
import {
  getCapabilities,
  Image,
  matchesKey,
  Text,
  truncateToWidth,
  type Component,
} from "@earendil-works/pi-tui";
import {
  fitClouds,
  paintCloud,
  projectPoint,
  POINT_LIMIT,
  type Cloud,
  type Camera,
} from "./points.js";
export { readCloud } from "./points.js";
import { accent, amber, clean, muted, StatusLine } from "./terminal-style.js";

/** A presentation of one immutable observation. View controls never execute a DimOS query. */
export class SpatialView implements Component {
  private angle = -0.58;
  private zoom = 1;
  private graphics = true;
  private details = false;
  private image?: Image;
  private revision = 0;
  private disposed = false;
  private animation?: ReturnType<typeof setInterval>;
  private readonly camera: Camera;
  private readonly selected: Set<number>;
  private readonly step: number;
  private readonly heading: StatusLine;
  expanded = false;
  constructor(
    readonly cloud: Cloud,
    readonly source: string,
    readonly sha256: string,
    private readonly changed: () => void,
  ) {
    this.camera = fitClouds([cloud]);
    this.selected = new Set(cloud.selectedIndices);
    this.step = Math.max(1, Math.ceil(cloud.points.length / POINT_LIMIT));
    this.heading = new StatusLine(() => [
      accent("⠿ PointCloud") + muted("  " + clean(basename(source))),
      muted(
        `${cloud.points.length.toLocaleString()} points · ${clean(cloud.frame ?? "frame unspecified")}`,
      ),
    ]);
    this.refresh();
  }
  invalidate(): void {
    this.image?.invalidate();
  }
  close(): void {
    this.disposed = true;
    this.revision++;
    clearInterval(this.animation);
  }
  private retained(i: number): boolean {
    return this.selected.has(i);
  }
  private async raster(revision: number): Promise<void> {
    const png = await paintCloud(
      this.cloud,
      this.camera,
      this.angle,
      this.zoom,
    );
    if (this.disposed || revision !== this.revision) return;
    this.image = new Image(
      png.toString("base64"),
      "image/png",
      { fallbackColor: muted },
      {
        maxHeightCells: this.expanded ? 30 : 20,
        maxWidthCells: this.expanded ? 150 : 100,
      },
    );
    this.changed();
  }
  private refresh(): void {
    const revision = ++this.revision;
    if (this.graphics && getCapabilities().images)
      void this.raster(revision).catch(() => {
        if (revision === this.revision) {
          this.graphics = false;
          this.changed();
        }
      });
    this.changed();
  }
  handleInput(key: string): void {
    if (key === "r" || matchesKey(key, "right") || matchesKey(key, "left")) {
      clearInterval(this.animation);
      const direction = matchesKey(key, "left") ? -1 : 1;
      let frames = 0;
      this.animation = setInterval(() => {
        this.angle += (direction * Math.PI) / 16;
        this.refresh();
        if (++frames === 4) clearInterval(this.animation);
      }, 80);
    } else {
      if (key === "+" || key === "=") this.zoom = Math.min(4, this.zoom * 1.2);
      else if (key === "-") this.zoom = Math.max(0.25, this.zoom / 1.2);
      else if (key === "g") {
        this.graphics = !this.graphics;
        this.image = undefined;
      } else if (key === "d") this.details = !this.details;
      else if (key === "0") {
        this.zoom = 1;
        this.angle = -0.58;
      } else return;
      this.refresh();
    }
  }
  setExpanded(value: boolean): void {
    this.expanded = value;
    this.image = undefined;
    this.refresh();
  }
  private braille(width: number): string[] {
    const rows = this.expanded ? 26 : 14,
      cols = Math.max(1, width - 2);
    const cells = Array.from({ length: rows }, () =>
      Array.from({ length: cols }, () => ({ dots: 0, selected: false })),
    );
    const bits = [
      [1, 8],
      [2, 16],
      [4, 32],
      [64, 128],
    ];
    for (let i = 0; i < this.cloud.points.length; i += this.step) {
      const [px, py] = projectPoint(
        this.cloud.points[i],
        this.camera,
        cols * 2,
        rows * 4,
        this.angle,
        this.zoom,
      );
      const x = Math.floor(px),
        y = Math.floor(py);
      if (x < 0 || y < 0 || x >= cols * 2 || y >= rows * 4) continue;
      const cell = cells[Math.floor(y / 4)][Math.floor(x / 2)];
      cell.dots |= bits[y % 4][x % 2];
      cell.selected ||= this.retained(i);
    }
    const masked = !!this.cloud.selectedIndices;
    return cells.map(
      (row) =>
        " " +
        row
          .map((cell) =>
            cell.dots
              ? (cell.selected ? amber : masked ? muted : accent)(
                  String.fromCharCode(0x2800 + cell.dots),
                )
              : " ",
          )
          .join(""),
    );
  }
  render(width: number): string[] {
    const lines = [
      "",
      ...this.heading.render(width),
      muted("─".repeat(Math.max(0, width))),
    ];
    lines.push(
      ...(this.graphics && getCapabilities().images && this.image
        ? this.image.render(width)
        : this.braille(width)),
    );
    lines.push(
      truncateToWidth(
        " " +
          (this.cloud.selectedIndices
            ? amber(
                `${this.selected.size.toLocaleString()} source-selected points`,
              )
            : muted("Source geometry")),
        width,
      ),
    );
    lines.push(
      truncateToWidth(
        muted(
          ` ${this.cloud.timestamp ?? "timestamp unspecified"} · display 1/${this.step} · ${this.graphics && getCapabilities().images ? "graphics" : "Braille"}`,
        ),
        width,
      ),
    );
    lines.push(
      ...new Text(
        muted(
          this.expanded
            ? " ←/→ rotate · +/- zoom · g graphics · d source · 0 reset · Esc close"
            : " /view rotate, zoom and inspect · /expand tool details",
        ),
        0,
        0,
      ).render(width),
    );
    if (this.details)
      lines.push(
        ...new Text(
          clean(this.source) + "\nSHA-256: " + this.sha256,
          1,
          0,
        ).render(width),
      );
    return lines;
  }
}

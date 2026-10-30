import type { Camera, Cloud } from "../../../shared/render/points.ts";
import type { RenderedView } from "../../../shared/protocol/media.ts";
import type { ViewPanel } from "./base.ts";
import { StatusLine, accent, amber, muted } from "../style.ts";
import { TerminalImage } from "../lib/terminal-image.ts";
import { Text, getCapabilities, matchesKey, truncateToWidth } from "@earendil-works/pi-tui";
import { basename } from "node:path";
import { clean } from "../../../shared/view-models/text.ts";
import { fitCamera, paintCloud, parseCloud, projectPoint } from "../../../shared/render/points.ts";
import { readVerified } from "../../../shared/render/files.ts";

type PointsView = Extract<RenderedView, { readonly kind: "points" }>;

const PAINT_SIZE = { width: 900, height: 448 };

const BRAILLE_ROWS = 14;

const BRAILLE_POINT_LIMIT = 20_000;

const BRAILLE_BITS = [
  [0x01, 0x08],
  [0x02, 0x10],
  [0x04, 0x20],
  [0x40, 0x80],
] as const;

/**
 * A cloud that rotates with the arrow keys and `r`, zooms 0.25-4 with `+`/`-`, `g` graphics or Braille, `d` details,
 * `0` reset. Reads the source cloud and repaints with shared/render/points.ts. Moved from dimcode spatial.ts.
 */
export class SpatialPanel implements ViewPanel<"points"> {
  readonly view: PointsView;
  private readonly changed: () => void;
  private readonly heading: StatusLine;
  private cloud: Cloud | null = null;
  private fitted: Camera | null = null;
  private camera: Camera | null = null;
  private image: TerminalImage | null = null;
  private graphics = true;
  private details = false;
  private revision = 0;
  private animation: NodeJS.Timeout | null = null;
  private error: string | null = null;
  private disposed = false;

  constructor(view: PointsView, changed: () => void) {
    this.view = view;
    this.changed = changed;
    this.heading = new StatusLine(() => [
      accent("⠿ PointCloud") + muted(`  ${clean(basename(view.source.path))}`),
      muted(`${view.points.toLocaleString()} points · ${clean(view.frame ?? "frame unspecified")}`),
    ]);
    readVerified(view.source).then(
      (bytes) => {
        if (this.disposed) return;
        this.cloud = parseCloud(JSON.parse(bytes.toString("utf8")));
        this.fitted = fitCamera(this.cloud);
        this.camera = this.fitted;
        this.refresh();
      },
      (error: unknown) => {
        if (this.disposed) return;
        this.error = clean(String(error));
        changed();
      },
    );
  }

  handleInput(data: string): void {
    const camera = this.camera;
    if (camera === null) return;
    if (data === "r" || matchesKey(data, "right") || matchesKey(data, "left")) {
      if (this.animation !== null) clearInterval(this.animation);
      const direction = matchesKey(data, "left") ? -1 : 1;
      let frames = 0;
      this.animation = setInterval(() => {
        const current = this.camera;
        if (current === null) return;
        this.camera = { ...current, yaw: current.yaw + (direction * Math.PI) / 16 };
        this.refresh();
        frames += 1;
        if (frames === 4 && this.animation !== null) clearInterval(this.animation);
      }, 80);
      return;
    }
    if (data === "+" || data === "=") this.camera = { ...camera, zoom: Math.min(4, camera.zoom * 1.2) };
    else if (data === "-") this.camera = { ...camera, zoom: Math.max(0.25, camera.zoom / 1.2) };
    else if (data === "g") {
      this.graphics = !this.graphics;
      this.image?.close();
      this.image = null;
    } else if (data === "d") this.details = !this.details;
    else if (data === "0") this.camera = this.fitted;
    else return;
    this.refresh();
  }

  render(width: number): string[] {
    const lines = ["", ...this.heading.render(width), muted("─".repeat(Math.max(0, width)))];
    if (this.cloud === null) {
      lines.push(
        ...new Text(
          muted(this.error === null ? " Reading the cloud…" : ` Cloud unavailable: ${this.error}`),
          0,
          0,
        ).render(width),
      );
      return lines;
    }
    lines.push(
      ...(this.showsGraphics() && this.image !== null ? this.image.render(width) : this.braille(this.cloud, width)),
    );
    const selected = this.cloud.selectedIndices;
    lines.push(
      truncateToWidth(
        ` ${selected === null ? muted("Source geometry") : amber(`${selected.length.toLocaleString()} source-selected points`)}`,
        width,
      ),
      truncateToWidth(
        muted(` ${this.cloud.timestamp ?? "timestamp unspecified"} · ${this.showsGraphics() ? "graphics" : "Braille"}`),
        width,
      ),
      ...new Text(muted(" ←/→ rotate · +/- zoom · g graphics · d source · 0 reset"), 0, 0).render(width),
    );
    if (this.details)
      lines.push(
        ...new Text(`${clean(this.view.source.path)}\nSHA-256: ${this.view.source.sha256}`, 1, 0).render(width),
      );
    return lines;
  }

  invalidate(): void {
    this.image?.invalidate();
  }

  dispose(): void {
    this.disposed = true;
    this.revision += 1;
    if (this.animation !== null) clearInterval(this.animation);
    this.image?.close();
  }

  private showsGraphics(): boolean {
    return this.graphics && Boolean(getCapabilities().images);
  }

  private refresh(): void {
    const revision = ++this.revision;
    const cloud = this.cloud;
    const camera = this.camera;
    if (cloud !== null && camera !== null && this.showsGraphics()) {
      paintCloud(cloud, camera, PAINT_SIZE).then(
        (png) => {
          if (this.disposed || revision !== this.revision) return;
          this.image?.close();
          this.image = new TerminalImage(png, { maxWidthCells: 100, maxHeightCells: 20 }, this.changed);
          this.changed();
        },
        () => {
          if (revision !== this.revision) return;
          this.graphics = false;
          this.changed();
        },
      );
    }
    this.changed();
  }

  /** Braille dots from the same projection paintCloud uses; selected points amber when the source selects some. */
  private braille(cloud: Cloud, width: number): string[] {
    const camera = this.camera;
    if (camera === null) return [];
    const columns = Math.max(1, width - 2);
    const size = { width: columns * 2, height: BRAILLE_ROWS * 4 };
    const dots = new Uint8Array(columns * BRAILLE_ROWS);
    const marked = new Uint8Array(columns * BRAILLE_ROWS);
    const plot = (index: number, mark: boolean): void => {
      const point = cloud.points[index];
      if (point === undefined) return;
      const [px, py] = projectPoint(point, camera, size);
      const x = Math.floor(px);
      const y = Math.floor(py);
      if (x < 0 || y < 0 || x >= size.width || y >= size.height) return;
      const cell = Math.floor(y / 4) * columns + Math.floor(x / 2);
      dots[cell] = (dots[cell] ?? 0) | (BRAILLE_BITS[y % 4]?.[x % 2] ?? 0);
      if (mark) marked[cell] = 1;
    };
    const step = Math.max(1, Math.ceil(cloud.points.length / BRAILLE_POINT_LIMIT));
    for (let index = 0; index < cloud.points.length; index += step) plot(index, false);
    const selected = [...new Set(cloud.selectedIndices)];
    const selectedStep = Math.max(1, Math.ceil(selected.length / BRAILLE_POINT_LIMIT));
    for (let n = 0; n < selected.length; n += selectedStep) plot(selected[n], true);
    const masked = cloud.selectedIndices !== null;
    return Array.from({ length: BRAILLE_ROWS }, (_, row) => {
      let line = " ";
      for (let column = 0; column < columns; column++) {
        const cell = row * columns + column;
        const value = dots[cell] ?? 0;
        if (value === 0) line += " ";
        else {
          const glyph = String.fromCharCode(0x2800 + value);
          line += marked[cell] === 1 ? amber(glyph) : masked ? muted(glyph) : accent(glyph);
        }
      }
      return line;
    });
  }
}

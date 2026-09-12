import { createHash } from "node:crypto";
import { readFile, stat } from "node:fs/promises";
import { basename } from "node:path";
import {
  getCapabilities,
  Image,
  matchesKey,
  Text,
  truncateToWidth,
  type Component,
} from "@earendil-works/pi-tui";
import sharp from "sharp";
import { z } from "zod";
import { accent, amber, clean, muted, StatusLine } from "./terminal-style.js";

export const cloudSchema = z
  .object({
    points: z
      .array(
        z.tuple([
          z.number().finite(),
          z.number().finite(),
          z.number().finite(),
        ]),
      )
      .min(1)
      .max(1_000_000),
    frame: z.string().optional(),
    timestamp: z.number().finite().optional(),
    selectedIndices: z.array(z.number().int().nonnegative()).optional(),
  })
  .refine(
    (cloud) =>
      cloud.selectedIndices?.every((i) => i < cloud.points.length) ?? true,
    "Selection index outside point cloud",
  );
type Cloud = z.infer<typeof cloudSchema>;
type XYZ = Cloud["points"][number];

export async function readCloud(
  path: string,
  digest?: string,
): Promise<{ cloud: Cloud; sha256: string }> {
  if ((await stat(path)).size > 32 * 1024 * 1024)
    throw new Error("Point-cloud export exceeds 32 MiB");
  const bytes = await readFile(path);
  const sha256 = createHash("sha256").update(bytes).digest("hex");
  if (digest && digest !== sha256)
    throw new Error(
      "Source changed since the agent inspected it; showing the saved result instead",
    );
  return {
    cloud: cloudSchema.parse(JSON.parse(bytes.toString("utf8"))),
    sha256,
  };
}

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
  private readonly center: XYZ;
  private readonly extent: number;
  private readonly floor: number;
  private readonly height: number;
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
    const low: XYZ = [Infinity, Infinity, Infinity],
      high: XYZ = [-Infinity, -Infinity, -Infinity];
    for (const point of cloud.points)
      for (let i = 0; i < 3; i++) {
        low[i] = Math.min(low[i], point[i]);
        high[i] = Math.max(high[i], point[i]);
      }
    this.center = low.map((n, i) => (n + high[i]) / 2) as XYZ;
    this.floor = low[2];
    this.height = Math.max(high[2] - low[2], 0.001);
    this.extent = Math.max(...high.map((n, i) => n - low[i]), 0.001);
    this.selected = new Set(cloud.selectedIndices);
    this.step = Math.max(1, Math.ceil(cloud.points.length / 20_000));
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
  private project(point: XYZ, width: number, height: number): [number, number] {
    const [x, y, z] = point.map((v, i) => v - this.center[i]);
    const c = Math.cos(this.angle),
      s = Math.sin(this.angle);
    const scale =
      (Math.min(width / 1.65, height / 1.2) / this.extent) * this.zoom;
    return [
      width / 2 + (x * c - y * s) * scale,
      height / 2 + (x * s + y * c) * scale * 0.5 - z * scale,
    ];
  }
  private async raster(revision: number): Promise<void> {
    const masked = !!this.cloud.selectedIndices;
    const circles: string[] = [];
    for (let i = 0; i < this.cloud.points.length; i += this.step) {
      const [x, y] = this.project(this.cloud.points[i], 900, 400);
      const keep = this.retained(i);
      const elevation = (this.cloud.points[i][2] - this.floor) / this.height;
      circles.push(
        `<circle cx="${x.toFixed(1)}" cy="${y.toFixed(1)}" r="${keep ? 1.8 : 1}" fill="${keep ? "#f5c078" : "#6ae5cf"}" opacity="${masked && !keep ? 0.09 : keep ? 0.9 : 0.12 + elevation * 0.75}"/>`,
      );
    }
    const grid: string[] = [];
    for (let i = -3; i <= 3; i++)
      for (const axis of [0, 1]) {
        const a = [...this.center] as XYZ,
          b = [...this.center] as XYZ;
        a[axis] += (i * this.extent) / 6;
        b[axis] = a[axis];
        a[1 - axis] -= this.extent / 2;
        b[1 - axis] += this.extent / 2;
        a[2] = this.floor;
        b[2] = a[2];
        const start = this.project(a, 900, 400),
          end = this.project(b, 900, 400);
        grid.push(
          `<path d="M${start.join(" ")}L${end.join(" ")}" stroke="#284047"/>`,
        );
      }
    const png = await sharp(
      Buffer.from(
        `<svg xmlns="http://www.w3.org/2000/svg" width="900" height="400"><rect width="900" height="400" fill="#101b21"/>${grid.join("")}${circles.join("")}<path d="M30 365h25M30 365v-25M30 365l-14 12" stroke="#8ba4a8"/><g fill="#8ba4a8" font-family="monospace" font-size="12"><text x="59" y="370">x</text><text x="26" y="334">z</text><text x="7" y="389">y</text></g></svg>`,
      ),
    )
      .removeAlpha()
      .png()
      .toBuffer();
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
      const [px, py] = this.project(this.cloud.points[i], cols * 2, rows * 4);
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

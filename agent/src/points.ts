import sharp from "sharp";
import { z } from "zod";
import { readArtifact } from "./artifacts.js";

const xyz = z.tuple([
  z.number().finite(),
  z.number().finite(),
  z.number().finite(),
]);
const rgb = z.tuple([
  z.number().int().min(0).max(255),
  z.number().int().min(0).max(255),
  z.number().int().min(0).max(255),
]);
export const cloudSchema = z
  .object({
    points: z.array(xyz).min(1).max(1_000_000),
    colors: z.array(rgb).optional(),
    frame: z.string().optional(),
    timestamp: z.number().finite().optional(),
    selectedIndices: z.array(z.number().int().nonnegative()).optional(),
  })
  .refine(
    (cloud) =>
      cloud.selectedIndices?.every((i) => i < cloud.points.length) ?? true,
    "Selection index outside point cloud",
  )
  .refine(
    (cloud) => !cloud.colors || cloud.colors.length === cloud.points.length,
    "RGB count must match point count",
  );
export type Cloud = z.infer<typeof cloudSchema>;
type XYZ = Cloud["points"][number];
export interface Camera {
  center: XYZ;
  extent: number;
  floor: number;
  height: number;
}
export const POINT_LIMIT = 20_000;

export async function readCloud(
  path: string,
  digest?: string,
): Promise<{ cloud: Cloud; sha256: string }> {
  const { bytes, sha256 } = await readArtifact(path, digest);
  return {
    cloud: cloudSchema.parse(JSON.parse(bytes.toString("utf8"))),
    sha256,
  };
}

/** Shared presentation bounds; no registration, reconstruction or filtering. */
export function fitClouds(clouds: readonly Cloud[]): Camera {
  const low: XYZ = [Infinity, Infinity, Infinity],
    high: XYZ = [-Infinity, -Infinity, -Infinity];
  for (const cloud of clouds)
    for (const point of cloud.points)
      for (let i = 0; i < 3; i++) {
        low[i] = Math.min(low[i], point[i]);
        high[i] = Math.max(high[i], point[i]);
      }
  return {
    center: low.map((n, i) => (n + high[i]) / 2) as XYZ,
    floor: low[2],
    height: Math.max(high[2] - low[2], 0.001),
    extent: Math.max(...high.map((n, i) => n - low[i]), 0.001),
  };
}
export function projectPoint(
  point: XYZ,
  camera: Camera,
  width: number,
  height: number,
  angle = -0.58,
  zoom = 1,
): [number, number] {
  const [x, y, z] = point.map((v, i) => v - camera.center[i]);
  const c = Math.cos(angle),
    s = Math.sin(angle);
  const scale = (Math.min(width / 1.65, height / 1.2) / camera.extent) * zoom;
  return [
    width / 2 + (x * c - y * s) * scale,
    height / 2 + (x * s + y * c) * scale * 0.5 - z * scale,
  ];
}
export async function paintCloud(
  cloud: Cloud,
  camera = fitClouds([cloud]),
  angle = -0.58,
  zoom = 1,
): Promise<Buffer> {
  const selected = new Set(cloud.selectedIndices),
    step = Math.max(1, Math.ceil(cloud.points.length / POINT_LIMIT));
  const circles: string[] = [],
    grid: string[] = [];
  for (let i = 0; i < cloud.points.length; i += step) {
    const [x, y] = projectPoint(cloud.points[i], camera, 900, 400, angle, zoom);
    const keep = selected.has(i),
      elevation = (cloud.points[i][2] - camera.floor) / camera.height;
    const color = keep
      ? "#f5c078"
      : cloud.colors
        ? `rgb(${cloud.colors[i].join(",")})`
        : "#6ae5cf";
    circles.push(
      `<circle cx="${x.toFixed(1)}" cy="${y.toFixed(1)}" r="${keep ? 1.8 : 1}" fill="${color}" opacity="${cloud.selectedIndices && !keep ? 0.09 : keep || cloud.colors ? 0.9 : 0.12 + elevation * 0.75}"/>`,
    );
  }
  for (let i = -3; i <= 3; i++)
    for (const axis of [0, 1]) {
      const a = [...camera.center] as XYZ,
        b = [...camera.center] as XYZ;
      a[axis] += (i * camera.extent) / 6;
      b[axis] = a[axis];
      a[1 - axis] -= camera.extent / 2;
      b[1 - axis] += camera.extent / 2;
      a[2] = camera.floor;
      b[2] = a[2];
      grid.push(
        `<path d="M${projectPoint(a, camera, 900, 400, angle, zoom).join(" ")}L${projectPoint(b, camera, 900, 400, angle, zoom).join(" ")}" stroke="#284047"/>`,
      );
    }
  const legend = `${cloud.points.length.toLocaleString()} points | display 1/${step} | ${cloud.colors ? "source RGB" : `height brightness z=${camera.floor.toFixed(2)}..${(camera.floor + camera.height).toFixed(2)}`} | ${selected.size} selected`;
  return sharp(
    Buffer.from(
      `<svg xmlns="http://www.w3.org/2000/svg" width="900" height="448"><rect width="900" height="448" fill="#101b21"/>${grid.join("")}${circles.join("")}<path d="M30 365h25M30 365v-25M30 365l-14 12" stroke="#8ba4a8"/><g fill="#8ba4a8" font-family="monospace" font-size="12"><text x="59" y="370">x</text><text x="26" y="334">z</text><text x="7" y="389">y</text><text x="16" y="425">${legend}</text></g></svg>`,
    ),
  )
    .removeAlpha()
    .png()
    .toBuffer();
}

// Shared PointCloud2 rasterizer — the height-coloured top-down draw extracted from WorldView so the
// CloudCompare panel renders raw / downsampled / Draco-decoded clouds identically. `cloud` is any
// object shaped like a decoded PointCloud2 (data:Uint8Array, point_step, fields:[{name,offset}],
// is_bigendian) — including the synthetic object CloudCompare builds from a Draco-decoded xyz buffer.
// W/H project world metres → canvas px (the caller owns the view transform, so a shared view can be
// applied to several canvases). Returns the total point count in the frame (pre-decimation) for labels.

export interface CloudLike {
  data: Uint8Array;
  point_step: number;
  fields: { name: string; offset: number }[];
  is_bigendian?: number | boolean;
}

/** xy bounds of a cloud (world metres), for computing a shared auto-fit view. null if empty. */
export function cloudBounds(
  cloud: CloudLike | null | undefined,
): { minX: number; maxX: number; minY: number; maxY: number } | null {
  if (!cloud?.data?.byteLength || !cloud.point_step || !cloud.fields?.length) return null;
  const fx = cloud.fields.find((f) => f.name === "x");
  const fy = cloud.fields.find((f) => f.name === "y");
  if (!fx || !fy) return null;
  const dv = new DataView(cloud.data.buffer, cloud.data.byteOffset, cloud.data.byteLength);
  const le = !cloud.is_bigendian;
  const n = Math.floor(cloud.data.byteLength / cloud.point_step);
  const stride = Math.max(1, Math.ceil(n / 4000)); // sample — exact bounds aren't needed to fit a view
  let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
  for (let i = 0; i < n; i += stride) {
    const b = i * cloud.point_step;
    const x = dv.getFloat32(b + fx.offset, le);
    const y = dv.getFloat32(b + fy.offset, le);
    if (!Number.isFinite(x) || !Number.isFinite(y)) continue;
    if (x < minX) minX = x;
    if (x > maxX) maxX = x;
    if (y < minY) minY = y;
    if (y > maxY) maxY = y;
  }
  return minX === Infinity ? null : { minX, maxX, minY, maxY };
}

/** Draw the cloud height-coloured (blue→amber by z), stride-decimated to `cap` points. Returns the
 *  total point count in the frame. */
export function drawCloud(
  ctx: CanvasRenderingContext2D,
  cloud: CloudLike | null | undefined,
  W: (wx: number) => number,
  H: (wy: number) => number,
  cap = 4000,
  size = 1.6,
): number {
  if (!cloud?.data?.byteLength || !cloud.point_step || !cloud.fields?.length) return 0;
  const fx = cloud.fields.find((f) => f.name === "x");
  const fy = cloud.fields.find((f) => f.name === "y");
  const fz = cloud.fields.find((f) => f.name === "z");
  if (!fx || !fy) return 0;
  const dv = new DataView(cloud.data.buffer, cloud.data.byteOffset, cloud.data.byteLength);
  const le = !cloud.is_bigendian;
  const n = Math.floor(cloud.data.byteLength / cloud.point_step);
  const stride = Math.max(1, Math.ceil(n / cap));
  for (let i = 0; i < n; i += stride) {
    const b = i * cloud.point_step;
    const x = dv.getFloat32(b + fx.offset, le);
    const y = dv.getFloat32(b + fy.offset, le);
    if (!Number.isFinite(x) || !Number.isFinite(y)) continue;
    const z = fz ? dv.getFloat32(b + fz.offset, le) : 0;
    const t = Math.max(0, Math.min(1, (z + 0.2) / 2)); // ~[-0.2,1.8]m → 0..1
    ctx.fillStyle = `hsl(${200 - t * 160}, 85%, ${42 + t * 18}%)`; // blue→amber by height
    ctx.fillRect(W(x), H(y), size, size);
  }
  return n;
}

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

/** Extract ALL points as an interleaved xyz Float32Array (N×3) — the vertex buffer a WebGL point
 *  renderer wants. Skips non-finite points; z defaults to 0 if the cloud has no z field. */
export function cloudToXYZ(cloud: CloudLike | null | undefined): Float32Array | null {
  if (!cloud?.data?.byteLength || !cloud.point_step || !cloud.fields?.length) return null;
  const fx = cloud.fields.find((f) => f.name === "x");
  const fy = cloud.fields.find((f) => f.name === "y");
  const fz = cloud.fields.find((f) => f.name === "z");
  if (!fx || !fy) return null;
  const dv = new DataView(cloud.data.buffer, cloud.data.byteOffset, cloud.data.byteLength);
  const le = !cloud.is_bigendian;
  const n = Math.floor(cloud.data.byteLength / cloud.point_step);
  const out = new Float32Array(n * 3);
  let m = 0;
  for (let i = 0; i < n; i++) {
    const b = i * cloud.point_step;
    const x = dv.getFloat32(b + fx.offset, le);
    const y = dv.getFloat32(b + fy.offset, le);
    if (!Number.isFinite(x) || !Number.isFinite(y)) continue;
    const z = fz ? dv.getFloat32(b + fz.offset, le) : 0;
    out[m++] = x;
    out[m++] = y;
    out[m++] = z;
  }
  return m === out.length ? out : out.subarray(0, m);
}

/** Centroid + a fit radius + z range from an interleaved xyz buffer, for auto-framing a 3D camera
 *  and for the shared height-colour range. */
export function cloudExtent(
  xyz: Float32Array,
): { center: [number, number, number]; radius: number; zMin: number; zMax: number } {
  let minX = Infinity, minY = Infinity, minZ = Infinity;
  let maxX = -Infinity, maxY = -Infinity, maxZ = -Infinity;
  for (let i = 0; i < xyz.length; i += 3) {
    const x = xyz[i], y = xyz[i + 1], z = xyz[i + 2];
    if (x < minX) minX = x;
    if (x > maxX) maxX = x;
    if (y < minY) minY = y;
    if (y > maxY) maxY = y;
    if (z < minZ) minZ = z;
    if (z > maxZ) maxZ = z;
  }
  if (minX === Infinity) return { center: [0, 0, 0], radius: 5, zMin: 0, zMax: 1 };
  return {
    center: [(minX + maxX) / 2, (minY + maxY) / 2, (minZ + maxZ) / 2],
    radius: Math.max(0.5, Math.hypot(maxX - minX, maxY - minY, maxZ - minZ) / 2),
    zMin: minZ,
    zMax: maxZ,
  };
}

/** Wrap a Draco-decoded interleaved xyz Float32Array as a PointCloud2-shaped object so the 2D
 *  `drawCloud` renders it unchanged (contiguous float32 xyz → point_step 12, little-endian). */
export function synthCloud(xyz: Float32Array): CloudLike {
  return {
    data: new Uint8Array(xyz.buffer, xyz.byteOffset, xyz.byteLength),
    point_step: 12,
    fields: [{ name: "x", offset: 0 }, { name: "y", offset: 4 }, { name: "z", offset: 8 }],
    is_bigendian: 0,
  };
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

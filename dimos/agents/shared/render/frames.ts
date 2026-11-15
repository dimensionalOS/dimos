import sharp from "sharp";
import type { Camera } from "#dimos/agents/shared/render/points.ts";

/** The pixels a card shows its picture in; every frame is painted at this size, so a swap sends only what shows. */
export type FrameBox = { readonly width: number; readonly height: number };

const BACKGROUND = [16, 27, 33] as const;

const POINT = [106, 229, 207] as const;

const UNKNOWN_GREY = 128;

const PNG = { compressionLevel: 1 } as const;

/**
 * Live pictures for stream cards, one PNG per frame. `points.ts`'s `paintCloud` draws an SVG for the model and takes
 * 389 to 699 ms; a live cloud is projected straight into a raw buffer instead.
 */
export async function imageFrame(jpeg: Uint8Array, box: FrameBox): Promise<Buffer> {
  return sharp(jpeg).resize({ width: box.width, height: box.height, fit: "inside" }).png(PNG).toBuffer();
}

/** A top-down view of one frame's finite points, +x right and +y up. */
export function topDownCamera(xyz: Float32Array): Camera {
  const low = [Infinity, Infinity, Infinity];
  const high = [-Infinity, -Infinity, -Infinity];
  for (let i = 0; i + 2 < xyz.length; i += 3) {
    const x = xyz[i];
    const y = xyz[i + 1];
    const z = xyz[i + 2];
    if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) continue;
    low[0] = Math.min(low[0], x);
    low[1] = Math.min(low[1], y);
    low[2] = Math.min(low[2], z);
    high[0] = Math.max(high[0], x);
    high[1] = Math.max(high[1], y);
    high[2] = Math.max(high[2], z);
  }
  if (low[0] > high[0]) return { center: [0, 0, 0], extent: 1, floor: 0, height: 1, yaw: 0, zoom: 1 };
  return {
    center: [(low[0] + high[0]) / 2, (low[1] + high[1]) / 2, (low[2] + high[2]) / 2],
    extent: Math.max(high[0] - low[0], high[1] - low[1], 0.001),
    floor: low[2],
    height: Math.max(high[2] - low[2], 0.001),
    yaw: 0,
    zoom: 1,
  };
}

/** The cloud from above through `camera`, rotated by its yaw; a higher point is brighter. */
export async function cloudFrame(xyz: Float32Array, camera: Camera, box: FrameBox): Promise<Buffer> {
  const { width, height } = box;
  const pixels = Buffer.alloc(width * height * 3, Buffer.from(BACKGROUND));
  const cos = Math.cos(camera.yaw);
  const sin = Math.sin(camera.yaw);
  // One pixel short of the box, so the farthest points land on the last row and column rather than past them.
  const scale = ((Math.min(width, height) - 1) / camera.extent) * camera.zoom;
  for (let i = 0; i + 2 < xyz.length; i += 3) {
    const dx = xyz[i] - camera.center[0];
    const dy = xyz[i + 1] - camera.center[1];
    const column = Math.floor(width / 2 + (dx * cos - dy * sin) * scale);
    const row = Math.floor(height / 2 - (dx * sin + dy * cos) * scale);
    if (!(column >= 0 && column < width && row >= 0 && row < height)) continue;
    const level = Math.min(1, Math.max(0, (xyz[i + 2] - camera.floor) / camera.height));
    const shade = 0.35 + 0.65 * (Number.isFinite(level) ? level : 0);
    const offset = (row * width + column) * 3;
    for (let channel = 0; channel < 3; channel++) {
      pixels[offset + channel] = Math.max(pixels[offset + channel], Math.round(POINT[channel] * shade));
    }
  }
  return sharp(pixels, { raw: { width, height, channels: 3 } })
    .png(PNG)
    .toBuffer();
}

/** An occupancy grid in grey, fitted in the box: unknown mid grey, free white, occupied black, +y up. */
export async function gridFrame(cells: Int8Array, width: number, height: number, box: FrameBox): Promise<Buffer> {
  const grey = Buffer.alloc(width * height);
  for (let i = 0; i < grey.length; i++) {
    const cell = cells[i];
    grey[i] = cell < 0 ? UNKNOWN_GREY : 255 - Math.round((Math.min(cell, 100) * 255) / 100);
  }
  const fit = Math.min(box.width / width, box.height / height);
  return sharp(grey, { raw: { width, height, channels: 1 } })
    .flip()
    .resize({
      width: Math.max(1, Math.round(width * fit)),
      height: Math.max(1, Math.round(height * fit)),
      kernel: fit >= 1 ? "nearest" : "lanczos3",
    })
    .toColourspace("b-w")
    .png(PNG)
    .toBuffer();
}

// World ↔ screen math for the occupancy-map canvas.
//
// Grid space: mapX = (x - originX) / resolution (0..width, increases right),
// mapY = (y - originY) / resolution (0..height, increases UP — world y-up).
// The recolored map layer is pre-flipped vertically so its pixel (mapX, height-mapY)
// sits at layer row 0 = top; drawing it at (tx, ty) scaled by `scale` therefore
// lines up exactly with worldToScreen below. No scaleY(-1) container hack.

export interface MapMeta {
  resolution: number;
  originX: number;
  originY: number;
  width: number;
  height: number;
}

export interface View {
  scale: number; // screen px per grid cell
  tx: number; // screen px offset of grid cell (0,0)-top-left
  ty: number;
}

export function worldToScreen(
  meta: MapMeta,
  view: View,
  x: number,
  y: number,
): { sx: number; sy: number } {
  const mapX = (x - meta.originX) / meta.resolution;
  const mapY = (y - meta.originY) / meta.resolution;
  return {
    sx: view.tx + view.scale * mapX,
    sy: view.ty + view.scale * (meta.height - mapY),
  };
}

export function screenToWorld(
  meta: MapMeta,
  view: View,
  sx: number,
  sy: number,
): { x: number; y: number } {
  const mapX = (sx - view.tx) / view.scale;
  const mapY = meta.height - (sy - view.ty) / view.scale;
  return {
    x: meta.originX + mapX * meta.resolution,
    y: meta.originY + mapY * meta.resolution,
  };
}

// Centered fit of the whole grid into a canvas of the given CSS size.
export function fitView(meta: MapMeta, cw: number, ch: number): View {
  const scale = Math.min(cw / meta.width, ch / meta.height) * 0.95;
  return {
    scale,
    tx: (cw - meta.width * scale) / 2,
    ty: (ch - meta.height * scale) / 2,
  };
}

// Zoom by `factor` while keeping the world point under (sx, sy) fixed.
export function zoomAt(view: View, factor: number, sx: number, sy: number): View {
  return {
    scale: view.scale * factor,
    tx: sx - factor * (sx - view.tx),
    ty: sy - factor * (sy - view.ty),
  };
}

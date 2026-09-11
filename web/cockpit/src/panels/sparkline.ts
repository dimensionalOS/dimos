// Canvas sparkline for the Stats page: CPU history on dtop's fixed 0..100
// scale (rows stay comparable), the newest sample at the right edge, a line
// in the given color with a faint fill beneath. Pure drawing on a context the
// caller sized in device pixels; `dpr` scales the stroke.

export function drawSparkline(
  ctx: CanvasRenderingContext2D,
  samples: readonly number[],
  window: number,
  w: number,
  h: number,
  color: string,
  dpr: number,
): void {
  ctx.clearRect(0, 0, w, h);
  const n = samples.length;
  if (n === 0) return;
  const pad = dpr; // half the stroke: 0 and 100 stay inside the box
  const step = w / Math.max(window - 1, 1);
  const x = (i: number): number => w - (n - 1 - i) * step;
  const y = (v: number): number => {
    const ratio = Math.min(Math.max(v, 0), 100) / 100;
    return pad + (h - 2 * pad) * (1 - ratio);
  };
  ctx.fillStyle = color;
  ctx.strokeStyle = color;
  if (n === 1) {
    ctx.fillRect(x(0) - dpr, y(samples[0]) - dpr, 2 * dpr, 2 * dpr);
    return;
  }
  ctx.beginPath();
  ctx.moveTo(x(0), y(samples[0]));
  for (let i = 1; i < n; i++) ctx.lineTo(x(i), y(samples[i]));
  ctx.lineWidth = 1.5 * dpr;
  ctx.lineJoin = "round";
  ctx.stroke();
  // The same path closed along the bottom edge, filled faintly.
  ctx.lineTo(x(n - 1), h);
  ctx.lineTo(x(0), h);
  ctx.closePath();
  ctx.globalAlpha = 0.2;
  ctx.fill();
  ctx.globalAlpha = 1;
}

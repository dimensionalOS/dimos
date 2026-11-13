export type PathPoint = [number, number];
export const PATH_COLOR = "#3fb950";
export const PATH_PX = 2;

export function readPath(v: unknown): PathPoint[] | null {
  if (!Array.isArray(v)) return null;
  const ok = v.every((p) =>
    Array.isArray(p) && p.length === 2 && Number.isFinite(p[0]) && Number.isFinite(p[1])
  );
  return ok ? v as PathPoint[] : null;
}

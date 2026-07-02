/**
 * Eval rubric helpers — building blocks for `workflow.success(ctx)`.
 *
 * Two layers of API:
 *
 * 1. High-level rubrics that match the common eval shapes and return an
 *    `EvalSuccess` ({passed, reason, score}) directly:
 *      - objectDistance({ target, thresholdM })
 *      - radiusContains({ targets, radiusM })
 *
 * 2. Low-level helpers if you want to write the scoring inline:
 *      - findAsset(query, sceneState) → AssetEntry | null
 *      - dist(a, b) → number
 *      - distToSurface(point, center, bbox?) → number
 */

export interface Vec3 { x: number; y: number; z: number; }

export interface AssetEntry {
  title?: string;
  id?: string;
  transform?: { x?: number; y?: number; z?: number };
  _bbox?: { w: number; h: number; d: number };
}

export interface SceneState {
  assets?: AssetEntry[];
  agentPos?: Vec3;
}

export interface EvalSuccess {
  passed: boolean;
  reason?: string;
  score?: number;
}

// ── Low-level helpers ────────────────────────────────────────────────────────

export function dist(a: Vec3, b: Vec3): number {
  const dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
  return Math.sqrt(dx * dx + dy * dy + dz * dz);
}

export function distToSurface(point: Vec3, center: Vec3, bbox?: { w: number; h: number; d: number }): number {
  if (!bbox) return dist(point, center);
  const hw = bbox.w / 2, hh = bbox.h / 2, hd = bbox.d / 2;
  const cx = Math.max(center.x - hw, Math.min(point.x, center.x + hw));
  const cy = Math.max(center.y - hh, Math.min(point.y, center.y + hh));
  const cz = Math.max(center.z - hd, Math.min(point.z, center.z + hd));
  return dist(point, { x: cx, y: cy, z: cz });
}

/** Find the first asset whose title or id contains `query` (case-insensitive). */
export function findAsset(query: string, sceneState: SceneState): AssetEntry | null {
  if (!sceneState.assets) return null;
  const lower = query.toLowerCase();
  for (const a of sceneState.assets) {
    if (a.title?.toLowerCase().includes(lower) || a.id?.toLowerCase().includes(lower)) return a;
  }
  return null;
}

function assetPos(a: AssetEntry): Vec3 | null {
  if (!a.transform) return null;
  return { x: a.transform.x ?? 0, y: a.transform.y ?? 0, z: a.transform.z ?? 0 };
}

// ── High-level rubrics (return EvalSuccess directly) ─────────────────────────

export interface ObjectDistanceOpts {
  target: string;
  thresholdM?: number;
}

/** Pass when the agent is within `thresholdM` of the target's bbox surface. */
export function objectDistance(
  ctx: { agentPos: Vec3; sceneState: SceneState },
  { target, thresholdM = 0.5 }: ObjectDistanceOpts,
): EvalSuccess {
  const hit = findAsset(target, ctx.sceneState);
  if (!hit) return { passed: false, reason: `target "${target}" not found in scene`, score: Infinity };
  const pos = assetPos(hit);
  if (!pos) return { passed: false, reason: `target "${target}" has no transform`, score: Infinity };
  const d = distToSurface(ctx.agentPos, pos, hit._bbox);
  return {
    passed: d <= thresholdM,
    score: Math.round(d * 1000) / 1000,
    reason: `${d.toFixed(3)}m to "${hit.title ?? target}" (threshold ${thresholdM}m)`,
  };
}

export interface RadiusContainsOpts {
  targets: string[];
  radiusM?: number;
}

/** Pass when the agent is within `radiusM` of the centroid of the listed targets. */
export function radiusContains(
  ctx: { agentPos: Vec3; sceneState: SceneState },
  { targets, radiusM = 3.0 }: RadiusContainsOpts,
): EvalSuccess {
  if (!targets || targets.length === 0) return { passed: false, reason: "no targets specified" };

  const found: Vec3[] = [];
  const missing: string[] = [];
  for (const name of targets) {
    const a = findAsset(name, ctx.sceneState);
    const p = a ? assetPos(a) : null;
    if (p) found.push(p);
    else missing.push(name);
  }
  if (found.length === 0) {
    return { passed: false, reason: `no targets found: ${missing.join(", ")}`, score: Infinity };
  }

  const centroid: Vec3 = { x: 0, y: 0, z: 0 };
  for (const p of found) { centroid.x += p.x; centroid.y += p.y; centroid.z += p.z; }
  centroid.x /= found.length; centroid.y /= found.length; centroid.z /= found.length;

  const d = Math.round(dist(ctx.agentPos, centroid) * 1000) / 1000;
  const missingNote = missing.length ? ` (missing: ${missing.join(", ")})` : "";
  return {
    passed: d <= radiusM,
    score: d,
    reason: `${d.toFixed(3)}m to centroid of ${found.length} targets${missingNote} (radius ${radiusM}m)`,
  };
}

/**
 * Eval rubric helpers — building blocks for `workflow.success(ctx)`.
 *
 * Two layers of API:
 *
 * 1. High-level rubrics that match the common eval shapes and return an
 *    `EvalSuccess` ({passed, reason, score}) directly:
 *      - objectDistance({ target, thresholdM })
 *      - radiusContains({ targets, radiusM })
 *      - searchEvidence({ minTravelM, minHeadingTravelDeg, minViewpoints })
 *      - orderedRegionVisits({ regions })
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

/** Pose-history evidence collected by the browser after `evalStart`. */
export interface EvalMetrics {
  pathLengthM: number;
  headingTravelDeg: number;
  distinctViewpoints: number;
  trajectory: Vec3[];
}

// ── Low-level helpers ────────────────────────────────────────────────────────

export function dist(a: Vec3, b: Vec3): number {
  const dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
  return Math.sqrt(dx * dx + dy * dy + dz * dz);
}

function distToSurface(point: Vec3, center: Vec3, bbox?: { w: number; h: number; d: number }): number {
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

export interface SearchEvidenceOpts {
  minTravelM?: number;
  minHeadingTravelDeg?: number;
  minViewpoints?: number;
}

export interface AxisAlignedRegion {
  name: string;
  minX: number;
  maxX: number;
  minZ: number;
  maxZ: number;
}

export interface OrderedRegionVisitsOpts {
  regions: AxisAlignedRegion[];
}

function regionContainsPoint(region: AxisAlignedRegion, point: Vec3): boolean {
  return point.x >= region.minX && point.x <= region.maxX &&
    point.z >= region.minZ && point.z <= region.maxZ;
}

/**
 * Require a physical trajectory to visit calibrated regions in order and end
 * in the final region.
 *
 * Region coordinates are scorer-only ground truth. They are never included in
 * the task string or exposed as an agent tool.
 */
export function orderedRegionVisits(
  ctx: { metrics: EvalMetrics },
  { regions }: OrderedRegionVisitsOpts,
): EvalSuccess {
  if (regions.length === 0) {
    return { passed: false, reason: "no ordered regions specified" };
  }

  let nextRegion = 0;
  for (const point of ctx.metrics.trajectory) {
    if (
      nextRegion < regions.length &&
      regionContainsPoint(regions[nextRegion], point)
    ) {
      nextRegion++;
    }
  }

  const finalRegion = regions[regions.length - 1];
  const finalPoint = ctx.metrics.trajectory.at(-1);
  const endedInFinalRegion = finalPoint !== undefined &&
    regionContainsPoint(finalRegion, finalPoint);
  const passed = nextRegion === regions.length && endedInFinalRegion;
  const visitedNames = regions.slice(0, nextRegion).map((region) =>
    region.name
  );
  const nextName = regions[nextRegion]?.name;
  return {
    passed,
    reason: passed
      ? `visited ${
        regions.map((region) => region.name).join(" -> ")
      } and ended in ${finalRegion.name}`
      : `visited ${visitedNames.join(" -> ") || "none"}; ${
        nextName
          ? `next required ${nextName}`
          : `did not end in ${finalRegion.name}`
      }`,
  };
}

/**
 * Require physical search motion before accepting a final-position rubric.
 *
 * A viewpoint is a pose separated from every prior viewpoint by at least
 * 0.75m or 45 degrees. This deliberately measures only externally observable
 * robot motion; it does not claim that continuous camera publication proves
 * the model called `observe`.
 */
export function searchEvidence(
  ctx: { metrics: EvalMetrics },
  {
    minTravelM = 0,
    minHeadingTravelDeg = 0,
    minViewpoints = 1,
  }: SearchEvidenceOpts,
): EvalSuccess {
  const { pathLengthM, headingTravelDeg, distinctViewpoints } = ctx.metrics;
  const passed = pathLengthM >= minTravelM &&
    headingTravelDeg >= minHeadingTravelDeg &&
    distinctViewpoints >= minViewpoints;
  return {
    passed,
    reason: `${pathLengthM.toFixed(2)}m travelled (min ${minTravelM}m), ` +
      `${
        headingTravelDeg.toFixed(0)
      }° heading travel (min ${minHeadingTravelDeg}°), ` +
      `${distinctViewpoints} distinct viewpoints (min ${minViewpoints})`,
  };
}

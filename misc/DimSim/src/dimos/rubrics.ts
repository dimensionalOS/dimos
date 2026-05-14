/**
 * Eval Rubrics — deterministic scoring for eval workflows.
 *
 * Rubrics:
 *   objectDistance  — Euclidean distance from agent to target bbox surface
 *   radiusContains — agent is within radius of centroid computed from multiple targets
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

export interface ObjectDistanceCriteria {
  object: string;
  target: string;
  thresholdM?: number;
}

export interface ObjectDistanceResult {
  pass: boolean;
  distanceM: number;
  details: string;
}

export function scoreObjectDistance(criteria: ObjectDistanceCriteria, sceneState: SceneState): ObjectDistanceResult {
  const { target: targetName, thresholdM = 0.5 } = criteria;

  if (!sceneState.agentPos) {
    return { pass: false, distanceM: Infinity, details: "Agent position not available" };
  }

  const targetHit = _findTarget(targetName, sceneState);
  if (!targetHit) {
    return { pass: false, distanceM: Infinity, details: `Target "${targetName}" not found in scene` };
  }

  const dist = _distToSurface(sceneState.agentPos, targetHit.pos, targetHit.bbox);

  return {
    pass: dist <= thresholdM,
    distanceM: Math.round(dist * 1000) / 1000,
    details: `agent is ${dist.toFixed(3)}m from "${targetName}" surface (threshold: ${thresholdM}m)`,
  };
}

// -- radiusContains -----------------------------------------------------------

export interface RadiusContainsCriteria {
  object?: string;       // defaults to "agent"
  targets: string[];     // scene objects whose centroid defines the region
  radiusM?: number;      // max distance from centroid (default 3.0)
}

export interface RadiusContainsResult {
  pass: boolean;
  distanceM: number;
  centroid: Vec3;
  foundTargets: string[];
  missingTargets: string[];
  details: string;
}

export function scoreRadiusContains(criteria: RadiusContainsCriteria, sceneState: SceneState): RadiusContainsResult {
  const { targets, radiusM = 3.0 } = criteria;

  const fail = (details: string): RadiusContainsResult => ({
    pass: false, distanceM: Infinity, centroid: { x: 0, y: 0, z: 0 },
    foundTargets: [], missingTargets: targets, details,
  });

  if (!sceneState.agentPos) return fail("Agent position not available");
  if (!targets || targets.length === 0) return fail("No targets specified");

  const found: { name: string; pos: Vec3 }[] = [];
  const missing: string[] = [];
  for (const name of targets) {
    const hit = _findTarget(name, sceneState);
    if (hit) found.push({ name, pos: hit.pos });
    else missing.push(name);
  }

  if (found.length < 2 && found.length < targets.length) {
    // Need at least 2 targets found, or all if fewer than 2 specified
    if (found.length === 0) return fail(`No targets found: ${missing.join(", ")}`);
  }

  // Compute centroid of found targets
  const centroid: Vec3 = { x: 0, y: 0, z: 0 };
  for (const f of found) {
    centroid.x += f.pos.x;
    centroid.y += f.pos.y;
    centroid.z += f.pos.z;
  }
  centroid.x /= found.length;
  centroid.y /= found.length;
  centroid.z /= found.length;

  const dx = sceneState.agentPos.x - centroid.x;
  const dy = sceneState.agentPos.y - centroid.y;
  const dz = sceneState.agentPos.z - centroid.z;
  const dist = Math.round(Math.sqrt(dx * dx + dy * dy + dz * dz) * 1000) / 1000;

  const foundNames = found.map((f) => f.name);
  const pass = dist <= radiusM;
  const missingNote = missing.length > 0 ? ` (missing: ${missing.join(", ")})` : "";

  return {
    pass,
    distanceM: dist,
    centroid,
    foundTargets: foundNames,
    missingTargets: missing,
    details: `agent is ${dist.toFixed(3)}m from centroid of [${foundNames.join(", ")}]${missingNote} (radius: ${radiusM}m)`,
  };
}

// -- helpers ------------------------------------------------------------------

function _distToSurface(from: Vec3, center: Vec3, bbox?: { w: number; h: number; d: number }): number {
  if (!bbox) {
    const dx = from.x - center.x, dy = from.y - center.y, dz = from.z - center.z;
    return Math.sqrt(dx * dx + dy * dy + dz * dz);
  }
  const hw = bbox.w / 2, hh = bbox.h / 2, hd = bbox.d / 2;
  const cx = Math.max(center.x - hw, Math.min(from.x, center.x + hw));
  const cy = Math.max(center.y - hh, Math.min(from.y, center.y + hh));
  const cz = Math.max(center.z - hd, Math.min(from.z, center.z + hd));
  const dx = from.x - cx, dy = from.y - cy, dz = from.z - cz;
  return Math.sqrt(dx * dx + dy * dy + dz * dz);
}

function _findTarget(name: string, sceneState: SceneState): { pos: Vec3; bbox?: { w: number; h: number; d: number } } | null {
  const lower = name.toLowerCase();
  if (!sceneState.assets) return null;
  for (const asset of sceneState.assets) {
    if (asset.title?.toLowerCase().includes(lower) || asset.id?.toLowerCase().includes(lower)) {
      if (asset.transform) {
        return {
          pos: { x: asset.transform.x || 0, y: asset.transform.y || 0, z: asset.transform.z || 0 },
          bbox: asset._bbox,
        };
      }
    }
  }
  return null;
}

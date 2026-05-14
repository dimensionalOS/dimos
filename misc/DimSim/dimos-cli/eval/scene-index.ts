/**
 * Scene Index — parse scene JSON and search for objects by name.
 *
 * Uses the same substring matching logic as the browser-side rubric
 * (`_findObject` in rubrics.ts) so validation matches scoring exactly.
 */

export interface SceneObject {
  title: string;
  id: string;
  position: { x: number; y: number; z: number };
}

export interface SceneIndex {
  sceneName: string;
  objects: SceneObject[];
}

/**
 * Load a scene JSON and extract all titled assets with positions.
 * Skips assets without titles (structural delta patches, etc).
 */
export function loadSceneIndex(scenePath: string, sceneName: string): SceneIndex {
  const text = Deno.readTextFileSync(scenePath);
  const json = JSON.parse(text);
  const objects: SceneObject[] = [];

  if (Array.isArray(json.assets)) {
    for (const asset of json.assets) {
      const title = asset.title;
      if (!title || typeof title !== "string") continue;
      const pos = asset.transform?.position || asset.transform || {};
      objects.push({
        title: title.trim(),
        id: asset.id || "",
        position: {
          x: Math.round((pos.x || 0) * 10) / 10,
          y: Math.round((pos.y || 0) * 10) / 10,
          z: Math.round((pos.z || 0) * 10) / 10,
        },
      });
    }
  }

  // Sort by title for display
  objects.sort((a, b) => a.title.localeCompare(b.title));
  return { sceneName, objects };
}

/**
 * Find an object by name — same case-insensitive substring match as the rubric.
 * Returns the first match (same as rubric behavior).
 */
export function findObject(searchTerm: string, index: SceneIndex): SceneObject | null {
  const lower = searchTerm.toLowerCase();
  for (const obj of index.objects) {
    if (obj.title.toLowerCase().includes(lower) || obj.id.toLowerCase().includes(lower)) {
      return obj;
    }
  }
  return null;
}

/**
 * Find ALL objects matching the search term (for duplicate warnings).
 */
export function findAllObjects(searchTerm: string, index: SceneIndex): SceneObject[] {
  const lower = searchTerm.toLowerCase();
  return index.objects.filter(
    (obj) => obj.title.toLowerCase().includes(lower) || obj.id.toLowerCase().includes(lower),
  );
}

/**
 * Suggest similar objects when search fails (simple substring overlap).
 */
export function suggestObjects(searchTerm: string, index: SceneIndex, limit = 5): string[] {
  const lower = searchTerm.toLowerCase();
  const scored: { title: string; score: number }[] = [];

  for (const obj of index.objects) {
    const t = obj.title.toLowerCase();
    // Score: longest common substring length
    let best = 0;
    for (let len = 1; len <= lower.length; len++) {
      for (let start = 0; start + len <= lower.length; start++) {
        if (t.includes(lower.substring(start, start + len))) {
          best = len;
        }
      }
    }
    if (best > 0) scored.push({ title: obj.title, score: best });
  }

  scored.sort((a, b) => b.score - a.score);
  // Deduplicate titles
  const seen = new Set<string>();
  const result: string[] = [];
  for (const s of scored) {
    if (!seen.has(s.title)) {
      seen.add(s.title);
      result.push(s.title);
      if (result.length >= limit) break;
    }
  }
  return result;
}

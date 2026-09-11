export type Vec3 = [number, number, number];
export type Quat = [number, number, number, number];
export interface VisualGeom {
  id: number;
  name: string;
  body: number;
  kind:
    | "plane"
    | "sphere"
    | "capsule"
    | "ellipsoid"
    | "cylinder"
    | "box"
    | "mesh";
  size: Vec3;
  position: Vec3;
  quaternion: Quat;
  rgba: Quat;
  mesh?: string;
  part?: string;
  texture?: { id: string; repeat: [number, number] };
}
export interface WorldDefinition {
  version: 1;
  up: "z";
  bodyIds: number[];
  focusBody: number;
  camera?: {
    body: number;
    position: Vec3;
    quaternion: Quat;
    fovy: number;
    near: number;
    far: number;
  } | null;
  actors?: {
    id: string;
    bodyIds: number[];
    focusBody: number;
    camera: NonNullable<WorldDefinition["camera"]>;
    color: string | null;
  }[];
  geoms: VisualGeom[];
  textures?: Record<string, string>;
  meshes: Record<string, { positions: string; indices: string }>;
  appearance: {
    background?: string;
    ground?: string;
    exposure?: number;
    colors?: Record<string, string>;
    camera?: { position: Vec3; target: Vec3 };
    views?: Record<string, { position: Vec3; target: Vec3; extent?: Vec3 }>;
  };
}
export interface WorldSnapshot {
  model: string;
  t: number;
  poses: number[][];
  actors?: { id: string; active: boolean; generation: string }[];
  football?: { scores: [number, number]; lit: number[]; drops?: Record<string, number>; scorers?: { handle: string; goals: number }[] };
}

export function readSnapshot(value: unknown): WorldSnapshot | null {
  if (!value || typeof value !== "object") return null;
  const data = value as Partial<WorldSnapshot>;
  if (
    typeof data.model !== "string" ||
    !/^\/world-assets\/scene-[a-f0-9]{20}\.json$/.test(data.model)
  ) return null;
  if (
    typeof data.t !== "number" || !Number.isFinite(data.t) ||
    !Array.isArray(data.poses)
  ) {
    return null;
  }
  if (
    !data.poses.every((p) => Array.isArray(p) && p.length === 7 && p.every(Number.isFinite))
  ) {
    return null;
  }
  if (
    data.actors !== undefined &&
    (!Array.isArray(data.actors) ||
      !data.actors.every((a) =>
        a && typeof a.id === "string" && typeof a.active === "boolean" &&
        typeof a.generation === "string"
      ))
  ) return null;
  if (
    data.football !== undefined && (
      !data.football || !Array.isArray(data.football.scores) || data.football.scores.length !== 2 ||
      !data.football.scores.every((s) => Number.isSafeInteger(s) && s >= 0) ||
      !Array.isArray(data.football.lit) ||
      !data.football.lit.every((id) => Number.isSafeInteger(id) && id >= 0) ||
      (data.football.drops !== undefined && (!data.football.drops || typeof data.football.drops !== "object" || Array.isArray(data.football.drops) || Object.values(data.football.drops).some(n => !Number.isSafeInteger(n) || n < 0))) ||
      (data.football.scorers !== undefined && (!Array.isArray(data.football.scorers) || data.football.scorers.length > 20 || !data.football.scorers.every(row => row && typeof row.handle === "string" && /^[a-zA-Z0-9-]{1,39}$/.test(row.handle) && Number.isSafeInteger(row.goals) && row.goals >= 0)))
    )
  ) return null;
  return data as WorldSnapshot;
}

export function readDefinition(value: unknown): WorldDefinition {
  const model = value as Partial<WorldDefinition> | null;
  if (
    !model || model.version !== 1 || model.up !== "z" ||
    !Array.isArray(model.bodyIds) ||
    !model.bodyIds.every(Number.isInteger) || !Array.isArray(model.geoms) ||
    !model.meshes || !model.appearance ||
    !model.bodyIds.includes(model.focusBody!)
  ) {
    throw new Error("The world model format is unsupported.");
  }
  const kinds = new Set([
    "plane",
    "sphere",
    "capsule",
    "ellipsoid",
    "cylinder",
    "box",
    "mesh",
  ]);
  for (const geom of model.geoms) {
    if (
      (geom?.texture &&
        (typeof model.textures?.[geom.texture.id] !== "string" ||
          !Array.isArray(geom.texture.repeat) || geom.texture.repeat.length !== 2 ||
          !geom.texture.repeat.every(Number.isFinite))) ||
      (geom?.part !== undefined && typeof geom.part !== "string") ||
      !geom || !kinds.has(geom.kind) || !model.bodyIds.includes(geom.body) ||
      ![geom.size, geom.position, geom.quaternion, geom.rgba].every((v) =>
        Array.isArray(v) && v.every(Number.isFinite)
      ) ||
      geom.size.length !== 3 || geom.position.length !== 3 ||
      geom.quaternion.length !== 4 ||
      geom.rgba.length !== 4 ||
      (geom.kind === "mesh" &&
        (geom.mesh === undefined || !model.meshes[geom.mesh]))
    ) {
      throw new Error("The world contains an invalid visual geometry.");
    }
  }
  const pov = model.camera;
  if (
    pov && (!model.bodyIds.includes(pov.body) || !Array.isArray(pov.position) ||
      pov.position.length !== 3 || !pov.position.every(Number.isFinite) ||
      !Array.isArray(pov.quaternion) || pov.quaternion.length !== 4 ||
      !pov.quaternion.every(Number.isFinite) || !Number.isFinite(pov.fovy) ||
      pov.fovy <= 0 ||
      pov.fovy >= 180 || !Number.isFinite(pov.near) || pov.near <= 0 ||
      !Number.isFinite(pov.far) ||
      pov.far <= pov.near)
  ) {
    throw new Error("The robot camera configuration is invalid.");
  }
  if (
    model.appearance.views && Object.values(model.appearance.views).some((view) =>
      !view ||
      (view.extent !== undefined &&
        (!Array.isArray(view.extent) || view.extent.length !== 3 ||
          !view.extent.every((n) => Number.isFinite(n) && n > 0))) ||
      ![view.position, view.target].every((v) =>
        Array.isArray(v) && v.length === 3 && v.every(Number.isFinite)
      )
    )
  ) throw new Error("The room camera configuration is invalid.");
  const camera = model.appearance.camera;
  if (
    camera &&
    ![camera.position, camera.target].every((v) =>
      Array.isArray(v) && v.length === 3 && v.every(Number.isFinite)
    )
  ) {
    throw new Error("The world camera configuration is invalid.");
  }
  if (
    model.appearance.exposure !== undefined &&
    (!Number.isFinite(model.appearance.exposure) ||
      model.appearance.exposure <= 0)
  ) {
    throw new Error("The world exposure must be positive.");
  }
  if (
    model.actors !== undefined &&
    (!Array.isArray(model.actors) ||
      !model.actors.every((a) =>
        a && typeof a.id === "string" && Array.isArray(a.bodyIds) &&
        a.bodyIds.every((id) => model.bodyIds!.includes(id)) &&
        a.bodyIds.includes(a.focusBody) &&
        a.camera && a.bodyIds.includes(a.camera.body)
      ))
  ) {
    throw new Error("The world actor configuration is invalid.");
  }
  return model as WorldDefinition;
}

export function unpack(base64: string): ArrayBuffer {
  return Uint8Array.from(atob(base64), (c) => c.charCodeAt(0)).buffer;
}

let cachedModel: { url: string; value: Promise<WorldDefinition> } | null = null;
/** Both panels share one download and parsed model; GPU resources remain panel-owned. */
export function fetchDefinition(url: string): Promise<WorldDefinition> {
  if (cachedModel?.url === url) return cachedModel.value;
  const value = fetch(url).then(async (response) => {
    if (!response.ok) {
      throw new Error(`World model could not load (${response.status}).`);
    }
    return readDefinition(await response.json());
  }).catch((error) => {
    if (cachedModel?.value === value) cachedModel = null;
    throw error;
  });
  cachedModel = { url, value };
  return value;
}

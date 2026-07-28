/**
 * @dimsim/scene-api — the surface scene modules import.
 *
 * Usage (in a scene module at scenes/<name>/index.js):
 *
 *     import { scene, THREE, agent, physics } from '@dimsim/scene-api';
 *
 *     export default async function build() {
 *       scene.add(new THREE.DirectionalLight(0xffffff, 1.2));
 *       return { embodiment: 'unitree-go2', spawnPoint: { x: 2, y: 0.5, z: 3 } };
 *     }
 *
 * Lifecycle: engine.js calls `_init()` once before dynamic-importing scenes,
 * populating the module-level `export let` bindings.  ESM live-binding makes
 * `import { scene } from '@dimsim/scene-api'` return the engine's live ref.
 *
 * Implementation note: the collider helpers mirror what `sceneEditor.ts`
 * already does in its exec sandbox — browser-side Rapier collider creation
 * *and* a `physicsColliderAdd` WS message so server-side physics + LiDAR see
 * the same colliders.  Factored here so scene modules (load-time) and the
 * exec sandbox (runtime) share one implementation.
 */

export interface SceneApiContext {
  scene: any;
  THREE: any;
  RAPIER: any;
  rapierWorld: any;
  renderer: any;
  camera: any;
  agent: any;
  gltfLoader: any;
  /** Browser → bridge WS send.  Used for physicsColliderAdd / Remove. */
  sendPhysics: (msg: Record<string, any>) => void;
  /** Where the scene module lives (e.g. "/scenes/apartment/").  loadGLTF resolves against this. */
  sceneBaseUrl: string;
  /** Engine's existing JSON scene loader, kept so `loadLevel` can populate primitives. */
  importLevelFromJSON: (json: any) => Promise<void>;
  /** Apply sky settings ({topColor, horizonColor, bottomColor, brightness, softness, sunStrength, sunHeight, enabled?}). */
  setSky: (opts: Record<string, any>) => void;
}

// ── Live bindings (ESM re-export pattern) ────────────────────────────────────

export let scene: any = null;
export let THREE: any = null;
export let RAPIER: any = null;
export let rapierWorld: any = null;
export let renderer: any = null;
export let camera: any = null;
export let agent: any = null;
export let gltfLoader: any = null;

// ── Internal state ───────────────────────────────────────────────────────────

let _sendPhysics: ((msg: Record<string, any>) => void) | null = null;
let _sceneBaseUrl: string = "/";
let _importLevelFromJSON: ((json: any) => Promise<void>) | null = null;
let _setSky: ((opts: Record<string, any>) => void) | null = null;

const _colliderMap = new Map<string, any>();
const _dynamicBodies = new Map<string, { body: any; mesh: any }>();
let _dynamicSyncRaf: number | null = null;

let _lastLoadedKey: string | null = null;

// ── Initialization (called by engine.js once at boot) ────────────────────────

export function _init(ctx: SceneApiContext): void {
  scene = ctx.scene;
  THREE = ctx.THREE;
  RAPIER = ctx.RAPIER;
  rapierWorld = ctx.rapierWorld;
  renderer = ctx.renderer;
  camera = ctx.camera;
  agent = ctx.agent;
  gltfLoader = ctx.gltfLoader;
  _sendPhysics = ctx.sendPhysics;
  _sceneBaseUrl = ctx.sceneBaseUrl;
  _importLevelFromJSON = ctx.importLevelFromJSON;
  _setSky = ctx.setSky;
}

export function setSky(opts: Record<string, any>): void {
  _setSky?.(opts);
}

/**
 * Declare the agent's embodiment — avatar mesh + body dimensions + physics
 * mode + control parameters.  Sent to the bridge so server-side physics +
 * lidar mount reconfigure live, AND applied locally so the browser swaps
 * the GLB and re-asserts visibility immediately.
 *
 * Typical configs:
 *   setEmbodiment({ embodimentType: 'ground', avatarUrl: '/embodiment/dimsim_unitree_stub.glb',
 *                   radius: 0.18, halfHeight: 0.25, maxSpeed: 1.5, turnRate: 2.5 });
 *
 *   setEmbodiment({ embodimentType: 'drone',  avatarUrl: '/embodiment/drone.glb',
 *                   radius: 0.3, halfHeight: 0.1, gravity: 0, maxSpeed: 3.0 });
 *
 * All fields are forwarded to the bridge's EmbodimentConfig (see
 * cli/bridge/physics.ts).  Falsy fields are passed through unchanged so
 * partial reconfigures work — e.g. just bumping maxSpeed mid-scene.
 */
let _pendingEmbodiment: Record<string, any> | null = null;

export function _getPendingEmbodiment(): Record<string, any> | null {
  return _pendingEmbodiment;
}

export function setEmbodiment(config: Record<string, any>): void {
  if (!_sendPhysics) throw new Error("scene-api not initialized");
  const w = window as any;
  if (w.__dimosBridge) {
    console.log("[sceneApi] setEmbodiment applying:", config.embodimentType, "gravity=" + config.gravity);
    if (w.__dimosBridge._handleEmbodimentConfig) {
      w.__dimosBridge._handleEmbodimentConfig(config);
    }
    _sendPhysics({ type: "embodimentConfig", ...config });
  } else {
    console.log("[sceneApi] setEmbodiment queued (bridge not ready):", config.embodimentType);
    _pendingEmbodiment = config;
  }
}

/** engine.js calls this right after `window.__dimosBridge = bridge`. */
export function _flushPendingEmbodiment(): void {
  // Always declare an embodiment per load (the scene's, else default ground), so a
  // scene without setEmbodiment doesn't inherit the bridge's last one (e.g. a drone).
  const cfg = _pendingEmbodiment ?? { embodimentType: "ground", motionModel: "holonomic" };
  _pendingEmbodiment = null;
  setEmbodiment(cfg);
}

/** Engine.js calls this once the agent is constructed (post-scene-build). */
export function _setAgent(a: any): void {
  agent = a;
}

// ── Helpers ──────────────────────────────────────────────────────────────────

export function loadGLTF(url: string): Promise<any> {
  const abs = new URL(url, _sceneBaseUrl).toString();
  return new Promise((resolve, reject) =>
    gltfLoader.load(abs, resolve, undefined, reject),
  );
}

/**
 * Feed an in-memory level blob directly to the engine.  Idempotent — if the
 * caller hands us the same object reference twice we skip the second import.
 * Materials with `texturePath: 'foo.jpg'` are rewritten to absolute
 * `textureDataUrl` against the scene base before import.
 */
export async function loadLevel(data: any): Promise<void> {
  if (!_importLevelFromJSON) throw new Error("scene-api not initialized");
  const key = `level:${_identityOf(data)}`;
  if (_lastLoadedKey === key) return;
  _rewriteTexturePaths(data);
  await _importLevelFromJSON(data);
  _lastLoadedKey = key;
}

const _identitySym = Symbol("dimsim.loadLevel.id");
let _identityCounter = 0;
function _identityOf(o: any): string {
  if (!o || typeof o !== "object") return String(o);
  if (!o[_identitySym]) {
    Object.defineProperty(o, _identitySym, {
      value: String(++_identityCounter), enumerable: false, writable: false,
    });
  }
  return o[_identitySym];
}

function _rewriteTexturePaths(node: any): void {
  if (!node) return;
  if (Array.isArray(node)) {
    for (const item of node) _rewriteTexturePaths(item);
    return;
  }
  if (typeof node !== "object") return;
  const mat = node.material;
  if (mat && typeof mat === "object" && typeof mat.texturePath === "string" && !mat.textureDataUrl) {
    mat.textureDataUrl = new URL(mat.texturePath, _sceneBaseUrl).toString();
  }
  for (const k in node) {
    const v = (node as any)[k];
    if (v && typeof v === "object") _rewriteTexturePaths(v);
  }
}

export interface ColliderOpts {
  shape?: "trimesh" | "box" | "sphere";
  dynamic?: boolean;
  mass?: number;
  restitution?: number;
}

/** Static collider — no rigid body, attached to the world. */
export function staticCollider(mesh: any, shape: ColliderOpts["shape"] = "trimesh"): any {
  return addCollider(mesh, { shape, dynamic: false });
}

/** Dynamic collider — gets a rigid body, responds to gravity. */
export function dynamicCollider(mesh: any, opts: Omit<ColliderOpts, "dynamic"> = {}): any {
  return addCollider(mesh, { ...opts, dynamic: true });
}

/**
 * Full-control collider helper.  Mirrors sceneEditor.ts addCollider so scene
 * modules and the exec sandbox produce identical results.
 */
export function addCollider(obj: any, shapeOrOpts?: ColliderOpts["shape"] | ColliderOpts): any {
  if (!_sendPhysics) throw new Error("scene-api not initialized");

  let shape: ColliderOpts["shape"] = "trimesh";
  let dynamic = false;
  let mass = 1.0;
  let restitution = 0.3;
  if (typeof shapeOrOpts === "string") {
    shape = shapeOrOpts;
  } else if (shapeOrOpts) {
    shape = shapeOrOpts.shape || "trimesh";
    dynamic = !!shapeOrOpts.dynamic;
    mass = shapeOrOpts.mass ?? 1.0;
    restitution = shapeOrOpts.restitution ?? 0.3;
  }

  removeCollider(obj);

  const bbox = new THREE.Box3().setFromObject(obj);
  const size = new THREE.Vector3();
  const center = new THREE.Vector3();
  bbox.getSize(size);
  bbox.getCenter(center);
  const clamp = (v: number) => Math.max(v, 0.001);

  const serverDesc: any = {
    shape,
    position: { x: center.x, y: center.y, z: center.z },
  };

  if (shape === "sphere") {
    serverDesc.radius = clamp(Math.max(size.x, size.y, size.z) / 2);
  } else if (shape === "trimesh") {
    const verts: number[] = [];
    const indices: number[] = [];
    let vertBase = 0;
    obj.traverse((m: any) => {
      if (!m.isMesh) return;
      const geom = m.geometry;
      const posAttr = geom?.attributes?.position;
      if (!posAttr) return;
      const tmpPos = new THREE.Vector3();
      for (let i = 0; i < posAttr.count; i++) {
        tmpPos.fromBufferAttribute(posAttr, i).applyMatrix4(m.matrixWorld);
        verts.push(tmpPos.x, tmpPos.y, tmpPos.z);
      }
      if (geom.index) {
        for (let i = 0; i < geom.index.count; i++) indices.push(geom.index.getX(i) + vertBase);
      } else {
        for (let i = 0; i < posAttr.count; i++) indices.push(vertBase + i);
      }
      vertBase += posAttr.count;
    });
    if (verts.length < 9 || indices.length < 3) throw new Error("Not enough geometry for trimesh");
    serverDesc.vertices = verts;
    serverDesc.indices = indices;
  } else {
    serverDesc.halfExtents = {
      x: clamp(size.x / 2),
      y: clamp(size.y / 2),
      z: clamp(size.z / 2),
    };
  }

  if (RAPIER && rapierWorld) {
    let desc: any;
    if (shape === "sphere") {
      desc = RAPIER.ColliderDesc.ball(serverDesc.radius);
      if (!dynamic) desc.setTranslation(center.x, center.y, center.z);
    } else if (shape === "trimesh") {
      desc = RAPIER.ColliderDesc.trimesh(
        new Float32Array(serverDesc.vertices),
        new Uint32Array(serverDesc.indices),
      );
    } else {
      const h = serverDesc.halfExtents;
      desc = RAPIER.ColliderDesc.cuboid(h.x, h.y, h.z);
      if (!dynamic) desc.setTranslation(center.x, center.y, center.z);
    }
    desc.setFriction(0.9);
    desc.setRestitution(restitution);

    if (dynamic && shape !== "trimesh") {
      const bodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(
        center.x, center.y, center.z,
      );
      const body = rapierWorld.createRigidBody(bodyDesc);
      body.setAdditionalMass(mass);
      const collider = rapierWorld.createCollider(desc, body);
      _colliderMap.set(obj.uuid, collider);
      _dynamicBodies.set(obj.uuid, { body, mesh: obj });
      _ensureDynamicSyncLoop();
    } else {
      const collider = rapierWorld.createCollider(desc);
      _colliderMap.set(obj.uuid, collider);
    }
  }

  serverDesc.dynamic = dynamic;
  if (dynamic) {
    serverDesc.mass = mass;
    serverDesc.restitution = restitution;
  }
  _sendPhysics({ type: "physicsColliderAdd", uuid: obj.uuid, desc: serverDesc });

  return {
    shape, dynamic, uuid: obj.uuid,
    size: { x: +size.x.toFixed(3), y: +size.y.toFixed(3), z: +size.z.toFixed(3) },
  };
}

export function removeCollider(obj: any): boolean {
  const existing = _colliderMap.get(obj.uuid);
  if (existing) {
    try { rapierWorld?.removeCollider(existing, true); } catch { /* already removed */ }
    _colliderMap.delete(obj.uuid);
  }
  const dyn = _dynamicBodies.get(obj.uuid);
  if (dyn) {
    try { rapierWorld?.removeRigidBody(dyn.body); } catch { /* already removed */ }
    _dynamicBodies.delete(obj.uuid);
  }
  _sendPhysics?.({ type: "physicsColliderRemove", uuid: obj.uuid });
  return !!existing;
}

/**
 * Kinematic-position-based body for script-driven actors (NPCs, doors).
 * Caller updates the world position each frame via
 *   body.setNextKinematicTranslation({ x, y, z })
 * The body collides with the static world (walls, floor) and pushes dynamic
 * bodies, but is not itself pushed.
 *
 * Returns the Rapier RigidBody handle (so the caller can drive it).  The body
 * is also registered for cleanup via removeCollider(mesh).
 */
/** Bundled namespace so scene modules can `import { physics } from '@dimsim/scene-api'`. */
export const physics = {
  staticCollider,
  dynamicCollider,
  addCollider,
  removeCollider,
};

// ── Dynamic body → mesh sync loop ────────────────────────────────────────────

function _ensureDynamicSyncLoop(): void {
  if (_dynamicSyncRaf != null) return;
  const tick = () => {
    if (_dynamicBodies.size === 0) {
      _dynamicSyncRaf = null;
      return;
    }
    for (const { body, mesh } of _dynamicBodies.values()) {
      const t = body.translation();
      const r = body.rotation();
      mesh.position.set(t.x, t.y, t.z);
      mesh.quaternion.set(r.x, r.y, r.z, r.w);
    }
    _dynamicSyncRaf = requestAnimationFrame(tick);
  };
  _dynamicSyncRaf = requestAnimationFrame(tick);
}

// Remove the engine's default lamps + image-based light so the scene lights
// itself. Returns the number of lamps removed.
export function clearDefaultLights(): number {
  if (!scene) throw new Error("scene-api not initialized");
  const doomed: any[] = [];
  scene.traverse((o: any) => {
    if (o.isLight && o.userData?.dimsimDefault) doomed.push(o);
  });
  for (const l of doomed) {
    try { l.parent?.remove(l); } catch { /* already gone */ }
    try { l.dispose?.(); } catch { /* no-op */ }
  }
  scene.environment = null; // drop the image-based light (the main "too bright" source)
  return doomed.length;
}

// Turn shadows on. Then set castShadow on lights/meshes and receiveShadow on surfaces.
export function enableShadows(): void {
  if (!renderer) throw new Error("scene-api not initialized");
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.autoUpdate = true;
  renderer.shadowMap.__dimsimForced = true; // stop syncShadowMapEnabled() clobbering it
}

// Lowest surface Y under (x, z), or null. Lowest so a ceiling/roof above the floor doesn't win.
function _floorYAt(x: number, z: number, fromY: number): number | null {
  if (!scene || !THREE) throw new Error("scene-api not initialized");
  scene.updateMatrixWorld(true);
  const ray = new THREE.Raycaster(
    new THREE.Vector3(x, fromY, z),
    new THREE.Vector3(0, -1, 0),
  );
  const hits = ray
    .intersectObjects(scene.children, true)
    .filter((h: any) => h.object?.isMesh && h.object?.visible !== false);
  if (hits.length === 0) return null;
  return hits.reduce((lo: number, h: any) => Math.min(lo, h.point.y), Infinity);
}

// Capsule centre→foot offset, from the declared embodiment (else default capsule).
function _capsuleOffset(opts: { halfHeight?: number; radius?: number }): number {
  const emb = _pendingEmbodiment ?? {};
  const halfHeight = opts.halfHeight ?? emb.halfHeight ?? 0.25;
  const radius = opts.radius ?? emb.radius ?? 0.12;
  return halfHeight + radius;
}

// Spawn point on the floor at (x, z); y = floor + capsule offset. Pass { fromY }
// for multi-story. Warns and uses { fallbackY } (default 0.5) if there's no floor.
export function placeOnGround(
  x: number,
  z: number,
  opts: { halfHeight?: number; radius?: number; fromY?: number; fallbackY?: number } = {},
): { x: number; y: number; z: number } {
  const floorY = _floorYAt(x, z, opts.fromY ?? 1000);
  if (floorY == null) {
    console.warn(
      `[placeOnGround] no ground under (x=${x}, z=${z}) — robot will float/fall. ` +
        `Pick a spot over the floor, or pass { fallbackY }.`,
    );
    return { x, y: opts.fallbackY ?? 0.5, z };
  }
  return { x, y: floorY + _capsuleOffset(opts), z };
}

// Airborne spawn point: altitude metres above the floor at (x, z), for drones.
export function placeInAir(
  x: number,
  z: number,
  altitude: number,
  opts: { fromY?: number } = {},
): { x: number; y: number; z: number } {
  const floorY = _floorYAt(x, z, opts.fromY ?? 1000);
  if (floorY == null) {
    console.warn(
      `[placeInAir] no ground under (x=${x}, z=${z}) — measuring altitude from y=0.`,
    );
    return { x, y: altitude, z };
  }
  return { x, y: floorY + altitude, z };
}

// Auto-pick a collision-free spawn: spiral outward from opts.near (default origin),
// return the nearest spot with floor under it and no collider overlapping the capsule.
// Call after colliders exist. opts: near, altitude (drone hover), maxRadius (40),
// margin (0.1), radius/halfHeight (default embodiment), step, fromY, maxSamples (800).
export function findOpenSpawn(
  opts: {
    near?: { x: number; z: number };
    altitude?: number;
    maxRadius?: number; step?: number; margin?: number; maxSamples?: number;
    halfHeight?: number; radius?: number; fromY?: number;
  } = {},
): { x: number; y: number; z: number } {
  if (!scene || !THREE || !RAPIER || !rapierWorld) throw new Error("scene-api not initialized");
  scene.updateMatrixWorld(true);

  // index colliders into the query pipeline (safe outside the step loop)
  try { rapierWorld.queryPipeline?.update?.(rapierWorld.colliders); } catch { /* stepped elsewhere */ }

  const near = opts.near ?? { x: 0, z: 0 };
  if (!Number.isFinite(near.x) || !Number.isFinite(near.z)) {
    throw new Error(`findOpenSpawn: near must be { x, z } with finite numbers, got ${JSON.stringify(opts.near)} (did you write y instead of z?)`);
  }
  const emb = _pendingEmbodiment ?? {};
  const halfHeight = opts.halfHeight ?? emb.halfHeight ?? 0.25;
  const radius = opts.radius ?? emb.radius ?? 0.12;
  const margin = opts.margin ?? 0.1;
  const offset = halfHeight + radius;
  const altitude = opts.altitude; // set => hover this high (drones)
  const fromY = opts.fromY ?? 1000;
  const maxRadius = opts.maxRadius ?? 40;
  const step = opts.step ?? Math.max(0.5, radius + margin);
  const maxSamples = opts.maxSamples ?? 800; // each sample is a raycast; bound the work

  const shape = new RAPIER.Ball(radius + margin);
  const rot = { x: 0, y: 0, z: 0, w: 1 };

  const clearAt = (x: number, z: number): { x: number; y: number; z: number } | null => {
    const floorY = _floorYAt(x, z, fromY);
    if (floorY == null) return null;                 // no ground here
    const y = floorY + (altitude != null ? altitude : offset);
    const hit = rapierWorld.queryPipeline.intersectionWithShape(
      rapierWorld.bodies, rapierWorld.colliders,
      { x, y, z }, rot, shape, RAPIER.QueryFilterFlags.EXCLUDE_SENSORS,
    );
    return hit == null ? { x, y, z } : null;
  };

  // spiral outward from near; first clear + grounded hit wins. Bounded by
  // maxSamples (each sample is a raycast) so a closed-in `near` can't hang.
  let tried = 0;
  const c0 = clearAt(near.x, near.z); tried++;
  if (c0) return c0;
  for (let r = step; r <= maxRadius && tried < maxSamples; r += step) {
    const n = Math.min(48, Math.max(8, Math.ceil((2 * Math.PI * r) / step)));
    for (let k = 0; k < n && tried < maxSamples; k++) {
      const a = (2 * Math.PI * k) / n;
      const c = clearAt(near.x + r * Math.cos(a), near.z + r * Math.sin(a)); tried++;
      if (c) return c;
    }
  }

  console.warn(`[findOpenSpawn] no clear spot in ${tried} samples within ${maxRadius} m of (${near.x}, ${near.z}) — using near.`);
  const cy = _floorYAt(near.x, near.z, fromY);
  return { x: near.x, y: (cy ?? 0) + (altitude != null ? altitude : offset), z: near.z };
}

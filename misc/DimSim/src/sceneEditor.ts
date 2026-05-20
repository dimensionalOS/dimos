/**
 * SceneEditor — Browser-side script execution engine.
 *
 * Receives {type: "exec", code, id?} commands via the DimosBridge control WS,
 * evaluates user JS with full Three.js + Rapier globals exposed, and returns
 * {type: "execResult", id, success, result?, error?}.
 *
 * Must NOT modify engine.js — hooks into DimosBridge WS the same way EvalHarness does.
 */

import type { DimosBridge } from "./bridge.ts";

export interface SceneEditorGlobals {
  scene: any;         // THREE.Scene
  THREE: any;         // Three.js namespace
  RAPIER: any;        // Rapier namespace (may be null until ensureRapierLoaded)
  rapierWorld: any;   // Rapier.World (may be null)
  worldBody: any;     // Fixed RigidBody for static colliders
  renderer: any;      // THREE.WebGLRenderer
  camera: any;        // THREE.PerspectiveCamera
  agent: any;         // Player agent (has getPosition, setPosition, group)
  assets: any[];      // Scene assets array
  assetsGroup: any;   // THREE.Group containing loaded asset meshes
  gltfLoader: any;    // THREE GLTFLoader instance
}

export interface SceneEditorOptions {
  bridge: DimosBridge;
  globals: SceneEditorGlobals;
  channel?: string;
}

// AsyncFunction constructor — allows top-level await in user scripts
const AsyncFunction = Object.getPrototypeOf(async function () {}).constructor;

export class SceneEditor {
  bridge: DimosBridge;
  globals: SceneEditorGlobals;
  channel: string;

  constructor({ bridge, globals, channel }: SceneEditorOptions) {
    this.bridge = bridge;
    this.globals = globals;
    this.channel = channel || "";
    this._hookBridgeMessages();
  }

  _hookBridgeMessages(): void {
    const origConnect = this.bridge.connect.bind(this.bridge);
    this.bridge.connect = () => {
      origConnect();
      setTimeout(() => {
        const ws = this.bridge.ws;
        if (ws) this._patchWsOnMessage(ws);
      }, 100);
    };
    const ws = this.bridge.ws;
    if (ws) this._patchWsOnMessage(ws);
  }

  _patchWsOnMessage(ws: WebSocket): void {
    const origOnMessage = ws.onmessage;
    ws.onmessage = (event: MessageEvent) => {
      if (typeof event.data === "string") {
        try {
          const cmd = JSON.parse(event.data);
          if (cmd.type === "exec" || cmd.type === "loadScript") {
            this._handleCommand(cmd);
            return;
          }
        } catch { /* not JSON or not for us */ }
      }
      // Pass through to existing handlers (EvalHarness, DimosBridge)
      if (origOnMessage) (origOnMessage as (e: MessageEvent) => void).call(ws, event);
    };
  }

  _send(msg: Record<string, any>): void {
    if (this.channel) msg.channel = this.channel;
    this.bridge.sendCommand(msg);
  }

  async _handleCommand(cmd: { type: string; code?: string; url?: string; id?: string; channel?: string }): Promise<void> {
    if (this.channel && cmd.channel && cmd.channel !== this.channel) return;

    if (cmd.type === "exec" && cmd.code) {
      await this._execCode(cmd.code, cmd.id);
    } else if (cmd.type === "loadScript" && cmd.url) {
      await this._loadScript(cmd.url, cmd.id);
    }
  }

  // Track colliders created by addCollider so removeCollider can clean up
  _colliderMap: Map<string, any> = new Map(); // mesh.uuid → Rapier collider
  // Track dynamic rigid bodies (body + mesh ref for position sync)
  _dynamicBodies: Map<string, { body: any; mesh: any }> = new Map();
  // Track NPC mixers for animation updates
  _npcMixers: Map<string, any> = new Map(); // npc name → THREE.AnimationMixer
  _npcAnimFrame: number | null = null;
  _npcClock: any = null; // THREE.Clock

  async _execCode(code: string, id?: string): Promise<void> {
    console.log(`[sceneEditor] exec${id ? ` (${id})` : ""}:`, code.slice(0, 100));
    try {
      const g = this.globals;
      const colliderMap = this._colliderMap;

      // loadGLTF: convenience async helper for loading GLTF/GLB models
      const loadGLTF = (url: string): Promise<any> =>
        new Promise((resolve, reject) =>
          g.gltfLoader.load(url, resolve, undefined, reject),
        );

      // addCollider: create a physics collider for a mesh/group
      // shape: "trimesh" (default) | "box" | "sphere"
      // opts.dynamic: if true, creates a dynamic rigid body (responds to gravity/collisions)
      // opts.mass: mass in kg (default 1.0, only for dynamic)
      // opts.restitution: bounciness 0-1 (default 0.3, only for dynamic)
      // Creates collider browser-side AND sends command to server (for lidar/physics)
      const sendPhysics = this._send.bind(this);
      const dynamicBodies = this._dynamicBodies;
      const selfRef = this;
      const addCollider = (obj: any, shapeOrOpts?: string | { shape?: string; dynamic?: boolean; mass?: number; restitution?: number }): any => {
        let shape = "trimesh";
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

        // Remove existing collider if any
        removeCollider(obj);

        const bbox = new g.THREE.Box3().setFromObject(obj);
        const size = new g.THREE.Vector3();
        const center = new g.THREE.Vector3();
        bbox.getSize(size);
        bbox.getCenter(center);

        const clamp = (v: number) => Math.max(v, 0.001);

        // Build server-side descriptor (shape-agnostic)
        const serverDesc: any = {
          shape,
          position: { x: center.x, y: center.y, z: center.z },
        };

        if (shape === "sphere") {
          const r = clamp(Math.max(size.x, size.y, size.z) / 2);
          serverDesc.radius = r;
        } else if (shape === "trimesh") {
          const verts: number[] = [];
          const indices: number[] = [];
          let vertBase = 0;
          obj.traverse((m: any) => {
            if (!m.isMesh) return;
            const geom = m.geometry;
            const posAttr = geom?.attributes?.position;
            if (!posAttr) return;
            const tmpPos = new g.THREE.Vector3();
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
          serverDesc.vertices = Array.from(verts);
          serverDesc.indices = Array.from(indices);
        } else {
          // box (default)
          serverDesc.halfExtents = {
            x: clamp(size.x / 2), y: clamp(size.y / 2), z: clamp(size.z / 2),
          };
        }

        // Browser-side collider (for standalone / non-dimos mode)
        if (g.RAPIER && g.rapierWorld) {
          let desc: any;
          if (shape === "sphere") {
            desc = g.RAPIER.ColliderDesc.ball(serverDesc.radius);
            if (!dynamic) desc.setTranslation(center.x, center.y, center.z);
          } else if (shape === "trimesh") {
            desc = g.RAPIER.ColliderDesc.trimesh(
              new Float32Array(serverDesc.vertices), new Uint32Array(serverDesc.indices)
            );
          } else {
            const h = serverDesc.halfExtents;
            desc = g.RAPIER.ColliderDesc.cuboid(h.x, h.y, h.z);
            if (!dynamic) desc.setTranslation(center.x, center.y, center.z);
          }
          desc.setFriction(0.9);
          desc.setRestitution(restitution);

          if (dynamic && shape !== "trimesh") {
            // Dynamic: create rigid body + attach collider
            const bodyDesc = g.RAPIER.RigidBodyDesc.dynamic()
              .setTranslation(center.x, center.y, center.z);
            const body = g.rapierWorld.createRigidBody(bodyDesc);
            body.setAdditionalMass(mass);
            const collider = g.rapierWorld.createCollider(desc, body);
            colliderMap.set(obj.uuid, collider);
            dynamicBodies.set(obj.uuid, { body, mesh: obj });
            selfRef._ensureDynamicSyncLoop();
          } else {
            // Static: collider with no parent body
            const collider = g.rapierWorld.createCollider(desc);
            colliderMap.set(obj.uuid, collider);
          }
        }

        // Server-side collider (for lidar + dimos physics)
        serverDesc.dynamic = dynamic;
        if (dynamic) { serverDesc.mass = mass; serverDesc.restitution = restitution; }
        sendPhysics({ type: "physicsColliderAdd", uuid: obj.uuid, desc: serverDesc });

        return { shape, dynamic, uuid: obj.uuid, size: { x: +size.x.toFixed(3), y: +size.y.toFixed(3), z: +size.z.toFixed(3) } };
      };

      // removeCollider: remove collider browser-side + server-side
      const removeCollider = (obj: any): boolean => {
        const existing = colliderMap.get(obj.uuid);
        if (existing) {
          try {
            g.rapierWorld?.removeCollider(existing, true);
          } catch { /* already removed */ }
          colliderMap.delete(obj.uuid);
        }
        // Clean up dynamic rigid body if any
        const dynEntry = dynamicBodies.get(obj.uuid);
        if (dynEntry) {
          try { g.rapierWorld?.removeRigidBody(dynEntry.body); } catch { /* already removed */ }
          dynamicBodies.delete(obj.uuid);
        }
        // Always tell server to remove (even if browser didn't have it)
        sendPhysics({ type: "physicsColliderRemove", uuid: obj.uuid });
        return !!existing;
      };

      // addNPC: load an animated GLTF character, place it, and play an animation
      const npcMixers = this._npcMixers;
      const self = this;
      const addNPC = async (opts: {
        url: string;
        name?: string;
        position?: { x: number; y: number; z: number };
        rotation?: number; // yaw in radians
        scale?: number;
        animation?: string | number; // clip name or index (default: 0)
        collider?: boolean; // add trimesh collider (default: true)
      }): Promise<any> => {
        const gltf = await loadGLTF(opts.url);
        const model = gltf.scene;
        const npcName = opts.name || `npc-${Date.now().toString(36)}`;
        model.name = npcName;
        if (opts.position) model.position.set(opts.position.x, opts.position.y, opts.position.z);
        if (opts.rotation != null) model.rotation.y = opts.rotation;
        if (opts.scale != null) model.scale.setScalar(opts.scale);
        model.traverse((child: any) => { if (child.isMesh) { child.castShadow = true; child.receiveShadow = true; } });
        g.scene.add(model);

        // Set up animation
        let activeClipName = "";
        const clipNames: string[] = [];
        if (gltf.animations && gltf.animations.length > 0) {
          const mixer = new g.THREE.AnimationMixer(model);
          npcMixers.set(npcName, mixer);

          for (const clip of gltf.animations) clipNames.push(clip.name);
          // Store clips on model so they can be switched at runtime
          model.animations = gltf.animations;

          // Select animation clip
          let clipIndex = 0;
          if (typeof opts.animation === "string") {
            const idx = gltf.animations.findIndex((c: any) => c.name.toLowerCase().includes(opts.animation!.toString().toLowerCase()));
            if (idx >= 0) clipIndex = idx;
          } else if (typeof opts.animation === "number") {
            clipIndex = Math.min(opts.animation, gltf.animations.length - 1);
          }

          const clip = gltf.animations[clipIndex];
          activeClipName = clip.name;
          const action = mixer.clipAction(clip);
          action.play();

          // Start the animation loop if not already running
          self._ensureNpcAnimLoop();
        }

        // Add collider (default: trimesh)
        let colliderInfo = null;
        if (opts.collider !== false) {
          colliderInfo = addCollider(model, "trimesh");
        }

        return {
          name: npcName,
          animations: clipNames,
          activeAnimation: activeClipName,
          collider: colliderInfo,
        };
      };

      // removeNPC: remove an NPC from scene and clean up its mixer
      const removeNPC = (name: string): boolean => {
        const obj = g.scene.getObjectByName(name);
        if (!obj) return false;
        // Stop animation mixer
        const mixer = npcMixers.get(name);
        if (mixer) { mixer.stopAllAction(); npcMixers.delete(name); }
        // Remove collider
        removeCollider(obj);
        // Clear name before removal so getObjectByName won't find stale refs
        obj.name = "";
        // Remove from scene
        obj.traverse((child: any) => {
          if (child.isMesh) { child.geometry?.dispose(); child.material?.dispose(); }
        });
        g.scene.remove(obj);
        return true;
      };

      // autoScale: detect cm/m mismatch and normalize model to scene scale.
      // Heuristic: if bounding box exceeds targetMaxDim (default 50m) in any axis,
      // assume the model is in centimeters and scale by 0.01. For intermediate cases
      // (10-50m), scale proportionally so the largest dimension equals targetMaxDim.
      // Returns the scale factor applied (1.0 if no change).
      const autoScale = (obj: any, targetMaxDim = 50): number => {
        const bbox = new g.THREE.Box3().setFromObject(obj);
        const size = new g.THREE.Vector3();
        bbox.getSize(size);
        const maxDim = Math.max(size.x, size.y, size.z);
        if (maxDim <= 0.001) return 1.0; // degenerate
        let scaleFactor = 1.0;
        if (maxDim > targetMaxDim * 2) {
          // Very large — likely centimeters (100x off)
          scaleFactor = 0.01;
        } else if (maxDim > targetMaxDim) {
          // Moderately large — scale down proportionally
          scaleFactor = targetMaxDim / maxDim;
        }
        if (scaleFactor !== 1.0) {
          obj.scale.multiplyScalar(scaleFactor);
          obj.updateMatrixWorld(true);
          console.log(`[sceneEditor] autoScale: ${maxDim.toFixed(1)}m → ${(maxDim * scaleFactor).toFixed(1)}m (×${scaleFactor.toFixed(4)})`);
        }
        return scaleFactor;
      };

      const fn = new AsyncFunction(
        "scene", "THREE", "RAPIER", "rapierWorld", "renderer", "camera",
        "agent", "playerBody", "assets", "assetsGroup",
        "loadGLTF", "addCollider", "removeCollider", "addNPC", "removeNPC", "autoScale",
        code,
      );
      const result = await fn(
        g.scene, g.THREE, g.RAPIER, g.rapierWorld, g.renderer, g.camera,
        g.agent, g.agent, g.assets, g.assetsGroup,
        loadGLTF, addCollider, removeCollider, addNPC, removeNPC, autoScale,
      );
      this._send({ type: "execResult", id, success: true, result: _serialize(result) });
    } catch (err: any) {
      console.error("[sceneEditor] exec error:", err);
      this._send({ type: "execResult", id, success: false, error: String(err) });
    }
  }

  async _loadScript(url: string, id?: string): Promise<void> {
    console.log(`[sceneEditor] loadScript${id ? ` (${id})` : ""}:`, url);
    try {
      const resp = await fetch(url);
      if (!resp.ok) throw new Error(`HTTP ${resp.status}: ${resp.statusText}`);
      const code = await resp.text();
      await this._execCode(code, id);
    } catch (err: any) {
      console.error("[sceneEditor] loadScript error:", err);
      this._send({ type: "execResult", id, success: false, error: String(err) });
    }
  }

  _ensureNpcAnimLoop(): void {
    if (this._npcAnimFrame != null) return;
    this._startUpdateLoop();
  }

  _ensureDynamicSyncLoop(): void {
    if (this._npcAnimFrame != null) return;
    this._startUpdateLoop();
  }

  /** Shared rAF loop for NPC animations + dynamic body position sync. */
  _startUpdateLoop(): void {
    if (this._npcAnimFrame != null) return;
    if (!this._npcClock) {
      this._npcClock = new this.globals.THREE.Clock();
    }
    const tick = () => {
      const hasWork = this._npcMixers.size > 0 || this._dynamicBodies.size > 0;
      if (!hasWork) {
        this._npcAnimFrame = null;
        return;
      }
      const dt = this._npcClock.getDelta();

      // Update NPC animations
      for (const mixer of this._npcMixers.values()) {
        mixer.update(dt);
      }

      // Sync dynamic body positions → Three.js meshes
      for (const { body, mesh } of this._dynamicBodies.values()) {
        const t = body.translation();
        const r = body.rotation();
        mesh.position.set(t.x, t.y, t.z);
        mesh.quaternion.set(r.x, r.y, r.z, r.w);
      }

      this._npcAnimFrame = requestAnimationFrame(tick);
    };
    this._npcAnimFrame = requestAnimationFrame(tick);
  }

  dispose(): void {
    if (this._npcAnimFrame != null) cancelAnimationFrame(this._npcAnimFrame);
    for (const mixer of this._npcMixers.values()) mixer.stopAllAction();
    this._npcMixers.clear();
    this._dynamicBodies.clear();
  }
}

/** Safely serialize a return value for JSON transport. */
function _serialize(val: any): any {
  if (val === undefined || val === null) return val;
  if (typeof val === "number" || typeof val === "string" || typeof val === "boolean") return val;
  if (Array.isArray(val)) return val.map(_serialize);
  // Three.js objects have .toJSON() but it's huge — just return type + id
  if (val.isObject3D) return { _type: "Object3D", type: val.type, name: val.name, uuid: val.uuid };
  if (val.isMesh) return { _type: "Mesh", name: val.name, uuid: val.uuid };
  try { return JSON.parse(JSON.stringify(val)); } catch { return String(val); }
}

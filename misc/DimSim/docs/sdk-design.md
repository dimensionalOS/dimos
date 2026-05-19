# DimSim Scene SDK — Design + Tickets

**Issue**: dimensionalOS/dimos#1691 (Simulation Editing)
**Date**: 2026-03-27

---

## Coverage Matrix

Every request from #1691 mapped to a ticket:

| #1691 Request | Ticket |
|---------------|--------|
| Model importing (GLTFLoader API) | SDK-1 |
| Standard Three.js API (primitives, scenegraph, materials, textures, lights, cameras, shadows, fog) | SDK-1 |
| Optional physics for models (collisions toggle) | SDK-1, ENG-1 |
| Robot definition API (drone, holonomic, plane, car) | SDK-2 |
| Validation: deploy robot in random Three.js environment | VAL-1 |
| Validation: third-party map load (Sketchfab GLB) | VAL-1 |
| Validation: add red ball with collisions | VAL-1 |
| Validation: editing flow (change position, reload, check) | ENG-2 |
| Validation: new embodiment (code a drone) | VAL-2 |

---

## Architecture

```
Developer code (TypeScript)          DimSim Runtime (browser)
┌─────────────────────────┐         ┌──────────────────────┐
│  import { Scene, Robot  │         │  engine.js            │
│    } from "dimsim/sdk"  │         │  importLevelFromJSON()│
│                         │  JSON   │  AiAvatar / Robot cfg │
│  scene.addModel(...)    │ ──────► │  Rapier physics       │
│  scene.addBox(...)      │         │  Three.js renderer    │
│  scene.setRobot(...)    │         │  Eval harness         │
│  scene.export("x.json") │         └──────────────────────┘
└─────────────────────────┘
         │
         ▼
┌─────────────────────────┐
│  dimsim dev --scene x   │
│  dimsim eval --scene x  │
└─────────────────────────┘
```

**Key principle**: The SDK outputs DimSim's existing scene JSON format.
The runtime already consumes it. Most tickets need zero engine changes.

---

## Tickets

### SDK-1: Scene builder API — models, primitives, lights, materials, physics

**What**: TypeScript `Scene` class that builds scene JSON. Covers model importing, Three.js primitives, materials, lights, optional physics — the core of the issue.

**Scope**:
- `Scene` class with `addBox`, `addSphere`, `addCylinder`, `addCone`, `addTorus`, `addPlane`
- `addModel(id, { url, physics, collider, position, rotation, scale })` — GLB/GLTF by URL
- `addPointLight`, `addDirectionalLight`, `addSpotLight`, `addAmbientLight`
- `sky({ topColor, horizonColor, bottomColor, brightness, sunStrength })`
- `fog({ color, near, far })` — Three.js fog
- `addTag(name, { position })` — navigation waypoints for evals
- `group(name)` — scenegraph grouping, children inherit transform
- `setSpawnPoint({ x, y, z, yaw })`
- PBR materials: `{ color, roughness, metalness, specularIntensity, envMapIntensity }`
- Shadow config per object: `castShadow`, `receiveShadow`
- Physics per object: `physics: true/false`, `dynamic: true/false`, `collider: "auto" | "box" | "trimesh" | "convex"`
- `scene.export(filepath)` writes JSON, `scene.toJSON()` returns raw object
- `dimsim dev --scene ./file.json` accepts local file paths (not just S3 manifest names)
- Published as `@antim/dimsim/sdk` export from existing JSR package

```typescript
import { Scene } from "@antim/dimsim/sdk";

const scene = new Scene("fps-arena");
scene.addModel("map", { url: "./models/lowpoly-fps-map.glb", physics: true });
scene.addSphere("ball", {
  radius: 0.3, position: [2, 2, 0],
  material: { color: "#FF0000" },
  physics: true, dynamic: true,
});
scene.addPointLight("sun", { position: [0, 10, 0], intensity: 3 });
scene.setSpawnPoint({ x: 0, y: 0.5, z: 3 });
await scene.export("./scenes/fps-arena.json");
```

**Files**: `sdk/mod.ts`, `sdk/scene.ts`, `sdk/primitives.ts`, `sdk/model.ts`, `sdk/light.ts`, `sdk/types.ts`, `cli.ts` (--scene filepath)

---

### SDK-2: Robot definition in scene JSON

**What**: `robot` field in scene JSON so developers configure the agent type, collider, avatar, and controller from code. AiAvatar reads this at spawn instead of using hardcoded values.

**Scope**:
- SDK: `scene.setRobot(Robot.capsule({ ... }))`, `Robot.drone({ ... })`, `Robot.car({ ... })`, `Robot.holonomic({ ... })`
- Scene JSON gets `robot` field:
  ```json
  { "robot": { "type": "drone", "radius": 0.15, "height": 0.08, "avatar": "./drone.glb", "walkSpeed": 5.0, "hoverHeight": 1.5 } }
  ```
- AiAvatar reads `sceneJson.robot` for collider size, avatar GLB, speed, and controller type
- `capsule` = current behavior (kinematic character controller)
- `drone` = same controller but Y-axis movement enabled, hover height default, cmd_vel.linear.z maps to vertical
- `holonomic` = omnidirectional, cmd_vel.linear.y maps to strafe
- `car` = box collider, linear.x = throttle, angular.z = steering angle
- All types consume standard `cmd_vel` Twist — dimos nav stack works unchanged

```typescript
scene.setRobot(Robot.drone({
  radius: 0.15,
  height: 0.08,
  walkSpeed: 5.0,
  hoverHeight: 2.0,
  avatar: "./models/quadcopter.glb",
}));
```

**Files**: `sdk/robot.ts`, `src/AiAvatar.js` (read config from scene), `src/engine.js` (pass robot config)

---

### ENG-1: Dynamic rigid bodies in engine

**What**: Support `dynamic: true` on primitives and models so objects respond to gravity and collisions (not just static walls/floors).

**Scope**:
- Currently all colliders use `RigidBodyDesc.fixed()` (immovable)
- Add `RigidBodyDesc.dynamic()` path when scene JSON has `dynamic: true`
- Expose `mass`, `friction`, `restitution` in primitive/model JSON
- Rapier already supports all of this — just need to wire the properties through

**Covers**: "Add a red ball with collisions enabled" — ball should fall and roll, not float in place.

**Files**: `src/engine.js` (collider builder sections: `buildPrimitiveCollider`, `buildRapierTriMeshColliderFromObject`)

---

### ENG-2: Hot reload (`--watch`)

**What**: `dimsim dev --scene ./file.json --watch` watches the JSON file and auto-reloads when it changes. The "reasonable editing flow" from the issue.

**Scope**:
- `Deno.watchFs()` on scene file path in bridge server
- On change: read new JSON, send `{ type: "reloadScene", scene: <data> }` to browser via WS
- Browser calls `importLevelFromJSON()` (already exists)
- Developer flow: edit `build-scene.ts` → save → re-run → scene JSON updates → browser reloads automatically
- ~20 lines of code in bridge server

**Covers**: "change the position of an object, reload, check, add a light, reload"

**Files**: `bridge/server.ts`, `cli.ts` (--watch flag)

---

### VAL-1: Demo scenes — Three.js env, Sketchfab map, custom objects

**What**: Three example scripts proving the SDK works end-to-end. Each one is `deno run examples/X.ts` → produces JSON → `dimsim dev --scene ./output.json`.

**Scope**:
- `examples/threejs-env.ts` — load a Three.js example GLB (e.g. LittlestTokyo), add lights, deploy robot
- `examples/sketchfab-map.ts` — load a Sketchfab low-poly FPS map GLB with physics, deploy robot
- `examples/custom-scene.ts` — floor + walls + red ball (dynamic) + lights + fog, robot navigates
- README with run instructions

**Covers**: All three validation scenarios from #1691

**Files**: `examples/*.ts`, `examples/README.md`

---

### VAL-2: Demo — drone embodiment

**What**: Example script deploying a drone robot in a scene, validating the robot definition API works.

**Scope**:
- `examples/drone-demo.ts` — scene with obstacles at varying heights, drone robot config
- Demonstrates: drone avatar loads, drone hovers at configured height, cmd_vel.linear.z controls altitude
- Should require "similar amount of code to doing it in plain Three.js" per the issue

```typescript
const scene = new Scene("drone-test");
scene.addModel("building", { url: "./models/building.glb", physics: true });
scene.setRobot(Robot.drone({ hoverHeight: 2.0, avatar: "./models/drone.glb" }));
scene.setSpawnPoint({ x: 0, y: 2, z: 0 });
await scene.export("./scenes/drone-test.json");
```

**Covers**: "Code a drone, deploy within a world above"

**Files**: `examples/drone-demo.ts`

---

## Dependency Graph

```
SDK-1 (scene builder)  ─────┬──► VAL-1 (3 demo scenes)
                             │
SDK-2 (robot config)   ──────┼──► VAL-2 (drone demo)
                             │
ENG-1 (dynamic bodies) ─────┘
ENG-2 (hot reload)     ── independent
```

**Build order**: SDK-1 → (ENG-1 + ENG-2 + SDK-2 in parallel) → (VAL-1 + VAL-2)

---

## File Structure

```
DimSim/
├── sdk/
│   ├── mod.ts              # Public API: export { Scene, Robot }
│   ├── scene.ts            # Scene builder class
│   ├── primitives.ts       # Primitive type definitions + helpers
│   ├── model.ts            # GLB/GLTF model wrapper
│   ├── light.ts            # Light types
│   ├── robot.ts            # Robot.capsule(), Robot.drone(), etc.
│   └── types.ts            # Vec3, Transform, Material, etc.
├── examples/
│   ├── threejs-env.ts
│   ├── sketchfab-map.ts
│   ├── custom-scene.ts
│   ├── drone-demo.ts
│   └── README.md
├── dimos-cli/
│   ├── cli.ts              # --scene filepath, --watch flag
│   ├── bridge/server.ts    # hot reload watcher
│   └── ...
├── src/
│   ├── engine.js           # dynamic bodies, robot config passthrough
│   ├── AiAvatar.js         # read robot config from scene JSON
│   └── ...
└── docs/
    └── sdk-design.md       # this file
```

## JSR exports

```json
{
  "exports": {
    ".": "./cli.ts",
    "./mod": "./mod.ts",
    "./sdk": "./sdk/mod.ts"
  }
}
```

Usage: `import { Scene, Robot } from "@antim/dimsim/sdk";`

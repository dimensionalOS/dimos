# Scenes

A scene is one JS file: `scenes/<name>/index.js`. It default-exports an async `build(api)` function. DimSim hot-reloads it on save.

## Create a new scene

```bash
mkdir -p misc/DimSim/scenes/my-room
```

`misc/DimSim/scenes/my-room/index.js`:

```js
export default async function build({ scene, THREE, physics, setSky }) {
  setSky({ topColor: '#3a4654', horizonColor: '#cfd6df', brightness: 0.8 });

  // floor
  const floor = new THREE.Mesh(
    new THREE.BoxGeometry(20, 0.2, 20),
    new THREE.MeshPhysicalMaterial({ color: 0x808080, roughness: 0.9 }),
  );
  floor.position.y = -0.1;
  scene.add(floor);
  physics.staticCollider(floor, 'box');

  // a wall
  const wall = new THREE.Mesh(
    new THREE.BoxGeometry(20, 3, 0.2),
    new THREE.MeshPhysicalMaterial({ color: 0xc4c1b8, roughness: 0.8 }),
  );
  wall.position.set(0, 1.5, -10);
  scene.add(wall);
  physics.staticCollider(wall, 'box');

  // lighting
  scene.add(new THREE.HemisphereLight(0xffffff, 0x404040, 0.6));
  const sun = new THREE.DirectionalLight(0xffffff, 1.0);
  sun.position.set(10, 20, 10);
  sun.castShadow = true;
  scene.add(sun);

  return { spawnPoint: { x: 0, y: 0.5, z: 0 } };
}
```

Run it:

```bash
dimsim dev --scene my-room
# or via dimos:
dimos --simulation dimsim --dimsim-scene=my-room run unitree-go2-basic
```

## Edit a scene

Open the file, edit, save. The browser HMR-reloads — no full refresh.

The whole `build()` re-runs on every save, so iteration is cheap. Try changing `setSky({ brightness: 0.8 })` to `1.5` — the sky brightens within a second.

## The `api` argument

`build(api)` gets one argument — destructure what you need:

| Field | What |
|---|---|
| `scene` | The `THREE.Scene`. `scene.add(mesh)` anything you want rendered. |
| `THREE` | The engine's THREE module. Use this rather than re-importing. |
| `physics.staticCollider(mesh, shape)` | Make a mesh solid (agent can't walk through, lidar hits it). `shape`: `'box'` \| `'trimesh'` \| `'sphere'`. |
| `physics.dynamicCollider(mesh, {mass, shape})` | Mesh becomes a rigid body — falls, can be pushed. Engine syncs `mesh.position` each frame. |
| `setSky({...})` | Atmosphere. Keys: `topColor`, `horizonColor`, `bottomColor`, `brightness`, `softness`, `sunStrength`, `sunHeight`. |
| `setEmbodiment({...})` | Declare the agent — avatar GLB + capsule dimensions + physics mode + control params. See "Robot embodiment" below. |
| `loadGLTF(url)` | Async GLB load — `await loadGLTF('./forklift.glb')` returns `{scene, animations, …}`. |
| `agent`, `camera`, `renderer`, `RAPIER`, `rapierWorld` | Live engine refs if you need them. |

## Robot embodiment

`setEmbodiment(config)` swaps the avatar mesh **and** reconfigures the bridge's server-side physics + lidar mount — so changing `embodimentType` from `'ground'` to `'drone'` instantly switches the cmd_vel → motion mapping from differential-drive (with gravity) to 6DoF flight (no gravity, altitude clamp).

```js
// ground robot (default)
setEmbodiment({
  embodimentType: 'ground',
  avatarUrl:    '/agent-model/dimsim_unitree_stub.glb',
  radius:       0.18,
  halfHeight:   0.25,
  maxSpeed:     1.5,
  turnRate:     2.5,
  gravity:      -9.81,
});

// drone
setEmbodiment({
  embodimentType: 'drone',
  avatarUrl:    '/agent-model/dimsim_unitree_stub.glb',
  radius:       0.3,
  halfHeight:   0.1,
  gravity:       0,
  maxSpeed:      3.0,
  turnRate:      2.0,
  maxAltitude:   8,
});
```

| Field | What |
|---|---|
| `embodimentType` | `'ground'` (differential-drive + gravity) or `'drone'` (6DoF). |
| `avatarUrl` | URL of the GLB to render. Bridge serves `/agent-model/*` from `public/`. |
| `radius`, `halfHeight` | Capsule collider dimensions. Also drive lidar mount height. |
| `maxSpeed` | Linear cmd_vel scale. |
| `turnRate` | Angular.z cmd_vel scale. |
| `gravity` | m/s². `0` for flight, `-9.81` for ground. |
| `maxAltitude` | Drone-only ceiling. |
| `lidarMountHeight`, `maxStepHeight`, `groundSnapDist`, `maxSlopeAngle`, `friction` | Optional fine-tuning. |

Call it once at scene-build time (anywhere in `build()`), or call it again later to swap mid-scene.  `scenes/warehouse/index.js` declares a drone as its first line — see it for a working example.

## Return value

`build()` can return `{ spawnPoint?, embodiment? }`:

```js
return { spawnPoint: { x: 2, y: 0.5, z: -5 } };
```

If omitted, the agent spawns at `(2, 0.5, 3)`.

## Common patterns

**Loops for repeated geometry.** The whole module re-runs on HMR, so spawning 50 crates via a loop is fine — no `_disposed` bookkeeping needed.

```js
const crateGeo = new THREE.BoxGeometry(1, 1, 1);
const crateMat = new THREE.MeshPhysicalMaterial({ color: 0x8b5a2b, roughness: 0.75 });
for (let i = 0; i < 8; i++) {
  const c = new THREE.Mesh(crateGeo, crateMat);
  c.position.set((i % 4) - 1.5, 0.5 + Math.floor(i / 4), 3);
  scene.add(c);
  physics.staticCollider(c, 'box');
}
```

**GLB props.** Drop a GLB next to `index.js`, then:

```js
const gltf = await loadGLTF('./forklift.glb');
const forklift = gltf.scene;
forklift.position.set(5, 0, 0);
scene.add(forklift);
physics.staticCollider(forklift, 'trimesh');
```

**Lighting** — `HemisphereLight` for ambient fill + one `DirectionalLight` with `castShadow = true` is enough for most interiors. `PointLight`s give nice volumetric pendant effects but turn off `castShadow` to keep frame rate sane.

**Tune the shadow camera** if shadows pop:

```js
const sun = new THREE.DirectionalLight(0xffffff, 1.0);
sun.castShadow = true;
sun.shadow.mapSize.set(2048, 2048);
sun.shadow.camera.left = -20;
sun.shadow.camera.right = 20;
sun.shadow.camera.top = 25;
sun.shadow.camera.bottom = -25;
scene.add(sun);
```

**Materials** — `MeshPhysicalMaterial` supports clearcoat, sheen, transmission, iridescence — use it for anything you care about visually. The engine sets the renderer up for HDR/PBR.

## Always pair `scene.add(mesh)` with a collider

Otherwise the agent walks through it and lidar passes straight through:

```js
scene.add(mesh);
physics.staticCollider(mesh, 'box');     // ← don't skip this
```

Pure-visual meshes (decoration the agent can't bump into) — skip the collider on purpose. Just be aware.

## Tip: start from `scenes/empty/`

It's a one-file scene with a floor and a sky. Copy it as a template:

```bash
cp -r misc/DimSim/scenes/empty misc/DimSim/scenes/my-room
```

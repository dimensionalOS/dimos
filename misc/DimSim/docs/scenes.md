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

  // No lights needed — the engine already lights every scene (see "Lighting" below).
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
| `placeOnGround(x, z)` | Returns a floor-resting `{x, y, z}` spawn point — no Y guessing. See "Placing the robot". |
| `placeInAir(x, z, altitude)` | Like `placeOnGround`, but `altitude` metres above the floor — for drones. |
| `clearDefaultLights()` | Drop the engine's default lighting (fill lamps + image-based light) so you can light the scene yourself. See "Lighting". |
| `enableShadows()` | Turn real shadows on (off by default). See "Lighting". |
| `agent`, `camera`, `renderer`, `RAPIER`, `rapierWorld` | Live engine refs if you need them. |

## Robot embodiment

`setEmbodiment(config)` swaps the avatar mesh **and** reconfigures the bridge's server-side physics + lidar mount — so changing `embodimentType` from `'ground'` to `'drone'` instantly switches the cmd_vel → motion mapping from differential-drive (with gravity) to 6DoF flight (no gravity, altitude clamp).

```js
// ground robot (default)
setEmbodiment({
  embodimentType: 'ground',
  avatarUrl:    '/embodiment/dimsim_unitree_stub.glb',
  radius:       0.18,
  halfHeight:   0.25,
  maxSpeed:     1.5,
  turnRate:     2.5,
  gravity:      -9.81,
});

// drone (flight)
setEmbodiment({
  motionModel: 'flight',
  embodimentType: 'drone',   // also set this so the browser visual matches
  radius: 0.3, halfHeight: 0.1, gravity: 0, maxSpeed: 3.0, maxAltitude: 8,
});

// car (ackermann steering — arcs through turns, can't pivot in place)
setEmbodiment({
  motionModel: 'ackermann',
  radius: 0.4, halfHeight: 0.3, maxSpeed: 4.0, wheelBase: 1.2, maxSteerAngle: 0.6,
});
```

The motion model decides how `cmd_vel` becomes movement. Three ship today:

| `motionModel` | Behaviour | Key params |
|---|---|---|
| `holonomic` (default) | ground robot — drives along heading, gravity | `maxSpeed`, `turnRate`, `gravity` |
| `flight` | drone — 6DoF, no gravity, altitude clamp | `maxSpeed`, `maxAltitude` |
| `ackermann` | car — steers, turn rate scales with speed | `maxSpeed`, `wheelBase`, `maxSteerAngle` |

Other fields:

| Field | What |
|---|---|
| `avatarUrl` | URL of the GLB to render. Bridge serves `/embodiment/*` from `public/`. Defaults to the robot dog. |
| `embodimentType` | Browser visual: `'ground'` or `'drone'`. Legacy alias — maps to `holonomic`/`flight` if `motionModel` is omitted. |
| `radius`, `halfHeight` | Capsule collider dimensions. Also drive lidar mount height. |
| `gravity` | m/s². `0` for flight, `-9.81` for ground. |
| `lidarMountHeight`, `maxStepHeight`, `groundSnapDist`, `maxSlopeAngle`, `friction` | Optional fine-tuning. |

Call it once at scene-build time (anywhere in `build()`), or again later to swap mid-scene. `scenes/warehouse/index.js` declares a drone on its first line. To add a brand-new motion model (legged, tank, boat), add one function to `MOTION_MODELS` in `cli/bridge/physics.ts`.

## Return value

`build()` can return `{ spawnPoint?, embodiment? }`:

```js
return { spawnPoint: { x: 2, y: 0.5, z: -5 } };
```

If omitted, the agent spawns at `(2, 0.5, 3)`.

## Placing the robot

`spawnPoint` is in Three.js world coords (Y-up) — the same coords you place meshes at.

Pick `x, z`, and let `placeOnGround` resolve the height:

```js
return { spawnPoint: placeOnGround(2, -5) };   // ground robot, on the floor
```

Drone? Spawn it in the air instead:

```js
return { spawnPoint: placeInAir(2, -5, 5) };   // hover 5 m above the floor
```

> `placeOnGround` / `placeInAir` use the lowest surface under `x, z` as the floor.
> For a multi-story building, pass `{ fromY }` (a Y just above the floor you want).
> To pick a clear `x, z`, drop a marker mesh there, reload, and see where it lands.

## Common patterns

**Loops for repeated geometry.** The whole module re-runs on each save, so a loop that spawns 50 crates is fine.

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

## Lighting

The engine lights every scene by default — fill lamps **plus** an image-based light
(a soft environment that lights everything from all directions). Most scenes need no
lights of their own; just `setSky(...)`.

To light a scene yourself, call `clearDefaultLights()` first — it drops both default
sources so you start from black — then add your own:

```js
clearDefaultLights();      // removes the fill lamps + the image-based light
scene.add(new THREE.HemisphereLight(0xffffff, 0x404040, 0.5));
const sun = new THREE.DirectionalLight(0xffffff, 1.0);
sun.position.set(10, 20, 10);
scene.add(sun);
```

> Skip `clearDefaultLights()` and your lights stack on the defaults — the scene washes
> out to white. The image-based light is the main culprit, which is why this removes it too.

## Shadows

Shadows are off by default. Turn them on, then mark casters and receivers:

```js
enableShadows();
sun.castShadow = true;        // a light that casts
floor.receiveShadow = true;   // a surface that catches it
```

Leave `PointLight.castShadow = false` to save frame rate.

## Materials

`MeshPhysicalMaterial` supports clearcoat, sheen, transmission, and iridescence — use it for anything you care about visually.

## Always pair `scene.add(mesh)` with a collider

Otherwise the agent walks through it and lidar passes straight through:

```js
scene.add(mesh);
physics.staticCollider(mesh, 'box');     // ← don't skip this
```

Skip the collider only for pure-visual decoration the agent never touches.

## Tip: start from `scenes/empty/`

It's a one-file scene with a floor and a sky. Copy it as a template:

```bash
cp -r misc/DimSim/scenes/empty misc/DimSim/scenes/my-room
```

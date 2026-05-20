# Authoring scenes

A scene is a JS module at `scenes/<name>/index.js` that default-exports an async `build(api)` function. dimsim dynamically imports it at boot, hands you the scene API, and lets you do whatever you'd do in a normal Three.js script — `scene.add(mesh)`, attach lights, register physics colliders.

## Minimal scene

```js
// scenes/warehouse/index.js
export default async function build({ scene, THREE, physics, setSky }) {
  setSky({ topColor: '#3a4654', horizonColor: '#cfd6df', brightness: 0.7 });

  // floor
  const floor = new THREE.Mesh(
    new THREE.BoxGeometry(30, 0.2, 40),
    new THREE.MeshPhysicalMaterial({ color: 0x6b6f74, roughness: 0.95 }),
  );
  floor.position.y = -0.1;
  scene.add(floor);
  physics.staticCollider(floor, 'box');     // make it solid for the agent + lidar

  // sun + ambient
  scene.add(new THREE.HemisphereLight(0xffffff, 0x404040, 0.8));
  const sun = new THREE.DirectionalLight(0xffffff, 1.2);
  sun.position.set(10, 20, 10);
  sun.castShadow = true;
  scene.add(sun);

  return {
    embodiment: null,                       // dimos sends the embodiment over WS
    spawnPoint: { x: 0, y: 0.5, z: 0 },
  };
}
```

Run it:

```bash
dimsim dev --scene warehouse
# or via dimos:
dimos --simulation dimsim --dimsim-scene=warehouse run unitree-go2-basic
```

A live example with stacked crates, pallet racks, and pendant lights is at `scenes/warehouse/index.js`.

## The `api` argument

`build(api)` receives an object — destructure what you need:

| Field | Description |
|---|---|
| `scene` | `THREE.Scene` — add meshes, lights, groups here |
| `THREE` | The engine's THREE module; use this rather than re-importing |
| `physics.staticCollider(mesh, shape)` | Attach a static collider. `shape`: `"box"` / `"trimesh"` / `"sphere"` |
| `physics.dynamicCollider(mesh, {mass, shape})` | Rigid body — falls, bounces. Engine syncs its position into `mesh` each frame |
| `setSky(opts)` | `{topColor, horizonColor, bottomColor, brightness, softness, sunStrength, sunHeight}` |
| `loadGLTF(url)` | Async GLB loader. Returns `{ scene, animations, ... }` |
| `loadLevel(data)` | Feed an apt-shape blob through the engine's importLevelFromJSON — populates the assets registry so E-key pickup / multi-state objects work |
| `loadJson(url)` | Like loadLevel, but fetches a JSON file first |
| `agent`, `camera`, `renderer`, `RAPIER`, `rapierWorld` | Live references — same instances the engine uses |

You can also `import * as THREE from 'three'` if you prefer the explicit form, but the `api.THREE` is guaranteed to be the same instance the engine uses (avoiding "two THREEs" bugs).

## Adding interactive objects (limitation)

Right now, anything you add with `scene.add(mesh)` is **invisible to the E-key interaction system**. Pickable items, multi-state cabinets, openable doors, the TV-toggle pattern — all of these live in the engine's `assets[]` registry, which is only populated by `loadLevel(data)` / `loadJson(url)` with apt-shape data.

Two options today:

1. **Decompose your scene's interactive objects into apt-shape data** (see `scenes/apartment/data/objects.js` for the format) and feed them through `loadLevel`. Tedious but works.
2. **Hold for the `registerAsset(group, {pickable, states, actions})` API** — planned but not yet built. Will let `new THREE.Group(...)` participate in interactions without the apt-shape detour.

The apartment scene (`scenes/apartment/index.js`) uses option 1 — its `data/` folder has 123 assets including 28 interactive ones.

## Hot reload

dimsim's bridge watches `scenes/<active-scene>/` and re-fires the engine's HMR handler on any file save. You'll see a `[bridge] hot-reload …` line in the bridge logs, then the engine logs `[dimos] hot-reloaded <scene>`. No browser refresh needed.

Limitation: HMR re-runs `build(api)` which re-adds everything — but the engine doesn't currently scrub interactive state from `loadLevel`. If your scene uses `loadLevel`, expect duplicate assets after HMR; refresh the browser instead.

## Tips

- **Always pair `scene.add(mesh)` with `physics.staticCollider(mesh, ...)` if the agent / lidar should see it.** Visual-only meshes are fine — lidar won't hit them.
- **Use `MeshPhysicalMaterial`** for anything you care about visually — supports clearcoat, sheen, transmission, iridescence.
- **For repeating elements** (racks, pillars, crates) use loops in `build`. The whole module re-runs on HMR, so iteration is cheap.
- **Spawn point**: `spawnPoint` from `build`'s return value is where the agent gets placed at scene-load time. dimos may override it later.
- **`scenes/empty/index.js`** is the bare-minimum scene — a floor and a sky. Useful as a starting template.

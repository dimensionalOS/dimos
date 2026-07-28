# System Validation: Scene Recipes

Each item from [issue #1691](https://github.com/dimensionalOS/dimos/issues/1691) as a
runnable DimSim scene. Full authoring reference: [scenes.md](./scenes.md).

## Workflow

Create `misc/DimSim/scenes/<name>/index.js` exporting a default async `build(api)`, and
drop any `.glb` it loads into the same folder. Then run:

```bash
export OPENAI_API_KEY=sk-...    # the agentic blueprint needs it
uv run dimos --simulation dimsim --dimsim-scene=<name> run unitree-go2-agentic
```

Headless by default. Add `--no-dimsim-headless` to watch at localhost:8090 (free-orbit
camera). To reload after an edit, hard-refresh the tab (headed) or restart the command
(headless). No rebuild needed.

`build(api)` provides `scene`, `THREE`, `RAPIER`, `physics`, `loadGLTF`, `setSky`,
`setEmbodiment`, `placeOnGround`, `placeInAir`, `findOpenSpawn`, `clearDefaultLights`,
`enableShadows`.

Scenes are lit (and shadow-free) by default. Call `clearDefaultLights()` to take over
lighting, `enableShadows()` for shadows. See [scenes.md](./scenes.md).

---

## 1. Third-party map (+ collisions)

Drop `map.glb` in the scene folder:

```js
export default async function build({ scene, physics, loadGLTF, setSky, findOpenSpawn }) {
  setSky({ topColor: "#1a6be0", horizonColor: "#c8ddf5" });
  const map = (await loadGLTF("./map.glb")).scene;
  scene.add(map);
  physics.staticCollider(map, "trimesh");   // so the robot can't fall through
  return { spawnPoint: findOpenSpawn() };   // auto-pick a clear spot (after colliders exist)
}
```

Observe: the robot spawns on the map and collides with walls/floor.

`findOpenSpawn()` is the safe default for a third-party map: you don't know where the
walls are, and it searches outward from the origin for a collision-free spot instead of
guessing coords. If you *do* know a good clear spot, `placeOnGround(x, z)` (or
`placeInAir(x, z, altitude)` for drones) resolves the floor height at that `x, z` — but a
guessed origin can land inside geometry, and `placeOnGround` only *warns* if it floats.
`findOpenSpawn` must run after the colliders exist, so call it in the `return`.

---

## 2. Add a car (+ collisions)

Drop `car.glb` in the folder, then in `build()`:

```js
const car = (await loadGLTF("./car.glb")).scene;
car.position.set(3, 0, -2);
scene.add(car);
physics.staticCollider(car, "box");   // dynamicCollider(car, { shape: "box", mass: 50 }) to knock it around
```

Observe: the robot is blocked by the car, and LiDAR registers it.

---

## 3. Editing loop

Run headed (`--no-dimsim-headless`), then edit `index.js`. Move the car
(`car.position.set(2, 0, 0)`) or add a light (`scene.add(new THREE.PointLight(0xff0000, 5, 10))`),
then hard-refresh the tab. Changes appear immediately, no rebuild.

---

## 4. New embodiment: drone, car, or your own

`setEmbodiment({ motionModel, ...params })` picks the motion model the server physics
executes (`cmd_vel → /odom`):

| `motionModel` | Behaviour | Key params |
|---|---|---|
| `holonomic` (default) | ground robot: drives along heading, gravity | `maxSpeed`, `turnRate`, `gravity` |
| `flight` | drone: 6DoF, no gravity, altitude clamp | `maxSpeed`, `maxAltitude` |
| `ackermann` | car: steers, can't pivot in place | `maxSpeed`, `wheelBase`, `maxSteerAngle` |

```js
// drone — set embodimentType too so the browser visual matches the flight physics
setEmbodiment({ motionModel: "flight", embodimentType: "drone",
  radius: 0.3, halfHeight: 0.1, gravity: 0, maxSpeed: 3, maxAltitude: 20 });
return { spawnPoint: findOpenSpawn({ altitude: 5 }) };   // clear column, hover 5 m up

// car
setEmbodiment({ motionModel: "ackermann",
  radius: 0.4, halfHeight: 0.3, maxSpeed: 4, wheelBase: 1.2, maxSteerAngle: 0.6 });
return { spawnPoint: findOpenSpawn() };
```

To add a new model (legged, tank, boat), add one function to `MOTION_MODELS` in
`cli/bridge/physics.ts`.

Observe: the drone holds altitude, the car arcs through turns, and both collide with scene
geometry.

---

## Coverage vs issue #1691

| Item | Recipe |
|---|---|
| Third-party map load | §1 |
| New element + collisions | §2 |
| Editing loop | §3 |
| New embodiment | §4 |

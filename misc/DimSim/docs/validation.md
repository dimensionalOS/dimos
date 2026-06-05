# System Validation — Scene-Authoring How-Tos

This doc maps each **System Validation** item from
[issue #1691](https://github.com/dimensionalOS/dimos/issues/1691) to a concrete,
runnable recipe in the DimSim **JS scene-authoring workflow**.

Everything here is done by writing/editing a scene module — no Python
`SceneClient` runtime calls. Each recipe is a complete `index.js` you can drop in
and run.

> The full `THREE` and `RAPIER` namespaces are passed to every scene's `build()`
> by default — there's nothing to enable, so "ThreeJS API exposed" needs no
> separate example; every recipe below uses it.

---

## The workflow

To make a scene, **create a new folder** `misc/DimSim/scenes/<name>/` with an
`index.js` that exports a default async `build(api)` function — and drop any `.glb`
assets it loads (maps, cars) into that **same folder**. Scenes are served **from source**
(`misc/DimSim/scenes/`), so editing `index.js` and reloading takes effect with **no
rebuild**.

### `build(api)` surface

```js
export default async function build({
  scene,        // THREE.Scene — add anything to it
  THREE,        // full three.js namespace (primitives, lights, materials, fog, ...)
  RAPIER,       // full Rapier namespace (custom physics, if needed)
  physics,      // { staticCollider(mesh, shape), dynamicCollider(mesh, opts) }
  loadGLTF,     // (url) => Promise<gltf>  — load a .glb/.gltf
  setSky,       // ({ enabled, topColor, horizonColor, bottomColor, ... })
  setEmbodiment,// ({ embodimentType, radius, halfHeight, gravity, maxSpeed, ... })
}) {
  // ... build the world ...
  return {
    embodiment: null,                 // or set via setEmbodiment() above
    spawnPoint: { x: 2, y: 0.5, z: 3 } // Three.js coords (Y-up); maps to ROS (z, x)
  };
}
```

- `physics.staticCollider(mesh, "trimesh" | "box" | "sphere")` — fixed obstacle the
  robot collides with (and LiDAR sees). Created at boot → captured by the Rapier
  snapshot the bridge restores.
- `physics.dynamicCollider(mesh, { shape, mass, restitution })` — body that responds
  to gravity (ball, knock-around obstacle).
- `spawnPoint` is in **Three.js** coords (Y-up). It maps to ROS/odom `(x, y) = (z, x)`
  via the bridge's `(x,y,z)->(z,x,y)` swap.

### Run a scene

```bash
export OPENAI_API_KEY=sk-...          # the agentic blueprint needs it at startup
uv run dimos --simulation dimsim --dimsim-scene=<name> run unitree-go2-agentic
```

This runs **headless** by default (a capture page is auto-launched for the sensor
pipeline). To **watch in a browser**, add `--no-dimsim-headless` and open
**http://localhost:8090** — that gives a free-orbit ("ghost") camera, and the tab
itself becomes the sensor source (keep it foregrounded):

```bash
uv run dimos --simulation dimsim --dimsim-scene=<name> --no-dimsim-headless run unitree-go2-agentic
```

### Reload after editing

Scenes are dynamically imported by the browser. After editing `scenes/<name>/index.js`:

- **Headed (browser tab open):** hard-refresh the tab (`Cmd/Ctrl+Shift+R`) to re-import
  the scene module.
- **Headless / agentic run:** restart the `uv run dimos ...` command.

> Reference scenes: `scenes/apartment/` (GLB structure + colliders) and
> `scenes/warehouse/` (sky + embodiment + GLB) are complete worked examples.

---

## 1. Third-party map load (+ collisions)

**Validates:** load an external/Sketchfab map and deploy the robot inside it without
falling through.

1. Download the `.glb` (e.g. a Sketchfab map) and place it in the scene folder:
   `scenes/game-map/map.glb`.
2. `scenes/game-map/index.js`:

```js
export default async function build({ scene, THREE, physics, loadGLTF, setSky }) {
  setSky({ enabled: true, topColor: "#1a6be0", horizonColor: "#c8ddf5", bottomColor: "#4a7a4a" });
  scene.add(new THREE.AmbientLight(0xffffff, 0.7));
  const sun = new THREE.DirectionalLight(0xffffff, 1.5);
  sun.position.set(10, 20, 10); sun.castShadow = true; scene.add(sun);

  const gltf = await loadGLTF("./map.glb");      // local file in the scene folder
  const map = gltf.scene;
  map.traverse((c) => { if (c.isMesh) { c.castShadow = c.receiveShadow = true; } });
  scene.add(map);

  // trimesh collider over the whole map so the robot doesn't fall through
  physics.staticCollider(map, "trimesh");

  return { embodiment: null, spawnPoint: { x: 0, y: 1.0, z: 0 } };
}
```

**Run:** `--dimsim-scene=game-map`
**Observe:** robot spawns on the map geometry, collides with walls/floor.

> **Note:** download the Sketchfab `.glb` and drop it in the scene folder, then load it
> with `loadGLTF("./map.glb")` (as above). For a drone city map, use the Birmingham
> city GLB the same way with the drone embodiment from §4.

---

## 2. Add a car (new element) with collisions

**Validates:** add a model as a collidable obstacle.

Drop `car.glb` into the scene folder, then in `build()`:

```js
const car = (await loadGLTF("./car.glb")).scene;
car.position.set(3, 0, -2);
car.traverse((c) => { if (c.isMesh) c.castShadow = true; });
scene.add(car);
physics.staticCollider(car, "box");     // box collider = robot can't drive through it
```

**Observe:** the robot is blocked by the car; LiDAR registers it (a dropped-wall-style
obstacle). Use `physics.dynamicCollider(car, { shape: "box", mass: 50 })` instead if you
want it to be knocked around.

---

## 3. Editing flow (move object, reload, add a light, reload)

**Validates:** a fast threejs-like edit loop.

1. Run a scene **headed** so you get a browser view to edit against — add
   `--no-dimsim-headless` (the default is headless):
   `uv run dimos --simulation dimsim --dimsim-scene=game-map --no-dimsim-headless run unitree-go2-agentic`
   (a scene you created in §1/§2), then open `localhost:8090`.
2. Edit `scenes/game-map/index.js` — e.g. move the car: `car.position.set(2, 0, 0);`
3. **Hard-refresh the tab** (`Cmd/Ctrl+Shift+R`) → the moved object appears.
4. Add a light: `scene.add(new THREE.PointLight(0xff0000, 5, 10));` near an object →
   hard-refresh → the red glow appears.

No build step — scenes are served from source and re-imported on refresh.

---

## 4. New embodiment — drone, car, or your own

**Validates:** define a robot embodiment other than the default ground robot, and
deploy it in a world.

An embodiment is **config in the scene** — `setEmbodiment({ motionModel, ...params })`
— which the server-side physics executes (`cmd_vel → motion → /odom`). Three motion
models ship today:

| `motionModel` | Behaviour | Key params |
|---|---|---|
| `holonomic` (default) | ground robot: drives along heading, gravity | `maxSpeed`, `turnRate`, `gravity`, `maxStepHeight`, `maxSlopeAngle` |
| `flight` | drone: 6DoF, no gravity, altitude clamp | `maxSpeed`, `maxAltitude` |
| `ackermann` | car: steers (can't pivot in place), turn rate scales with speed | `maxSpeed`, `wheelBase`, `maxSteerAngle` |

**Drone:**
```js
setEmbodiment({
  embodimentType: "drone",   // the browser keys off this for the flight visual/camera
  motionModel: "flight",     // the server motion model (drone -> flight)
  radius: 0.3, halfHeight: 0.1, gravity: 0,
  maxSpeed: 3.0, maxAltitude: 20.0,
});
return { embodiment: null, spawnPoint: { x: 0, y: 3, z: 0 } }; // spawn in the air
```
> Note: the browser-side embodiment visual keys off `embodimentType` (`"ground"` /
> `"drone"`), while the server motion is driven by `motionModel`. Set both for a
> drone so the flight physics *and* the browser visual agree.

**Car (Ackermann steering):**
```js
setEmbodiment({
  motionModel: "ackermann",
  radius: 0.4, halfHeight: 0.3,
  maxSpeed: 4.0, wheelBase: 1.2, maxSteerAngle: 0.6, gravity: -9.81,
});
return { embodiment: null, spawnPoint: { x: 0, y: 0.5, z: 0 } };
```

> `avatarUrl` is optional — the default is the robot dog
> (`/embodiment/dimsim_unitree_stub.glb`). Set it only for a custom drone/car visual
> (drop a `.glb` into `public/embodiment/`).
> **If the agent shows as a bare capsule**, your `dist/` avatar GLB is a stale Git-LFS
> pointer (a ~133-byte file) — run `git lfs pull`, then rebuild dist
> (`cd misc/DimSim && npm run build`) so the real model is served.
> Legacy `embodimentType: "ground" | "drone"` still works (maps to holonomic/flight).

### Custom embodiments

- **Compose an existing model** (no code change): any robot is just `motionModel` +
  params in the scene — size, speed, gravity, altitude, steering. The server executes it.

- **Add a brand-new motion model** (legged gait, tank, boat, plane): add **one
  function** to the `MOTION_MODELS` registry in **`cli/bridge/physics.ts`**. That's the
  one place motion models live — they run server-side (where `cmd_vel → /odom` happens),
  so a new embodiment is a small bridge edit, not scene JS. Signature:

  ```
  (cmd, yaw, cfg, dt) => { dx, dy, dz, dyaw, clampMaxY? }
  ```
  Map the (speed-scaled) `cmd_vel` + current `yaw` to a world displacement + yaw delta;
  the shared step path handles collision resolution + odom. Example — a tank (drives and
  turns, but no strafing):

  ```js
  // cli/bridge/physics.ts  →  MOTION_MODELS = { holonomic, flight, ackermann, ... }
  tank(cmd, yaw, cfg, dt) {
    const dyaw = cmd.angZ * dt, y = yaw + dyaw;
    return { dx: cmd.linX * Math.sin(y) * dt, dz: cmd.linX * Math.cos(y) * dt,
             dy: cfg.gravity * dt * dt * 0.5, dyaw };
  },
  ```
  Once added, any scene selects it with `setEmbodiment({ motionModel: "tank", ... })`.

**Observe:** the drone holds altitude and moves in 6DoF; the car arcs through turns
instead of spinning in place; both collide with scene geometry.

---

## Coverage vs issue #1691

| Validation item | Recipe | Status |
|---|---|---|
| Third-party map load | §1 | ✅ via **local** GLB (download the Sketchfab model into the scene folder) |
| New elements (car + collisions) | §2 | ✅ works today |
| Editing flow | §3 | ✅ works today |
| New embodiment (drone / car / custom) | §4 | ✅ `motionModel`: holonomic + flight + ackermann; new models = 1 fn in `physics.ts` |

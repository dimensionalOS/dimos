# Scene Editing SDK
** Status: In Progress — Eval creation workflow and additional rubric types are under active development.

Python SDK for runtime 3D scene manipulation in DimSim. SceneClient connects to a running DimSim bridge server via WebSocket and provides high-level methods for loading scenes, managing objects, configuring sensors, and swapping robot embodiments.

## Architecture

```
Python (SceneClient)                    Deno Bridge Server                  Browser (Three.js)
─────────────────────                   ──────────────────                  ──────────────────
                                              :8090
scene.load_map(url)  ───WS text───►  relay to browser  ───WS text───►  SceneEditor._execCode()
scene.set_embodiment()  ──────────►  store + reconfigure  ──────────►  DimosBridge._handleEmbodimentConfig()
                                     ServerPhysics                      avatar model swap
                                     ServerLidar

scene.set_agent_position()  ──────►  ServerPhysics.setPosition()  ────►  pose broadcast (visual sync)

Sensor data flows back:
                                     ServerPhysics (50Hz odom)  ──LCM──►  dimos Python
                           ◄──LCM──  ServerLidar (10Hz raycasts)
Browser captures:
  color_image (5Hz)  ───WS binary──►  LCM relay  ───LCM multicast──►  dimos Python
  depth_image (5Hz)  ───WS binary──►  LCM relay  ───LCM multicast──►  dimos Python
```

SceneClient uses a **separate WebSocket** (`?ch=control`) from the LCM sensor data path. It sends JSON text commands; the bridge relays them to the browser where `SceneEditor` executes them in a sandboxed `AsyncFunction` context.

## Quick Start

```bash
# Terminal 1: start sim with empty scene
DIMSIM_SCENE=empty dimos --simulation run sim-nav

# Terminal 2: manipulate scene
python -c "
from dimos.robot.sim.scene_client import SceneClient

with SceneClient() as scene:
    scene.load_map('/local-assets/my-room.glb')
    scene.set_embodiment('drone')
    scene.set_agent_position(0, 2, 0)
"
```

## Connection

```python
from dimos.robot.sim.scene_client import SceneClient

# Default: localhost:8090
with SceneClient() as scene:
    ...

# Custom host/port
scene = SceneClient(host="192.168.1.10", port=9090)
scene.connect()
# ... use scene ...
scene.close()
```

## API Reference

### Scene Management

| Method | Description |
|---|---|
| `load_map(url, position, scale, collider, name, auto_scale)` | Load GLB scene with trimesh colliders. Auto-scales cm to m by default. |
| `clear_scene()` | Remove all user-added objects (preserves agent, camera, lights) |
| `get_scene_info()` | List scene objects. Slow on large scenes (1000+ objects) — use `exec()` instead. |

### Objects

| Method | Description |
|---|---|
| `add_object(geometry, size, color, position, name, dynamic, mass, restitution, collider)` | Add primitive: `"box"`, `"sphere"`, or `"cylinder"` |
| `remove_object(name)` | Remove object by name, dispose geometry/materials |
| `add_collider(name, shape)` | Add physics collider to existing object (`"trimesh"`, `"box"`, `"sphere"`) |
| `remove_collider(name)` | Remove physics collider |

### NPCs

| Method | Description |
|---|---|
| `add_npc(url, name, position, rotation, scale, animation, collider)` | Load animated GLTF character. Animation selected by name substring or index. |
| `remove_npc(name)` | Remove NPC, stop animations, remove collider |

### Agent

| Method | Description |
|---|---|
| `get_agent_position()` | Returns `{x, y, z}` in world frame |
| `set_agent_position(x, y, z)` | Teleport agent via server physics |
| `set_embodiment(preset, **overrides)` | Change robot type, physics params, and avatar model |

### Custom Code

```python
result = scene.exec("""
    const geo = new THREE.SphereGeometry(0.5);
    const mat = new THREE.MeshStandardMaterial({ color: 0xff0000 });
    const mesh = new THREE.Mesh(geo, mat);
    mesh.name = "my-sphere";
    mesh.position.set(0, 1, 0);
    scene.add(mesh);
    return addCollider(mesh, { shape: "sphere", dynamic: true, mass: 1.0, restitution: 0.5 });
""")
```

**Sandbox globals:** `scene`, `THREE`, `RAPIER`, `rapierWorld`, `renderer`, `camera`, `agent`, `loadGLTF(url)`, `addCollider(obj, opts)`, `removeCollider(obj)`, `addNPC(opts)`, `removeNPC(name)`, `autoScale(obj, maxDim?)`.

Top-level `await` is supported. Return value is serialized to JSON and sent back to Python.

## Embodiment Presets

8 built-in presets covering common robot types:

| Preset | Type | Key Parameters |
|---|---|---|
| `unitree-go2` | Quadruped (default) | radius=0.12, halfHeight=0.25, maxSpeed=3.0, gravity=-9.81 |
| `quadruped` | Generic quadruped | Same as unitree-go2 |
| `differential-drive` | Two-wheel robot | radius=0.15, maxSpeed=2.0, maxStepHeight=0.05 |
| `ackermann` | Car-like steering | radius=0.3, maxSpeed=5.0, turnRate=1.2 |
| `holonomic` | Omnidirectional | radius=0.2, maxSpeed=2.5, turnRate=4.0 |
| `humanoid` | Biped | radius=0.2, halfHeight=0.8, lidarMount=1.6m |
| `small-robot` | Compact ground robot | radius=0.08, maxSpeed=1.0 |
| `drone` | 6DoF flight | gravity=0, maxAltitude=20m, maxSpeed=5.0 |

Override any parameter:

```python
scene.set_embodiment("drone",
    avatar_url="http://localhost:8090/proxy?url=https://example.com/drone.glb",
    max_speed=8.0,
    max_altitude=50.0,
)
```

When you change embodiment:
- **Server physics** rebuilds the agent's colliders, changes gravity/speed params
- **Server lidar** updates the mount offset
- **Browser** swaps the avatar GLB model

## Auto-Scaling

Models from Sketchfab and other sources are often in centimeters. `load_map()` auto-detects this:

- **> 100m** bounding box → scale by 0.01 (cm to m)
- **50-100m** → scale proportionally so max dimension = 50m
- **< 50m** → no change

```python
scene.load_map("/local-assets/room.glb")                # auto-scale on
scene.load_map("/local-assets/room.glb", auto_scale=20)  # custom max dim
scene.load_map("/local-assets/room.glb", auto_scale=False)  # off
```

Also available in exec context: `autoScale(obj, targetMaxDim)`.

## Sensor Configuration

Three levels of configuration (later overrides earlier):

### Blueprint Config (Python)

```python
DimSimBridgeConfig(
    image_rate_ms=100,       # 10 Hz images (default: 200 = 5 Hz)
    enable_depth=False,      # skip depth publishing
    camera_fov=46,           # vertical FOV (default: 46 = Go2 D435i)
)
```

### Environment Variables

```bash
DIMSIM_IMAGE_RATE=100 DIMSIM_DISABLE_DEPTH=1 DIMSIM_CAMERA_FOV=46 dimos --simulation run sim-nav
```

Sensor config is launch-time only — not changeable at runtime via SceneClient.

### Default Rates

| Sensor | Rate | Channel |
|--------|------|---------|
| Color image | 200ms (5 Hz) | `/color_image` |
| Depth image | 500ms (2 Hz) | `/depth_image` |
| LiDAR | 100ms (10 Hz) | `/lidar` |
| Odom | 20ms (50 Hz) | `/odom` |

Depth is the only toggleable channel. Color and lidar are essential for navigation and perception.

## Dynamic Rigid Bodies

Objects that respond to gravity and collisions:

```python
scene.add_object(
    "sphere", size=(0.3,), color=0xFF0000,
    position=(0, 3, 0),
    dynamic=True, mass=0.5, restitution=0.8,
)
```

Or via exec for full control:

```python
scene.exec("""
    const mesh = new THREE.Mesh(
        new THREE.SphereGeometry(0.3),
        new THREE.MeshStandardMaterial({ color: 0xff0000 })
    );
    mesh.position.set(0, 3, 0);
    scene.add(mesh);
    addCollider(mesh, { shape: "sphere", dynamic: true, mass: 0.5, restitution: 0.8 });
""")
```

Dynamic bodies have physics on both browser and server side.

## Loading Assets

### Remote URLs

Use the CORS proxy for cross-origin models:

```python
url = "/proxy?url=https://example.com/model.glb"
scene.load_map(url)
```

### Empty scene

```bash
DIMSIM_SCENE=empty dimos --simulation run sim-nav
```

Then build the scene programmatically. Use `clear_scene()` to reset.

## Environment Variables

| Variable | Description | Default |
|---|---|---|
| `DIMSIM_SCENE` | Scene to load (`apt`, `empty`) | `apt` |
| `DIMSIM_LOCAL` | Use local DimSim repo instead of binary | unset |
| `DIMSIM_HEADLESS` | Launch headless Chrome | unset |
| `DIMSIM_RENDER` | Headless rendering: `gpu` or `cpu` | `cpu` |
| `DIMSIM_IMAGE_RATE` | Image publish interval in ms | `200` (5 Hz) |
| `DIMSIM_DISABLE_DEPTH` | Disable depth image publishing | unset |
| `DIMSIM_CAMERA_FOV` | Vertical camera FOV in degrees | `46` (Go2) |

## Known Quirks

- **GLTF 2.0+ required** — Older models fail silently. Check model version before loading.
- **NPC Y positioning** — No auto-ground-snapping. Apt floor is ~Y=0.1. Use agent position as reference.
- **CORS for remote URLs** — Browser blocks cross-origin GLBs. Use `/proxy?url=<encoded-url>`.
- **`get_scene_info()` timeout** — Slow on 1000+ object scenes. Use targeted `exec()` queries.
- **Server restart for DimSim changes** — `DimSim/src/` needs `npm run build`. Bridge server needs restart. Python changes are immediate.
- **`DIMSIM_LOCAL=1` for development** — Required when working with the local DimSim repo.

## Examples

See `examples/scene_editing/`:

| Script | Description |
|---|---|
| Example | Status | Description |
|---|---|---|
| `load_object.py` | Ready | Load a GLB object ([Avocado sample](https://github.com/KhronosGroup/glTF-Sample-Models/blob/main/2.0/Avocado/glTF/Avocado.gltf) by default) |
| `load_custom_object.py` | Ready | Create objects from code (box, sphere with physics) |
| `load_scene.py` | Template | Load a full GLB scene — update `GLB_PATH` to your file |
| `remove_object.py` | Ready | Remove a named object |
| `load_robot.py` | Ready | Swap robot embodiment (drone, Go2) |

Run any example while a sim is running:

```bash
# Terminal 1
DIMSIM_LOCAL=1 dimos --simulation run sim-nav

# Terminal 2
python examples/scene_editing/load_object.py
python examples/scene_editing/load_robot.py
```

Both `load_object.py` and `load_scene.py` take a local file path as argument. Download a `.glb` model and pass the path:

- [Avocado (GitHub)](https://github.com/KhronosGroup/glTF-Sample-Models/blob/main/2.0/Avocado/glTF-Binary/Avocado.glb) — small test object
- [TDM Game Map (Sketchfab)](https://sketchfab.com/3d-models/lowpoly-fps-tdm-game-map-by-resoforge-d41a19f699ea421a9aa32b407cb7537b) — full scene

```bash
python examples/scene_editing/load_object.py ~/Downloads/Avocado.glb
python examples/scene_editing/load_scene.py ~/Downloads/scene.glb
```

## Tests

```bash
# SceneClient unit tests (against live headless server)
pytest dimos/e2e_tests/test_scene_client.py -v -s -m slow

# Scene editing integration tests (sensor rates, FOV, embodiment, auto-scale)
pytest dimos/e2e_tests/test_scene_editing.py -v -s -m slow
```

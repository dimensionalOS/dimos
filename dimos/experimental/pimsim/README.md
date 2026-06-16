# PimSim — Overview & API

**The canonical PimSim doc: what it is, how to use it, the API.** The API *in
code* is [`spec/protocols.py`](spec/protocols.py) (backend contracts) +
[`api.py`](api.py) (the `PimSim` usage facade). The separate G1-reachability
deliverable spec is [`SPEC.md`](SPEC.md).

---

## What PimSim is, in one sentence

PimSim is the sim half of a **sim‑equals‑hardware** contract: you run your normal
Dimos blueprint with a `--simulation` flag, point it at a portable **scene
package**, and drive/observe it through the **same LCM topics and client API**
you'd use on the real robot — Dimos never learns it's in sim, or which simulator
it is.

The simulator is a *pluggable physics backend*. Two exist today and implement the
same interface (`spec/protocols.py::PhysicsAuthority`):

| Backend | Module | Profile |
|---|---|---|
| **MuJoCo** | `simulation/engines/mujoco_sim_module.py` | headless, deterministic — the eval/CI authority; full RGBD/lidar/IMU |
| **Babylon + Havok** | `experimental/pimsim/babylon/module.py` | browser, interactive, high visual fidelity (splat); the human-in-the-loop authority |

Adding a third simulator = implement `PhysicsAuthority` + `SceneControl`. Nothing
downstream changes.

---

## The API — the four surfaces you touch

### 1. Run it — same blueprint as hardware, a flag picks the backend

```bash
# Babylon + Havok authority — browser viewer at http://localhost:8091
dimos --simulation pimsim --scene dimos-office run g1-groot-wbc   # 'pimsim' aliases 'babylon'

# MuJoCo authority — headless, deterministic (evals/CI)
dimos --simulation mujoco --scene dimos-office run g1-groot-wbc

# the parametrized agentic-eval tests on the pimsim backend
pytest -m pimsim dimos/e2e_tests/

# global flags (--simulation, --scene) precede the `run` subcommand.
# --scene <name|path|none>;  none = bare robot, no scene.
```

The blueprint (`g1-groot-wbc`, `unitree-go2-agentic`, …) is **identical** to the
real-hardware run; only the flag changes.

### 2. Scenes — cook any mesh/map into a portable package

```bash
python -m dimos.experimental.pimsim.scene.cook <mesh.glb> [--cook-spec <scene>.cook.json]
# → data/scene_packages/<name>/  { browser/visual.glb, browser/collision.glb,
#   browser/objects.json, mujoco/<key>/wrapper.xml + hull OBJs, scene.meta.json }
```

One source asset → one package → consumed **identically** by Babylon and MuJoCo.
Third-party sketchfab maps work (the office and the low-poly TDM map were cooked
this way). Reference a package by catalog name or path via `--scene`.

### 3. The shapes — what you're actually passing around

All defined in code; import them, don't reinvent:

```python
from dimos.experimental.pimsim.entity import EntityDescriptor, EntityStateBatch
from dimos.experimental.pimsim.spec.models import SceneObject       # proposed unified noun
from dimos.experimental.pimsim.spec.enums import AuthorityMode      # OWNS | MIRROR
from dimos.simulation.scene_assets.spec import ScenePackage         # the cooked package

# what an entity IS (identity + how to instantiate; no pose):
EntityDescriptor(entity_id="cup", kind="dynamic", mesh_ref="cup.glb",
                 shape_hint="mesh", extents=(), mass=0.2)

# where everything IS now (one tick, streamed as JSON-over-LCM):
EntityStateBatch(entries=[(descriptor, pose), ...])
```

| Shape | Is | Lives |
|---|---|---|
| `EntityDescriptor` | what an entity *is* (id, kind, mesh/shape, mass) | `entity.py` |
| `EntityStateBatch` | where everything *is now* (timestamped `(descriptor, pose)` snapshot) | `entity.py` |
| `SceneObject` | the proposed merge of `Obstacle`+`EntityDescriptor` (identity+geometry, no pose) | `spec/models.py` |
| `ScenePackage` | the cooked, portable scene (visual+collision GLB, hulls, objects, MJCF) | `simulation/scene_assets/spec.py` |

### 4. Drive & observe — backend-agnostic, identical to hardware

**From code/tests/evals — the `PimSim` facade** (`api.py`):

```python
from dimos.experimental.pimsim.api import PimSim
sim = PimSim(backend="pimsim"); sim.start()
sim.set_agent_position(1, 2)        # place the robot
sim.cmd_vel(vx=0.5)                 # drive it (open-loop)        ┐ work today
sim.goto(10.9, 0.6)                 # nav goal (closed-loop)      │
sim.add_wall(7, -2.5, 7, 3.5)       # collidable wall             │
sim.add_box((0, 1, 0.5))            # collidable box              │
sim.add_object("cup", shape="cylinder", extents=(0.03, 0.1), at=(0.4, 0.8, 0.7))  # spawn entity ┘
sim.add_npc("walker", path=[...])                   # NotImplemented — needs a moving-entity driver
sim.add_robot("go2"); sim.set_embodiment("drone")   # NotImplemented — needs an embodiment WS command
```

`add_robot` is the one backend-specific verb: runtime on the browser/Havok
backend, but on MuJoCo the robot is welded into the model + driven by the
coordinator, so it raises "relaunch with the blueprint." The unimplemented verbs
are defined with that per-backend guidance — reaching DimSim parity is the
scoped build-out below. `PimSim` extends `PimSimClient` (the e2e `SceneControl`
surface), so `DimSimClient` and `PimSim` share the contract that lets one e2e
test run on either backend (surface 5).

**Live dynamic entities** (the authority's `@rpc`s): `spawn_entity(descriptor, pose)`,
`set_entity_pose`, `apply_entity_velocity`, `despawn_entity` — moving cars, people, balls.

**Topics** (your modules subscribe these whether sim or real): `/odom`,
`/cmd_vel`, `/entity_state_batch`, `/color_image`+`/depth_image`+`/camera_info`,
`/pointcloud` (lidar), `/point_goal`.

### 5. Test / eval — write once, run on any backend (incl. hardware)

```python
@pytest.mark.parametrize("sim_client",
    [pytest.param("dimsim", marks=pytest.mark.dimsim),
     pytest.param("pimsim", marks=pytest.mark.pimsim)], indirect=True)
def test_walk_forward(sim_client, start_blueprint, human_input, lcm_spy):
    start_blueprint("run", "unitree-go2-agentic", simulator=...)
    sim_client.set_agent_position(1, 2)
    human_input("move forward 3 meter")
    lcm_spy.wait_until_odom_position(4, 2, threshold=0.4)
```

The body is identical across backends. `dimos/e2e_tests/test_dimsim_*.py` are the
agentic-eval demos (walk-forward, nav replanning, spatial memory).

---

## Architecture

```
  BACKEND (PhysicsAuthority):   MuJoCo (headless)   ||   Babylon+Havok (browser)
                                       │  owns scene + embodiment, steps physics
            ┌──────────────────────────┼──────────────────────────┐
            │   three authority-agnostic contracts                 │
            │   • ScenePackage   (cooked geometry, portable)       │
            │   • EntityStateBatch  (entity stream, JSON-over-LCM) │
            │   • LCM bus  (+ LCM-over-WebSocket bridge for browser)│
            └──────────────────────────┬──────────────────────────┘
   CONSUMERS (EntityConsumer, backend-blind):
     planning world · rust scene-lidar · splat camera · nav stack · perception · agent
   CONTROL (SceneControl, backend-blind):  PimSimClient / DimSimClient / (real robot)
```

---

## Adding a new simulator

1. Make it a Dimos `Module` that satisfies **`PhysicsAuthority`**: publish
   `entity_state_batch` + `odom`, accept `cmd_vel`, expose `authority_mode` +
   `capabilities`, implement `spawn_entity`. Ingest a `ScenePackage` at start.
2. Provide a **`SceneControl`** client (`start/stop/set_agent_position/add_wall/
   publish_goal`) so tests and scripts drive it unchanged.
3. Register it in `dimos/e2e_tests/conftest.py::sim_client` and add a
   `pytest.param("<name>")` to the parametrized tests.

That's it — consumers, blueprints, and the agent are untouched.

---

## Status vs the wishlist (#1120, #1691)

| Requirement | Status |
|---|---|
| Sim interfaces exactly like hardware (Dimos-agnostic) | ✅ `PhysicsAuthority` + `SceneControl`; e2e tests parametrized over backends |
| Easy 3rd-party 3D/map import | ✅ `scene_assets.cook` (office + TDM sketchfab map cooked) |
| Good visual fidelity (VLMs) | ✅ Babylon + splat cameras |
| Low-fidelity physics / collisions | ✅ Havok (browser) / MuJoCo; rust lidar vs collision GLB |
| Basic nav eval on PimSim | ✅ `test_walk_forward` parametrized over `pimsim` |
| Nav replanning / spatial-memory evals | ⚠️ wired for `dimsim` only — parametrize over `pimsim` to claim |
| Faster-than-realtime / parallel CI | ⚠️ MuJoCo headless is fast; Babylon is wall-clock; parallelism uncharacterized |
| Live threejs scene-editing, drone embodiment (#1691) | ❌ **DimSim's lane** (threejs-native), not PimSim |

---

## Known gaps

- Still under `experimental/`; `PhysicsAuthority` names the shared backend
  contract but `MujocoSimModule`/`BabylonSceneViewerModule` don't yet inherit a
  common `SimulationModule` base.
- Viewer WS command surface covers `respawn`/`wall`/`box`/`clear`/`entity_spawn`
  (so `add_object` works); `add_npc` (a moving-entity driver) and runtime
  `add_robot`/`set_embodiment` remain scoped build-out (see `api.py`).
- MuJoCo runtime `spawn_entity` is a gap (it seeds from `scene_entities` config).
- Scene-package articulation (doors/drawers) has no schema yet.
- Headless Babylon boots a real Chromium tab (`pip install 'dimos[pimsim]' &&
  playwright install chromium`); a `BABYLON.NullEngine` path would drop the
  browser dep.

## Where things live — the structure *is* the architecture

```
experimental/pimsim/
  README.md  SPEC.md                 ← this doc + the reachability spec
  api.py                             PimSim — the usage facade (add_robot/move/add_object…)
  client.py                          PimSimClient — SceneControl transport
  entity.py                          EntityDescriptor / EntityStateBatch — the wire shapes
  spec/                              the contracts in code
    protocols.py                       PhysicsAuthority · EntityConsumer · SceneControl · SceneObjectWorld
    models.py · enums.py               SceneObject · AuthorityMode
  babylon/                           ← one authority: Babylon + Havok (swappable)
    module.py                          BabylonSceneViewerModule (Havok / external mirror)
    browser.py config.py geometry.py robot_meshes.py headless.py   viewer support
    static/                            the browser frontend (app.js, ui.js, index.html)
  sensors/                           ← authority-blind consumers (any backend feeds them)
    mesh_camera.py  splat_camera.py  scene_lidar.py  rust/scene_lidar/
  scene/                             ← the cooking pipeline (build-time tool)
    cook.py plan.py visual_*.py browser_collision.py entity_collision.py sidecar.py inspect.py
  tests/test_client.py

core (shared — NOT in pimsim, by design):
  simulation/engines/mujoco_sim_module.py       MujocoSimModule — the other authority (headless)
  simulation/scene_assets/{spec,mesh_scene}.py  ScenePackage format/loader (what the cooker emits)
  e2e_tests/test_dimsim_*.py                     agentic-eval tests, parametrized over backends
```

The layout states the design: `babylon/` is one authority (MuJoCo is the other, in core),
`sensors/` are backend-blind, `scene/` is the build-time cooker, and `api`+`spec` are the interface.

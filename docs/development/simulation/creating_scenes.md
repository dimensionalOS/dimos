# Creating Scenes from Scratch

Build a scene programmatically with Python, save it as JSON, and reload it on the next bridge boot. Hot-reload on JSON edits is supported out of the box.

## Where scenes live


| Location                        | What lives there                                                                                               |
| ------------------------------- | -------------------------------------------------------------------------------------------------------------- |
| `~/.dimsim/dist/sims/*.json`    | Built-in scenes shipped with the DimSim binary (`empty`, `apt`, ...). Replaced on dimsim upgrade — don't edit. |
| `dimos/robot/sim/scenes/*.json` | User-authored scenes. Checked into the dimos repo, survives upgrades. `SceneClient.save()` writes here.        |


When you launch with `--scene foo`, dimos resolves `foo` against `dimos/robot/sim/scenes/` first, then falls back to the bridge's built-ins.

## The empty scene

`empty.json` ships with no embodiment and no primitives — just lights and a sky. Boot with:

```bash
DIMSIM_SCENE=empty dimos --simulation run sim-basic
```

You'll see a blank sky and no robot. The agent's physics body is created but kept invisible until you set an embodiment.

## Authoring with SceneClient

`SceneClient` is a Python WebSocket client that issues commands to the running bridge. Each authoring call mutates the live browser scene **and** records into a structured journal. `save(name)` collapses the journal with the bridge's currently-loaded base scene to produce a self-contained scene JSON.

### Walkthrough — `examples/scene_editing/author_warehouse.py`

```python
from dimos.robot.sim.scene_client import SceneClient

with SceneClient() as scene:
    scene.reset()                          # idempotent: drop any prior state
    scene.set_embodiment("quadruped")      # empty has no embodiment — pick one
    scene.set_spawn_point(0, 0.5, 0)       # where the agent spawns next boot

    scene.add_object("box", size=(20, 0.1, 20),
                     color=0x808080, position=(0, -0.05, 0), name="floor")

    for i in range(4):
        scene.add_object("box", size=(1, 1, 1), color=0x8B4513,
                         position=(3 + i * 1.5, 0.5, 2), name=f"crate-{i}")

    scene.add_object("sphere", size=(0.3,), color=0xff4444,
                     position=(-2, 2, -2), name="ball",
                     dynamic=True, mass=0.5, restitution=0.8)

    scene.save("warehouse")                # → dimos/robot/sim/scenes/warehouse.json
```

Run it against an already-running sim:

```bash
# terminal 1
DIMSIM_SCENE=empty dimos --simulation run sim-basic

# terminal 2
python examples/scene_editing/author_warehouse.py
```

Refresh the browser at `http://localhost:8090` and you'll see the saved warehouse — the bridge's in-memory cache was sticky-updated by `save()`.

### Booting straight into a saved scene

```bash
DIMSIM_SCENE=warehouse dimos --simulation run sim-basic
```

dimos resolves `warehouse` to `dimos/robot/sim/scenes/warehouse.json`, points the bridge at it via `DIMSIM_SCENE_FILE`, and the browser loads it on first connect.

## Hot reload

The bridge watches `dimos/robot/sim/scenes/` (file watcher set via `DIMOS_SCENES_DIR`). When the **active** scene's JSON file changes on disk, the bridge re-reads it and pushes the new content to:

- the browser, which calls `importLevelFromJSON` to rebuild visual primitives + browser-side colliders
- `ServerPhysics`, which clears its tracked user colliders and adds fresh ones from the new content

So edits to `warehouse.json` (positions, dimensions, colors, embodiment, sky, lights) propagate to both the visual scene and `/odom` physics within a few hundred ms. No restart needed.

```python
# alternatively, force a reload from Python
scene.reload("warehouse")
```

## Hand-writing a JSON

You can also skip Python entirely and hand-write a scene file. The schema (with `embodiment`, `assets`, `primitives`, `lights`, `sceneSettings`, `dimosSpawnPoint`, `tags`) is documented by `empty.json` and `apt.json` (in `~/.dimsim/dist/sims/`). Drop your file into `dimos/robot/sim/scenes/` and boot with `--scene <name>`.

This works for one-off scenes, but iterating on positions / dimensions by typing numbers is painful — Python authoring is faster for non-trivial layouts.

## Authoring vs editing — when does which run?


| Action                        | Browser meshes            | Browser colliders     | Server colliders                | JSON file       |
| ----------------------------- | ------------------------- | --------------------- | ------------------------------- | --------------- |
| `add_object()` (live)         | added directly            | added via SceneEditor | synced via `physicsColliderAdd` | unchanged       |
| `save(name)`                  | unchanged                 | unchanged             | unchanged                       | written to disk |
| File edit + save (hot reload) | wiped + rebuilt from JSON | wiped + rebuilt       | cleared + rebuilt               | source of truth |
| `reload(name)`                | same as hot reload        | same                  | same                            | re-read         |


`save()` is non-destructive; `reload()` and hot reload are destructive (they replace the live state with the JSON's view).

## Future / TODO — Gizmo integration

The current authoring loop is "write Python, run, refresh browser." A more direct workflow is to spawn scenes and assets visually from [Gizmo](https://gizmo.antimlabs.com) . We expect to add a Gizmo → DimSim export path so you can author in Gizmo and consume in `dimos/robot/sim/scenes/` without round-tripping through Python.
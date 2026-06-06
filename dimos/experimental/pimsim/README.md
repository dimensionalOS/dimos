# pimsim

A set of dimos modules that share scene packages, an entity wire format,
and the LCM bus. Listed below with file paths.

## Scene authoring and scene packages

`dimos/simulation/scene_assets/`. Offline cooker that turns a source mesh
plus a `<scene>.cook.json` sidecar into a portable package: visual GLB,
decimated collision GLB, per-prim semantic sidecar (`objects.json`),
per-entity GLBs, and a MuJoCo wrapper with the robot MJCF and assets
bundled in. Same package is consumed by pimsim and by MuJoCo. Full doc:
[`scene_assets/README.md`](../../simulation/scene_assets/README.md).

## Entity types

`dimos/experimental/pimsim/entity.py`. `EntityDescriptor` (mesh ref,
kind, mass), `EntityState` (timestamped pose + twist), `EntityStateBatch`
(the LCM message). Same shape regardless of which simulator owns
physics; downstream consumers (rust lidar, viewers) don't know which
one.

## Babylon scene viewer + Havok physics

`dimos/experimental/pimsim/module.py` — `BabylonSceneViewerModule`.
Boots Babylon.js + Havok WASM in a browser tab, serves the page from
uvicorn, exposes `/lcm-ws` over `@dimos/msgs`. Right-handed z-up to
match ROS. Has an `entity_authority` config:

- `"browser"` — Havok owns `/odom` and `/entity_state_batch`.
- `"external"` — mirrors states from another publisher (MuJoCo) as
  kinematic bodies.

Physics steps inside Babylon's render loop;
`scene.onAfterPhysicsObservable` drives the entity broadcast (see
`static/app.js`).

## MuJoCo (reworked)

- `dimos/simulation/engines/mujoco_sim_module.py` — gained
  `scene_entities` config and `entity_state_batch: Out[EntityStateBatch]`.
- `dimos/simulation/mujoco/entity_scene.py` — `compose_entity_model(pkg)`
  extends the cooked wrapper MJCF with one body per scene-package
  entity. Audits spawn-time contact and welds anything that starts
  >2 cm inside the static scene. Compiled binary cached next to the
  wrapper.
- Robot MJCFs are now scene-free. The `manip_table` / `manip_cube`
  rig that used to be hardcoded in `g1_gear_wbc.xml` is removed; both
  spawn from the office sidecar as synthetic entities.

## Splat cameras

`dimos/visualization/viser/splat_camera.py`. `SplatCameraModule`
renders a Gaussian splat scene from the robot camera pose and publishes
`color_image: Out[Image]`. `WorkspaceSplatCameraModule` is a sibling
class using the down-pitched D435 mount; publishes `/workspace_image`.
Both composite arm convex hulls and live entity poses on top of the
splat render so the operator sees them in the feed.

## Rust scene lidar

`dimos/simulation/sensors/rust/scene_lidar/` (rust) and
`dimos/simulation/sensors/scene_lidar.py` (the `NativeModule` wrapper).
BVH-accelerated raycaster against the cooked collision GLB plus
dynamic entities streamed via `/entity_state_batch`. 720×16 rays at
10 Hz default; scan model is configurable (`uniform`, `mid360`).

## Wire-up

```bash
dimos --simulation pimsim --scene dimos-office run g1-groot-wbc
dimos --simulation pimsim --scene street-lite  run unitree-go2-agentic
pytest -m pimsim dimos/e2e_tests/
```

`--simulation pimsim` is an alias for `babylon` and resolves through
`dimos/robot/cli/dimos.py:57` plus the per-robot `make_connection` /
blueprint branch. Wired blueprints: `g1-groot-wbc`,
`unitree-go2-basic`, `unitree-go2-agentic`, `agibot-x2-policy-sim`.
Headless mode launches Chromium via
`dimos/experimental/pimsim/headless.py` and requires
`pip install 'dimos[pimsim]' && playwright install chromium`.

## Relationship to `examples/language-interop/ts`

That example ships `@dimos/lcm` and `@dimos/msgs` on JSR plus a
standalone Deno LCM↔WS bridge. Pimsim uses the same `@dimos/msgs` wire
format in its browser code and exposes `/lcm-ws` from its own uvicorn
instead of spawning a separate Deno process. A client written against
the example bridge connects to pimsim unchanged. Pimsim's bridge
already has per-channel min-interval rate caps and a
latest-per-channel drain loop (see `module.py:_lcm_websocket`); those
patterns transfer to the standalone bridge when needed.

## Open

- Still under `experimental/`. Move to `dimos/simulation/pimsim/` is
  pending and not blocking anything.
- No shared `SimulationModule` base. Pimsim and `MujocoSimModule` both
  extend `Module` directly while sharing the entity-authority
  contract.
- Scene-package articulation (doors, drawers) has no schema yet —
  `joints[]` on `InteractableSpec` is the next extension.
- Headless mode boots a real Chromium tab. A `BABYLON.NullEngine` in
  Node would drop the browser dependency but needs the scene-building
  JS factored out of `static/app.js`.

# Click-to-Grasp end-to-end plan (PimSim proof + real-robot path)

> Implementation plan written 2026-06-16 to survive a context compaction. When
> implementing cold: read this file top to bottom first. Branch is **pim/dev**.
> I (Claude) am on **macOS**; the user (Nabla7/pim) tests visuals on **Linux**.

## Goal

Click a graspable object in the Babylon viewer → the robot reaches/grasps it.
Same flow in sim and on the real robot; the only thing that differs is **how a
clicked object resolves to a pose**. This proves PimSim end-to-end and gives a
clean answer to "what is PimSim."

## The architecture (the one seam that keeps it clean)

The reach is **already source-blind**: `pick`/`move_to_pose` consume a *pose*
(IK→plan→execute). So we add ONE resolver seam with swappable pose sources and
touch the manipulator almost not at all.

```
Babylon click ─► grasp intent ─► ObjectPoseSource.resolve(query) ─► SceneObject(id, pose, geometry)
                                   ├─ PrivilegedObjectSource : latest EntityStateBatch (sim ground truth)
                                   └─ PerceivedObjectSource  : perception detection DB (sim RGBD OR real RealSense)
                                                              │
                                                              ▼
                                          pick / move_to_pose  (UNCHANGED, source-blind)
```

Camera/perception is **source-blind** too — everything matches the RealSense
contract (`color_image` + `depth_image` DEPTH16 + `camera_info`), so the three
modes are one contract:

| Mode | RGB+depth source | Status |
|---|---|---|
| True sim (MuJoCo authority) | `MujocoSimModule` render | **RGBD already published today** |
| Real robot | RealSense | works today |
| Babylon-as-authority | splat RGB + splat depth (gsplat/mlx compute depth, unexposed) | only real new camera work (Phase 3) |

**Key realization:** the "new mesh camera with depth" is NOT the move. For
perception use the MuJoCo render (sim) or RealSense (real). The mesh camera
stays a lightweight viewer; the splat camera only needs a depth pass if we want
perception under Babylon-physics authority (Phase 3).

## Duplication we kill

- 3 object nouns (`Object`/`DetObject`, `EntityDescriptor`, `Obstacle`) → one
  `SceneObject` returned by the resolver (the §7-A convergence, finally paying off).
- 2 sim cameras (mesh vs splat) → perception eats the MuJoCo render; splat stays a view.
- N pose sources → one `ObjectPoseSource` protocol, 2 impls.

## Key existing pieces (file:line — so I don't re-investigate post-compaction)

**Reach flow (unchanged, the consumer):**
- `dimos/manipulation/manipulation_module.py`: `move_to_pose` (:1282), `plan_to_pose`
  (:508 — `self._kinematics.solve(world, robot_id, target_pose, seed, check_collision)`
  → `_plan_path_only`), `execute` (:971 — `_translate_trajectory_to_coordinator` →
  `client.task_invoke(config.coordinator_task_name, "execute", {trajectory})`),
  `entity_states: In[EntityStateBatch]` (:124), config `scene_package`/`world_backend`/`enable_viz` (:99-103).
- `dimos/manipulation/pick_and_place_module.py`: `pick`/`place`/`scan_objects` skills,
  `objects: In[list[DetObject]]` (:81,101), `_detection_snapshot` (:92),
  `_find_object_in_detections(name, id)` (:194), `_generate_grasps_for_pick` (:276),
  `refresh_obstacles` (:128).
- `dimos/robot/unitree/g1/g1_manipulation.py`: `G1ManipulationModule(PickAndPlaceModule)`;
  `odom: In[PoseStamped]` (:59), `point_goal: In[PointStamped]` (:64) → `_on_point_goal`
  (:94) → `point_at` (:186, CLOSED-FORM aim, not IK); `reach_for_sim_object(body_name)`
  (:393, MJCF ground-truth → move_to_pose); `point_at_sim_object` (:362);
  `_sync_floating_base`/`_on_odom` (pushes /odom into the world).

**Babylon viewer + entity machinery (the interaction surface):**
- `dimos/experimental/pimsim/module.py`: `BabylonSceneViewerModule`;
  `entity_state_batch: Out[EntityStateBatch]` (:130), `entity_descriptors: Out` (:127);
  emits `clicked_point` + `point_goal` over the LCM-over-WebSocket bridge (~:540, 723-724);
  RPCs `spawn_entity` (:939), `despawn_entity` (:966), `set_entity_pose` (:977),
  `apply_entity_velocity` (:990); `_entities`/`_entity_poses` dicts keyed by entity_id
  (:609-610, 943-944); `entity_authority: Literal["browser","external"]` (:166).
- Browser meshes carry per-entity metadata `dimosSceneMesh` (see `perf_probe.py:259`),
  so a browser pick can report which entity_id was hit.

**Entity stream (privileged poses):**
- `dimos/experimental/pimsim/entity.py`: `EntityDescriptor` (:45 — entity_id, kind,
  mesh_ref, shape_hint, extents, mass, rgba), `EntityStateBatch` (:201 — entries of
  `(EntityDescriptor, Pose)`, JSON-over-LCM).
- Sim publishers: `MujocoSimModule.entity_state_batch` (mujoco_sim_module.py:349, `_publish_entity_states` ~:916);
  `BabylonSceneViewerModule.entity_state_batch` (module.py:130, ~:1126).
- Planning consumer: `dimos/manipulation/planning/monitor/world_monitor.py`
  `on_entity_state_batch` (:219) → `world.sync_entity_poses(poses)` (:228) — TODAY used
  only for collision, NOT target selection.

**Perception (perceived poses) — RealSense contract:**
- `dimos/perception/object_scene_registration.py`: `ObjectSceneRegistrationModule`;
  In `color_image`/`depth_image`/`camera_info` (:52-54); Out `detections_3d`/`objects`
  (`list[DetObject]`)/`pointcloud` (:57-59); `detect(prompts)` skill (:240),
  `get_detected_objects()` (:153). depth auto-scaled `/1000` (:296).
- `dimos/perception/.../object.py`: `Object`/`DetObject` (center/size/pose/pointcloud :54-57),
  `from_2d_to_list` (:146, o3d backproject RGBD→3D, world-transform).
- `dimos/hardware/sensors/camera/realsense/camera.py`: contract to match —
  `color_image` (RGB), `depth_image` (DEPTH16, mm), `camera_info` (:77-81, 292-316).

**Sim cameras:**
- `dimos/mapping/mesh_camera.py` (:102-399): RGB only; raycasts (`t_hit` = depth, :354-356)
  but does NOT publish depth; RGB is barycentric (low quality — wrong for YOLO-E).
- `dimos/visualization/viser/splat_camera.py` (:771-1221): RGB only; gsplat/mlx backends
  compute depth but don't expose it.
- `MujocoSimModule` ALREADY publishes `/color_image` + `/depth_image` + `/camera_info`
  + `/depth_camera_info` (confirmed in the groot-wbc sim run) → perception works in true sim TODAY.

**Planning world (collision/IK target):**
- `dimos/manipulation/planning/world/mujoco_world.py`: `MujocoWorld(WorldSpec)`;
  `sync_entity_poses` (:565), `add_obstacle` (:736), `update_obstacle_pose` (:894),
  `get_visualization_url` (:919 — NO-OP, so mujoco backend has NO standalone viewer).
- Drake has meshcat viz; cc's `viser-vis-rework` will add a manipulation viser viewer
  (the eventual MujocoWorld standalone view).

**Runnable entry already added this session:**
- `g1-office-planner` blueprint (`dimos/manipulation/blueprints.py` `g1_office_planner`):
  `G1ManipulationModule` + g1 left arm (`dimos/robot/catalog/g1.py` `g1_left_arm(backend="mujoco").robot_model_config`)
  + `world_backend="mujoco"` + `scene_package="dimos-office"` + transports for
  `/odom`, `/entity_state_batch`, `/point_goal`. enable_viz is a no-op on mujoco backend
  (see get_visualization_url). Boots clean; needs the sim alongside for /odom + visuals.

**PimSim spec (the contracts doc):**
- `dimos/experimental/pimsim/spec/` — protocols.py (`EntityAuthority`, `EntityConsumer`,
  `SceneObjectWorld` PROPOSED), models.py (`SceneObject` PROPOSED), enums.py, README.md.
- `dimos/experimental/pimsim/DESIGN.md` + `SPEC.md` (prose).

## The new abstractions to introduce (small, clean)

1. **`SceneObject`** — the unified graspable-object noun the resolver returns:
   `id`, `pose` (PoseStamped, world), plus geometry (`mesh_ref`/`shape_hint`/`extents`)
   and optional `kind`. It's the proposed §7-A noun (already stubbed in
   `spec/models.py`). For this work, make it a real lightweight type (do NOT yet
   refactor `Obstacle`/the whole WorldSpec — that's cc's domain + joint-agreed).
2. **`ObjectPoseSource`** (Protocol): `resolve(query) -> SceneObject | None`, where
   `query` is an entity_id (str) OR a world `PointStamped`. Plus maybe
   `list_objects() -> list[SceneObject]`. Lives in a new small module, e.g.
   `dimos/manipulation/scene_objects.py` (or `dimos/experimental/pimsim/` if it's
   considered pimsim-side — decide at impl time; keep it where the manipulator can
   import it without heavy deps).
   - `PrivilegedObjectSource`: caches the latest `EntityStateBatch`; `resolve(id)`
     → the matching `(descriptor, pose)`; `resolve(point)` → nearest entity.
   - `PerceivedObjectSource`: wraps the perception detection snapshot
     (`get_detected_objects` / `objects` stream); `resolve(name/id)` → DetObject→SceneObject;
     `resolve(point)` → nearest detection.
3. **Grasp intent**: a click→grasp signal. Phase 1 reuses the EXISTING
   `clicked_point`/`point_goal` (a world `PointStamped`) + nearest-entity match in
   the PrivilegedSource — no browser JS change. Phase 1b (optional) lets the browser
   report the picked `entity_id` directly (precise) via a small JS pick handler +
   a stamped-string/entity_id message. Decide a topic name: `/grasp_goal`.

## Phases

### Phase 1 — Privileged click→grasp (the demo, proves the seam)
Minimal, no perception/depth. Goal: click an object in Babylon → robot reaches it.
1. Add `SceneObject` (real type) + `ObjectPoseSource` protocol + `PrivilegedObjectSource`
   (resolves from `EntityStateBatch`; by entity_id and by nearest-point).
2. Wire `G1ManipulationModule`: add `grasp_goal: In[PointStamped]` (or reuse
   `point_goal` semantics) → `_on_grasp_goal` → `obj = privileged_source.resolve(click)`
   → `self.move_to_pose(obj.pose...)` (Phase 1 = reach-to-object; defer full grasp-gen).
   Feed the source the entity stream the module already subscribes to (`entity_states`,
   :124) — keep one cache.
3. Babylon: emit a grasp intent on click. Simplest: a viser/HUD "grasp clicked object"
   action that publishes `clicked_point` → `/grasp_goal`. (Reuse the existing
   clicked_point plumbing; ~:540/723.)
4. Runnable blueprint: a combined **sim + manipulation** blueprint so the Babylon
   viewer (from the sim) shows the scene + the planner reaches. Compose `g1-groot-wbc`
   (publishes /odom + /entity_state_batch + Babylon viewer) with the g1 manipulation
   planner (consumes them). Likely `autoconnect(<groot-wbc sim>, <g1 manip planner>)`.
   Name e.g. `g1-office-grasp`. Verify execution wiring: the planner executes via
   `coordinator_task_name` (e.g. `traj_left_arm`) — confirm the groot sim's coordinator
   has a trajectory task the planner can invoke, or add one (the servo_arms task may
   need a sibling trajectory task; check `dimos/control/tasks/`).
**Verify:** boot headless (see Workflow: PYTEST_VERSION trick or `--simulation mujoco`),
publish a `/grasp_goal` (a tiny script via LCM, or call the @rpc), confirm logs show
resolve→IK→plan→execute. Full visual click→reach = user on Linux in Babylon.

### Phase 2 — `detect` in true-sim (proves the real-robot detection path)
No privileged cheat; perception runs on the MuJoCo RGBD.
1. Wire `ObjectSceneRegistrationModule` onto the MuJoCo sim's existing
   `/color_image` + `/depth_image` + `/camera_info` (it already publishes them).
   Confirm depth units/format match perception's expectation (DEPTH16 mm; perception
   scales /1000 — the sim may publish float meters, so a depth-format adapter or a
   config flag may be needed — CHECK at impl).
2. `PerceivedObjectSource` resolves from the perception detection DB; the resolver
   gains a "perceived" mode. `detect("bottle")` then click → resolve from detections.
3. Same click→grasp path; only the source swaps (privileged → perceived). This is the
   moment `SceneObject` unifies perception `Object` and the entity descriptor.
**Verify:** in the MuJoCo sim, run `detect`, confirm `get_detected_objects` returns the
manip_* objects with sane poses; click→grasp via the perceived source.

### Phase 3 — Babylon-authority depth + real-robot deploy
1. Expose a depth pass on `splat_camera` (gsplat/mlx already compute depth) so the
   Babylon-physics authority can feed perception → RealSense-equivalent RGBD in browser.
2. Real robot: swap the source to RealSense perception (already the contract). Babylon
   runs in `external`/MIRROR mode as the true robot viewer. Click→grasp gated behind
   real detection.
**Verify:** user on the real G1 (Linux); out of scope for the Mac sandbox.

## Workflow / environment facts (so cold-start works)

- **Branch:** pim/dev. Commit trailer: `Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>`. Push: `git push origin pim/dev`.
- **Pre-commit:** runs ruff check + **ruff format** (run `uv run ruff format <files>`
  before committing or the hook fails on reformat), cargo fmt/clippy (rust only).
  Pre-commit STASHES unstaged changes → stage ALL related files together.
- **all_blueprints.py is auto-generated:** after adding/removing a blueprint run
  `uv run pytest dimos/robot/test_all_blueprints_generation.py` (it FAILS the first run
  = it rewrote the file; `git add` it + commit). Blueprint var name `foo_bar` →
  registry key `foo-bar`.
- **Booting `dimos run <bp>` headless on Mac:** `dimos run` (non-sim) does a
  `sudo route add -net 224.0.0.0/4` (LCM multicast) that FAILS in the no-sudo sandbox
  and is fatal. Bypass for local boot tests with `env PYTEST_VERSION=8.0 uv run dimos run <bp>`
  (system_configurator skips when PYTEST_VERSION is set — base.py:84). `dimos --simulation mujoco run`
  skips it (in-process transport). Single-process LCM works on loopback without the route.
- **Boot smoke pattern:** launch in background, watch the log for
  `MujocoWorld finalized` / `Added robot` / transports / `Traceback`, then kill. Clean
  up leftover procs (`pkill -9 -f "dimos run"`) + port holders (`lsof -ti:8091`).
- **MujocoWorld works** (18 tests pass: RRT, IK, G1 dual-arm). `g1-office-planner` boots
  via the PYTEST_VERSION trick.

## Constraints / coordination

- **cc's #2489 (`cc/spec/movegroup`)** redesigns `WorldSpec` to Planning Groups
  (group-scoped, resolved joint names `{robot}/{joint}`) in OpenSpec format — it lands
  the WorldSpec interface change. OUR stuff lands first on the CURRENT WorldSpec; do NOT
  fully adopt planning groups yet (rebase later). Don't break `WorldSpec`.
- **The §7-A `SceneObject`/`add_object`/`update_object_pose` rename across all WorldSpec
  backends is JOINT-AGREED — do NOT do the full rename here.** Introduce `SceneObject`
  only as the resolver's return noun; keep `Obstacle`/`sync_entity_poses` as-is.
- **cc's `viser-vis-rework`** adds the manipulation viser viewer (the eventual MujocoWorld
  standalone view) — don't build a competing one; the sim's Babylon viewer is our view for now.
- **cc's `pink-ik`** (planning KinematicsSpec) and **`roboplan-integration`** (another
  WorldSpec backend) exist — don't duplicate; the resolver/IK seam is orthogonal.

## Open decisions to confirm with the user at impl time

- Phase 1 grasp intent: clicked-point + nearest-entity (no JS change) vs browser reports
  entity_id (small JS, precise). Recommend start with clicked-point.
- Where `ObjectPoseSource`/`SceneObject` live (manipulation-side vs pimsim-side).
- Phase 1 = reach-to-object (`move_to_pose`) vs a real grasp (`pick` + grasp-gen).
  Recommend reach-to-object first.
- Whether to commit this plan file or keep it untracked.

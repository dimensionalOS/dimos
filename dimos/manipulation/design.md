# G1 manipulation stack — design

One document for the three layers that turn the G1 into a manipulation
platform, slotting into the existing coordinator without touching the
GR00T WBC (which keeps its 15 legs+waist joints at priority 50; the 14 arm
joints are free above `servo_arms` at priority 10).

```text
                  coordinator (tick loop, arbitration)
                  ├── groot_wbc          legs+waist   prio 50   (unchanged)
                  ├── mink_ik task       arms (14)    prio 20   ← layer 2
                  └── servo_arms         arms (14)    prio 10   (fallback hold)

  manipulation process
  ├── MujocoWorld (WorldSpec)            ← layer 1: collision/FK oracle
  │     model = cooked wrapper + G1 MJCF + scene entities
  │     live sync: /coordinator/joint_state, /odom, /entity_state_batch
  ├── RRT-Connect / JacobianIK           (existing, unchanged — WorldSpec-only)
  └── stance selection                   ← layer 3: where to stand
        proposer (capability map) → verifier (IK) → nav/twist
```

Drake drops out of the online loop entirely; `DrakeWorld` survives as a
non-default backend until parity is proven, then per-robot flips.

---

## Layer 1 — MujocoWorld

A `WorldSpec` implementation using MuJoCo as a kinematics + collision
*library* (`mj_kinematics`, `mj_comPos`, `mj_collision`, `mj_jac` — never
`mj_step`). Planning, sim, and rendering consume the same cooked scene
packages and the same entity wire format; the planning world becomes one
more consumer of the entity-authority contract.

### Model composition

```python
world = MujocoWorld(scene_package="dimos-office")
left  = world.add_robot(g1_left_arm_config)                      # attaches G1 MJCF once
right = world.add_robot(g1_right_arm_config, share_model_with=left)
world.finalize()                                                  # spec.compile()
```

`finalize()` mirrors `MujocoSimModule._compose_model`: load the cooked
wrapper into an `MjSpec`, attach each distinct robot model (robot first,
keeps the leading qpos block), `add_entities_to_spec` (freejoints + the
package's cooked CoACD hulls), obstacle slot pool, compile.

- **Model files**: MJCF for this backend (G1:
  `data/mujoco_sim/g1_gear_wbc.xml`; menagerie MJCFs for the arms).
  MuJoCo parses plain URDF but not xacro/`package://` — per-robot check
  before flipping a URDF-only robot.
- **`weld_base=False`** (G1): the robot keeps its freejoint; base pose is
  state written by `set_floating_base_pose()` — the same duck-typed hook
  `g1_manipulation.py` already calls. No plant rebuilds.
- **`share_model_with`**: a `WorldRobotID` resolves to a `RobotEntry`
  (config, prefix, qpos/dof index tables per `config.joint_names`,
  `ee_body_id` + grasp offset, moving-subtree geom set, excluded pairs).
  Two arms on one humanoid = one body tree, two entries.
- G1 catalog (`dimos/robot/catalog/g1.py`) gains an MJCF model path +
  MJCF joint-name mapping (`g1/left_shoulder_pitch` →
  `left_shoulder_pitch_joint`, the `RobotSimSpec` convention).

### Contexts

Context = `MjData`. Live context mirrors reality; `scratch_context()` is
a pooled `MjData` filled by `mj_copyData` from live — microseconds, vs
Drake's ~100 ms `CreateDefaultContext` that JacobianIK currently works
around. `mjModel` is read-only during queries (lock only for obstacle
mutation and pool bookkeeping); threads each query their own `MjData`.

### Query mapping

| `WorldSpec` method | MuJoCo |
|---|---|
| `set_joint_state(ctx, id, js)` | names → qpos adrs, write, `mj_kinematics` + `mj_comPos` |
| `get_ee_pose(ctx, id)` | `xpos/xquat[ee_body_id]` + grasp offset |
| `get_jacobian(ctx, id)` | `mj_jac` at grasp point → stack `[jacp; jacr]` (rows `[v; ω]`, matching the `Jacobian` alias), slice dof columns |
| `get_joint_limits(id)` | `model.jnt_range`, config overrides win |
| `is_collision_free(ctx, id)` | `mj_kinematics` + `mj_collision`, contact scan (below) |
| `check_edge_collision_free` | linear interpolation, all steps in one scratch context |
| `get_min_distance(ctx, id)` | min `contact.dist` over relevant contacts; **not** a global signed distance (MuJoCo only reports within margin) — documented deviation, nothing consumes it beyond diagnostics |
| `sync_from_joint_state` / `set_floating_base_pose` | writes into the live context |

`JacobianIK` and `RRTConnectPlanner` run unmodified (verified against
their call patterns).

### Collision semantics

A standing humanoid always has contacts (feet ↔ floor), so "any contact =
collision" rejects every config. The check is scoped to the **planning
group's moving subtree**: at finalize, each `RobotEntry` precomputes the
collision geoms of bodies downstream of its planned joints (shoulder →
hand). A config is in collision iff a contact involves one of those geoms
and the pair isn't excluded. Verdict: `contact.dist < 0`.

Self-collision audit of `g1_gear_wbc.xml`: 49 visual geoms at
`contype=0 conaffinity=0`; the ~52 collision geoms use defaults (1/1), no
`<pair>`/`<exclude>` elements → arm-vs-torso and arm-vs-arm pairs are
active out of the box. Remaining audit: wrist/hand collision geom
coverage (if hands are visual-only, clearance checks degrade to the wrist
geom — worst case add capsules at attach time).
`collision_exclusion_pairs` from `RobotModelConfig` maps to
`spec.add_exclude`.

### Entities vs Obstacles

Scene entities (chairs, scanned props) enter at finalize with their
cooked hulls; their **poses are state**: `sync_entity_states(batch)`
writes each `entity:<id>:free` qpos7 in the live context. A new
`EntityStateMonitor` (sibling of `RobotStateMonitor`) subscribes
`/entity_state_batch` — identical under MuJoCo sim authority, pimsim
Havok authority, and the real robot once perception publishes the same
message. This replaces the Drake obstacle-feeding path for scene objects.

`add_obstacle()` stays for perceived primitives: a pre-allocated slot
pool (static bodies parked underground, `contype=0`); add = mutate
`body_pos/quat`, `geom_size`, `geom_rbound` (broadphase radius!), enable
contype, under the model lock. MESH obstacles go through
`MjSpec.recompile(model, data)` (state-preserving, mujoco ≥ 3.2; present
in our 3.5.0) — seconds-scale on the office scene, log loudly. The
strategic path for perceived objects is the entity contract, not
`add_obstacle`.

### Visualization

The world does not render. `animate_path` publishes the waypoint path on
a `/planning/preview` channel and returns; ghost-robot rendering is
viewer work (pimsim's `robot_meshes.py` already builds robot links
browser-side). No render call exists in any query path — the old
Meshcat-latency bug class is gone structurally.

### Parity harness

pytest module (skipped without `pydrake`): same G1 arm config in both
worlds, N random configs — FK agreement (<1 mm, <0.5°), Jacobian
element-wise, collision-verdict agreement with a tolerated boundary band
(report, don't hard-fail at contact margins). Benchmarks for the PR:
`check_config_collision_free` ≤ 200 µs on the office scene, scratch
context ≤ 1 ms, full 7-DOF RRT plan well under 1 s — measured against
DrakeWorld on the same machine.

---

## Layer 2 — mink arm task (reactive cartesian control)

[mink](https://github.com/kevinzakka/mink) (Apache-2.0, MJCF-native,
ships a G1 example) replaces the unwired Pinocchio-DLS
`cartesian_ik_task` as the reactive layer: QP-based differential IK with
joint limits, velocity limits, posture regularization, and self-collision
avoidance as hard constraints — the things DLS can't express.

### Task shape

One coordinator task, `g1_mink_ik`, claiming **all 14 arm joints** at
priority 20 (above `servo_arms` 10, below WBC 50 — disjoint from the
WBC's joints anyway). One task for both arms, not one per arm: a single
QP handles arm-vs-arm collision constraints and (later) bimanual
coupling coherently.

```text
registry type: "mink_ik"   (dimos/control/tasks/mink_ik_task/)
claim: 14 arm joints, POSITION, prio 20
per tick (with decimation, GR00T-task pattern — 50 Hz solve on the
500 Hz real loop, every tick in sim):
  1. configuration.q ← measured qpos (state.joints) + pelvis freejoint
     pose from state.imu/odom            # never integrate open-loop drift
  2. targets: FrameTask(left_ee), FrameTask(right_ee) from the latest
     cartesian commands (world or pelvis frame, per command frame_id)
  3. v = mink.solve_ik(configuration, tasks, dt, solver, limits)
  4. q_cmd = q_measured_arms + v_arms · dt, rate-limited
     (max_joint_delta pattern from CartesianIKTaskConfig)
  5. emit JointCommandOutput for the 14 arm joints
```

- **Model**: the same `g1_gear_wbc.xml`. The coordinator process gains a
  `mujoco` import (CPU-only kinematics, fine on the real robot).
- **Tasks/limits**: `FrameTask` per EE (position+orientation costs),
  `PostureTask` (comfort pose, low cost — resolves the 7-DOF null space),
  `ConfigurationLimit`, `VelocityLimit`, `CollisionAvoidanceLimit` over
  the arm-relevant geom pairs (shared pair list with Layer 1's audit).
  Non-arm DOF are present in the QP but their velocities are discarded;
  the WBC owns them.
- **Inputs**: the coordinator's existing `on_cartesian_command` surface,
  with `PoseStamped.frame_id` selecting the EE (`left_ee` / `right_ee`)
  and the reference frame; timeout → hold current pose (servo semantics).
  Teleop, visual servoing, and trajectory tracking (stream waypoint poses)
  all ride this.
- **Stale-target behavior**: hold last solved arm pose, do not decay to
  defaults (that's `servo_arms`' job if this task deactivates).
- **Failure modes**: QP infeasible → keep previous command, warn at rate;
  target unreachable → converges to nearest feasible (FrameTask is a
  soft cost), which is the desired servo behavior.
- **Dependency**: `mink` + a QP solver (daqp/osqp) added to an extra
  (`[manipulation]` or a slimmer `[ik]`). Exact API pinned at impl time
  against the mink G1 example.

Division of labor with Layer 1: mink is *local* — it will not route an
arm around a chair. When clutter demands it, `plan_to_pose` (RRT over
MujocoWorld) produces a joint path and either executes via the trajectory
task or streams FK waypoint poses through mink. Same arbitration slot
either way.

mink-as-solver is also reused offline: solve-to-convergence with limits +
collision constraints is the IK verifier for Layer 3 and the
ground-truth oracle for its evaluation.

---

## Layer 3 — stance selection ("where should the robot stand?")

### Method judgment (why not RM4D wholesale)

The operative question for a mobile humanoid is the **inverse** one:
given grasp candidates, pick a pelvis (x, y, heading). Honest assessment
of the options:

- **Pure IK sampling** (no precomputation): sample candidate stances on
  an annulus around the target, verify each with a mink solve. Exact,
  zero new data structures, ~1–5 ms per (stance, grasp) pair. Fine for a
  handful of candidates; too slow for hundreds of grasps × dozens of
  stances × multiple objects, where stance scoring wants grid
  aggregation anyway.
- **Precomputed capability map** (Zacharias/Vahrenkamp/Burget lineage):
  the right tool exactly when aggregation across many target poses is
  the workload — which is the grasp-planning pipeline we're building
  toward.
- **RM4D specifically** contributes two reductions. Its base-yaw
  symmetry trick is **exactly valid** for the G1 — better than for any
  arm — because the WBC gives a true SE(2) base: free planar position,
  free heading (turn in place), fixed height, level pelvis. Its
  wrist-collapse to 4D is **invalid**: `wrist_yaw` is ±92.5° against an
  assumption of 360° (the paper's own ablation stops at ±150°). And its
  binary cells answer feasibility when stance *ranking* is what we
  actually need.

**Decision**: build a capability map in the gravity-aligned pelvis frame
using RM4D's canonical-base-position construction for the inverse query
(the valid half), keep the in-plane rotation as an explicit coarse
dimension (rejecting the invalid half), and store **scores, not bits**.
Don't call it RM4D; it's an inverse-reachability map that borrows RM4D's
indexing trick. The map is a *proposer*; mink IK is the *verifier* — the
map only needs high recall and decent precision, because every proposed
stance is IK-checked before the robot walks anywhere. Build the verifier
first (it works standalone with annulus sampling); add the map when
multi-candidate aggregation is actually exercised.

### Frame and structure

Map lives in the **gravity-aligned pelvis frame at ground level**:
origin at the pelvis ground projection, z along gravity, x along the
yaw-projected pelvis heading.

- **θ = ∠(approach vector, gravity ẑ)** — against gravity, *not* pelvis
  z. The symmetry axis is gravity-z; pelvis roll/pitch wobble is a
  disturbance, not a symmetry (the WBC re-levels). Construction samples
  with the pelvis exactly level at h₀ = 0.74 m, matching standstill
  execution. 3° residual tilt ≈ one 5 cm cell at full extension —
  erode for conservative filtering, dilate for candidate generation.
- Dimensions: `(p_z, θ, x*, y*, γ)` — heights from the ground plane,
  RM4D's canonical planar offset (translate TCP to origin, rotate by −ψ,
  ψ = atan2(r_zy, r_zx)), plus in-plane angle γ in ~12 coarse bins.
  ~28 × 36 × 30 × 30 × 12 ≈ 11 M cells.
- Cell payload: saturating uint8 sample count (a manipulability score is
  a drop-in upgrade), plus a ψ₀ heading-hint bitmask so the inverse query
  returns (x, y, **φ**) directly instead of heading-searching on arrival.
  Total ~20 MB; a binary 4D OR-reduction is one numpy line if a tiny
  prefilter is ever wanted.

Query semantics (the one real departure from the paper): forward
`query(T)` means *reachable from this position, possibly after turning in
place* — the right predicate, since turning is the quotiented symmetry
and cheap. "Reachable at the current heading right now" is a single mink
solve, not a map query. Inverse query is paper Eq. 7–9 verbatim: slice
the (p_z, θ) plane, back-rotate, translate, return scored stances.

Two free humanoid bonuses: (a) **height dilation** — the arm is rigid to
the pelvis, so a future `height_cmd` interface extends vertical reach as
a 1D band-dilation over p_z at query time, no new dimension; (b) **right
arm by mirror** — `S_r[p_z, θ, x*, y*, γ] = S_l[p_z, θ, x*, −y*, −γ]`,
validated once against a small right-arm sample run.

### Construction & evaluation

Sampling (per arm): pelvis fixed level at h₀, waist at default (the WBC
owns it — conservative), other arm at servo default; draw uniform 7-DOF
arm configs, MuJoCo FK + self-collision reject on the same MJCF
(~50 µs/sample → 10 M samples in minutes, parallel workers, OR-merge).
Stop at the paper's saturation criterion (<0.1% new cells per 1 M).

Evaluation settles how much the wrist violation costs: ~100 k poses
uniform in the workspace cylinder; ground truth = mink
solve-to-convergence, multi-restart, paper's acceptance threshold.
Heading-free semantics are matched exactly by canonicalizing each eval
pose to ψ = 0 and testing that representative with the pelvis fixed (map
and oracle then quotient heading identically — no pelvis-yaw search).
Report accuracy/TPR/FPR overall and per (θ, γ) bin; the γ-marginalized
map's FPR tells us whether the coarse-γ dimension is pulling its weight.
Proposed bar: FPR ≤ 10% is fine for a proposer (a false positive costs
one IK check).

### Pipeline

```text
grasp candidates → inverse query per candidate → scored stance grids
  → aggregate / intersect across objects
  → filter: footprint clearance (MujocoWorld pelvis-cylinder check vs
    scene entities) + nav reachability
  → walk (twist/nav), turn to heading hint
  → forward query prefilters grasps; mink verifies; planner/mink executes
```

Code: `dimos/manipulation/reachability/` — `capability_map.py`
(structure + queries + mirror), `construct.py` (CLI), `evaluate.py`
(oracle + metrics), anchor tests that need no built map (canonical
transform round-trip, mirror identity, known-reachable/unreachable
poses).

---

## Planner upgrades — only when latency hurts

The existing RRT-Connect over MujocoWorld is the v1 planner. When it
measurably hurts: OMPL 2.0 (pip wheels, Apr 2026; validity checker = one
MujocoWorld call) is the cheap robot-agnostic upgrade; VAMP
`g1_upper_body` codegen (foam spherization + cricket, compile-time joint
groups) is the µs-class endgame for replan-every-tick against
pointclouds. Neither blocks anything above.

## Phasing

1. **MujocoWorld core** — `mujoco_world.py`, factory `backend="mujoco"`,
   unit tests + Drake-parity harness. Standalone via
   `create_planning_stack`. (Keystone; everything else verifies against
   it.)
2. **mink task** — `mink_ik_task/`, registry entry, G1 blueprint wiring,
   sim smoke test. *Independent of 1 — parallelizable.*
3. **Monitors & wiring** — `EntityStateMonitor`, `WorldMonitor` backend
   param, `ManipulationModuleConfig.world_backend/scene_package`, G1
   catalog MJCF variant, flip G1 sim flows to MuJoCo backend.
4. **Stance selection** — annulus-sampling + mink verifier first (works
   with no map), then `construct.py`/`evaluate.py` and the capability
   map behind the same proposer interface; the eval report decides the
   map's final shape.
5. **Preview viz** — `/planning/preview` channel + Babylon ghost; retire
   the Meshcat thread wrapper.

## Risks

- Wrist/hand collision geometry coverage in the G1 MJCF (affects layers
  1–3 identically; one audit, shared pair list).
- `MjSpec.recompile` with attached sub-specs + entity freejoints needs a
  regression test before the mesh-obstacle path is advertised.
- mink API drift (young library) — pin version, adapt from its in-tree
  G1 example.
- Coordinator-process mujoco dependency for the mink task on real
  hardware (CPU-only; verify install on the deployment image).
- TCP definition (wrist vs palm vs hand, `grasp_offset_xyz`) must be
  identical across mink, the world, and map construction — single source
  in the G1 catalog config.

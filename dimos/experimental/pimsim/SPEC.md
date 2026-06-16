# PimSim & G1 Reachability — System Spec

> **Canonical usage doc: [`README.md`](README.md); API in code:
> [`spec/protocols.py`](spec/protocols.py) + [`api.py`](api.py).** This file is
> the G1-reachability deliverable spec — a separate track that consumes PimSim's
> scene/world contracts.

Status: draft for review (answers the "how does this fit the larger system"
question on #2479). Author: Pim.

## 0. Purpose and scope

**Deliverable:** reachability analysis for the G1 arms — answer "can this
arm reach this pose, and how well?" without running IK.

**This document:** how reachability fits the existing manipulation +
simulation stack, and the two shared contracts it depends on — the
*scene-object* description and the *world backend*. It documents mostly
what already runs end-to-end on the pimsim branch; it marks the few
genuine design decisions explicitly as **DECISION**.

**Non-goals:** this does not respecify the planning/IK protocols (they
exist and are correct); it does not cover the Babylon viewer or the Rust
lidar consumer (they stay on the experimental branch); it is not a
roboplan/pink critique — those are good and this spec accommodates them.

---

## 1. The layer map

```
   PHYSICS / PERCEPTION AUTHORITY          (sim, or perception later)
            │  EntityStateBatch  (versioned JSON-over-LCM, authority-agnostic)
            ▼
   ┌────────────────────┐
   │  scene-object       │   one description: SceneObject (§2)
   │  contract           │   one portable package: cooked GLB + CoACD hulls
   └─────────┬───────────┘
             │  each backend COMPILES the package into its native form once
   ┌─────────┼───────────────────────────────┐
   ▼         ▼                                ▼
┌────────┐ ┌──────────────┐            ┌──────────────┐
│ Drake  │ │ MujocoWorld   │            │ RoboPlanWorld│   WorldSpec backends (§3)
│ World  │ │ (humanoid)    │            │ (fast arm)   │   geometry/kinematics/collision ONLY
└───┬────┘ └──────┬────────┘            └──────┬───────┘
    └─────────────┴───────── consumed by ──────┴───────────┐
                  ▼                  ▼                      ▼
          ┌──────────────┐  ┌──────────────┐      ┌──────────────────┐
          │  PlannerSpec │  │ KinematicsSpec│      │  Reachability     │  (§4) — the deliverable
          │  RRT-Connect │  │ JacobianIK    │      │  capability map   │
          │  RRT*        │  │ pink / drake  │      │  build + query    │
          └──────────────┘  └──────────────┘      └──────────────────┘

   (separate stack) CONTROL: coordinator tick loop — trajectory task,
   GR00T WBC, mink cartesian servo. Executes plans; not part of planning.
```

Three facts this picture encodes, each verified in the current code:

- `WorldSpec` is geometry only. `PlannerSpec` and `KinematicsSpec` are
  *separate* protocols that take a `world` and call only its methods
  (`RRTConnectPlanner` collision-checks via `world.check_edge_collision_free`;
  `JacobianIK` does FK via `world.get_ee_pose`/`world.get_jacobian`).
- A backend ingests the shared scene package **once at `finalize`** and
  compiles it natively. Sharing the package therefore costs the planner
  nothing — there is no per-query penalty for a shared scene.
- Reachability is a fourth consumer of `WorldSpec`, at the same level as
  the planner and the IK solver. It does not belong to any one backend.

---

## 2. Scene-object contract

### The problem

Three types describe "a shaped thing at a pose," and they overlap:

| Concept | Where | Carries | Purpose |
|---|---|---|---|
| `Obstacle` | `planning/spec/models.py` | name, type, pose, dims, color, mesh_path | planning collision input (static add) |
| `EntityDescriptor` + `EntityStateBatch` | `experimental/pimsim/entity.py` | id, kind, shape, extents, mesh_ref, rgba, mass + streamed pose | live scene state from an authority |
| `Object` | `perception/.../detection3d/object.py` | object_id, center, size, pose, pointcloud, mask | perception detection output |

`Obstacle` and `EntityDescriptor` are the same data under two names.
`Object` is different in kind — it is the *detector output* (pulls in
open3d + cv2 + the Detection3D stack) and cannot be reused directly: the
Rust and browser consumers can't take that dependency, and it carries no
asset reference or physics fields.

### DECISION — one noun, two verbs

Collapse `Obstacle` and `EntityDescriptor` into one `SceneObject`
description. Expose two verbs on `WorldSpec`, because the two operations
are genuinely different (and both already exist):

```
add_object(obj: SceneObject, pose) -> id     # inject geometry the world didn't have
update_object_pose(id, pose) -> bool         # reposition geometry already present
remove_object(id) -> bool
```

- Perception stays upstream: `WorldObstacleMonitor` *converts* a detector
  `Object` into a `SceneObject` and calls `add_object`. `Object` is
  unchanged.
- The sim/physics authority streams `EntityStateBatch` (now carrying
  `SceneObject`s); the monitor calls `update_object_pose` per entry.
- `EntityStateBatch` is the wire form: versioned, length-prefixed JSON
  over LCM (the `EntityMarkers` pattern), hand-decodable, no codegen.

Scope: ~11 files in `manipulation/planning/` + `entity.py`. No mechanism
bodies change — it is a type merge plus renaming `add_obstacle` →
`add_object` across the three backends. **This touches all backends, so
it is agreed jointly, not landed unilaterally.**

---

## 3. World backends

`WorldSpec` is the geometry contract. Backends differ in capability, and
that is the point — pick the backend per robot:

| | Drake | MujocoWorld | RoboPlanWorld |
|---|---|---|---|
| floating base (mobile/humanoid) | no | **yes** | no |
| dual-arm on one model | no | **yes** | no (one robot/scene) |
| model format | URDF | MJCF (shared with sim) | URDF + SRDF |
| scene-entity sync | no | **yes** | no |
| native fast planner | no | no | **yes (compiled RRT)** |

### DECISION — name the two planner kinds

Planning has two legitimate shapes, and the spec should say so instead of
pretending all planners are interchangeable:

- **Backend-agnostic planner** (`RRT-Connect`, `RRT*`, `JacobianIK`):
  Python, consumes *any* `WorldSpec`. These MUST keep working on every
  backend — that is the line the abstraction protects.
- **Backend-coupled planner** (`roboplan`): a compiled library that owns
  its world+planner so it can collision-check in-process without a Python
  call per edge. It is exposed through `PlannerSpec` but is only valid
  with its own world. The factory already enforces this
  (`planner_name="roboplan"` requires `world_backend="roboplan"`); the
  spec sanctions it rather than treating it as a leak.

This dissolves the tension: roboplan owning its world is correct for a
compiled planner; the agnostic planners stay backend-free; both ingest the
same scene package (§1), so nothing about coupling forces a private scene.

---

## 4. Reachability — the deliverable

### What it is

A capability map for each G1 arm: a discretized record, in a
gravity-aligned, heading-quotiented **pelvis frame**, of which end-effector
poses are reachable and how dexterously. The pelvis frame is valid because
the WBC provides a true SE(2) base (heading-free queries = "reachable after
turning in place").

It is RM4D-style but **5D**: an explicit in-plane wrist dimension, because
the G1's ±92.5° wrist yaw breaks RM4D's 4D collapse. Measured, IK-verified:
false-positive rate 4.9% (5D) vs 13.9% (4D marginal) at equal recall.

### Where it sits — answers "must not need mujoco"

Reachability has two phases, and only one of them is offline:

- **Query** (runtime: "is pose T reachable?") is a pure array lookup into
  the saved map. **Zero backend dependency, zero mujoco** — this is the
  part that "sits high on the planning level," and Mustafa is right that
  it must be backend-free. It already is.
- **Construction** (offline, ~once per robot, minutes) needs FK +
  self-collision sampling. **DECISION:** sample through the `WorldSpec`
  interface (`get_ee_pose` + collision), so a map can be built on *any*
  backend, not bound to mujoco. Construction is a one-time offline cost, so
  the Python-per-query speed ceiling is acceptable in exchange for backend
  independence. (Today it calls `mujoco.MjSpec` directly; this is the
  refactor that lifts it above the backend.)

### Artifacts (what "done" looks like)

- `g1_{left,right}_capability.npz` maps (built offline, not committed).
- The standard reachability plots (green=reachable → red, with gradient) —
  the primary deliverable for review.
- An IK-verified evaluation report (TPR/FPR vs an oracle).
- A one-shot inspection viewer (workspace cloud + slices + drag-to-reach)
  — analysis tool, not a production surface.

### Integration

The map is an **instant feasibility oracle**: a planner or a visualizer can
ask "reachable here?" in microseconds instead of running IK. Two near-term
uses: gate/seed planning targets, and (later) propose where the base should
stand so a target becomes reachable (stance selection).

---

## 5. Ownership — stop crowding the kitchen

The overlaps surfaced because three efforts ran in parallel without an
agreed split. Proposed lanes, matching where each is already strong:

- **roboplan + pink** — fast fixed-base single-arm planning + IK. Mature,
  compiled, the right default for stationary manipulators.
- **MujocoWorld + scene/entity contract + reachability** — the
  humanoid/mobile-manipulator lane roboplan structurally can't serve today
  (floating base, dual-arm, shared sim MJCF), plus the shared scene layer.
- **Shared, owned here, serves everyone** — the `SceneObject` contract and
  the cooked scene package: the ingestion format every backend and the
  visualizer consume.

---

## 6. Landing plan

1. **Scene-entity contract** (#2479) — lands experimental; the wire format
   stabilizes after the §2 convergence is agreed.
2. **SceneObject convergence** (§2) — agreed with the planning/roboplan
   owners, then applied across backends.
3. **MujocoWorld + G1 reachability** — the deliverable; query layer is
   backend-free, construction samples through `WorldSpec`.
4. **Scene cooking pipeline** — produces the packages §1 depends on.

Babylon viewer and Rust lidar consumer remain on the experimental branch;
Linux gets demo parity through the native MuJoCo viewer.

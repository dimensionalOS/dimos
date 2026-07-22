---
title: "The Planning Stack"
description: "How a pose target becomes motion — and where to plug in your own planner, IK solver, or learned policy."
---

Ask the arm to move its gripper somewhere, and three things happen in order:

```
"put the gripper here"
        │
   IK solver          which joint angles reach that pose?
        │
   motion planner     which path gets there without hitting anything?
        │
   trajectory task    stream it to the motors, 100 times a second
```

The first two live in `ManipulationModule` — that is the planning stack. The third lives in the `ControlCoordinator` and you will rarely touch it.

This layer is where your time goes. It is deliberately pluggable: the planner, the IK solver, and the collision world are all protocols you can swap — and a learned policy can replace the whole thing.

## Your first plan

Start the planner and a mock arm, then drive it from Python:

```bash
dimos run xarm7-planner-coordinator            # planner + coordinator + Meshcat
python -m dimos.manipulation.planning.examples.manipulation_client
```

```python skip
joints()            # where is the arm?
plan([0.1] * 7)     # plan a collision-free path to these joint angles
preview()           # ghost animation in Meshcat — nothing moves yet
execute()           # now it moves
```

That `plan → preview → execute` rhythm is the whole API in miniature. Preview is cheap and execute is explicit, so you always see a motion before the robot performs it.

## The API

Everything below is an RPC on `ManipulationModule`. The building blocks:

| Call | What it does |
|------|--------------|
| `solve_ik(pose)` | Pose in, `IKResult` out. No planning, nothing moves. |
| `plan_to_pose(pose)` / `plan_to_joints(joints)` | Solve, then plan a collision-free path. Stores it. |
| `preview_path()` | Animate the stored path in the visualizer. |
| `execute()` | Send the stored path to the coordinator for real. |
| `add_obstacle(name, pose, shape, dimensions)` | Box, sphere, cylinder, or mesh the planner must avoid. |
| `is_collision_free(joints)` | Check a configuration without planning. |

And the one-shot skills that agents call — `move_to_pose(x, y, z)`, `move_to_joints("0.1, -0.5, ...")`, `go_home()`, `open_gripper()` — each wrap plan-and-execute into a single step.

Poses land in IK, IK lands in the planner, and the planner only ever talks to the world through `WorldSpec` — which is what makes the backends swappable.

## Picking backends

Three independent choices, set on `ManipulationModuleConfig` or with `-o` overrides:

```bash
dimos run xarm7-planner-coordinator \
  -o manipulationmodule.world_backend=drake \
  -o manipulationmodule.planner_name=rrt_connect \
  -o manipulationmodule.kinematics.backend=pink
```

| Choice | Options | Default |
|--------|---------|---------|
| `world_backend` — collision & FK | `drake`, `roboplan` | `drake` |
| `planner_name` — path search | `rrt_connect`, `roboplan` | `rrt_connect` |
| `kinematics.backend` — IK | `pink`, `jacobian`, `drake_optimization` | `pink` |

Two rules, enforced at startup rather than at your first plan: the `roboplan` planner needs the `roboplan` world, and `drake_optimization` IK needs the `drake` world. Everything else combines freely.

If you are not sure, the defaults are right: Drake world, RRT-Connect, Pink IK.

## Plugging in your own

The three protocols live in `dimos/manipulation/planning/spec/protocols.py`, and they are duck-typed — no base class, just the methods.

A planner is one method:

```python skip
class MyPlanner:
    def plan_joint_path(self, world, robot_id, start, goal, timeout=10.0) -> PlanningResult:
        ...   # talk to the world only via WorldSpec: check_edge_collision_free, joint limits, FK
    def get_name(self) -> str:
        return "MyPlanner"
```

An IK solver is one method too — `solve(world, robot_id, target_pose, seed, ...) -> IKResult`.

Wiring it up is a two-line change in `dimos/manipulation/planning/factory.py`: add your name to the `PlannerName` (or `KinematicsName`) literal, and add a branch in `create_planner` (or `create_kinematics`) that constructs your class. There is no plugin registry to learn — the factory is the registry.

Stay on `WorldSpec` methods inside your implementation and it will work with every world backend, current and future. `RRTConnectPlanner` (`planning/planners/rrt_planner.py`) is a readable reference — pure Python, backend-agnostic.

One naming trap: `PinocchioIK` exists in the kinematics folder but is **not** a planning backend — it is the fast in-loop IK used by the teleop and servo control tasks. You cannot select it via `kinematics.backend`.

## Where learned policies fit

A learned policy does not extend the planner — it **replaces** it. Both are just producers of joint targets, and the coordinator accepts either:

```
planner  ── JointTrajectory ──▶  trajectory task ─┐
                                                  ├─▶ arm
policy   ── joint commands  ──▶  servo task ──────┘
```

`LeRobotPolicyModule` streams `joint_command` at the policy's rate into a coordinator servo task — no IK, no collision world, no planner in the loop. Which path to use is a per-task decision: structured, obstacle-aware motion suits planning; contact-rich or hard-to-model skills suit a policy trained from demonstrations.

Collecting those demonstrations and training that policy is [the learning loop](/docs/capabilities/manipulation/learning.md).

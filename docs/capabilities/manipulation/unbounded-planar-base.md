# Unbounded Planar-Base Planning

**Status:** Implemented
**Stack:** follow-up to PR #3784

R1 Pro mobile manipulation uses the same canonical joint-space semantics as every other robot. The planar base contributes two unbounded prismatic coordinates and one continuous revolute coordinate:

```text
URDF                         prepared joint space

base/x   prismatic, no bounds  ───────► LINE
base/y   prismatic, no bounds  ───────► LINE
base/yaw continuous            ───────► CIRCLE
```

`PlanarBaseDefinition` owns the generated joint names and their velocity and acceleration limits. It does not define a workspace. The final materialized URDF is compiled once by `prepare_robot_model()`, and all planning adapters consume that prepared description and its ordered `JointSpace`.

## Query-local planning domains

An unbounded model does not imply unbounded sampling. For each selected-joint request, dimOS builds a finite domain:

- `INTERVAL`: use the physical lower and upper limits.
- `LINE`: span the request start and goal, then add a margin on each side.
- `CIRCLE`: sample the canonical interval `[-pi, pi]`.

The shared RRT begins with a one-metre line margin. If the request is blocked and budget remains, it retries with 2, 4, 8, then 16 metres. Attempts share the request deadline and total node budget. The domain never mutates the robot model.

Distance, interpolation, edge subdivision, simplification, and reported path length use the same metric: shortest circular deltas divided by each coordinate's velocity maximum, followed by L2 distance. A final circular path is lifted before trajectory parametrization so a short crossing at `pi` does not become an almost-full rotation.

## Backend behavior

| Backend | Canonical adaptation |
|---|---|
| Pink / Pinocchio | Maps every circle scalar to cosine/sine, repairs every line coordinate to infinite native limits, preserves locked-joint constraints, and uses the shared finite retry domain. |
| Drake | Keeps native scalar coordinates and takes physical limits and finite request domains from the prepared joint space. |
| RoboPlan | Uses native planning for interval-only selections. A selection containing a line or circle uses dimOS RRT with RoboPlan collision and FK queries. Cartesian waypoint planning remains interval-only. RoboPlan itself is unchanged. |
| Viser | Renders interval sliders, unbounded line number inputs, and wrapped circle sliders from coordinate topology. |
| Trajectory | RoboPlan TOPP-RA expands continuous scalar inputs to cosine/sine coordinates, then collapses and lifts its output back into a continuous canonical path. Unbounded translations pass through unchanged. |

Base execution remains disabled on the R1 Pro blueprint until a feedback-controlled base trajectory executor exists. That safety check is tied to the planar-base controller capability, not to circular topology; a continuous arm joint is not rejected merely because it is periodic.

## R1 Pro manual test

Run the fake-hardware blueprint and open the Viser URL printed in the log:

```bash
uv sync --extra all
dimos run r1pro-planner-coordinator
```

| Check | Action | Expected result |
|---|---|---|
| Positive line coordinate | Select `moving_base`, set `base/x` to `6.0`, plan | Planning succeeds beyond the former +5 m bound without rebuilding the world. |
| Negative line coordinate | Set `base/y` to `-6.0`, plan | Planning succeeds with the same prepared model. |
| Circle crossing | Start `base/yaw` near `3.04`, target `-3.04`, plan | Preview follows the short crossing, not an almost-full turn. |
| Whole-body target | Select `left_arm`, `torso`, and `moving_base`; move the left TCP target; plan | Pink solves one selected goal; shared RRT produces a collision-checked joint path. |
| Locked coordinates | Repeat with only `left_arm` selected | Torso, right arm, and base remain fixed. |
| Cartesian guard | Choose Cartesian mode with `moving_base` selected | The request reports that Cartesian planning requires interval-only coordinates. |
| Preview safety | Preview a base plan, then execute | Preview works; execution reports the missing feedback base-controller capability. |

To set the wrapped-yaw start precisely, use `set_init_joints()` in `dimos shell`, preserve every configured joint name and non-base position, change only `base/yaw`, then choose the **Init** preset in Viser.

For an obstructed scene, follow logs with `dimos log -f`. `RRT planning-domain attempt` should show margins of 1, 2, 4, 8, and 16 metres as needed, no more than 1,000 nodes per attempt, and no more than 5,000 nodes for the request.

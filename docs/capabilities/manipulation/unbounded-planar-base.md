# Unbounded Planar-Base Planning Handoff

**Status:** Implemented
**Version:** 2 (2026-08-31)
**Depends on:** PR #3784

This document specifies the stacked follow-up to PR #3784. That PR deliberately
uses finite `workspace_lower` and `workspace_upper` values so the existing
planners work end to end. Those values are planner search bounds, not physical
stops. The follow-up removes permanent planar-base position bounds while keeping
each planning query finite and deterministic.

## Target contract

`PlanarBaseDefinition` continues to define the generated root and ordered
`x`, `y`, and `yaw` coordinates, plus their velocity and acceleration limits.
It no longer contains `workspace_lower` or `workspace_upper`.

- `x` and `y` are globally unbounded translations.
- `yaw` is continuous modulo $2\pi$.
- Equivalent yaw values represent the same configuration.
- Planning and interpolation use the shortest wrapped yaw displacement.
- Hardware execution remains out of scope until a feedback-controlled base
  trajectory executor exists.

The portable model cannot rely on parser defaults for this contract. With the
currently installed dependencies, a URDF prismatic joint without lower/upper
limits parses as unbounded in Drake but as zero-range in Pinocchio. A continuous
URDF joint is one scalar coordinate in Drake but a two-value cosine/sine
configuration in Pinocchio. RoboPlan currently rejects non-finite model limits.
The backend adapters therefore need explicit unbounded planar-coordinate
handling instead of treating raw parser limits as the public contract.

## Per-request planning domain

The backend-independent selected-joint planning space creates a finite domain
for each request. RoboPlan-backed worlds route selections containing planar-base
joints to this shared RRT implementation. The request domain never mutates the
world model or cached global joint limits.

For planar translation:

1. Start with the axis-aligned box spanning the start and solved goal base
   positions.
2. Expand each x/y side by one metre.
3. If a bounded search reports no solution while request budget remains, double
   the margin and retry.
4. All attempts share the original planning deadline and node budget. Timeout
   and iteration reporting remain totals for the request, not per-attempt values.

This makes every individual search finite while allowing any global start or
goal coordinate and progressively larger obstacle detours.

## Coordinate metric

Replace raw mixed-unit L2 operations in planning with a common metric:

1. Compute x/y and revolute-joint deltas normally.
2. Compute yaw with the shortest modulo-$2\pi$ displacement.
3. Divide every delta by that coordinate's positive velocity limit.
4. Apply L2 distance in the normalized coordinates.

Use the same metric for nearest-node selection, steering/interpolation step
sizing, edge subdivision, shortcutting, and reported path length. This prevents
one metre and one radian from receiving an accidental identical weight.

## Backend work

- **Portable model:** emit semantic unbounded x/y and continuous yaw without
  presenting temporary search bounds as physical URDF stops.
- **Pinocchio/Pink:** map continuous yaw's two-value configuration to one public
  scalar yaw coordinate, repair unbounded prismatic limits after parsing, and
  preserve locked-joint QP constraints.
- **Drake:** retain native infinite translation/continuous-yaw semantics while
  applying the per-request domain only to sampling and planning constraints.
- **RoboPlan:** keep native planning for bounded non-planar selections and route
  selections containing planar coordinates through the shared DimOS RRT. Convert
  scalar yaw to RoboPlan's cosine/sine scene representation where needed.
- **Viser:** expose unbounded numeric x/y inputs and a yaw slider wrapped to
  $[-\pi, \pi]$; planner domains are never shown as physical control limits.
- **Trajectory generation:** use the simple trapezoid parametrizer for the R1 Pro
  blueprint and apply the planar base's declared velocity and acceleration limits.

## Acceptance criteria

- The same robot model can start and finish beyond the old R1 Pro $\pm5$ metre
  workspace without rebuilding or mutating the world.
- A blocked direct path succeeds after at least one query-domain expansion.
- Crossing yaw from $+\pi-\epsilon$ to $-\pi+\epsilon$ follows the short rotation.
- Planar selections on Drake and RoboPlan worlds use the same initial/expanded
  domains and normalized metric implementation.
- Pink and Drake FK agree for arbitrary x/y and equivalent wrapped yaw values.
- Arm-only planning still locks torso and base coordinates.
- Plans containing base joints remain preview-only until base execution support
  is implemented separately.

## R1 Pro manual test

Run the blueprint and use the Viser URL printed in its log:

```bash
uv sync --extra all
dimos run r1pro-planner-coordinator
```

The blueprint uses fake hardware. It is safe for planning and preview, but base
execution is intentionally rejected.

| Check | Viser action | Expected result |
|---|---|---|
| Unbounded translation | Select `moving_base`, set `base/x` to `6.0`, and plan | Planning succeeds even though the goal is beyond the former +5 m workspace |
| Negative unbounded translation | Set `base/y` to `-6.0`, and plan again | Planning succeeds without rebuilding the world |
| Wrapped yaw | Set yaw near `3.04`, plan, then use a start near `3.04` and target near `-3.04` | Preview uses the short rotation across the wrap boundary, not an almost-full turn |
| Mobile manipulation | Select `left_arm`, `torso`, and `moving_base`; move the left TCP target and plan | Pink solves one joint goal and shared RRT produces a collision-checked joint path |
| Cartesian guard | Choose Cartesian mode while `moving_base` is selected | The request reports that Cartesian waypoint planning does not support a moving planar base |
| Preview-only safety | Preview a base plan, then press Execute | Preview works; execution is rejected with the feedback-base-controller message |

For the wrapped-yaw check, set the planning start through the manipulation
module's `set_init_joints()` RPC in `dimos shell`, then choose the **Init** preset
in Viser. Preserve every configured joint name and current non-base position;
change only `base/yaw`.

While testing an obstructed scene, inspect `dimos log -f` for
`RRT planning-domain attempt`. The margins should progress through 1, 2, 4, 8,
and 16 metres as needed, with no more than 1,000 nodes per attempt and 5,000
nodes for the request.

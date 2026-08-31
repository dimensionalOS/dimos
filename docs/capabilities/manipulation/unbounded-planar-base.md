# Unbounded Planar-Base Planning Handoff

**Status:** Follow-up contract  
**Version:** 1 (2026-08-31)  
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

Introduce one backend-independent selected-joint planning-domain value. It is
created for each plan request and passed to RRT and RoboPlan; it must not mutate
the world model or cached global joint limits.

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
- **RoboPlan:** remove the requirement that global model limits are finite for
  planar coordinates and apply the shared finite domain to each native planning
  attempt.
- **Viser:** do not use temporary planner bounds as physical slider stops; expose
  a movable local viewport for x/y and a wrapped yaw control.

## Acceptance criteria

- The same robot model can start and finish beyond the old R1 Pro $\pm5$ metre
  workspace without rebuilding or mutating the world.
- A blocked direct path succeeds after at least one query-domain expansion.
- Crossing yaw from $+\pi-\epsilon$ to $-\pi+\epsilon$ follows the short rotation.
- RRT and RoboPlan use identical initial/expanded domains and normalized metric
  semantics.
- Pink and Drake FK agree for arbitrary x/y and equivalent wrapped yaw values.
- Arm-only planning still locks torso and base coordinates.
- Plans containing base joints remain preview-only until base execution support
  is implemented separately.

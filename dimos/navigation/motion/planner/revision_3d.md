# 3D era: conclusions ahead of time

Discussion notes, not a spec. Nothing here is in scope for [revision.md](revision.md);
everything in revision.md is designed to survive this.

## Principles

- The search stays an SE(2) lattice **on the support manifold**: states are
  (u, v, yaw) on the walkable surface; z, pitch, roll are DERIVED from the
  surface, never searched. Full SE(3) is out.
- 3D enters through the body (ObstacleModel, Embodiment), not the search.
- No slack anywhere: an "obstacle" that is actually the next stair tread is a
  frame error, and frame errors are fixed, not tolerated.

## Division of labor

- **MLS owns the manifold**: surface extraction, multi-level atlas, topology
  (which stairs to take). It already does a form of manifold discovery.
- **The local planner gets one chart**: the patch its 5 m horizon rides,
  disambiguated by the carrot when levels stack. It never sees the atlas.

## Local planner changes (bounded, all in layers already refactored)

1. ObstacleModel gains orientation: the band slices along the local surface
   normal (chart input at the seam), not gravity.
2. The lattice lays on the chart in (u, v); absolute anchoring carries over
   verbatim in chart coordinates.
3. Envelope and steppable gain slope conditioning (a pitch column later).

## Stairs without assuming collision

- **Per-station slicing**: every search state evaluates obstacles in its OWN
  support frame (per-cell ground height + normal join the clearance
  precompute). A tread is an obstacle from the floor and support when stood
  on — both exact, no contradiction.
- **Plane fit at body scale**: support per station = plane over a
  footprint-sized window (four feet span ~3 treads), which low-passes steps
  into a ramp — the pitch profile interpolates over ~a body length. "Slowly
  angling" is geometry, not a scripted behavior.
- **Steppable classifies locally**: < steppable above the LOCAL plane =
  negotiable terrain; steppable..height = obstacle; overhead checked along
  the station normal (pitched body under a stairwell ceiling: nose clears,
  rump does not).
- **Pitch-rate as an edge constraint**, priced like yaw_w; mostly free after
  the plane fit, catches mis-stitched charts.

## Gold economics

- Exhaustive gold stays affordable only on **single-chart worlds** (a ramp, one
  flight): heightfield + normal per cell, still 2D-grid-shaped. The atlas-level
  problem (volumetric precompute, adjacency-as-physics) is never gold's job.
- 3D edge feasibility depends on the PLANT (can the body climb this rise?), so
  gold inherits a measured capability model — truth stops being purely
  geometric. Keep the capability table small and measured in the fitted sim.
- **Witness paths** for whole-route truth: a recorded traversal is a
  feasibility certificate + cost bound. Verifying a path is cheap in any
  dimension; finding the optimum is what explodes. Recorded worlds score as
  "locally verified feasible, within X% of the witness".

## Dynamics / the seam

- The core planner stays deterministic-on-a-snapshot forever. Every dynamic-
  world defect found so far (breathing crop, clock replans, MLS republish,
  stamp dialects, map age, grid phase, churn) lived at the adapter seam or
  below — none needed time in the search.
- Dynamic robustness is scored in the CONTROL battery as a handful of seam
  scenarios driven by **recorded churn traces, replayed deterministically** —
  real dynamics, deterministic tests, zero new machinery.

## Prerequisites (in order)

1. **MLS stability + honest stamps** — it becomes the owner of all topological
   truth; door2 shows it flapping on flat ground today. Foundation, not polish.
2. **Plant capability**: does the walking policy climb stairs at all? Measure
   in the fitted sim before any geometry work.
3. Recorded-worlds pipeline as the 3D world supply (extractor already keeps
   full 3D voxels; skip the band-rect flattening, keep terrain).

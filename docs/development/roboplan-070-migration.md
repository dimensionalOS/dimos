# RoboPlan 0.7 migration

The migration uses RoboPlan 0.7.0, Pinocchio 4.1.0, Coal 3.0.3 and
cmeel-tinyxml2 11.0.0. Scene construction loads a URDF description, then imports
SRDF and the prepared model's velocity and acceleration limits through the
supported YAML interface. This avoids infinite acceleration limits when the
URDF parser ignores nonstandard acceleration attributes.

## Point clouds

The old `point_cloud_self_filter.py` utility and its blueprint are removed.
`ManipulationModule` optionally filters camera returns before mapping, calling
`RoboPlanWorld.robot_body_mask` and upstream `RobotBodyFilter.computeMask` with
`Narrowphase`. It reuses the prepared Scene, model limits, base pose and full-q
conversion; no second URDF parser, trimesh geometry tests, primitive sampling or
collision-link geometry cache remains.

Canonical JointState messages keep their original timestamps in a bounded buffer.
Each capture matches state and sensor-to-world TF within the configured tolerance;
missing, malformed or stale alignment drops the capture. There is no latest-state
fallback. The bidirectional TF port receives camera transforms as well as
publishing planning transforms. Continuous, mimic and supported prepared planar-base models reuse the
planning model's native configuration conversion. Raw floating/planar URDF joints
remain subject to the existing prepared-model validation rules.

The native filter is serialized with scene queries/updates. A consumer lock keeps
capture order and output publication consistent. Original sensor coordinates,
intensities and ancillary fields survive in `filtered_pointcloud`; the grasp
blueprint connects that output to the mapper's lidar input. Only the wrist camera's
attachment link needs extra TF publication.

Solid-volume samples, current/previous volume clearing, and the unused Python/Rust
clear-mask transport are removed. This intentionally changes map cleanup: the
filter excludes observed surface returns before insertion, and the mapper retains
ordinary ray-tracing cleanup. Existing deep occupied cells inside triangle meshes
are not explicitly erased. Coal triangle-mesh Narrowphase classifies proximity to
surfaces, so this follows the user-approved surface filtering semantics. PaddedObb
remains disabled because its box corners can erase nearby real obstacles.

## Planning contexts

Each dimOS scratch context owns a native `SceneContext` for FK, collision queries
and partial-to-full configuration conversion. Geometry changes recreate stale
contexts; placement-only updates keep them. These queries no longer change the
Scene's current configuration. Contexts cannot be reused across worlds.

The scene lock still excludes geometry updates while queries execute. Native
planners, TOPPRA and the Python Jacobian/path bindings retain their existing
locks. This layer improves scratch isolation; it does not claim parallel query
throughput while those consistency locks remain.

## Validation

CPU validation on Linux x86_64 / Python 3.12.14 / AMD Ryzen 7 8700F:

- 125 CPU tests: planning, native RRT/Cartesian/TOPPRA, manipulation-module behavior,
  direct point-cloud filtering, blueprint registry and documentation branding.
- Native surface-filter tests cover STL and primitives, capture-time state/TF,
  fields, nonzero base poses, continuous/mimic/fixed joints and prepared planar bases.
- Blueprint registry and wiring checks, focused mypy and changed-file pre-commit.
- 88 Rust mapper tests after removing its obsolete self-filter clear-mask pipeline.

A fixed-seed 100,000-point sphere fixture compares filtering against main 943ce13c.
Both versions removed the same 69 points over 50 iterations after five warmups.
The old path also builds solid clear masks, so timings measure different cleanup
semantics. The observed p50/p95 was 2.08/2.66 ms on main and 2.79/3.69 ms for the
direct native integration. No speedup claim is made. Evidence is kept in the
isolated validation directory.

Lock generation and pre-commit used the repository-supported uv 0.9.25. uv
0.12.21 rejected a fresh universal resolution because the existing a750-control
constraint admits Python 3.10/3.11 while its available wheel is cp312-only.
No unrelated constraint was changed.

A bounded synthetic MuJoCo depth-frame smoke used llvmpipe software rendering:
576 robot surface returns were removed and all 175 nearby obstacle returns retained,
with colors intact. This tests the camera conversion and native filtering path.
After the room archive was fetched and its SHA256 verified, a bounded CPU launch
of the actual `xarm-grasp` MuJoCo blueprint passed. The transport observer received
247 raw captures, 228 filtered captures and 228 mapper outputs; each matched
capture removed 263 points, and the map contained up to 1,338 occupied points.
The camera TF reached the planning consumer and viser returned HTTP 200 on port
8095. Early captures were dropped while consumers started. No grasp or motion
command was issued. System configurators were skipped during the successful
validation run; no driver or security setting was changed.

Hardware, GPU execution, Windows, macOS, ARM and the full self-hosted suite were not
run locally. Selected native tests marked self_hosted were explicitly run on CPU.

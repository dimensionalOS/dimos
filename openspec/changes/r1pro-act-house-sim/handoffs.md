# R1Pro ACT / house simulation handoff

Date: 2026-09-10
Branch: `feat/r1pro-act-sim`
Worktree: `/home/mustafa/dimos-wt/r1pro-act-sim`
Base: `hackmit/t7-resume` at `0b816edf49`.

## User intent and boundaries

The user wants mobile household manipulation, possibly a virtual mobile OpenYAM
rig, preferably R1Pro using its existing mobile base. The immediate request is to
verify ACT deployment in R1Pro simulation before extending to learned grabbing.
They requested pushing the local OpenYAM commits as well.

Preserve the user's main checkout (`cc/feat/unbounded-planar-base`) and the live
OpenYAM simulation. All new code and runs are isolated in this worktree.

## Findings

- `feat/r1pro-mobile-manip` and current planar-base work include R1Pro planning
  previews with mock whole-body state; these do not prove contact-based ACT sim.
- R1Pro hardware blueprints use real ROS/Zenoh adapters and must not be launched
  under the assumption that they are MuJoCo blueprints.
- The completed OpenYAM branch supplies the isolated LeRobot rollout runtime,
  generic MuJoCo whole-body adapter, and explicit simulator joint mappings.
- Pinned Galaxea URDF compiles in MuJoCo 3.10.0. Its grippers have zero limits, so
  the first profile deliberately controls only the 18 upper-body joints.
- HSSD `102344115` is cached under the main checkout's `data/scene_packages`.
  Its eight dynamic props are listed in metadata separately from room MJCF and
  must be composed too. Initial composition with R1Pro has 101 qpos, 18 actuators,
  3163 geoms, eight free props, and no initial contacts.

## Implemented

See `dimos/robot/galaxea/r1pro/ACT_SIM.md` for commands and limitations.

- Versioned, explicit R1Pro simulation IO profile and isolated LeRobot binding.
- Local MJCF builder using pinned R1Pro assets, position actuators, gravity
  compensation, fixed base/grippers, native viewer, and optional house package.
- Diagnostic ACT checkpoint generator and bounded real-runtime rollout command.
- Physics and checkpoint-compatibility checks. Model and checkpoint artifacts
  remain gitignored under `recordings/r1pro-act-check/`.

## Verification so far

- Three asset-backed physics integration tests passed: actuator contract, stable
  pose/wrist motion, and scene/free-prop isolation from robot action indices.
- Six registry generation tests passed; no registry changes needed.
- Host mypy passed for five changed production files, including the engine.
- GLFW camera rendering verified visually; `overview.png` shows the full R1Pro.
- Seven isolated LeRobot contract tests passed, including rejecting wrong state
  and action widths. Isolated mypy passed for runtime and checkpoint generator.
- Nine simulator timing/reset/camera tests passed. The test invocation selected
  the MuJoCo-marked tests; nine unrelated non-MuJoCo tests were deselected.
- Native GLFW + CUDA studio rollout passed: 21 accepted chunks, wrists at
  +/- 0.05 rad, stop in 5.33 ms, max wrist error 1.45e-8 rad.
- Native GLFW + CUDA HSSD house rollout passed after the scheduling fix: 21
  accepted chunks in six seconds, stop in 6.45 ms, max wrist error 7.46e-10 rad,
  max subsequent half-second joint drift 1.70e-8 rad, no new chunks after stop.
- Evidence: `recordings/r1pro-act-check/native-studio/result.json` and
  `recordings/r1pro-act-check/native-house/result.json`; their logs and the
  diagnostic checkpoint remain local and ignored.

## Simulator timing issue found and fixed

The first six-second house rollout accepted 20 chunks but only reached +/- 0.0382
rad, outside the unchanged 0.01 rad error threshold. Raw house physics simulated
2 seconds in 0.317 wall seconds, so physics compute was not the bottleneck.
The engine previously performed one physics step per loop and synchronized the
viewer on every step, letting display/render time slow simulated motion.

The engine now schedules fixed physics steps using monotonic deadlines, catches
up after rendering, and limits viewer synchronization to 60 Hz. Recovery after a
long pause is bounded to 64 steps, and stop interrupts the wait/batch. Renderer
cleanup now runs in a finally block. The repeat house test met the original
six-second and error limits; no tolerance relaxation was needed.

This is a general engine change in this isolated branch, covered by timing,
reset and multi-camera regression tests. It has not been installed into the
user's running OpenYAM checkout. Do not infer that it fixes the OpenYAM ACT
checkpoint's earlier 0/10 placement score without a separate evaluation.

## Remaining limitations

- No R1Pro trained grasping checkpoint or validated gripper dynamics are included.
- The mobile base is still parked. House composition does not establish collision
  avoidance, navigation, manipulation reachability, or object transport.
- Existing Python 3.12 shared-memory resource-tracker KeyError messages appear
  during teardown. The rollout exits successfully and workers shut down; this
  separate bookkeeping issue was not changed as part of the ACT integration.
  Both diagnostic shared-memory namespaces were checked after shutdown: zero
  segments remained. The original OpenYAM process (PID 3599494) remained alive.
- Initial runtime-declaration and diagnostic lookup failures were corrected;
  do not use the earlier failed logs as the final acceptance result.

## Push status

No push completed. The automatic approval reviewer rejected pushing
`hackmit/t7-resume` to `git@github.com:dimensionalOS/dimos.git`, citing exact
payload/destination authorization and repository trust. A question explicitly
naming that branch, destination and commit `0b816edf49` is pending with the user.
Do not bypass the rejection. No recordings, weights, logs, or credentials were
included in the proposed push.

The exact desired house and locomanipulation branch have also been requested;
HSSD and the locally available R1Pro branches are the working assumptions.

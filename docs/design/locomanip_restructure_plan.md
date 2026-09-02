# Loco-Manipulation Restructure — Execution Plan

**Status**: active · **Branch**: `manip/locomanip-restructure` (this branch) · **Founded**: 2026-09-02
**Human dashboard**: the "Locomanip Merge Ladder" artifact (Mustafa has the link; status chips there mirror this file's Execution Log)
**Read-only reference branches**: `origin/manip/mobile-manipulation-demo-aug18` (the demo, 48 commits — quarry, never merge), `origin/feat/r1pro-mobile-manip` (CC's planar base — moving target, re-audit at Wave 4), `origin/pim/feat/g1-quest-teleop-stacked` (Pim's G1 teleop + locked-joint QP fix + canonical reachability).

## Mission (Mustafa, verbatim)

> I had made hacks to simplify the mobile manipulation implementation, with a basic state machine. The hack is essentially: use perception to identify position → create a reachability map over the target location → generate a 2D PoseStamped path (with orientation) → follow the path and reach the location → then make the current position of base_link the new home position, and perform manipulation.
>
> We need a holistic approach here, with a floating base link being the right design architecture, as it is better generalizable. The overall state machine and implementation can remain the same. The target: restructure mobile manipulation to work from manipulation's planning perspective.

Translation into this plan: the demo pipeline's *shape* survives (perceive → stance from reachability → approach → manipulate). The re-homing mechanics (the "latch") are never ported — the base becomes part of the planning model (planar base: `base/x, base/y, base/yaw` as ordinary joints), so planning always knows where the base is. End state: the manipulation planner can treat base + arm as one kinematic chain; the approach path itself becomes a planning artifact.

## Execution discipline (read before every session)

1. **One commit per rung, in rung order.** Commit title: `rung(R<N>): <imperative summary>`. Body: what was done, deviations from this spec, source SHAs consulted. This is non-negotiable — the commits ARE the future PR ladder; each will be cherry-picked onto main as its own ≤10-file PR.
2. **Every commit green**: the scoped test suite for the touched area + `mypy` + `ruff` pass before committing. New behavior gets a test in the same commit.
3. **Re-express, never cherry-pick through conflicts.** Main refactored under the demo branch (single-robot model #3420/#3431, RPC simplify #3447, servo_task deleted by #3610, Rust camera #3746, pick&place rewrite #3715). Read the demo commit with `git show`, then write the change fresh against this branch. Take main's `uv.lock` always.
4. **Architecture rules** (house doctrine): one arbitration authority (ControlCoordinator) — command sources are coordinator tasks with priorities, no topic muxes, no side-channels. Planning goes through ManipulationModule/RoboPlan/Pink. World-frame targets only; no re-zeroing, no hardcoded base offsets. Sim-privileged data enters through the same typed inputs perception uses. Failures loud and recoverable — nothing may refuse work silently.
5. **On surprise or design smell**: stop the rung, append an Execution Log entry (facts + proposal), and report. Do not improvise around a ratified decision. A rung that balloons past ~10 files gets a log note proposing a split — fine on this branch, decided at extraction time.
6. **Session shape**: one wave per session recommended. Launcher: "Read docs/design/locomanip_restructure_plan.md. Execute Wave N per the discipline section. Append to the Execution Log."
7. `SKIP=lfs_check` on commits is permitted only with a log note (fix lands in R3).

## Ratified decisions (defaults; only Mustafa overturns)

| ID | Decision |
|----|----------|
| D-1 | Base mechanism = CC's planar base (three ordinary joints prepended to the model). The demo's latch is never ported. |
| D-2 | Hardware base-pose source = **pelvis-anchored explicit mode** (world = ground under the pelvis at task start, as a named convention with validation), behind a swappable pose-source port. Real odometry (Point-LIO / cuVSLAM) later. Sim uses ground truth. |
| D-3 | Base-velocity arbitration = **virtual base joints in the coordinator** (`make_twist_base_joints("g1")` precedent): teleop velocity task outranks the last-mile servo task; the arbitrated value feeds the walking policy's velocity command. No MovementManager. |
| D-4 | IK solver tuning (damping/lm_damping/tolerances) = **G1 per-robot config overrides**. Global Pink defaults untouched. |
| D-5 | Reachability package ownership = **Pim's** (`pim/feat/g1-reachability` canonical). This branch consumes the already-present lifted copy but claims no ownership; R19 generalizes the stance API around it and flags reconciliation. G1 model config: written fresh here; reconcile with Pim's before extraction. |
| D-6 | Fault policy = execution FAULT **auto-clears on the next plan request with a loud log line**; manual `reset()` RPC also exists. |
| D-7 | The water-demo blueprint is **not** a main deliverable. A demo/E2E blueprint may exist on this branch for verification (R20 acceptance); general capability lives in the manip variant + skills. |
| D-8 | TCP tool frames = Wave 5 (R23), not blocking. |
| D-9 | Pelvis height: **measure, don't assume.** Hardware once measured ~0.57 m vs the assumed 0.74 m. R16/R17 must set `base_pose.z` from measurement (or document the live source) and record the number in the log. Sim value 0.745 stands for sim. |
| D-10 | Wave 4 builds on a **labeled merge of `origin/feat/r1pro-mobile-manip`** (or its landed PR if merged by then). CC rebased that branch on 2026-09-01/02 (SHAs changed; the unbounded-planar-base follow-up doc was deleted) — re-audit its tip before the Wave 4 merge and log what changed. Expect rebase churn when his PR goes through review. |

## Rungs

### Wave 0 — unblockers (independent)

- **R1 · Preserve callable blueprint-config values.** Port demo `78ab3d748` essentially verbatim: blueprint config materialization uses explicit model values instead of `model_dump()`, so callable config (e.g. Rerun factories) survives CLI deploys. Verify main's parser still has the bug first (it did at audit: `blueprint_config/parser.py` ~:424). Acceptance: the commit's repro test + blueprint suite green. ~3 files.
- **R2 · @rpc-override guard.** A subclass overriding `start`/`stop` silently loses `@rpc`, killing workers (bit the demo twice). Add a framework-level check (metaclass / `__init_subclass__`) or lint that overriding an `@rpc` method preserves the decoration. Acceptance: guard test; existing modules unaffected. ~2 files.
- **R3 · `lfs_check` ignores untracked paths.** Today untracked local data blocks every commit. 1 file (`bin` hook).

### Wave 1 — manipulation quality (robot-agnostic)

- **R4 · `ik_posture` reference-posture seeds.** Port demo `82be7d8af` onto the current single-robot `RobotModelConfig`: opt-in `{joint: value}` overlay biasing IK seeds toward a chosen posture (G1 elbow reads ~90°-bent at joint zero; without this every solution parks there). Note CC solved the same class differently for R1 Pro (`test_r1pro_arm_ik_leaves_the_singular_zero_pose`) — coexist, don't fight. ~3 files.
- **R5 · Multi-group selection planning.** Re-express demo `3fe636240` (plan across several planning groups of one robot, e.g. both arms) on the single-model world's composite-group machinery. ~4 files.
- **R6 · Group-addressed pose skills + multi-arm `go_init`.** Re-express `03e8030da` + `874fb57db` on the simplified RPC surface (#3447): pose skills take a `group_id`; `go_init` works on multi-arm robots (and a failed safe-waypoint plan proceeds to the joint goal instead of dead-ending — document this behavior change in the commit). Replace the bare `assert config is not None` with a proper error return. ~4 files.
- **R7 · IK failure-reason reporting.** Re-express `3ff5e50e1` on main's `pink_solver` split: every failed attempt records why (QP exception / no-convergence / limits / collision); the exhaustion result carries the closest candidate's status (API change — check callers). ~3 files.
- **R8 · G1 IK tuning as per-robot overrides (D-4).** The demo's reach-edge fix (`2d9c73eb7`: damping 1e-8→1e-6, lm_damping→0.1; measured 4/8→8/8 at arm's length) lands as G1 config values with rationale comments. Global defaults untouched. ~2 files.
- **R9 · Fault recovery (D-6).** `reset()` RPC + viser Reset button (from `2fa1ede59`) + execution-FAULT auto-clear on next plan request with a loud log. Main already returns planning failures to IDLE; this closes the execution-side wedge. ~3 files.

### Wave 2 — sim & perception symmetry

- **R10 · Ground-truth body poses from MuJoCo.** Port `f23561e75` (`MujocoEngine.get_body_pose` + `@rpc get_body_poses`), rebased over main's sim timing changes (`764e4a98c`). ~3 files.
- **R11 · In-repo MJCF props merged into composed scenes.** Port `9c5e4df85` (`extra_mjcf`) + the potted-plant asset — scene props that survive scene-package re-pulls. ~3 files.
- **R12 · `SimBodyPose`.** Port `df5289550`: publish one named sim body as a world-frame `PoseStamped`, shaped identically to perception output. Imports per current perception layout. ~3 files.
- **R13 · `MarkerLatchModule`.** Port from `b988ecaa2` + the `@rpc` start/stop fix `f32e8fc85`: N consistent AprilTag sightings → one held world-frame `PoseStamped` (same contract as R12 — that symmetry is the point). Marker dictionary/length/ID are config. ~4 files.

### Wave 3 — control & hardware

- **R14 · Gravity feedforward.** Port `e867e29c5` + `d5b2a1651` + `e3b781c8f` rebased onto main's evolved `hardware/whole_body/spec.py`: RNEA holding torque for configured joints, point-mass payload hook, per-joint clamp to model effort limits, **hardware default off** until calibrated (sim scale 0.7 is sim-only; demo measured droop 3.86 cm→0.37 cm). Include the calibration procedure (scale sweep vs measured droop; harnesses exist on the demo branch as `*_harness.ignore.py`). ~6 files.
- **R15 · Camera resilience + head camera, on the Rust module.** The demo's Python camera fixes target code deleted by #3746. First **verify** the Rust RealSense module's behavior for (a) device-busy at open and (b) mid-stream `wait_for_frames` death (this silently killed multiple hardware runs); add supervisor recovery in `rust/src` if absent. Head camera: feed **live CameraInfo** into marker detection (no committed per-unit intrinsics — the demo committed one physical unit's calibration as "the" G1's); dedupe the waist-chain TF with main's `g1_tf_publisher` (#3527). ~4 files.

### Wave 4 — the loco-manip core (gate: labeled merge of CC's branch per D-10)

Before R16: `git merge origin/feat/r1pro-mobile-manip` (or main if his PR landed) as a clearly labeled merge commit; re-audit his tip (it moved since the audit — SHAs rewritten, follow-up doc deleted); log the differences.

- **R16 · G1 planar-base model config.** Fresh `manip_config` in the new idiom: `RobotModel.from_file(<LFS g1.urdf, pelvis root>).with_planar_base(G1_PLANAR_BASE)`; `base_link = planar_base_root`; 29 body + 3 base joints in `joint_names`; `left_arm`/`right_arm` groups + a `moving_base` group; fold in R4's posture + R8's overrides + the demo's collision exclusions (`d264ccc1f`). Workspace bounds office-sized. `base_pose.z` per D-9 (measured). Verify the URDF has no `planar_base_root`/`base/*` name collisions. ~3 files.
- **R17 · Pose→base-joint bridge.** Publish live `base/x, base/y, base/yaw` into the coordinator JointState from the pelvis pose source (sim: ground truth; hardware: pelvis-anchored mode per D-2), behind a swappable port. **This single rung replaces the demo's entire latch.** Acceptance = the old latch acceptance, latch-free: reach the same world point from two different stances in sim, error ≤ a few cm, no rebuild, no re-homing anywhere. ~4 files.
- **R18 · Blueprint factory + manip variant.** Public `make_g1_wbc_parts()` on the groot-WBC blueprint (no underscore imports — the demo variants imported `_backend`, `_G1GrootCoordinator`, etc.); manip variant composes: trajectory-task arm control on the #3610 base, gravity FF (R14), and **base-velocity arbitration per D-3** (teleop task > last-mile servo task over virtual base joints; arbitrated value → walking-policy velocity command). ~5 files.
- **R19 · Generalized stance selection.** Task-conditioned API under `dimos/manipulation`: input = target pose + a reach-grid artifact; output = stance `(x, y, yaw)`. Demo pour constants (`POUR_Z`, tip radians, arm side, grasp-center offset) move to per-robot/task config. Derived-artifact convention: provenance (generator version, model hash, parameters) embedded + a regeneration entry point. Consumes the reachability package copy in place; ownership stays Pim's (D-5) — flag, don't fork further. ~6 files.
- **R20 · Pour as a canonical skill.** Re-express the pour on main's canonical composition (#3715): approach → settle → act (wrist-roll pour) → home, consuming R19 stances and R12/R13 object poses. Orientation recovery is joint-space (`go_init`) — pose-target levelling fails from tipped configurations (measured 3/3 in the demo). Retires the demo script. **Acceptance = the wave gate**: full sim E2E on this branch — spawn anywhere → stance → last-mile → pour on the potted plant → home, ~cm accuracy, zero latch/re-homing code anywhere in the path. ~5 files.
- **R21 · Base-trajectory follower (flagged LATER — requires CC sync).** A tracking task for planner-emitted `(x, y, yaw)(t)` → velocity commands (candidate: Mustafa's holonomic path-following controller), an opt-in flag relaxing the planar-base execute gate in `manipulation_module`, and the first base-inclusive planning test (`moving_base` in the selection — none exists anywhere yet). This is "the approach path becomes a planning artifact" — the mission's end state. ~5 files.

### Wave 5 — frames & robustness

- **R22 · Arming safety.** `auto_arm`/`auto_dry_run` resolution out of import time (a mis-resolved environment could arm a hardware robot at launch); explicit hardware-mode assertion in the coordinator factory. ~3 files.
- **R23 · TCP tool frames (D-8).** Optional per-planning-group tool transform (`tip_link` + `tcp_pose`, z = approach convention), consumed by IK target construction and EE-pose reporting; the G1 grasp-center offset moves from stance math into model config. ~6 files.
- **R24 · Self-collision margin + start-state tolerance.** Per-model margin (or per-pair thresholds) in the RoboPlan collision check + an epsilon start-state tolerance, so a pose the planner itself just delivered can never be rejected as "in collision at start" (this refused all planning until reset during the demo). ~4 files.

## Out of scope (logged follow-ups, do not do here)

Walking velocity undershoot (~0.23 of 0.3 m/s + veer — instrument with the control benchmarking harness first; the 100→50 Hz decimation is a suspect). Rerun bridge rate-limiting for 100 Hz joint streams (shared-bridge fix, not per-blueprint). Real odometry integration (post-D-2). Capability-map inverse query (Pim's).

## Execution Log (append-only; newest first)

- 2026-09-02 · Plan founded on `manip/locomanip-restructure` off main. No rungs executed yet.

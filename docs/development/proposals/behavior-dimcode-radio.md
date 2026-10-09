# Development radio interaction through the Python SDK

This child change builds on BEHAVIOR integration PR #4342. It binds the existing
manipulation SDK to simulated R1 Pro arm/gripper execution and adds checked radio
interaction, sensor observations and terminal-outcome handling. It does not modify
the parent integration, task goal, asset dynamics or upstream toggle implementation.

The current evidence is **development validation with oracle-assisted preparation**.
It is not an autonomous agent success or a fair benchmark score. Generic runtime
isolation and cheating audit remain separate work.

## Interaction contract

Dimcode's existing Pi write/read/bash tools can write Python that connects using
`Dimos.connect(timeout=5)` and calls the Python SDK. No MCP wrapper is required.
Select the intended single development transport bus and module instance explicitly;
`Dimos.connect` has no run-ID selector. Stopping a borrowed client disconnects it;
the owner remains responsible for simulator shutdown.

The conceptual flow is task intent -> action intent -> embodied execution. Action
intent describes an end-effector pose, approach, press or gripper target rather than
raw joint angles. These contracts do not assign each whole module to one layer.

`Arm.from_app(app, group="right_arm", instance_name="ManipulationModule")`
provides encoder-derived `state()` and `pose()`. `move_pose`, `move_linear` and
`set_gripper_position` route to the selected arm and coupled gripper. Pose and
linear motion accept explicit auxiliary groups; unselected joints stay frozen.
The development radio blueprint has no locomotion controller. World-frame robot
feedback uses simulator localization/calibration assumptions, not a demonstrated
realistic localization model.

`RadioPolicy.from_app(app)` provides:

- `observe("head" | "left_wrist")`: actual RGB/depth, intrinsics and camera-to-base
  transform from one atomic sensor capture.
- `ground(observation_id, u, v)`: base-frame point from that retained sensor frame,
  or a typed error for expired/inconsistent/invalid depth. It does not infer
  grasp orientation, press normal, control identity or task success.
- Robot feedback and bounded motion operations with action IDs, cancellation and
  confirmed-stop/uncertain outcomes. Gripper command acceptance is distinct from
  measured travel or a blocked grasp.

With `bimanual=True`, `arm="right_arm"` and `policy_supervisor=True`,
`radio_blueprint` uses the checkpoint motion contract. The owner initializes the
private development scene through `RadioPolicyModule.initialize_development_scene`.
The policy's `move_checkpoint_pose` target is base-frame caller intent. The adapter
preserves that target and converts it using the same robot transform; it does not
replace it with an evaluator button coordinate. A tabletop press must explicitly
supply a contact point and outward normal. The owner checks the declared physical
surface against the radio mesh and permits only the designated leading-finger
contact corridor. This verifies admission, not sensor provenance.

The owner validates robot limits, selected/frozen groups, radio/table geometry,
start freshness, nominal and coordinator-anchored trajectory digests, stored plan
identity and measured arrival. Required torso assistance is explicit. The scene
includes robot/self collision, all 14 accepted radio convex parts and a padded
support table; it does **not** include the entire house or promise a tracking-safe
continuous-clearance margin.

Episode termination is independent of trajectory completion. The owner retains
full `TASK_GOAL_MET`, ordinary terminal or runtime-fault evidence and confirms the
stop. The runtime policy receives sanitized episode-ended/stop status, without
BDDL success or evaluator geometry. A goal event cannot override a physical
protection or unconfirmed-stop failure.

## Ordered physical baseline

Official `turning_on_radio`, `house_double_floor_lower`, instance 0 completed via:

1. Grasp, lift, reorient, lower and place using the SDK and stock assisted grasping.
2. Verify actual table support, open fully, confirm assisted attachment release
   and verify stable released support before backing off the gripper end stop.
3. Retract, configure the empty gripper and execute a shallow leading-finger press.
4. Read the independent official BDDL outcome and stop.

The development initialization used base position `[3.6, 4.15, 0.005]`, yaw pi/2,
privileged radio/table geometry and a manually prepared contact intent. No radio
freeze, friction/mass change, symbolic toggle or task-goal change was used.

Run `20261002l` reached official `TASK_GOAL_MET` at step 3093, goal 0 satisfied
and no unsatisfied goals, with a confirmed stop and clean process exit. This
terminal event ended the episode before ordinary motion-arrival completion.
Released support had five real table-contact samples over 0.418 s, detached
assistance and no finger contacts. Sampled press translation was at most 6.13 µm.
Contact evidence is distinct from the upstream semantic annotation: this does not
prove actuation of a modeled articulated electrical switch.

The installed contact cache can lose resting table pairs after the radio sleeps.
The ordered flow verifies actual support immediately after detachment, before
backoff. It does not fabricate contacts or wake/freeze the asset to pass a check.

Labeled baseline video is retained as Library artifact
`libfile_3153d0cb597881918ea3c72a87b40114`, version 1. Raw source hashes, report,
contact timeline and original camera evidence are retained in the durable task
workspace; large recordings and generated scene geometry are not committed here.
This Library artifact is not a publicly accessible GitHub attachment.

## Actual sensor coverage and agent status

After placement, the initial head and left-wrist views missed the radio. A later
operator-assisted left-arm/torso inspection motion passed collision validation and
completed in 9.52 s with base and right-arm joints frozen. Actual wrist RGB then
showed the casing, speaker, handle and tabletop. The top was clipped and fingers
partly occluded it; power-control identity remains unverified. Head RGB showed the
room/ceiling. All 76,800 depth pixels were valid, and RGB/depth/intrinsics/TF shared
one capture timestamp. Sampled radio displacement was zero. This observation-only
run made no model request or press; its independent BDDL goal remained unsatisfied.

Actual camera image is retained as Library artifact
`libfile_c5fbe0e9e5708191ac1d346e2d80948e`. Images contain no hidden-target
projection or evaluator overlays. The viewing pose was oracle-assisted setup,
not autonomous target acquisition.

An earlier bounded CPU pilot verified genuine Dimcode write/execute/SDK feedback;
it reproduced a supplied script and did not prove observation-driven robot control.
The current genuine agent-written observation trial has **not run**. Official Node
24.21.0 and Dimcode 0.1.0-next.7 are restored in an isolated durable directory;
system Node and existing credentials are unchanged. Model registration and local
image loading passed. Specific approval to transmit simulator images/selected SDK
feedback to the configured external model is pending; no request has been sent.

The eventual agent receives actual robot sensors/proprioception and legitimate SDK
feedback, never the successful evaluator report or hidden control coordinates.
BDDL/object truth stays owner/evaluator evidence. Bash/Python introspection is not
currently an enforced fairness boundary; a prompt or this facade cannot provide
complete filesystem/process/network isolation.

## Reproduction and checks

Keep code, environments, checkpoints and experiment outputs in durable project/task
directories. Use temporary directories only for disposable scratch. The prepared
operator replay and accepted geometry are retained under the durable task experiment
root; the generic repository demo below is a plumbing check, not a one-command
reproduction of the ordered successful replay.

```bash
mkdir -p radio-development/evidence
python -m dimos.simulation.behavior.demo_radio --stage motion \
  --report radio-development/evidence/motion.json
```

A checked contact replay requires an explicitly labelled target/scene description,
all 14 checksum-verified accepted convex meshes, table pose/extents, robot calibration
and a single initialized owner. Keep these private development inputs out of the
agent prompt. Missing runtime/assets or permissions must fail explicitly; do not
silently fall back to symbolic success or another model.

CPU verification does not import Isaac Sim or run a physical robot:

```bash
python -m pytest -q -o addopts='' \
  dimos/simulation/behavior/test_radio*.py \
  dimos/simulation/behavior/test_demo_radio.py \
  dimos/simulation/behavior/test_policy_visuals.py \
  dimos/manipulation/test_sdk.py \
  dimos/manipulation/test_plan_execution.py
```

Native planner regressions additionally check equivalent translated robot/target/
obstacle scenes, frozen base coordinates, selected groups and auxiliary routing.
The URDF import repair preserves omitted unbounded prismatic limits at the native
boundary rather than letting URDFDOM clamp translated base X/Y to zero.

Tests cover collision/scene ownership, exact-frame grounding and expiry,
trajectory/dispatch identity, limit and frozen-context admission, caller contact
geometry, support/release, cancellation/uncertain-stop behavior and terminal races.
They do not establish robust perception, semantic control identity, full-house
collision coverage, autonomous task completion or benchmark fairness.

Publication checks: 326 scoped CPU tests passed (two optional native backend skips
in the local environment), five real native planner cases passed in the existing
prepared CPU environment, Ruff check/format passed across the changed Python
files, and scoped mypy passed for seven production files with imported dependency
errors suppressed via `--follow-imports=silent`. The broader dependency-following
mypy run was blocked by missing optional packages and existing imported-module
type errors; it is not reported as a full-repository type-check pass.

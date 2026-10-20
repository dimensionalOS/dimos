# Run Dreamlabs on dual YAM

This integration uses the existing dual OpenYAM coordinator, canonical joint state,
and three `Image` streams. It starts idle. Only `start_rollout()` creates a Dreamlabs
client or requests inference. `stop_rollout()` cancels the coordinator before waiting
for an outstanding request. A late reply cannot restart motion.

## Prepare the station

1. Confirm two standard six-joint YAM follower arms with standard linear grippers.
   The supported hardware adapter is `dual_openyam_damiao`, not R1Pro, YAM Pro,
   YAM Ultra, GenRobot, Unitree, or Galaxea. Confirm physical left and right CAN
   assignments, encoder zero positions, joint directions, and gripper calibration.
2. Confirm the three camera roles by their actual view: `overhead_image`,
   `left_wrist_image`, and `right_wrist_image`. Use the station's existing camera
   modules, not a second process opening the same devices. The adapter requires
   timestamped uint8 RGB images. It resizes them to 320 by 176 and encodes JPEG
   at quality 85. Check camera timestamps against the coordinator clock.
3. Compare the measured tool and camera calibration with the SDK's standard YAM.
   The reference convention uses `joint1` through `joint6` in radians and a TCP 0.1347 m along
   `link_6` z. The DimOS model's `grasp_frame` is a different frame and is not an
   interchangeable TCP. The SDK FK fixture at `[0, .5, .5, -.35, 0, 0]` gives
   approximately `[.262523, 0, .206904]` m in the arm base frame. This fixture
   checks the SDK model, not the physical station. Do not mark an unverified or
   modified station as standard YAM. A custom tool needs a separately measured
   profile and an EEF integration, which this PR does not implement.
4. Confirm the workspace, operator, physical E-stop, and stop conditions before
   enabling execution. Keep existing coordinator checks and higher-priority
   operator tasks. This adapter uses joint trajectories and adds no scene collision
   planner. Its position and speed checks do not validate table or inter-arm clearance.

Both adapters use normalized grippers, 0 closed and 1 open. The policy maps
`q_l_chunk` and `q_r_chunk` into DimOS order: left six joints, right six joints,
left gripper, right gripper. It never sends raw `per_key`, degrees, motor positions,
or a different robot's joint profile to the coordinator. Only YAM joint chunks are
supported; an EEF-only reply is rejected rather than sent to hardware.

## Install and compose

Install the lightweight official client in the normal DimOS environment:

```bash
uv sync --extra dreamlabs --inexact
```

`DreamlabsModule` is a normal module. Its optional `dreamlabs-client==1.0.1`
dependency adds no full robot SDK, isolated process, JAX solver, or dependency
downgrades. The client imports only when an episode or live probe is explicitly
started. Existing Pinocchio FK computes the required EEF observations from a
kinematic-only copy of the SDK's reference YAM description. That file retains the
source geometry and MIT notice, and adds no hardware model or physical calibration.
The coordinator still uses its existing dual OpenYAM model and bounds.

Follow the provider's [Setup API](https://platform.dreamlabs.ai/setup-api) instructions
to supply an approved credential through `DREAM_API_KEY` or an existing owner-only
credentials file. The client reads provider endpoints from that file. Do not put
keys in a blueprint, command argument, source file, log, or recording.

For a new stack, compose `dreamlabs_dual_openyam` with the station's three existing
camera blueprints. Remap each camera instance's `color_image` output to the role
above. Set both `DualOpenYamCoordinator` CAN ports only for an approved hardware
launch. Omitting both CAN ports selects the existing mock adapter.

For an existing stack, add `DreamlabsModule.blueprint()` to its composition.
Use exactly one coordinator. Its sole `joint_trajectory` task must cover
`DUAL_OPENYAM_JOINTS`, including both grippers. The supplied
[`dreamlabs_dual_openyam` blueprint](../../../robot/manipulators/dual_openyam/blueprints/dreamlabs.py)
shows this task configuration. Preserve other operator and E-stop tasks.

Configure `DreamlabsModule` through the usual blueprint or CLI settings. Set
`prompt` to the requested task. Keep `allow_remote`, `enable_execution`, and
`standard_yam_verified` false until the applicable approvals and station checks
are complete. Setting them true permits a subsequent explicit start to upload
camera images, joint state, and the prompt to Dreamlabs and execute its answer.
SDK background recording is explicitly disabled with `_record=False`.

In an attached `dimos shell`, use these RPCs after the operator has approved the run:

```python
app.DreamlabsModule.preflight_rollout()
app.DreamlabsModule.start_rollout()
app.DreamlabsModule.rollout_status()
app.DreamlabsModule.stop_rollout()
```

Preflight checks gates, fresh synchronized observations, and model bounds locally.
It makes no Dreamlabs call. The explicit YAM schema supplies joint, grip, and EEF
fields without a model metadata lookup or client-side IK. The policy requests one
chunk at a time, plays all 48 rows at 20 Hz, waits for coordinator completion, and
holds during the next inference.
The SDK keeps nine frames per camera at 5 Hz from capture timestamps while fresh
observations arrive. This synchronous first version does not refill while moving.

Malformed, non-finite, out-of-range, discontinuous, or stale chunks stop the episode.
Coordinator rejection, timeout, missing observations, task abort, or a manual grip
also stops it. Uncertain cancellation blocks another start until the module is
restarted after hardware inspection. RPC exceptions and status contain sanitized
categories rather than SDK payloads.

## Run offline checks

These commands use no credential, camera, robot driver, or paid inference:

```bash
uv run --extra dreamlabs pytest dimos/imitation/policy/dreamlabs
```

Tests cover literal left/right action fixtures, gripper units, JPEG color order,
capture timestamps, SDK history, disabled recording, chunk seams, bounded commands,
timeout, stop during inference, manual takeover, cancellation uncertainty, and 38
fixed SDK reference FK poses, including seeded varied joint configurations. Tests
also exercise the real lightweight client with its network transport replaced.
No full SDK is needed to run these checks. No live inference or physical rollout
has been validated. Real camera placement, calibration, motion tracking, E-stop
latency, and task success require an approved station session.

After the provider's price and account limit are visible and one call is approved,
run the bounded probe separately from the robot stack:

```bash
uv run --extra dreamlabs python -m dimos.imitation.policy.dreamlabs.probe --live
```

The probe sends exactly one inference request, three generated black JPEGs,
the synthetic FK fixture state, and `hold position`. Session close may make an
additional control request. It uses the existing provider credential, disables recording, prints no returned actions, and opens no hardware.
It validates the API response contract, not task performance. Do not use this
command before approval of the payload and inference cost.

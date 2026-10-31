# Run Dreamlabs policy previews

The Dreamlabs backend uses the shared policy runtime. Joint mode accepts only the
canonical dual YAM model and named joints. EEF mode produces absolute TCP targets
for `EefTrajectoryTask`, which uses the existing bounded Pink controller.

## Prepare an isolated environment

Keep the backend SDK in `native/python/dreamlabs`. The runtime uses its pinned
`dreamlabs-client==1.0.1` lock and does not add the SDK to host dependencies.
Use the repository's supported manipulation environment for the coordinator and
planning UI. The blueprint preflights RoboPlan's XML loader before modules start.
No credentials are included in the repository. Configure credentials through the
SDK's existing setup separately.

## Start an idle stack

Set all three camera device paths explicitly. Omitting CAN ports uses the existing
mock hardware configuration. Real CAN deployment requires station-specific setup.

```bash
uv run dimos run dreamlabs-dual-openyam-joint \
  --OverheadCamera.device=/dev/video0 \
  --LeftWristCamera.device=/dev/video2 \
  --RightWristCamera.device=/dev/video4
```

For EEF mode, select `dreamlabs-dual-openyam-eef` with the same camera settings.
The EEF blueprint retains the joint trajectory task for the existing planning UI.
Neither blueprint starts rollout automatically.

## Preflight and preview

Attach with `dimos shell` and use the policy module's RPCs:

```python
policy.preflight_rollout()
policy.start_rollout()  # Remote inference and image/state upload, preview only.
policy.rollout_status()
policy.get_last_preview()
policy.get_last_action_response()
policy.stop_rollout()
```

Starting rollout sends camera images, robot state, and task language to Dreamlabs
and can incur billing. Local preflight does not send requests. `_record=False`
disables the SDK's additional background recording upload.

Wait for `busy=False` after stop before another episode. Use `set_task(text)` to
stop, drain, and replace task language. An in-flight request may finish after stop;
its result cannot preview or dispatch. Stop is not an E-stop.

Actuator dispatch is a separate opt-in:

```python
policy.start_rollout(dispatch_actions=True)
```

Physical dispatch through this integrated backend requires separate station
acceptance. Previous standalone Dreamlabs trials do not validate this runtime or
EEF tracking. Stop and drain policy before manual planning execution.

## Configure another robot for EEF mode

Provide `DreamlabsBackendConfig(mode="eef", model=..., arms=...)` to the shared
`policy_module`. Each `DreamlabsArmConfig` names its controlled joints, fixed arm
base, target frame, normalized gripper joint, and measured target-to-SDK-TCP
transform. The arm base must be fixed relative to the model root.
The model and target frames must match the coordinator's EEF task configuration.

The adapter uses metres, radians, and normalized `[0,1]` gripper openings. SDK
poses are arm-base-local. The adapter removes and reapplies mount/tool transforms
explicitly and outputs model-root poses. The EEF task converts model-root targets
through the configured world base pose. It rejects other reference frames.

The YAM default preserves the tested SDK tool-axis convention and 0.1347 m TCP.
The canonical grasp frame is at 0.1 m, so its explicit additional offset is 0.0347 m.
These transforms do not establish physical encoder or tool calibration.

## Protocol and execution limits

The pinned SDK owns nine-frame history at 5 Hz. Actions contain 48 steps at 20 Hz.
Conflicting `fps` or `execution_steps` settings are rejected.
Joint mode selects `robot="yam"`; EEF mode selects `robot="other_embodiment"` and
supplies no SDK IK. EEF `per_key` poses are already absolute. Only joint chunks
receive SDK smoothing; the adapter does not smooth EEF poses again.

Malformed shapes, non-finite values, and degenerate rot6 columns are rejected.
Finite gripper outputs are clipped to `[0,1]` with raw diagnostics retained.
Joint mode preserves the latest standalone implementation's explicit policy:
arm overshoot up to 2 degrees is clipped to model limits and logged; larger
overshoot rejects the whole chunk. Shared hardware limits remain authoritative.

EEF execution samples Cartesian targets on coordinator ticks, solves all target
frames together, and bounds commands using the existing Pink control core.
Tracking error, invalid feedback, IK failure, and settling timeout fault the task.
Completion requires measured TCP and gripper error within configured tolerances.
Dreamlabs dispatch retains the standalone adapter's coordinator deadman, with a
one-second default. The runtime refreshes it while awaiting execution. Lost
heartbeats abort the task; ordinary planning trajectories have no default deadman.
This is bounded tracking, not a collision-free motion planner.

## Run offline checks

```bash
uv run pytest dimos/imitation/policy/test_runtime.py \
  dimos/imitation/policy/dreamlabs dimos/control/tasks/eef_trajectory_task \
  dimos/msgs/trajectory_msgs/test_eef_trajectory.py
cd native/python/dreamlabs
uv run --frozen --with pytest --with pytest-mock --with-editable ../../.. \
  python -m pytest test_sdk_contract.py
```

The pinned SDK tests replace credential loading and transport requests. They do
not read real credentials, upload images, or contact Dreamlabs.

## Dual YAM learning setup

Use `dual-openyam-dreamlabs-collection` to record overhead and both wrist cameras
with the existing Quest episode controls. The two-camera
`dual-openyam-quest-collection` remains available. Configure capture devices through
`--left-wrist.hardware.camera-index`, `--right-wrist.hardware.camera-index`, and
`--overhead.hardware.camera-index`, preferably using stable `/dev/v4l/by-id` paths.

Prepare and inspect the saved recording through `dimos imitation prepare` and
`dimos imitation inspect`. The saved schema records measured state and attempted
joint commands separately; it does not claim that a commanded pose was reached.

Run `dreamlabs-dual-openyam-joint` or `dreamlabs-dual-openyam-eef` for three-camera
rollout. Both consume `overhead_image`, `left_wrist_image`, and `right_wrist_image`
as `overhead`, `wrist_l`, and `wrist_r`. Rollout camera selectors are
`--OverheadCamera.device`, `--LeftWristCamera.device`, and `--RightWristCamera.device`.
No policy starts at blueprint startup. Preflight is local; preview requests remote
inference without actuator dispatch. Explicitly pass `dispatch_actions=True` to
`start_rollout` to execute. Do not infer execution from the preview counters.

# Collect and preview policies on MakerMods Metal

These integration blueprints combine the existing Metal hardware stack with the
shared learning flow. They reuse the Metal firmware profiles and CAN driver.
See the [single-arm hardware guide](/docs/capabilities/manipulation/makermods_metal_integration.md) and
[dual-arm hardware guide](/docs/capabilities/manipulation/dual_makermods_metal_integration.md) for commissioning.

## Choose a setup

| Setup | Collection blueprint | Dreamlabs rollout blueprint |
| --- | --- | --- |
| Single Metal, overhead | `makermods-metal-learning-collection` | `dreamlabs-makermods-metal-eef` |
| Dual Metal, two wrists | `dual-makermods-metal-learning-collection-2cam` | `dreamlabs-dual-makermods-metal-eef-2cam` |
| Dual Metal, overhead and wrists | `dual-makermods-metal-learning-collection-3cam` | `dreamlabs-dual-makermods-metal-eef-3cam` |

Camera roles are `overhead`, `wrist_l`, and `wrist_r`. Each setup sends exactly its
configured views. Missing views are not replaced with duplicate or blank images.
The pinned SDK accepts these layouts in offline transport tests; remote checkpoint
support and task success still require live validation.

Single Metal uses the right protocol slot and right Quest controller. Dual-arm
controllers operate their corresponding arms. Controller grip takeover stops
policy; releasing the grip does not resume it. Joint trajectories remain available
for manual plans. Dreamlabs joint output is rejected for Metal because it names
YAM joints.

## Record demonstrations

Start a single-arm collection with a stable camera device path:

```bash
dimos run makermods-metal-learning-collection \
  --OverheadCamera.device /dev/v4l/by-id/OVERHEAD \
  --recorder.recording ./recordings/metal-demo \
  --episodes.task 'pick up the object'
```

Without CAN selectors all actuators are mocks. Physical single-arm deployment uses
`--ControlCoordinator.can-port`. Dual-arm deployment requires both distinct
`--ControlCoordinator.left-can-port` and `--ControlCoordinator.right-can-port`.
Physical startup enables the arms and calibrates the grippers.

For wrist views, configure `--LeftWristCamera.device` and `--RightWristCamera.device`.
Attach with `dimos imitation control`, then start, save, or discard demonstrations.
Detach leaves the stack running; Stop stack asks for shutdown confirmation.
Recordings preserve measured state separately from attempted joint commands.
Grippers use normalized values from 0 closed to 1 open.

```bash
dimos imitation inspect ./recordings/metal-demo
dimos imitation prepare ./recordings/metal-demo --output ./datasets/metal-demo
dimos imitation inspect ./datasets/metal-demo
```

## Supply station geometry

The dual model's default mounting layout is a mock example. Physical bimanual work
requires measured base positions and orientations. Pass one `RobotModelConfig` to
teleop, manual planning, EEF execution, and policy FK. Do not override only the
planning module's model.

`gripper_base` and its side-prefixed variants are mount frames, not calibrated
fingertip TCPs. Rollout preflight rejects missing target-to-SDK tool transforms.
Translations are metres; rotations are unit quaternions in xyzw order. No YAM tool
offset is substituted for Metal.

Define a station blueprint using `build_metal_learning` from
`dimos.robot.manipulators.makermods_metal.learning`. For a dual rollout, pass:

- `dual=True` and `collection=False`.
- The selected `camera_roles`.
- `model=dual_metal_model_config(...)` with measured mounts.
- `tcp_transforms={"left": (xyz, xyzw), "right": (xyz, xyzw)}` with measured tool transforms.
- `checkpoint` naming the model to evaluate.

Single-arm stations supply only the `"right"` transform. Registered rollout
blueprints can also receive the complete calibrated arm mapping through
`policy.dreamlabs.arms` configuration. Use a station blueprint for dual mount
changes so all model consumers receive identical geometry.

## Preview before execution

Rollout blueprints start idle. `dimos imitation control` starts Dreamlabs in
preview mode. For explicit control, use these RPCs in `dimos shell`:

```python
policy.preflight_rollout()  # Local validation, no inference.
policy.start_rollout()      # Remote inference and upload, preview only.
policy.get_last_preview()
policy.stop_rollout()
```

Wait for `busy=False` and inspect target frames and gripper directions. Explicitly
request `policy.start_rollout(dispatch_actions=True)` to execute. Stop and drain
policy before manual planning. Preview counters do not confirm trajectory execution.

## Validate each station

Offline tests cover station schemas, camera selection, CLI parsing, missing
calibration, rejection of YAM joint output on Metal, SDK request layouts, and
policy/trajectory lifecycle regressions. Self-hosted mock tests use the actual
cached Metal URDF and Pink with mock adapters to instantiate all tasks and compute
joint and EEF commands. They do not operate physical hardware.

Commission deterministic joint holds and small moves before model output. Then
check EEF holds and small tool-frame moves, each gripper, synchronized dual-arm
motion, cancellation, controller takeover, and deadman expiry. Finally test real
policy previews, short executions, and repeated tasks. Record station geometry,
camera roles, checkpoint, initial state, outcome, and stop/fault reasons per run.
Hardware commissioning and live inference remain separate acceptance steps.

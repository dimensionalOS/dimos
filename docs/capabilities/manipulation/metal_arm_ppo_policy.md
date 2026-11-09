# Metal dual-arm PPO policy (station, CPU/ONNX)

Opt-in deployment of the accepted Metal station bimanual PPO controller. This
page covers the learned policy only; the planning, IK and default joint
controller paths are unchanged. The policy is registered as a coordinator task
and is inert until it is engaged with a fresh goal.

**Model identity.** The deployed model is the accepted `ori10raw001` lineage at
iteration 38,000 (`cmdlp005-ori10-rawex001-v1`), *not* the newest nominal
producer checkpoint. The manifest pins:

| Field | Value |
|-------|-------|
| checkpoint sha256 | `5bc5a75fa9e2762567958b5b4b110bdf864fea5988e2419746f5c7730aebb23f` |
| ONNX sha256 | `6624c9b1bee76e2942810ceef768f832a0420c0c5bde773594ca27c1456b66ad` |
| normalizer sha256 | `8d0b25eaba9d19bbceac7327c9c73f387a43d9fa3b9ca8ea171c3da7d63d8957` |
| observation | 6 frames × 96 values (576), 20 Hz |
| action | 12 arm joints, deterministic mean |

The artifacts are build evidence and are **not** committed. Export them from
the trusted checkpoint:

```bash
python -m dimos.robot.manipulators.dual_makermods_metal.policy.export_onnx \
  --checkpoint /path/to/checkpoint_038000.pt \
  --out ~/.local/share/dimos/metal_policy
```

The exporter rebuilds the rsl_rl actor semantics (ELU MLP with
`(x - mean) / (std + eps)` normalization, deterministic mean output) and writes
`metal_arm_policy.onnx`, `metal_arm_policy.normalizer.npz` and a hash-pinned
manifest. It refuses a checkpoint whose contract block or actor keys do not
match. Deployment itself needs neither torch, the training repository, nor
mjlab.

## Run the fake/no-CAN station

```bash
export DIMOS_METAL_POLICY_MODEL_DIR=~/.local/share/dimos/metal_policy
dimos run dual-makermods-metal-policy-fake
```

The blueprint composes the coordinator task with `MockJointConnection`; it
never constructs `DualMetalConnection`, so this path cannot open a CAN bus even
when hardware selectors exist elsewhere in the environment. Both jaws keep
separate gripper tasks and the policy claims only the 12 arm joints. Mock
feedback echoes accepted positions: it is plumbing, not a dynamics or contact
simulation.

## Goals and engagement

Goals are absolute TCP poses for both hands in the station frame, laid out as
`[left_xyz, right_xyz, left_R, right_R]` (24 values). The runtime observes
`goal - current_tcp`, so the goal must use the station TCP, not the repo's
default planning tip.

```python
from dimos.robot.manipulators.dual_makermods_metal.policy import DualHandGoal, HandGoal

goal = DualHandGoal(
    left=HandGoal.from_quaternion((0.33, -0.35, 0.15), (0.0, 0.0, 0.0, 1.0)),
    right=HandGoal.from_quaternion((0.33, 0.35, 0.15), (0.0, 0.0, 0.0, 1.0)),
)
task.set_goal_pose24(goal.to_pose24().tolist())
task.engage()
# ... task.release()
```

`set_goal(left_xyz, left_quaternion_xyzw, right_xyz, right_quaternion_xyzw)`
is the equivalent position/quaternion entry.

With `teleop=True` the task also accepts the WebXR controller streams. The
deadman anchors the controller pose at engagement, so a relative hand motion
becomes an absolute station goal; releasing the deadman, a timeout or a fault
drops the anchor and invalidates the session.

Engagement is deliberately strict:

- a fresh, complete 12-joint feedback frame and fresh intent are required;
- the issued command is seeded from the **measured** joints, never from home,
  so engaging mid-pose cannot snap the arms;
- release, e-stop, preemption, feedback/goal timeout, an inference fault or an
  incomplete frame invalidate the session: no replay, no silent IK fallback,
  and the task must be re-engaged with a fresh same-or-new goal;
- resetting runtime state is separate from engagement and never re-activates
  the task.

## Timing and latency

The coordinator tick is 20 Hz. Each tick consumes the six most recent
observation frames, calls the ONNX actor, then applies the training decode
order: clip the action, form `home + 0.6 * action`, clamp to the training joint
limits, low-pass (`0.05`), then slew-limit (`0.06` rad/tick). The frame the
robot reaches during a tick is recorded *after* the action, matching the
one-period feedback latency the controller was trained with.

Measured on the deployment host CPU with `CPUExecutionProvider` over 140
recorded ticks:

- actor inference: p50 0.04 ms, p95 0.04 ms
- full runtime tick (FK + observation + actor + decode): p50 0.11 ms,
  p95 0.11 ms

That leaves essentially the whole 50 ms tick budget for transport and plant
jitter. The repository test suite asserts a p95 budget of 5 ms for inference
and 20 ms for the full tick, so a machine more than two orders of magnitude
slower fails loudly instead of silently missing the control rate.

## Station geometry

The trained station is not the repository's example bench layout. It is
carried explicitly in the manifest:

- left mount `(0, -0.345, 0.008)`, right mount `(0, 0.345, 0.008)`, no rotation
- TCP site 60 mm out along the gripper-base frame's local +Z

The runtime's FK and the MuJoCo evidence plant agree on those TCP sites to
1e-6. Reusing a different mount or TCP silently changes both the observation
and the goal, so do not override the geometry unless the physical station
really changed and the policy is re-validated.

## Evidence

- `policy/test_sim_closed_loop.py`: CPU MuJoCo closed loop on the trained
  station plant (training PD gains, effort limits, gravity compensation and
  20 Hz decimation). The original checkpoint actor and the exported ONNX
  runtime drive two identical plants in lockstep, consuming only the current
  measured state and the current causal goal. Result: actor parity 6.1e-6,
  trajectory gap 4.8e-3 rad, 12.6 cm of goal travel tracked to 2.7 mm, with a
  held-goal tail.
- `policy/test_evidence.py`: FK against 6400 recorded station poses (max
  5.1e-7) and multi-tick reconstruction of recorded actions/commands.
- `policy/test_latency.py`: CPU p50/p95 budget above.
- `control/tasks/metal_arm_policy_task/` and `blueprints/test_policy.py`:
  activation, fault/recovery, accepted-but-unemitted command, 20 Hz hold,
  release, preemption, e-stop, reset and dry-run, plus a registry-driven run
  through the real ONNX model and mock connection.

Run the artifact-gated tests with:

```bash
export DIMOS_METAL_POLICY_MODEL_DIR=~/.local/share/dimos/metal_policy
export DIMOS_METAL_POLICY_EVIDENCE_DIR=$DIMOS_METAL_POLICY_MODEL_DIR
export DIMOS_METAL_POLICY_SIM_DIR=/path/to/station/sim/assets
pytest dimos/robot/manipulators/dual_makermods_metal/policy
```

Without these variables the artifact tests skip rather than fabricate
fixtures.

## Limitations

- **No hardware validation.** The policy has not been run on real motors or a
  CAN bus. The blueprint's only launch path is the mock connection.
- **Training plant only.** Evidence comes from the recorded training station
  (gains, gravity compensation, mounts, TCP). A different bench, payload or
  controller will change the closed loop; re-run the evidence against the
  actual plant before use.
- **CPU measured; GPU untested.** Latency numbers are CPU
  `CPUExecutionProvider` on this host. `onnxruntime-gpu` is not exercised.
- **Station frame only.** Goals are absolute station-frame TCP poses. There is
  no world-frame, mobile-base or collision-aware goal handling in this layer.
- **Jaws are separate.** The policy never commands the grippers; they keep
  their normal gripper tasks.
- **No recovery automation.** Faults require a human or supervisor to
  re-engage. The task intentionally does not replay the last action, retry, or
  fall back to IK.
- **CI is not part of this evidence.** These tests are artifact-gated and were
  run locally against the hashed checkpoint.

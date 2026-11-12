# OpenYAM history policy in DimOS

Quest/WebXR controller input drives the frozen OpenYAM actor inside a DimOS
control stack, against the MuJoCo plant the actor was trained on. No hardware
path exists in these blueprints.

```
WebXR controller (deadman grip)
  -> QuestReferenceGenerator: 30 Hz anchored pose -> Pink IK q_ref (clipped to
     the trained envelope) -> dq_ref over the actual dt -> FK goal TCP
  -> OpenYamPolicyTask: 16x20 history + 19 goal -> exported ONNX actor
     (q_des = q_ref + 0.1 * raw, the trained decode law)
  -> TeleopControlCoordinator (60 Hz tick, actor every 2nd tick = 30 Hz)
  -> sim_mujoco adapter -> MujocoSimModule -> trained plant, 1/480 s step
  -> joint state back over shared memory
```

## Blueprints

| Blueprint | Input | Use |
| --- | --- | --- |
| `openyam-policy-sim` | Quest headset (`ArmTeleopModule`) | Operator teleoperation |
| `openyam-policy-sim-replay` | `WebXRReplayModule` (scripted) | Machines without a headset; **input replay, not a headset session** |

Both pin a private zenoh domain (`openyam-policy-sim`) **and** a private
multicast scout group (`224.0.0.231:7446`). Stream keys are domain-scoped but
`dimos/rpc/<name>` keys are not, so another DimOS stack on the host would
otherwise answer this run's RPCs and feed it foreign teleop targets.

## Model configuration

Artifacts are not committed. Point the stack at a directory holding:

* `openyam_policy.onnx` (actor), `openyam_policy.normalizer.npz`,
* `openyam_policy.manifest.json` (pins contract, actor hash, plant model and the
  plant hash),
* the plant XML named by `metadata.plant_model` (or `DIMOS_OPENYAM_POLICY_PLANT`).

`--model-dir` defaults to `~/.local/share/dimos/openyam_policy`
(`DIMOS_OPENYAM_POLICY_DIR`). The manifest, not this README, defines the joint
order, the observation layout and the simulated joint limits; a mismatch fails
at task construction.

## Launch

Quest teleoperation:

```bash
MUJOCO_GL=egl python -m dimos.cli run openyam-policy-sim \
    --model-dir /path/to/openyam_policy
```

Deterministic replay (evidence / no headset):

```bash
MUJOCO_GL=egl python -m dimos.cli run openyam-policy-sim-replay \
    --model-dir /path/to/openyam_policy
```

Closed-loop evidence run (writes `closed_loop.npz/json`, `demo.webm` (VP9
yuv420p, plays everywhere) and `demo.mp4`):

```bash
MUJOCO_GL=egl python -m dimos.robot.manipulators.openyam.policy.demo_policy_sim \
    --model-dir /path/to/openyam_policy --output-dir ./evidence --duration 9
```

## Training-chain comparison (evidence only, not a deployment input)

`openyam_policy_sim_clip` / `demo_clip_compare.py` run the same stack from a
frozen recorded reference clip instead of WebXR input, with the clip start index
resolved from the measured reset pose. The shipped clip is recorded at **60 Hz**
and decimated by the policy rate (`ReferenceClip` infers this; a 30 Hz
assumption is wrong). Result on env 0 (anchor 50 = `start_tick`, initial qpos
diff 0.0): actor 30.16 Hz, physics 466.6 Hz, joint error 19.6 mrad mean vs the
training rollout, TCP 13.9 mm mean. Not claimed equivalent: the training chain
had 8 joints (fingers parked within 0.0546-0.0556 rad) and a different command
path; replaying the recorded commands open loop reproduces training to
13.2 mrad mean but not exactly.

## Operator contract

* **Deadman**: hold the right grip. Releasing closes the session immediately.
* **Anchor**: the operator pose is anchored to the measured TCP at engagement,
  so pressing the grip never moves the arm.
* **Stale input**: no controller sample for 0.25 s closes the session
  (`fault=intent`); a release/drop does not re-arm until the grip is released
  and pressed again.
* **E-stop**: `task_invoke("openyam_policy", "set_estop", {"estopped": true})`
  latches; clearing it does not resume a session.
* **Episode limit**: 180 actor ticks (6 s at 30 Hz), matching the trained
  episode; the session ends with `fault=episode`.
* Ending a session stops emitting new targets; the plant holds its last
  commanded position.

## Honesty boundaries

* The reference is **IK-generated**, not the recorded retargeted training
  motion: it is a deployment adaptation, not proven inside the training
  distribution. `requested` (operator target), `achievable` (IK projection) and
  `executed` (plant TCP) are reported separately.
* The plant is the **welded-finger 6-joint** version of the trained 8-joint
  model; FK agrees, the finger bodies do not articulate.
* Joint limits differ between the IK URDF (yam) and the trained envelope; the
  generator clips every reference into `sim_lower+0.015 .. sim_upper-0.015`.
* The coordinator runs at 60 Hz with the actor decimated to every second tick;
  the measured cadence is 30 Hz, physics ~465 Hz on this host (480 configured).
* The reference dt guard accepts 0.9-1.1 x the policy period: measured jitter is
  <0.3% (33.11 ms mean, 33.42 ms max over 149 ticks), while a dropped/doubled
  60 Hz tick is 50% off and is refused.
* Report the TCP error channels separately and never mix frames: `goal` and
  `executed` are both the plant `arm_tcp` body (**policy-goal tracking**), while
  `requested`, `achievable` and the FK of the measured joints live in the
  deployed URDF `gripper_tip` frame, which sits ~0.08-0.10 m / ~100 deg away from
  `arm_tcp`. Cross-frame channels are marked `"valid": false` in the comparison
  JSON and are a measured frame offset, not tracking error. Orientation is the
  great-circle quaternion angle from `tcp.quat_angle_deg` (degrees, converted
  exactly once).
* The anchor diagnostic reports `achievable` in the same URDF frame as
  `requested` (the IK target at engagement is the measured pose, so the
  projection is zero); it must never carry the plant goal frame, or the frame
  offset re-enters the projection channel as a fake ~80 mm / ~100 deg error.
* Closed-loop numbers are published over the **complete active window** (every
  recorded sample whose session was active, with counts/mask/finite coverage/
  faults), not over a first-sample-per-tick subset; per-tick figures stay as
  labelled secondary diagnostics. The actor-11000 run is 299 samples / 150 ticks:
  policy-goal tracking 6.24 / 13.66 / 36.65 mm and 1.30 / 3.18 / 9.34 deg
  (mean / P95 / max), operator-request tracking in the URDF tool frame
  7.63 / 13.59 / 33.79 mm and 1.25 / 3.15 / 9.34 deg, IK projection
  2.38 / 5.84 / 5.89 mm.
* Actor **6000** is the primary artifact; a separate **11000** artifact
  (`actor_a00011000.pt`, ONNX, normalizer, manifest with its own checkpoint
  hash) was exported from the frozen `preeval_a00011000_r000027550.pt` and run
  through the same blueprint for a matched comparison. Note that local
  presence/absence of a checkpoint on any one host says nothing about the
  training producer's state.

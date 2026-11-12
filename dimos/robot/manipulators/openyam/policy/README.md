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
* The current evaluation artifacts are actor **6000** updates. An actor 11000
  run is not available: that checkpoint is no longer on disk (only its
  evaluation receipts remain), so no 6000-vs-11000 integrated comparison exists.

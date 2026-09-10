# Five-bottle ACT packing

Status: implementation and physical demonstrations are verified; learned ACT
validation is still running. Do not interpret teacher success as policy success.
The existing one-bottle delivery demo remains documented in [ACT_SIM.md](ACT_SIM.md).

This version uses five of the original 5 cm diameter, 14 cm tall bottles and the
original 21 cm square tray. A local MJCF overlay arranges the workstation within
right-arm reach; the house package is unchanged. The tray rests on the worktop.
Bottle shapes and learned tray grasping are deferred until this baseline passes.

A geometry planner chooses an accessible source and an empty slot. ACT receives
the two RGB cameras, 20 measured joint positions, and an explicit eight-value
simulator vector: selected source XYZ, destination XYZ, radius, half-height.
ACT executes the entire pick, release and return-home skill through the
ControlCoordinator `policy_rollout` trajectory task. Then physics verifies the
pick and the next goal is selected. This is a new goal-conditioned checkpoint;
`policy-free-tray` cannot be used with the packing profile.

The planner clears front bottles before rear bottles whose transfer corridor is
blocked, places bottles in rows, and includes gripper-opening clearance. If no
safe slot remains it stops with `tray_full`, leaving remaining objects alone.
It does not rearrange bottles or promise optimal packing. Per-bottle evidence
requires an actual bilateral grasp/lift, upright placement (within 15 degrees
of the tray normal), full containment, release and settling. Tilted objects use
a larger conservative footprint for subsequent fit checks.
All five must still be contained at the end for complete success. No object is
attached, teleported or repositioned after reset, and ACT evaluation has no
scripted motion fallback.

## Background training on this machine

The initial four-scene, 3,000-step pilot failed all three learned test sequences.
A second run adds paired choices (identical initial RGB/joints, different selected
bottles). Its 4,000-step checkpoint completed a first pick in two scenes, but
none of the three full sequences passed. One native coordinator run also
completed the first pick and stopped cleanly after the next failed pick.

`packing-conditioned` completed 8,000 steps on 12 paired picks and 40 picks from
full scenes; it still failed all three complete learned sequences. The larger `packing-full-conditioned` continuation combines all 20 full
scenes (100 picks) with 30 paired picks, fine-tunes for up to 20,000 steps with
batch size 32, and physically evaluates fresh scenes. This replaces the original
unpaired 5,000-step baseline after preserving its completed collection. Checkpoint
selection still depends on full physical outcomes. These jobs run detached:

```bash
cd /home/mustafa/dimos-wt/r1pro-act-sim
cat recordings/r1pro-act-task/jobs/packing-full-conditioned/stage
cat recordings/r1pro-act-task/jobs/packing-conditioned/stage
tail -f recordings/r1pro-act-task/jobs/packing-full-conditioned/run.log
```

Each job directory contains the exact `run.sh`, `pid`, and an `exit-code` when it
finishes. Do not start a second pipeline against its output directories.
The collector resumes completed scenes from `manifest.json`; interrupted partial
scenes are not admitted to the dataset. Raw NPZ files, datasets and checkpoints
are local ignored artifacts, separate from source commits.

Measured pilot throughput is approximately 6–7 training steps/second, so 5,000
steps take roughly 12–15 minutes. Camera demonstration collection and physical
rollouts take longer. More training or data may be required based on evaluation.

## Native demo command after policy validation

The following path is the larger continuation's planned output; check its evaluation
report before treating it as a working policy. Run from a desktop terminal with
`DISPLAY` set. GLFW provides the full native MuJoCo display.

```bash
cd /home/mustafa/dimos-wt/r1pro-act-sim
source .venv/bin/activate
export PYTHONPATH="$PWD"
export MUJOCO_GL=glfw

python -m dimos.robot.galaxea.r1pro.demo_packing_stack \
  --artifact "$PWD/recordings/r1pro-act-task/policy-packing-full-conditioned" \
  --output "$PWD/recordings/r1pro-act-task/my-five-bottle-run" \
  --zenoh-scout-addr 224.0.0.224:19467 \
  --scene-package /home/mustafa/dimos/data/scene_packages/hssd_102344115 \
  --seed 9000 --stay-open
```

Use a fresh output directory and close any previous R1Pro stack using the same
multicast address. The launcher reserves both resources. `result.json` retains
per-pick history and final all-bottle evidence, including failures, before
teardown. The native viewer stays open after the run.

## Verification so far

- Teacher: 20/20 full randomized sequences, 100/100 picks, seeds 8200–8219,
  source jitter ±3 mm. No rejected sequences in this batch.
- Five packing physics tests pass, including upright scoring and stopping when
  a fallen bottle obstructs every slot. Seven existing grasp tests also passed.
- 33 planner/profile/runtime tests pass; generated registry checks pass.
- Host and isolated LeRobot typing pass.
- Native stack preflight passed twice with the goal stream and CUDA policy
  connected, zero action chunks executed; this is an integration check only.
- A one-scene conversion, model initialization and two-step training smoke test
  passed. Its checkpoint is not a trained packing policy.
- Complete learned five-bottle success and repeated native learned runs: pending.
- Runtime waits briefly for a complete camera pair while preserving age/skew
  limits; 17 focused runtime tests pass, including timeout and stop behavior.

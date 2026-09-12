# Random-object ACT execution handoff

## Current state — 2026-09-12

The user requires ACT grasps for four or five randomly generated objects and requested sparse checks while training continues across terminal disconnects. Nothing further is needed from the user for this phase. Work is in `/home/mustafa/dimos-wt/r1pro-act-sim`; the shared root checkout and its environment are untouched.

The first learned pilot is complete: **6/12 unseen single-object picks, 0/8 complete scenes** (4/12 successful attempted picks within those sequences). There were no classical grasp interventions. This is not ready to replace the working bottle demo. Main failures include approaches that hover or miss, disturbing neighboring objects, and failing to finish the return-home requirement. New evaluation reports distinguish those failure categories.

A controlled fitting experiment is running in `recordings/r1pro-act-task/jobs/random-objects-act-refine-v1`: 30,000 additional updates from the pilot weights, on exactly the same demonstrations and diagnostic split, followed by 12 single-pick and eight full-sequence evaluations. This tests whether additional fitting improves precision before spending more time collecting data. Initial measured speed remains about five updates/second; allow approximately 1.5–2 hours including evaluation. The run does not promote any artifact automatically. The native random-object blueprint is still pending.

Supervisor PID at launch: **1018189**, with its own session ID and no terminal stdin. Training was confirmed advancing past 146/30000 updates. Do not assume that PID or stage remains current: read `status.json`, `progress.json`, and `exit-code`. The supervisor now writes a compact progress file every five minutes itself, without an LLM/API call. `train.log` and `evaluate-*.log` contain details. The job survives terminal disconnects; a machine reboot requires restarting the command below.

```bash
cd /home/mustafa/dimos-wt/r1pro-act-sim
.home-venv/bin/python -m dimos.robot.galaxea.r1pro.demo_train_objects \
  --output "$PWD/recordings/r1pro-act-task/jobs/random-objects-act-refine-v1" \
  --reuse-job "$PWD/recordings/r1pro-act-task/jobs/random-objects-act-v1" \
  --scene-package /home/mustafa/dimos/data/scene_packages/hssd_102344115 \
  --steps 30000 --background
```

Completed stages are reused. Training can resume from its saved optimizer checkpoint. A failure before the first checkpoint can still require examining the incomplete training directory; do not discard or overwrite artifacts blindly. A file lock prevents duplicate supervisors for one output directory. No external service or API key is used.

## Implementation and actual evidence

- `object_packing_scene.py` generates independently positioned, sized, coloured and weighted boxes, cylinders and compound bottles. There are four or five objects, with randomized partial occupancy. Bounded whole-layout restarts preserve source finger clearance. A local overlay gives the new task a 29 cm internal tray and enough release clearance for short objects; the existing bottle scenes are unchanged.
- `object_packing.py` defines 52 simulator-ground-truth features: selected object relative to TCP, destination relative to TCP, object orientation/dimensions/shape, home relative to TCP, and sorted neighboring poses/extents with padding masks. There are also two RGB cameras and 20 measured joints. ACT outputs all 20 joint targets. This is a bounded primitive-shape distribution, not arbitrary unseen household categories.
- `object_packing_task.py` uses the new manipulator SDK for offline demonstrations only. Learned evaluation never calls teacher actions or IK. Success requires physical bilateral grasp/lift, upright supported release, containment, settled motion and return home; non-target motion/collisions are checked. Geometry and contact support are separate predicates so tiny momentary support-force changes do not falsely report spills.
- Teacher pilots 01–06 exposed arm/furniture contacts, infeasible torso routes and insufficient short-object release clearance. Pilot 07 passed **15/15** physical picks across eight layouts, all three families and 0–3 initial occupants. Image smoke passed **4/4**, followed by successful conversion, initialization, five-update fitting, export and a deliberately short learned execution. That smoke did not prove learned success.
- Full collection in `jobs/random-objects-act-v1`: **115 accepted / 118 attempted**, 64 layouts, up to two selected targets per layout, with 43 cylinders, 39 boxes and 33 bottles. Each successful pick is an independent episode; this is not an enumeration of full-scene permutations.
- Pilot training completed **10,000 updates**, batch 32, about five dataset epochs, in **32m14s**. Diagnostic loss fell from .0379 at 2500 updates to .0239 at 10000; that loss is not the acceptance metric. Paired choices from the same layout stay on the same side of the diagnostic split.
- Initial evaluation attempts crashed when a successful pick returned `numpy.bool_` to JSON. `pick_complete()` now returns a Python bool, with a real physical grasp + JSON regression. Incomplete reports are archived under `eval-*-incomplete*`; use only the current complete `eval-single/result.json` and `eval-sequences/result.json`. The corrected evaluation finished at **6/12 and 0/8**. The pipeline now rejects incomplete evaluation reports instead of treating partial totals as a completed evaluation.
- Recorded-state prediction diagnostics are saved in `fit-diagnostics.json`: typical active-joint MAE .003–.016 rad, with some held-out lift errors up to .027 rad. Additional fitting is a hypothesis being tested, not an established fix. Current physical seeds (200000+ and 210000+) are development evaluations; freeze a new final test set before acceptance.

## Verification and git

The latest combined object geometry and physical-completion suite passed all 21 tests (15.99 seconds). Related geometry/recovery/tray checks also passed. Strict mypy passed for the six production task/pipeline files and four backend integration files. The original single-bottle physical regression passed. The new physical box-pick/JSON-completion regression also passed (10.8 seconds). When running explicit physical tests, disable unrelated ROS pytest plugin autoload rather than altering the shared environment.

Prior interactive work was committed as `14f90296c6`; random-object collection/evaluation as `cf2346856c`. Both were pushed to `origin/feat/r1pro-act-sim`, without coauthor trailers. Follow-up evaluation/pipeline fixes are committed separately with this handoff. Recordings, weights, logs, the `.venv` symlink and local data links are not committed.

Main integration remains pending: a rebase tried to replay 92 dependency commits and conflicted with main's relocation of the isolated Python runtime. It was aborted cleanly; `backup/r1pro-before-random-objects-20260911` preserves the previous branch. Do not claim this branch is rebased or force-push it casually.

## Next work

1. Read the refinement's final physical results. Compare the unchanged development scenes with the pilot; do not keep extending training if it fails to help.
2. If necessary collect broader independent layouts and off-demonstration start/approach corrections, preserving requested-object conditioning and all failure records. ACT must continue to execute grasps; do not add a hidden classical fallback.
3. Implement and validate the new profile through the native ControlCoordinator, simulator observation stream and agent skills. The current learned evaluator is a direct MuJoCo runner; a new `dimos run` random-object blueprint has not been shipped.
4. Test repeated requested targets, stop-when-full, infeasible requests and recovery in the native stack, plus a fresh final test set. Do not promote merely because loss decreases.
5. Integrate current main carefully. The earlier multi-stop unloading/regrasp SDK QP failure is separate and remains unresolved; existing tray carrying remains classical.

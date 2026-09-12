# Random-object ACT execution handoff

## Current state — 2026-09-12

The user requires ACT grasps for four or five randomly generated objects, and sparse checks while long jobs survive terminal disconnects. Work is in `/home/mustafa/dimos-wt/r1pro-act-sim`. The shared root checkout and its environment are untouched.

**Latest steering:** everyday objects such as mugs, stationery and toys remain the eventual goal, but the user explicitly asked not to restart or discard the current work to pursue that expansion now. Preserve the 115 image demonstrations and trained weights; reuse them with targeted new examples when further fitting is justified. The 30,000 figure was optimizer updates, **not episodes**. No new long training run has been started. Downloaded household scans are preparation for later, not a claim of learned household grasping.

The original 10,000-update pilot scored **6/12 single picks and 0/8 complete scenes**. Refinement completed another **30,000 updates on those same 115 demonstrations**, taking 97m35s and scoring **9/12 singles and 1/8 complete scenes** (14/21 attempted sequence picks). There were no classical grasp interventions. The current checkpoint is `recordings/r1pro-act-task/jobs/random-objects-act-refine-v1/policy`; its deployment default remains 20 actions per inference. Do not promote it as reliable full-scene or agentic random-object support.

A completed execution-horizon comparison reused the same saved model and development seeds, without training:

| Actions executed per inference | Single picks | Complete scenes | Successful sequence picks |
|---|---:|---:|---:|
| 20 (checkpoint default) | 9/12 | 1/8 | 14/21 |
| 30 | 11/12 | 2/8 | 14/20 |
| 10 | 7/12 | 0/8 | 13/21 |

Evidence: `jobs/random-objects-act-horizons-v1/comparison.json`. These small development sets do not establish acceptance or justify silently changing the saved artifact. Refinement and horizon supervisors have finished; read each job's `status.json` and exit marker for status instead of relying on an old PID.

Native Zenoh + ControlCoordinator integration now has a standalone validation runner, `dimos.robot.galaxea.r1pro.demo_object_packing_stack`. The selected-object run in `jobs/random-objects-native-01` physically passed for seed 210000, index 2. The complete native sequence in `jobs/random-objects-native-02` placed indices 2, 0 and 3; index 1 timed out without lifting. ACT stopped cleanly. Existing SHM resource-tracker teardown warnings were also observed. **A registered random-object agentic blueprint and recovery integration remain pending.**

The start/occupancy variation physics pilot completed **23 accepted / 24 attempted** demonstrations across 12 layouts. It perturbs the initial right-arm posture by up to .02 rad and preloaded tray occupants by .012 m, preserving geometric clearance. One approach failed at seed 120007, target 1; all failures are retained. This was a no-image demonstrator check, **not 23 additional trainable RGB episodes**. Results: `jobs/random-objects-variation-smoke/manifest.json`.

The demonstration generator uses the SDK's Pink IK with a weak current-posture task (`posture_cost=1e-5`) and an inward joint-limit margin. It favors continuity near the preceding pose, not a fixed neutral elbow/torso posture. ACT rollout itself commands joint targets; it does not run this teacher IK. Do not add a competing posture controller during ACT execution without validating the resulting behavior. Stronger demonstration posture preferences should be tested separately before changing the training distribution.

## Implementation and actual evidence

- `object_packing_scene.py` generates independently positioned, sized, coloured and weighted boxes, cylinders and compound bottles. There are four or five objects, with randomized partial occupancy. Bounded whole-layout restarts preserve source finger clearance. A local overlay gives the new task a 29 cm internal tray and enough release clearance for short objects; the existing bottle scenes are unchanged.
- `object_packing.py` defines 52 simulator-ground-truth features: selected object relative to TCP, destination relative to TCP, object orientation/dimensions/shape, home relative to TCP, and sorted neighboring poses/extents with padding masks. There are also two RGB cameras and 20 measured joints. ACT outputs all 20 joint targets. This is a bounded primitive-shape distribution, not arbitrary unseen household categories.
- `object_packing_task.py` uses the new manipulator SDK for offline demonstrations only. Learned evaluation never calls teacher actions or IK. Success requires physical bilateral grasp/lift, upright supported release, containment, settled motion and return home; non-target motion/collisions are checked. Geometry and contact support are separate predicates so tiny momentary support-force changes do not falsely report spills.
- Teacher pilots 01–06 exposed arm/furniture contacts, infeasible torso routes and insufficient short-object release clearance. Pilot 07 passed **15/15** physical picks across eight layouts, all three families and 0–3 initial occupants. Image smoke passed **4/4**, followed by successful conversion, initialization, five-update fitting, export and a deliberately short learned execution. That smoke did not prove learned success.
- Full collection in `jobs/random-objects-act-v1`: **115 accepted / 118 attempted**, 64 layouts, up to two selected targets per layout, with 43 cylinders, 39 boxes and 33 bottles. Each successful pick is an independent episode; this is not an enumeration of full-scene permutations.
- Pilot training completed **10,000 updates**, batch 32, about five dataset epochs, in **32m14s**. Diagnostic loss fell from .0379 at 2500 updates to .0239 at 10000; that loss is not the acceptance metric. Paired choices from the same layout stay on the same side of the diagnostic split.
- Initial evaluation attempts crashed when a successful pick returned `numpy.bool_` to JSON. `pick_complete()` now returns a Python bool, with a real physical grasp + JSON regression. Incomplete reports are archived under `eval-*-incomplete*`; use only the current complete `eval-single/result.json` and `eval-sequences/result.json`. The corrected evaluation finished at **6/12 and 0/8**. The pipeline now rejects incomplete evaluation reports instead of treating partial totals as a completed evaluation.
- Recorded-state prediction diagnostics are saved in `fit-diagnostics.json`: typical active-joint MAE .003–.016 rad, with some held-out lift errors up to .027 rad. Additional fitting is a hypothesis being tested, not an established fix. Current physical seeds (200000+ and 210000+) are development evaluations; freeze a new final test set before acceptance.

## Incremental training and verification

- `prepare_object_act.py` can initialize from an already-trained, matching object profile without reinitializing its environment projection. Same-width but incompatible profiles are rejected.
- Warm starting also preserves the original observation/action normalization. LeRobot otherwise replaces saved processor statistics with the new dataset statistics when fine-tuning, which can change physical predictions before the first update. The initializer saves the new dataset's original statistics in `normalization-before-warm-start.json`, then uses the checkpoint's mean/std. Pass a **newly converted dataset**, never the archived baseline dataset. The source checkpoint is read-only.
- A small local ACT regression proves bit-exact physical action predictions before/after warm-start preparation despite deliberately different new-data statistics. Both initializer tests pass. Mixing additional image demonstrations with old episodes and choosing the next training budget still require a deliberate follow-up; no expanded-data run has been launched.
- Shared `ObjectPackingState` provides the same read-only geometry, goal vector and physical evidence to native/offline execution. The physical regression compares both monitors without allowing metadata queries to change qpos/qvel/ctrl. Both nominal and perturbed initial postures pass.
- The geometry/physical tests passed (30 tests), and the generated blueprint registry check passed (6 tests) after its expected regeneration. Strict typing is checked separately in native and isolated LeRobot environments.

Previous commits `14f90296c6`, `cf2346856c`, and `a1137b2c81` were pushed to `origin/feat/r1pro-act-sim` without coauthor trailers. This follow-up contains native integration, variation collection support, exact training progress counts, warm-start preservation and this updated handoff. Recordings, weights, downloaded assets, logs, `.venv` and local data links are excluded from commits.

Main integration remains pending: a rebase tried to replay 92 dependency commits and conflicted with main's relocation of the isolated Python runtime. It was aborted cleanly; `backup/r1pro-before-random-objects-20260911` preserves the previous branch. Do not claim this branch is rebased or force-push it casually.

## Later household-asset work

The existing HSSD package includes a real `flower_mug` mesh and 12 convex collision pieces. Its local mesh is about 7.4 × 10.5 × 8.2 cm; world AABB dimensions must not be mistaken for local geometry. Other local assets include a whiskey bottle, bowl, journal and camera. Scans of a small wind-up dog, shark toy, tape roll, scissors and coffee mug were downloaded to `recordings/r1pro-act-task/household-assets`, with their Google Scanned Objects source metadata/license files retained. These are not integrated, grasp-qualified or used in training. Do not replace their geometry with disguised bottle colliders or shrink them silently to fit the hand. Preserve handles/cavities through appropriate collision decomposition, and establish grasp/support frames before changing the observation contract.

## Next work

1. Preserve the current image dataset and refined checkpoint. Continue improving repeated selected-object execution and explicit failure recovery; avoid another long fit on unchanged data without evidence.
2. Validate native requested-target selection, stop-when-full and repeated picks. The last native four-object sequence passed only three picks; do not call it accepted. Keep ACT as the grasp executor.
3. If collecting more training data, add targeted start-state/approach/occupancy corrections with images, retain the original demonstrations, and fine-tune a new artifact from the saved weights. Keep source artifacts and normalization intact; freeze a fresh final test set before acceptance.
4. Revisit actual household geometry after the current version is stable, per the user's latest steering. Current primitive data remain useful rehearsal/pretraining but are not demonstrations of handles, thin stationery or irregular toys.
5. Integrate current main carefully. The earlier multi-stop unloading/regrasp SDK QP failure remains separate; existing tray carrying remains classical.

# planner/research/ml — the learned local planner

Empty here; the lab exists and is further along than the control side.

| | |
|---|---|
| lab repo | `autoresearch-mlplanner` (split out of `autoresearch-planner`) |
| local checkout | `~/coding/autoresearch-mlplanner` |
| status | pipeline built 2026-08-02, **never trained** — `logs/` and `checkpoints/` are empty |
| candidate seam | `--planner ml.candidate:make` |

Its README paths still name `dimos/navigation/motion2/` as the referee. That is
this repo's `planner/referee/`: motion2 was moved and renamed (the crate is
still `dimos_motion2_target`, pinned at `d7c1b7c88`). Fix the paths there
before running it.

## What is already built

`obs.py` (cloud → egocentric truncated distance field, 128×128 at 0.05 m, plus
goal and embodiment vectors), `model.py` (strided-conv encoder → 16 waypoints
+ a refusal logit), `data.py` (gold-imitation shards), `train.py` (adaptive
budget: stops on plateau, failfast or cap), `candidate.py` (the episode factory
the referee loads). v1 is pure imitation of the gold SE(2) oracle.

Seed hygiene is already enforced: seeds `0..999` are the eval battery and
`data.build_sample` raises on them; `5_000..5_199` validate; `10_000+` train.

## The shape this should keep

**Propose, then verify.** A collision is `gate = 0`, so an unchecked network
plan cannot ship. What can ship is a net that proposes a path — or seeds the
existing optimizer's corridor — with the deterministic geometry check keeping
the veto. That also means the ML win here is *quality against the gold oracle*
where evo plateaus, not speed: the rust crate already meets the 20 ms budget,
and speed is the bottom-tier pillar anyway (`gate * (100*gold + 10*consist +
1*speed)`).

**Shipping means ONNX + tract**, single-threaded deterministic rust inside the
candidate crate, with `obs.py` ported alongside. Python inference in the lab
proves the accuracy ceiling, not the deployment path.

## Known blind spots in the baseline observation

All deliberate, all fair game: the distance field is *unsigned* (a box interior
wider than 2× truncation reads as free space); the 6.4 m window is smaller than
the 3.5–7 m goals, so the far half of a long maneuver comes from the goal vector
alone; there is no memory across `plan()` calls; and 16 arc-resampled waypoints
cannot express an in-place fan.

And the referee's clouds are noise-free — box surfaces on a 0.05 m grid, full
360° coverage including the far side of every obstacle. A net that overfits to
perfect clouds will not survive the robot's.

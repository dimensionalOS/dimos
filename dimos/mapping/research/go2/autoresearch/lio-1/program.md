# lio-autoresearch

This is an experiment to have the LLM do its own research: autonomously tune a
LiDAR-inertial odometry (Point-LIO) pipeline so its trajectory best agrees with
the robot's onboard leg-inertial odometry (`robot_odom`), which is our rough
ground truth.

## Setup

To set up a new experiment, work with the user to:

1. **Agree on a run tag**: propose a tag based on today's date (e.g. `mar5`). The branch `autoresearch/<tag>` must not already exist — this is a fresh run.
2. **Create the branch**: `git checkout -b autoresearch/<tag>` from current master.
3. **Read the in-scope files**: The repo is small. Read these files for full context:
   - `README.md` — repository context.
   - `prepare.py` — fixed data paths, the Point-LIO substrate location, and the evaluation (the `evaluate` ground-truth metric). Do not modify.
   - `train.py` — the file you modify. The Point-LIO `CONFIG` and the run/eval driver.
4. **Verify the build + data exist**: Check that `point_lio/build/pointlio_mapping` exists and that `python prepare.py` reports the input bin and ground truth present. If not, tell the human to run `./setup.sh`.
5. **Initialize results.tsv**: Create `results.tsv` with just the header row. The baseline will be recorded after the first run.
6. **Confirm and go**: Confirm setup looks good.

Once you get confirmation, kick off the experimentation.

## Experimentation

Each experiment runs the offline Point-LIO once on the recorded Go2 LiDAR+IMU
stream (a fixed input). It runs to completion on CPU — typically ~1-3 minutes.
You launch it simply as: `python train.py` (in the dimos venv / `nix develop`).

**What you CAN do:**
- Modify `train.py` — this is the only file you edit. The `CONFIG` dict (which maps 1:1 to the Point-LIO yaml) is fair game: covariances, plane threshold, match scale, voxel filter sizes, IMU integration dt, extrinsics, blind range, FoV, etc. You may also adjust the Python pre/post-processing in this file.

**What you CANNOT do:**
- Modify `prepare.py`. It is read-only. It contains the fixed data paths, the substrate location, and the evaluation harness.
- Modify the `point_lio/` C++ substrate. It is the fixed Point-LIO engine, built once by `setup.sh` (the analog of a fixed model framework). You tune its configuration via `CONFIG`, not its source.
- Install new packages or add dependencies. You can only use what's already in the dimos venv (numpy, matplotlib, and the dimos package).
- Modify the evaluation. The `evaluate` function in `prepare.py` is the ground truth metric.
- Use anything in `human-debug/`. The convert script (`mcap_to_plnr1.py`), the raw `.mcap` recording, and the `.rrd` rerun file there are for **human debugging only** — they are NOT part of the experiment. The input bin and ground truth are already prepared in `data/`; never regenerate them in the loop.

**The goal is simple: get the lowest `val_ate_xy`** — the 2D (xy) absolute trajectory error of the rigid-aligned LIO trajectory vs the `robot_odom` ground truth, in meters. The metric is **2D only** for now (this recording is flat and single-story, so the robot's z is near-constant and uninformative); 3D criteria (measured height/range change at specific timesteps) will be added later once a stairs recording is captured. The only constraint is that the code runs without crashing and finishes within the time budget.

**Simplicity criterion**: All else being equal, simpler is better. A small improvement that adds ugly complexity is not worth it. Conversely, removing something and getting equal or better results is a great outcome — that's a simplification win. When evaluating whether to keep a change, weigh the complexity cost against the improvement magnitude.

**The first run**: Your very first run should always be to establish the baseline, so you will run the training script as is.

## Output format

Once the script finishes it prints a summary like this:

```
---
val_ate_xy:        10.964000
final_err_xy:      46.178
loop_close_xy:     55.854
gt_loop_xy:        0.465
path_len:          182.41
gt_path_len:       16.52
num_poses:         576204
overlap_s:         63.0
run_seconds:       95.3
```

You can extract the key metric from the log file:

```
grep "^val_ate_xy:" run.log
```

Each run also writes `viz.png` (top-down LIO-vs-gt + error-over-time) and
`traj_ds.tsv` (a small downsampled trajectory). On a **keep**, commit these two
alongside the experiment (e.g. `git add viz.png traj_ds.tsv` then amend or a
follow-up commit) so each kept experiment in the git history carries its
visualization. The full trajectory stays in `point_lio/Log/` (untracked).

## Logging results

When an experiment is done, log it to `results.tsv` (tab-separated, NOT comma-separated — commas break in descriptions).

The TSV has a header row and 4 columns:

```
commit	val_ate_xy	status	description
```

1. git commit hash (short, 7 chars)
2. val_ate_xy achieved (e.g. 10.964000) — use 0.000000 for crashes
3. status: `keep`, `discard`, or `crash`
4. short text description of what this experiment tried

Example:

```
commit	val_ate_xy	status	description
a1b2c3d	10.964000	keep	baseline (v2_imu)
b2c3d4e	9.880000	keep	loosen lidar_meas_cov to 0.05
c3d4e5f	12.300000	discard	match_s 9 (lidar-only style)
d4e5f6g	0.000000	crash	extrinsic_R singular (typo)
```

## The experiment loop

The experiment runs on a dedicated branch (e.g. `autoresearch/mar5`).

LOOP FOREVER:

1. Look at the git state: the current branch/commit we're on
2. Tune `train.py`'s `CONFIG` with an experimental idea by directly hacking the code.
3. git commit
4. Run the experiment: `python train.py > run.log 2>&1` (redirect everything — do NOT use tee or let output flood your context)
5. Read out the results: `grep "^val_ate_xy:\|^run_seconds:" run.log`
6. If the grep output is empty, the run crashed. Run `tail -n 50 run.log` to read the trace and attempt a fix. If you can't get things to work after more than a few attempts, give up.
7. Record the results in the tsv (NOTE: do not commit the results.tsv file, leave it untracked by git)
8. If val_ate_xy improved (lower), you "advance" the branch, keeping the git commit
9. If val_ate_xy is equal or worse, you git reset back to where you started

The idea is that you are a completely autonomous researcher trying things out. If they work, keep. If they don't, discard. And you're advancing the branch so that you can iterate. If you feel like you're getting stuck in some way, you can rewind but you should probably do this very very sparingly (if ever).

**Timeout**: Each experiment should take ~1-3 minutes. If a run exceeds 10 minutes it is killed automatically (treated as a failure — discard and revert).

**Crashes**: If a run crashes (a bad config, a bug, etc.), use your judgment: If it's something dumb and easy to fix (e.g. a typo, a singular matrix), fix it and re-run. If the idea itself is fundamentally broken, just skip it, log "crash" as the status in the tsv, and move on.

**NEVER STOP**: Once the experiment loop has begun (after the initial setup), do NOT pause to ask the human if you should continue. Do NOT ask "should I keep going?" or "is this a good stopping point?". The human might be asleep, or gone from a computer and expects you to continue working *indefinitely* until you are manually stopped. You are autonomous. If you run out of ideas, think harder — re-read the in-scope files for new angles, try combining previous near-misses, try more radical configuration changes. The loop runs until the human interrupts you, period.

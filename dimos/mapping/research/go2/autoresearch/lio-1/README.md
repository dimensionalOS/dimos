# lio-autoresearch

An [autoresearch](https://github.com/karpathy/autoresearch)-style autonomous
experiment loop, adapted from LLM pretraining to **LiDAR-inertial odometry
tuning**. Give an AI agent a real Point-LIO pipeline and a recorded Go2
LiDAR+IMU stream, and let it experiment autonomously: modify the config, run
the LIO, check whether the trajectory agrees better with the robot's onboard
odometry, keep or discard, repeat.

## How it works

Three files matter (same idiom as the original autoresearch):

- **`prepare.py`** — fixed data paths, the Point-LIO substrate location, and the
  evaluation (`evaluate`, the ground-truth metric). Not modified.
- **`train.py`** — the single file the agent edits. Holds the Point-LIO `CONFIG`
  and the run/eval driver. **Edited and iterated on by the agent.**
- **`program.md`** — baseline instructions for the agent. **Edited by the human.**

The agent does NOT touch the Point-LIO C++ engine (`point_lio/`) — that's the
fixed substrate, built once by `setup.sh`, analogous to a fixed model framework.
The agent tunes its configuration.

The metric is **`val_ate_xy`**: the 2D (xy) absolute trajectory error (RMSE,
meters) of the rigid-aligned LIO trajectory against `robot_odom`, the robot's
leg-inertial odometry, which loop-closes to ~0.47 m over a 16.5 m path and is
our rough ground truth. Lower is better. It is **2D only** today; 3D criteria
(measured step heights from a stairs recording) come later.

## The experiment

The Go2's L1 lidar is sparse (18 lines) and Point-LIO tracks well on straight
motion but loses lock during in-place rotation (the loop turn-around). The
research question this harness lets an agent explore: how far can configuration
tuning alone close the gap to `robot_odom`?

## Quick start

This package runs in the **dimos environment** — it carries no venv of its own.
The C++ build deps (cmake, eigen, pcl, yaml-cpp, boost) come from the Nix dev
shell defined in `../flake.nix`; the python deps (numpy, matplotlib) and
`dimos.get_data` come from the dimos `.venv`. The LIO input + ground truth are
pulled from the dimos LFS data store (`go2dds_data1`) by `get_data` on first run.

```bash
# 1. Enter the dev shell (C++ build deps layered on the dimos dev shell)
cd ..            # -> dimos/mapping/research/go2/autoresearch
nix develop

# 2. Build the substrate + sanity check (pulls the data via get_data)
cd lio-1
./setup.sh

# 3. Run a single baseline experiment (~1-3 min)
python train.py
```

If those work, you're ready for autonomous mode: point your agent at
`program.md`.

## Project structure

```
prepare.py        — data paths (via get_data), substrate location, evaluation (do not modify)
train.py          — Point-LIO CONFIG + run/eval driver (agent modifies this)
program.md        — agent instructions
setup.sh          — environment prep (build point_lio; checks the dimos venv)
config/v2_imu.yaml— the baseline config, for reference
point_lio/        — the fixed Point-LIO C++ engine (built by setup.sh)
human-debug/      — convert script, raw .mcap, .rrd rerun — HUMAN USE ONLY, not in the loop
```

Data lives in the dimos LFS store, not here: `data/go2dds_data1/` holds
`go2-185959.bin` (LIO input) + `gt_robot_odom.tsv` (ground truth), fetched via
`get_data("go2dds_data1/...")`.

## License

The `point_lio/` engine is GPL (see `point_lio/LICENSE`); it derives from
unitreerobotics/point_lio_unilidar. The harness files are MIT.

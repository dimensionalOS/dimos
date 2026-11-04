# 2. Replay and simulation

These checks do not require a physical robot. Prepare the replay datasets and MuJoCo dependencies for the blueprints below. Run each command separately, stopping the previous blueprint before starting the next.

## 2.1 Replay mode (no hardware)

```bash
uv run dimos run unitree-go2 --replay.db go2_short
uv run dimos run unitree-go2 --replay.db go2_bigoffice
```

Expected result: each run starts, processes the replay data, and displays it in the configured viewer without tracebacks.

## 2.2 MuJoCo simulation

The first runs pull the MuJoCo scenes and models from LFS and download the MuJoCo Menagerie robot descriptions from GitHub. Allow the initial asset downloads to finish before checking the viewers.

```bash
uv run dimos --simulation run unitree-go2
uv run dimos --simulation run unitree-g1-agentic-sim
uv run dimos --simulation run unitree-g1-sim
```

* With `unitree-go2` running, click a visible point on the mapped floor in the Rerun 3D view. Clicking empty background/grid space only selects the view and sends no goal. The robot walks to the selected point, and the planner logs `arrived=True`.

Expected result: each blueprint starts its simulated robot in the office scene without requiring a hardware connection. The agentic blueprint also starts its agent using the configured credentials.

## 2.3 Recording and playback

```bash
# Ctrl+C after about 30 seconds. The "Recording to ..." log line names the file.
uv run dimos --record --simulation run unitree-go2
uv run dimos mem summary recordings/<run-id>/memory.db
uv run dimos mem rerun recordings/<run-id>/memory.db
uv run dimos --replay-db recordings/<run-id>/memory.db run replay
uv run dimos run unitree-go2 --replay.db recordings/<run-id>/memory.db
```

Expected result: the summary lists one row per stream, including `lidar`, `odom` and `color_image`. `mem rerun` opens the recording in Rerun. The `replay` blueprint plays every recorded stream back, and `unitree-go2` runs on the recording instead of a robot connection.

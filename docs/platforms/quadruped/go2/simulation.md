# Unitree Go2 Simulation

Run the full Go2 navigation stack without any hardware: replay recorded sessions or simulate in MuJoCo. Same code, no robot.

See [Setup](/docs/platforms/quadruped/go2/setup.md) for installing dimOS first.

## Try It: No Hardware Needed

```bash
# Replay a recorded Go2 session (a .db, a dimos .mcap, or a dataset name)
# First run downloads the dataset from LFS
dimos run unitree-go2 --replay.db go2_ranch_short
```

Opens the Rerun 3D visualization. Watch the Go2 map an office in real time.

## MuJoCo Simulation

```bash
# The installer's default extras include sim (dim-go2-sim).
dimos run unitree-go2 --simulation mujoco
```

Full navigation stack in MuJoCo: same code, simulated robot.

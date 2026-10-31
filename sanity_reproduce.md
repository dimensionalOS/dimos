# Reproduce the C-prime OpenYAM offset test

Run `scripts/openyam/transfer_cprime.sh user@host` on the source machine. The
transfer creates `~/openyam-cprime` on the destination. Pass `--remote-root
/absolute/path` to choose another new directory. It copies the C-prime Git
branch, the pinned `can-motor-control` source, the OpenYAM model meshes, and the
small calibration files needed by the offset script. It does not install
software or command motors.

The tested DimOS robot code is commit `c7ec34c94`; later commits on the same
branch add unrelated files. The CAN fork is fixed at
`20572f3a41dd98d692da44556de0a0d10c2c8d9d`. The transfer keeps the local
DimOS commits and checks both transferred Git revisions.

## Install on the destination

Set `ROOT` to the directory printed by the transfer script. The destination
needs `uv`, a Rust/Cargo toolchain, and the system dependencies installed by
DimOS's installer.

```bash
ROOT="$HOME/openyam-cprime"
cd "$ROOT/dimos"
bash scripts/install.sh --mode dev --project-dir .
bash scripts/openyam/install_anvil_parity.sh "$ROOT/can-motor-control"
```

Install the CAN fork **after** the DimOS environment. `uv.lock` still resolves
the public `can-motor-control` 0.0.8 package; the tested fork installs as 0.1.0.
Use `uv run --no-sync` for the experiment, or reinstall the fork after any
`uv sync` that replaces it.

## Verify before hardware use

```bash
cd "$ROOT/dimos"
git merge-base --is-ancestor c7ec34c94 HEAD
git -C "$ROOT/can-motor-control" rev-parse HEAD
.venv/bin/python -c 'import importlib.metadata as m, pinocchio, coal; assert m.version("can-motor-control") == "0.1.0"; print("CAN fork and workspace libraries OK")'

CAL="$ROOT/calibration/openyam_camera_calibration_20261027_i3h7w6ox"
uv run --no-sync pytest "$CAL/gravity_friction_identification/test_dimos_joint_offset.py" -q
uv run --no-sync python "$CAL/gravity_friction_identification/demo_dimos_joint_offset.py" --help
```

The original machine used **left `can1`, right `can0`, both at 1 Mbps**. Check
the destination's interface names and bitrate with `ip -details link show`.
The test script accepts `--left-can` and `--right-can` when the names differ.
The calibration bundle and collision model describe the original robot; use
equivalent calibration for a different physical setup.

## Run the same right-arm test

Stop the DimOS teleop blueprint first. The script moves both arms to the
12-angle target, tests all six right-arm joints with a 3° change in baseline
and candidate modes, returns to its measured starting pose, and disables the
arm motors. Use a fresh output directory for each run.

```bash
cd "$ROOT/dimos"
uv run --no-sync python "$CAL/gravity_friction_identification/demo_dimos_joint_offset.py" \
  --target-deg 1.0 0.2 1.7 -2.5 1.1 -0.1 -6.1 67.6 48.6 -14.9 -0.3 1.0 \
  --delta-deg 3 --side right --mode both \
  --output-dir "$CAL/results/dimos_offset_right_arm_3deg_reproduction_01" \
  --execute
```

The output directory contains `manifest.json`, `results.json`, `results.csv`,
`comparison.csv`, and raw CAN and per-cycle samples. `results.json` should say
`"finished": true` and `"all_arm_motors_disabled": true` after a complete run.

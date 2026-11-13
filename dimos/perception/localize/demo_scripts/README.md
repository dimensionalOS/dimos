# Demo scripts

Scripts that drive a running localize blueprint on a real arm.

## Piper X: install the camera SDK

The Gemini 335 driver needs the Orbbec SDK Python package `pyorbbecsdk2`. It is not in `pyproject.toml`: it pins `av==12.3.0` and DimOS needs `av>=16.1`, so the two cannot be locked together. Install it once, by hand, without its dependencies:

    uv pip install --no-deps pyorbbecsdk2==2.1.2

`uv sync` removes packages that are not in the lock file. Run `uv sync` with `--inexact`, or install the package again after a sync.

## Piper X: run the blueprint

1) Start with the arm in its rest pose. `go_init` returns to the pose the arm had at startup.
2) Start the blueprint from the repo root:

    dimos run piper-x-localize

It checks the Piper USB-CAN adapter and the Gemini 335 with `lsusb`, then asks for `sudo` for two things:

1) to bring `can0` up at 1 Mbit/s
2) to give your user read access to `/dev/input/event*`, so the SpaceMouse module can read the `Ctrl` key and the puck. It uses `setfacl` for your user only; without `setfacl` it falls back to `chmod a+r`, which lets every user on the machine read the input devices.

Then it starts the camera, localize, Rerun, SpaceMouse teleop, planning, grasping and the MCP server.

3) From a second terminal:

    dimos mcp call go_home     # opens the gripper, moves to the table scan pose
    dimos mcp call go_init     # moves back to the pose the arm had at startup

## Stopping. Read this.

Never hit Ctrl+C on the running blueprint while the arm is up. The arm will fall.

Always:

1) `dimos mcp call go_init`
2) wait until the arm is back in its rest pose
3) only then stop the blueprint

## SpaceMouse

1) Hold `Ctrl` (left or right) and move the puck. The arm moves only while `Ctrl` is held.
2) Release `Ctrl`: the arm holds its pose.
3) Left button: switches between ARM mode (puck moves the arm in X/Y/Z) and GRIPPER mode (puck tilts, pans and rolls the gripper).
4) Right button: opens and closes the gripper.

Full puck deflection is 0.10 m/s (`linear_speed` in `demo_blueprints/piper_x_localize.py`).

## piper_x_shuffle.py

Scans the table from three views (left, home, right), picks every object in order and places it at a random free spot in front of the arm. Repeats for a number of rounds.

Run it with the blueprint up and the arm in `go_home`:

    python dimos/perception/localize/demo_scripts/piper_x_shuffle.py OUT_DIR \
      --labels "white toothpaste tube" "tennis ball" "golf ball" \
      --tries 2 --cycles 5

1) `OUT_DIR`: where the logs go. Goes first, before `--labels`.
2) `--labels`: the objects to locate, each one quoted. Default: toothpaste tube, roll of tape, tennis ball, marker, golf ball, bottle cap.
3) `--tries`: picks of one object per round before moving on to the next. Default 2. A failed pick is tried again from a new scan.
4) `--cycles`: rounds over all objects. Default 25.
5) `--dry`: moves to the three views, scans, exits. No picks.
6) `--seed`: seed of the random place spots.

Stop it:

    touch OUT_DIR/STOP

It finishes the current place, returns home and exits.

It stops by itself when:

1) the gripper is holding something at start, or after a failed pick
2) the gripper does not open at a place pose
3) an object cannot be placed anywhere
4) every scan move of a round fails

Logs: `OUT_DIR/shuffle.log` (one line per event) and `OUT_DIR/shuffle.jsonl` (the same as JSON).

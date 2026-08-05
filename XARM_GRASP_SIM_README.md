# xArm Grasp Simulation

This branch tests learned xArm grasps in MuJoCo with ground-truth object poses.

1. Clone the branch, initialize LFS assets, and install the needed extras:
   `git lfs pull && uv sync --extra agents --extra manipulation --extra sim --extra graspgenx --inexact`
2. Use a Linux host with a working MuJoCo renderer; for headless NVIDIA hosts, configure EGL for that host.
3. Clear stale DimOS shared-memory segments before a new run: `rm -f /dev/shm/dmjm_*`.
4. Start the simulation: `uv run dimos run xarm-grasp-sim-agent`.
5. In a second terminal, run `uv run dimos humancli`, then ask it to scan and pick an object, for example `Pick up the can.`
6. Watch `uv run dimos log -f` and the Viser UI for the pick phases and candidate-rejection details.

The test scene uses simulator ground truth instead of camera perception.  During a pick,
all object collision obstacles are temporarily suppressed to isolate grasp execution; the
table remains a collision obstacle.  A pregrasp success followed by a grasp failure points
at approach, contact, gripper closure, or verification rather than scene-perception failure.

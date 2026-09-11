# R1Pro home demo

```bash
dimos run r1pro-home-sim
```

Run this from a desktop terminal. The blueprint opens the native MuJoCo window,
packs five bottles using ACT, grasps the tray with both hands, drives through the
house using KronkNav, and places the tray on the table beside the laptop. It keeps
the viewer open after finishing. Press **Ctrl-C** or use `dimos stop` to stop the
stack; `dimos restart` starts a fresh demonstration.

Zenoh is the default transport. Scene generation, the static house point cloud,
policy runtime, CPU thread limits and result directories are handled by the
blueprint. No OpenAI key, `PYTHONPATH`, `MUJOCO_GL`, LCM address, or scout address
is needed. This command uses the native display; EGL is unnecessary.

On this workstation the user-level `dimos` launcher is installed from the
`r1pro-act-sim` worktree. Use a fresh terminal if an activated environment from
another checkout is taking precedence over that launcher.

## Speed

The previous navigation cruise was **0.055 m/s**, with an actuator cap of
**0.08 m/s**. The new cruise and cap are **0.6 m/s**. The controller limits linear
acceleration/deceleration to 0.4 m/s² and lateral acceleration to 0.15 m/s²,
and slows for bends and the final approach. Initial clearance from the worktop
and the carrying turn remain cautious. Arm/policy timing is unchanged.

A physics replay of the loaded route took 19.64 seconds at the new setting,
compared with about 82 seconds at the old setting. All five bottles stayed
upright and contained, both hands kept contact, and no obstacle contacts occurred.
This is simulated carrying validation, not a hardware calibration.

## Assets and options

The existing trained checkpoint is
`recordings/r1pro-act-task/policy-packing-augmented`; the default house is
`dimos/data/scene_packages/hssd_102344115`. These large local assets must be
available alongside an installation of this branch with simulation, manipulation,
and learning dependencies. On this workstation they are already available.
A fresh checkout also needs the native planner built once:

```bash
cargo build --release --locked -p dimos-mls-planner --bin mls_planner
```

Optional overrides use the regular blueprint CLI:

```bash
dimos run r1pro-home-sim --seed 5001
dimos run r1pro-home-sim --artifact /path/to/policy --scene-package /path/to/house
```

Each launch writes a fresh directory under `recordings/r1pro-home-sim/` and logs
its path. `result.json` contains packing and delivery results, measured cargo
history, and the planned route. The overall `success` flag requires physical
release and support at the destination, as well as successful packing.
Use `dimos status` and `dimos log -f` for ordinary runtime inspection.

## How the trip runs

The simulated lidar supplies the complete static house point cloud. It includes
physical surfaces and the floor, while excluding the robot, tray and bottles.
This is perfect-map simulation without sensor noise or localization drift.

ACT controls each bottle pick through the coordinator's `policy_rollout` task.
After packing, `tray_manipulation` runs the two-handed pickup and placement.
KronkNav plans from measured odometry using the loaded carrying footprint; a
separate collision check validates the robot, tray and all five bottles along
the route. `tray_navigation`, a `HolonomicPoseFollowerTask`, executes that path
through the base's Twist interface. The twenty manipulation joints and three
base resources have separate ownership in the same ControlCoordinator.

The tray and bottles remain free physical bodies throughout the trip. There are
no attachments or live-pose teleports. Runtime checks stop on lost grasp,
spilled/tipped bottles, unexpected support contacts, or obstacles. Tray handling
and driving require no additional ACT training.

The older `demo_packing_stack` module remains an evaluation entry point; its
session-isolation arguments are unnecessary for the standard home blueprint.
The single-bottle demo is documented in [ACT_SIM.md](ACT_SIM.md).

## What is learned

A geometry planner selects an accessible bottle and an empty tray slot. ACT gets
the head and wrist RGB images, 20 measured joint positions, and eight simulator
goal values: source XYZ, destination XYZ, radius and half-height. It produces
joint commands for grasping, placing, releasing and returning home, executed by
the ControlCoordinator `policy_rollout` trajectory task. The model uses profile
`r1pro-sim-bottle-packing-v1`; the earlier `policy-free-tray` is incompatible.

The default source order works from left to right while clearing blocked rear
bottles. The legacy evaluation launcher can randomize accessible choices for robustness testing.
Placements fill rows and reserve space for opening the gripper. When no safe slot
remains, the planner reports `tray_full` and stops. It does not rearrange bottles
or claim optimal packing. A tilted bottle gets a larger conservative footprint.

Five original bottles (5 cm diameter, 14 cm tall) and the 21 cm square tray rest
on the worktop. A local MJCF overlay supplies them; the house package is unchanged.
This checkpoint covers matching bottles in a small workspace with ±3 mm source
variation. Mixed shapes, wider layouts, arbitrary objects, and learned tray
handling remain future work. Simulator geometry supplies goals; this demo does
not establish perception or grasp generalization to unseen items.

## Validation

The current `r1pro-home-sim` native-display Zenoh run (seed 5000) passed all five
picks, two-handed pickup, the fast KronkNav route and supported release at the
laptop table. Measured route time was 19.67 s, peak speed 0.6004 m/s, maximum tray
tilt 6.99°, and final tray position error 4.22 mm. All 696 navigation samples
kept both-hand contact and all bottles upright/contained, with no obstacle
contacts. Evidence: `recordings/r1pro-act-task/home-zenoh-5000/result.json` and
`recordings/r1pro-act-task/jobs/home-zenoh-5000/validation-summary.json`.
The following results document the earlier training and lower-speed runs.

Success requires bilateral finger contact, a real lift, upright placement within
15 degrees, full containment, release, settling, and an open gripper back at
home. ACT stops at completion, then physics is checked again while the controller
holds its last command. All five bottles must still pass at the end. Neither
native nor offline evaluation uses scripted motion fallback or attachments or
teleportation after reset.

- Teacher: 20/20 complete scenes and 100/100 picks, seeds 8200–8219. This validates
  demonstrations, not learned performance.
- Training dataset: 130 unique picks, with 120 training episodes and 10 held-out
  picks from two complete scenes. The 20,000-update continuation took 58 minutes;
  its best default-order offline test completed 3/5 scenes.
- Small image augmentation added 5,000 fine-tuning updates on the same dataset,
  about 15 minutes. The first fresh offline test completed **3/3 scenes and
  15/15 bottles**, seeds 9700–9702.
- Extended default-order offline check: **10/10 complete scenes**, all 50 bottles,
  seeds 9800–9809. Combined with the initial check, this is 13/13 scenes.
- Native augmented checkpoint: **6/6 complete runs**, all 30 bottles, seeds 9700,
  5000, 9800, 9801, 5001 and 5002, with clean cancellation and shutdown. The last
  run also verified the final display angle and captured the completed tray.
- Randomized accessible order: **5/5 complete scenes**, all 25 bottles, seeds
  9900–9904. These are five different source orders. Across the three offline
  evaluations, all **18/18 scenes and 90/90 bottles** passed.

The optional full tray delivery completed **two native end-to-end runs**, seeds
5000 and 5001, with all ten bottle deliveries successful. Both runs retained
four handle contacts and upright cargo throughout transport, then physically
released the tray on the laptop tabletop. The final placement error was below
5 mm in each run. The first development attempt stopped during the first ACT
pick on an observation synchronization error, before tray handling; the timing
guard remains unchanged. These two successes are repeat checks, not a broad
reliability estimate. Exact results and screenshots are in
`jobs/packing-delivery-5001/validation-summary.json`.

These tests cover the matching-bottle setup and small position changes described
above; they do not establish reliability for new shapes or larger rearrangements.

The cameras now render timestamped snapshots outside the physics lock. Full
native display is retained, and measured physics/control timing is approximately
real time. Camera initialization finishes before viewer creation to avoid the
GLFW X11 startup race. The initial viewer angle now faces the table from an
unobstructed side, showing the robot and tray. Policy camera views are unchanged.
Shutdown retains ownership of a blocked renderer and
rejects reconnect until it has actually stopped. Other stacks retain the default
of rendering on the simulation thread.

## Artifacts and background jobs

All paths below are under `recordings/r1pro-act-task` in this worktree:

| Path | Contents |
| --- | --- |
| `policy-packing-augmented` | Deployment checkpoint for the command above |
| `dataset-packing-full-conditioned` | Training and held-out demonstration episodes |
| `train-packing-augmented` | Fine-tuning configuration and checkpoints |
| `eval-packing-augmented-canonical/result.json` | Initial three-scene offline evaluation |
| `native-packing-resume-augmented-9700/result.json` | First complete native run |
| `native-packing-validated-5000/result.json` | Native run matching the documented seed |
| `eval-packing-augmented-extended/result.json` | Ten default-order offline trials |
| `eval-packing-augmented-random/result.json` | Five randomized-order offline trials |
| `jobs/packing-resume-validation` | Completed repeat-validation commands and logs |
| `jobs/packing-display-verified/native-final.png` | Captured native window after all five placements |
| `jobs/packing-display-verified/validation-summary.json` | Packing-only validation summary |
| `native-packing-delivery-5000-retry/result.json` | First full five-bottle tray delivery |
| `native-packing-delivery-5001/result.json` | Repeat delivery with elevated native view |
| `jobs/packing-delivery-5001/native-final.png` | Five bottles delivered beside the laptop |
| `jobs/packing-delivery-5001/validation-summary.json` | End-to-end measurements and earlier failure |

Each job saves its exact `run.sh`, `pid`, `stage`, log and final `exit-code`.
Jobs use independent sessions and survive terminal disconnection. A zero exit
code from the demo now means success; an unsuccessful completed rollout exits 1.
Closing with Ctrl-C exits 130 after saving result.json. Historical job wrappers
have their own exit behavior, so check physical success in result.json as well.
All fitting and validation jobs are complete, and their runtime processes have
exited. No further training is queued. Raw demonstrations, datasets and weights
are ignored local artifacts, separate from source commits. Keep them when moving
or cleaning this worktree.

```bash
cat recordings/r1pro-act-task/jobs/packing-resume-validation/stage
tail -f recordings/r1pro-act-task/jobs/packing-resume-validation/run.log
```

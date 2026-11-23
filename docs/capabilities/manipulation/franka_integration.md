# Franka Research 3 Integration

dimOS drives a Franka Research 3 (FR3) with the Franka Hand through
`FrankaConnection` (`dimos/robot/manipulators/franka/`), over Franka's
pylibfranka and the Franka Control Interface (FCI).

## Requirements

- Robot system version 5.9 or newer, with the FCI feature installed (Desk →
  Settings → System → Installed Features). Install it from Franka World if it
  is missing.
- `pip install pylibfranka` (0.18 or newer; tested with 0.21.3 on system 5.10.0).
- The workstation on Control's LAN port (the shop floor network), not the port
  on the arm's base. Franka's default is a static `172.16.0.2` for Control and
  `172.16.0.1/24` for the workstation.

## Prepare the robot in Desk

1. Release the joint brakes and switch to **Execution** mode.
2. Open the menu under the robot's address in Desk's top bar and choose
   **Activate FCI**. Keep Desk open while dimOS drives the arm.
3. Keep the emergency stop within reach.

## Run

```bash
dimos run fr3-planner-coordinator --frankaconnection.ip 172.16.0.2
```

The planner plans with the FR3 model from Franka's `franka_description`
(pinned in `config.py`) and the Hand's TCP as its tool frame. Its home pose
is Franka's ready pose.

## How it drives the arm

- **Arm.** Joint targets go to libfranka's `AsyncPositionControlHandler`: the
  robot plans toward each target itself, so targets arrive at the
  coordinator's 100 Hz instead of libfranka's 1 kHz callback. Franka marks
  this API experimental. Because the robot plans, the connection does not
  require a real-time kernel (`enforce_realtime=False`, as in Franka's own
  example for this API).
- **Sessions.** Starting an async session makes libfranka send all-zero joint
  positions, which the robot takes as a target, so the connection starts no
  session until the first joint target arrives, then immediately sends the
  measured pose as the target. A session ends by cancelling it (a StopMove),
  never with `stop_control()`, which finishes with all-zero positions too.
  While a session runs, a drift guard stops the robot if a joint leaves the
  span between where it was and its latest target by more than 0.02 rad.
- **Sessions end at their target.** An async session is a point-to-point
  motion: once the arm reaches its target and settles, the robot ends it, and
  libfranka reports "Unexpected reply to a Move command" through the session's
  feedback (not through `Robot.read_once()`), so the connection checks that
  feedback every cycle. Ended at the target, it is a finished session and the
  next new target starts another; a target within 0.01 rad of the arm starts
  none, so a move turning every joint less than that (a few mm at the tool)
  does not run. A robot can also end a session short of its target with no
  reflex and nothing in Desk (one FR3 did so on most moves, 1-10 deg short,
  while trailing a streamed trajectory). By default that faults the
  connection. With `hold_on_short_end` set (`--<arm>.hold_on_short_end true`)
  and the arm in no stop, the connection instead ends the session without a
  fault and holds where the arm is; a next target within one jump starts a
  fresh session that finishes the move, and one further away is refused by
  the jump check, so the caller re-plans from where the arm is. Three of these
  within 5 s, an ending in a stop, or the arm standing still over 0.02 rad
  from its target for 0.3 s while a session runs, faults the connection.
- **Speed.** The robot's planner is capped at `speed_fraction` of each joint's
  top speed (default 0.25, the value Franka's example uses). The model's
  velocity limits are scaled by the same share, so plans and the arm keep
  pace. Change the share in both places together (`speed_fraction` on the
  connection and in `fr3_model` / `make_fr3_model_config`), or a slower arm
  falls behind its plan and the jump guard halts it.
- **Safety checks.** A target more than `jump_limit_s` (0.1 s) of travel at
  the scaled speed from the measured position is refused, as is any target
  while the latest reading is older than 0.1 s. If commands stop for
  `deadman_timeout_s`, the arm is held where it is. Collision thresholds are
  20/40 Nm at the joints and `contact_force` (default 20/40 N) at the tool;
  the robot's tool-force estimate can read over 10 N with no contact when the
  arm is folded near its base.
- **Errors.** A reflex (collision, limit) or a stop from outside dimOS ends the
  control session and faults the connection, with libfranka's error names in
  `status().last_error`. Call `clear_errors()` (libfranka's automatic error
  recovery, as Desk's recovery does) once it is safe.
- **Hand.** The gripper joint is `arm/fr3_finger_joint1`, 0.0 (closed) to
  0.04 m (open) per finger. Closing calls `grasp()` with `grasp_force`
  (default 20 N), which holds an object rather than failing when the fingers
  are blocked; opening calls `move()`. The Hand is handled before the arm's
  checks in every command, so a refused arm target never drops a Hand command
  with it. pylibfranka's Gripper calls hold Python's GIL for as long as the
  fingers move (about a second), which would stall the arm's 100 Hz reading
  thread, so the Hand runs in a program of its own
  (`franka.hand_process`, a subprocess: dimOS modules may not have
  `multiprocessing` children). A `move()` holds a width without force: on
  startup the gripper task's first hold is a move, so restart the stack only
  with the Hands empty.
- **Straight paths.** `ManipulationModule.move_linear_path(waypoints, ...)`
  moves one tool through a list of world-frame displacements from where it is,
  in straight lines without stopping and at a fixed orientation, with an
  optional linear speed cap and corner blend (used to draw with a held pen).
- **What the planner does not know.** A trajectory reports `COMPLETED` once it
  has been played, also when the separation guard held the arm or the
  connection faulted: check the measured joints. Objects held in a Hand are not
  in the planning model: a long held object can be swept into the table by a
  planned move, so carry it with straight moves or check the plan's
  trajectory first (`examples/franka_tabletop_bimanual/pen_demo.py`, `carry`).

## Two FR3s clamped to a table

`fr3-tabletop-bimanual` runs two ordinary FR3s whose bases are clamped to the
same table. It is not Franka's bimanual product or any fixed dual-arm mount:
each arm has its own control box and FCI address, and where one base sits
relative to the other is measured on the actual table, not taken from a
drawing.

```bash
dimos run fr3-tabletop-bimanual --right_arm.ip 10.42.0.230 --left_arm.ip 10.43.0.53
```

- **Names.** Seen from behind the arms, looking the way they face, the arms
  are "right" and "left": joints `right/fr3_joint1` ... and `left/fr3_joint1`
  ..., Hands `right/fr3_finger_joint1` and `left/fr3_finger_joint1`, planning
  groups `right`, `left` and `both`. The world frame is the right arm's base.
- **Placement.** `franka/tabletop_bimanual.py` holds the left base's pose in
  the right base's frame, each wrist camera's hand-eye calibration, and the
  scene camera's pose: by default Dimensional's (`DIMENSIONAL_CALIBRATION`),
  or for another table a `TabletopCalibration` JSON file named by
  `DIMOS_FR3_TABLETOP_CALIBRATION` (example:
  `examples/franka_tabletop_bimanual/dimensional_tabletop_calibration.json`). The
  FCI addresses and camera serials default to Dimensional's too; override them
  on the command line (`--right_arm.ip`, `--scene.serial_number`, ...). They were measured with two ChArUco boards taped to the
  table, seen by both wrist cameras. Re-measure after unclamping or moving a
  base, moving a wrist camera on its mount, or moving the scene camera.
- **Keeping the arms apart.** Both arms, and a box standing in for each wrist
  camera and its bracket, are in one collision world, so every plan is
  checked against the other arm. Commands that do not come from the planner,
  such as a policy driving the coordinator, pass through a `SeparationGuard`
  before reaching either arm. With the other arm at its own latest target, it
  drops a joint target that would bring
  - the two grippers (hand and fingers) within `gripper_separation_m` (5 mm)
    of each other, which lets them meet for a handover at the risk of a light
    bump, or
  - anything else (arm links, wrist included, and the wrist cameras) within
    `arm_separation_m` (5 cm) of the other arm.

  The arm then stops where it is. A target that widens a too-small gap passes,
  so a stopped arm can be backed out. Face to face over the middle of the
  table, the grippers can close in until their TCPs are about 3.5 cm apart
  before a wrist or camera reaches its 5 cm limit.
- **Cameras.** A RealSense on each wrist and one on a tripod over the table,
  at 640x480, publish on `right_wrist_*`, `left_wrist_*` and `scene_*` streams.

## Skills and setup scripts

`examples/franka_tabletop_bimanual/` holds the scripts that run on a live
`fr3-tabletop-bimanual` stack: the calibration (hand-eye per wrist camera,
arm-to-arm, tripod), class-agnostic table perception, a handover skill (pick,
hand over, place any box-like object), a pen skill (pick, click, draw, click,
put down), a runbook, a write-up of the code, and notes of every hardware fault
seen. Its `README.md` lists them.

## Not supported yet

- Velocity, Cartesian and torque commands (positions only).
- Simulation: there is no FR3 MuJoCo scene, so `--simulation` is not wired.

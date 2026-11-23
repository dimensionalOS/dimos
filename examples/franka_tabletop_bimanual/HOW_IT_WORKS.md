# How the tabletop bimanual FR3 system works

Two ordinary Franka Research 3 arms, each with its own control box, are clamped to
one table. They are driven together by dimOS. This is not Franka's bimanual product
or a fixed dual-arm mount: where one base sits relative to the other was measured
on the table.

This folder is `examples/franka_tabletop_bimanual/` in the dimOS repo (it began as `~/franka_3_setup` on the lab laptop); paths below starting `~/franka_3_setup` mean this folder.

This document covers:

1. The physical setup and the network.
2. Coordinate frames and calibration.
3. The dimOS side: the code in `~/dimos-franka`, file by file.
4. The task side: the scripts in `~/franka_3_setup`, function by function.
5. The safety layers, from planner to robot.
6. What failed, and the rules it left.

Numbers are as of 2026-11-23.

---

## 1. Physical setup and network

| | Right = arm 1 (the original) | Left = arm 2 |
|---|---|---|
| FCI / Desk address | 10.42.0.230 | 10.43.0.53 |
| Laptop port | USB ethernet `enx9c69d3c7dc9f`, 10.42.0.1/24 | built-in `enp130s0`, 10.43.0.1/24 |
| NetworkManager profile | `franka-shared-usb` | `franka-shared-2` |
| Wrist camera (RealSense D435i) | 261922077925 | 261922072855 |

- **Network.** Each arm is on its own little network shared from the laptop. Both
  profiles have autoconnect priority 10. Each arm's address is pinned by MAC in
  `/etc/NetworkManager/dnsmasq-shared.d/franka.conf`, so a reboot can't move an arm
  to another address.
- **Tripod ("scene") camera.** A D435i, serial 261922070328, on a short USB cable
  (the long cable ran at USB 2 and gave no frames).
- **"Right" and "left".** Named as seen from behind the arms, looking the way they
  face.
- **Layout.**
  - The bases are about 78 cm apart, each turned inward.
  - A 4×3 ChArUco "marker" board is taped flat between the bases. The tripod uses it
    to re-measure itself.
  - The paper, taped to a board, sits on the left arm's side.

**What talks to what:**

```
laptop ──FCI (UDP, 1 kHz robot side)──► each control box ──FSoE safety bus──► its arm
   │                                                        (the bus that keeps faulting)
   └──USB──► 3 RealSense cameras
```

---

## 2. Frames and calibration

**World frame = the right arm's base frame.** Everything is expressed in it.

| Transform | Value | How it was measured | Where it lives |
|---|---|---|---|
| Left base in world | x 0.1597, y 0.7579, z 0, yaw −0.39176 rad (−22.4°) | Both wrist cameras saw two ChArUco boards; a planar fit (x, y, yaw only), 4.3 mm RMS | `LEFT_BASE_XYZ` / `LEFT_BASE_RPY` in `tabletop_bimanual.py` |
| Each wrist camera on its Hand ("hand-eye") | right t (0.0531, −0.0161, 0.0656), left t (0.0532, −0.0201, 0.0623) | `handeye.py`: about 15 views of a board each | `wrist_camera_transform(side)` |
| Tripod camera in world | re-measured every episode | From the taped marker board, whose world pose the right wrist camera measured | `bimanual_cal.json` (`arm1_to_marker`), `scene_pose.json` |
| The table | about z = 0 | Each camera fits the table plane in its own depth (RANSAC) | `tabletop.fit_table` |

**Why the left base fit is planar:** both bases sit flat on one level table. An
earlier full 3-D fit came out with a 2.7° roll and a 2 cm rise. That error was
just big enough that the left camera saw the table as an "object".

**Why the tripod re-measures itself:** the tripod can be bumped, and the
original calibration had drifted 3–5 cm by tonight.
- `tabletop.refresh_scene` finds the marker board in the tripod image, on a 2×
  enlargement so all 6 corners are read.
- It solves the camera's pose from the board's known world pose.
- It saves the result to `scene_pose.json`, which is used if the board is ever
  hidden.
- Check: the paper's centre measured by the tripod and by the left wrist camera
  agree within 2 mm.

---

## 3. The dimOS side (`~/dimos-franka`, branch `franka-fr3-adapter`)

### 3.1 How dimOS is put together

- **Modules.** A dimOS program is a set of **modules**, each in its own process,
  connected by typed **streams** (`In[...]`, `Out[...]`) and called through
  **RPCs** (`@rpc` methods).
- **Blueprints.** A **blueprint** lists the modules, their settings and how their
  streams connect. `dimos run <blueprint>` starts them all.
- **Scripts.** Our scripts attach to the running stack with
  `Dimos.connect()` (the "porcelain" API). They call RPCs such as
  `app.ManipulationModule.plan_to_joints(...)`, and read streams with
  `app.<module>.peek_stream(...)`.

### 3.2 The blueprint: `franka/blueprints/tabletop_bimanual.py` (`fr3-tabletop-bimanual`)

It starts these modules and wires them so:

```
            plans (RPC)                trajectories            joint targets 100 Hz
Scripts ──► ManipulationModule ──────► ControlCoordinator ──► SeparationGuard ──► FrankaConnection "right_arm" ──► arm 1
                ▲  planner + collision      (tasks, 100 Hz)        (drops unsafe     FrankaConnection "left_arm"  ──► arm 2
                │  world of BOTH arms           ▲                    targets)            │ joint_state (100 Hz)
                └───────────────────────────────┴────────────────────┴───────────────────┘
RealSenseCamera ×3 (right_wrist, left_wrist, scene) ──► <cam>_color_image, <cam>_depth_image, <cam>_camera_info
```

- **`ManipulationModule`:** the planner. It uses the combined two-arm model
  (§3.3), has planning groups `right`, `left` and `both`, and knows the camera
  transforms for the `tf` tree.
- **`ManipulationSkills`:** wraps the planner's actions as agent *skills*
  (`move_to_pose`, `move_to_joints`, `go_home`, `open_gripper`, `close_gripper`,
  `get_robot_state`, `cancel`). An LLM agent in dimOS calls these. Our scripts call
  the planner directly instead.
- **`FrankaTabletopCoordinator`:** a `ControlCoordinator` with one output and one
  input port per arm (`right_position_command`, `right_joint_state`, ...). It runs
  three **tasks**:
  - `joint_trajectory` on all 14 arm joints, priority 10. It plays planned
    trajectories.
  - `right_gripper` and `left_gripper`, priority 20. Each maps 0.0–1.0 to the
    finger joint, with 1 = open (0.04 m).
- **Two `FrankaConnection`s:** the instances `right_arm` and `left_arm`, one per FCI
  address.
- **`SeparationGuard`:** sits between the coordinator and the connections.
- **Three `RealSenseCamera`s** at 640×480, plus a Rerun bridge for visualisation.

**`_remaps()` does the wiring:**
- The coordinator's `<side>_position_command` goes into the guard's
  `<side>_request`.
- The guard's `<side>_command` becomes `<side>_guarded_command`, which feeds that
  connection's `position_command`.
- Each connection's `joint_state` goes back to both the coordinator and the guard.
- Camera outputs are renamed `<camera>_<stream>`.

**Command-line overrides** use the form `--<instance>.<field> value`. The ones used
tonight: `--right_arm.grasp_force 70`, `--left_arm.grasp_force 30`,
`--separationguard.arm_separation_m 0.015`.

**What the coordinator does every 10 ms:**
1. Read each robot's joints.
2. Run the tasks.
3. Arbitrate per joint (highest priority wins).
4. Publish a target for every joint.

When no trajectory is playing, it keeps sending the last target (a "hold"). Two
consequences:
- You must never publish onto `<side>_position_command` yourself; that stream is
  already full of holds.
- Motion is injected through the coordinator (`execute_trajectory`) or the planner.

### 3.3 The model: `franka/tabletop_bimanual.py`

- `tabletop_bimanual_model()` builds one robot from Franka's `fr3` URDF, pinned to a
  commit in `config.py`:
  - a root link `tabletop` at the right base;
  - the FR3 attached twice, with prefixes `right/` and `left/`, the left one at
    `LEFT_BASE_XYZ` / `LEFT_BASE_RPY`;
  - on each Hand, a **box standing in for the wrist camera and its bracket**
    (`WRIST_CAMERA_BOX_*`, 4 × 10.5 × 5.7 cm), so the planner and guard protect the
    cameras.
- `make_tabletop_bimanual_model_config()` turns that into the planner's config:
  - the planning groups;
  - acceleration limits;
  - the Hand's self-collision exclusions.
- **Speed:** every joint's velocity limit is scaled by `speed_fraction` = 0.25, the
  same share the connection gives the robot, so plans and the arm keep pace.
- **Calibration helpers:**
  - `wrist_camera_transform(side)`: the hand-eye result, `fr3_hand` to `camera_link`.
  - `scene_camera_transform()`: the tripod's pose from the original calibration.
- **Naming helpers:** `arm_joint_names(side)` gives `right/fr3_joint1 ...`;
  `finger_joint(side)`, `link(side, name)` and `wrist_camera_link(side)` work the
  same way.

`config.py` holds the single-arm definitions these build on:
- `fr3_model` (URDF to model, with velocity limits scaled by `speed_fraction`);
- `FR3_READY`;
- `FR3_VELOCITY_MAX`;
- the Hand's collision exclusions;
- the single-arm wrist-camera transform.

### 3.4 Driving one arm: `franka/connection.py` (`FrankaConnection`)

This is the bridge to Franka's `pylibfranka`. It has one **input**,
`position_command` (joint targets in rad, plus the finger in m), and one
**output**, `joint_state` (positions, velocities and measured joint torques
`tau_J`).

**Connecting (`connect`):**
1. Opens `libfranka.Robot(ip)` and does one `read_once`.
2. Sets the collision thresholds:
   - joints: 20/40 Nm (lower/upper);
   - tool: `contact_force` (20/40 N). Above the upper threshold, the robot stops
     on a *reflex*.
3. Starts the Hand worker (below).
4. Does **not** start motion yet.

**Reading (`_read_loop`), a thread at 100 Hz:**
1. `read_once()`.
2. Publish `joint_state`, and every 10th time the Hand's width.
3. Run the watchers:
   - `_watch_for_stop`: the robot mode `Reflex` or `UserStopped` raises a fault.
   - `_watch_session`: the robot ended the motion away from its target, a fault.
   - `_check_drift`: a joint leaves the band between where it was and its target by
     more than 0.02 rad.
   - `_check_following`: the arm stands still more than 0.02 rad from its target
     for 0.3 s.

   Any of these sets `_fault`, ends the session, and refuses commands until
   `clear_errors()`, which does Desk's automatic error recovery.

**Commanding (`write`), called by the coordinator at 100 Hz.** It refuses a target
when:
- the latest reading is more than **0.1 s** old (that's the "joints were last read
  2.03 s ago" error in the fault logs);
- the target is more than 0.1 s of motion, at the scaled top speed, from the
  previous target. That's 0.066 rad for joints 1–4, the "told 0.815 rad, 0.461 rad
  from where it is" error;
- the target is more than 1 s of motion from the measured position.

A target within 0.01 rad of where the arm already is, with no motion running,
starts nothing. That's why **moves under about 5 mm don't execute**, and why the
pen demo's touch steps go up 8 mm first.

**Motion (`_begin_control`, `_send_target`):**
- Uses libfranka's `AsyncPositionControlHandler`: the robot plans its own way to
  each target, at `speed_fraction` (25%) of each joint's top speed.
- When a session starts, libfranka sends an all-zero target, so the connection
  immediately sends the measured pose (a "hold").
- Sessions end by being cancelled, never with `stop_control()`, which would also
  send zeros.
- Once the arm reaches a target and settles, the robot ends the session itself.
  That's a finished session, not a fault.

**The Hand (`_HandWorker`, `_hand_loop`, `hand_process.py`):**
- **Why a separate program:** pylibfranka's `Gripper.move` and `grasp` hold Python's
  GIL for about a second. In the same process that froze the arm's reading thread,
  stale readings were refused, and the arm faulted silently.
- **How it runs:** the Hand runs as its own program, `python -m
  dimos.robot.manipulators.franka.hand_process <ip> <force>`. Widths go to it over
  stdin and readings come back on stdout, as JSON lines. It isn't a
  `multiprocessing` child because dimOS modules run in daemon processes, which
  can't have children.
- **Commands:** only the newest width request matters.
  - Closing by more than 5 mm becomes `grasp(width, 0.05 m/s, grasp_force,
    epsilons = full stroke)`: hold whatever is between the fingers with that force.
  - Anything else becomes `move(width)`: position only, **no squeezing force**.
  - The gripper task's first hold after a stack start is a *move* to the current
    width. That's how a held pen lost its grip force tonight.
- **Reconnects:** on a network error it opens a fresh `Gripper` and resends the last
  goal.

### 3.5 Keeping the arms apart: `franka/separation_guard.py` (`SeparationGuard`)

The planner avoids collisions in its own plans. The guard protects against
commands from anywhere else, such as a teleoperator, a policy or a buggy script.

For each incoming joint target, `_on_request`:
1. Takes **the other arm at its own latest target**, or its reading if it has no
   target.
2. Sets both arms in a Drake collision world (`_gaps`, using
   `DrakeWorld.get_min_distance_between`, a function added for this).
3. Measures two clearances:
   - **grippers** (hand and fingers) against the other arm's grippers: must stay at
     least `gripper_separation_m` = **5 mm** apart, small enough for a handover;
   - **everything else** (links 0–7 and the wrist-camera box) against anything on
     the other arm: must stay at least `arm_separation_m` = **5 cm** apart, or
     1.5 cm with the pen demo's flag.
4. Passes or drops the target:
   - **passes it** if both clearances hold;
   - **drops it** otherwise, so the arm stops at its last accepted target;
   - **exception:** a target that *widens* a too-small gap still passes, so a held
     arm can back out.

`status()` reports the last clearances and how many targets each arm has had
refused. The scripts compare that before and after every move.

### 3.6 The planner: `dimos/manipulation/manipulation_module.py` (the parts we use)

| RPC | What it does |
|---|---|
| `plan_to_joints({side: JointState})` | Collision-free joint-space plan (both arms and any obstacles), stored as the pending plan; the result includes the trajectory |
| `execute(blocking=True)` | Sends the pending plan to the coordinator's `joint_trajectory` task |
| `move_linear(dx, dy, dz, planning_group, speed_scale)` | Plans and executes a straight TCP line in world coordinates, orientation fixed |
| **`move_linear_path(waypoints, ...)`** | Added tonight (commit `c166f1ff8`): a list of world-frame displacements from the current TCP, passed in straight lines without stopping, with optional `max_linear_speed` and corner blend (used to draw the smiley and step the touch) |
| `add_obstacle` / `remove_obstacle` / `get_obstacles` | Boxes in the planner's world (the table, a pen top) |
| `set_gripper_position(0..1, side)` | Goes to that arm's gripper task |
| `get_state().groups[side].joints` | Current joints, used everywhere |
| `cancel()` | Stops the running trajectory |

Two quirks to know:
- **False "COMPLETED".** A trajectory reports `COMPLETED` even when the arm didn't
  move: the connection faulted, or the guard held it. The scripts therefore verify
  every move against the measured joints or TCP.
- **Held objects are invisible.** The planner models the two arms, the camera boxes
  and any obstacles. It knows nothing about what a Hand holds.

### 3.7 Tests and docs

- **Tests:** `franka/test_connection.py` (the connection, against a fake
  pylibfranka), `franka/test_tabletop_bimanual.py` (the model, the guard and
  handover poses) and `blueprints/test_tabletop_bimanual.py` (the wiring), plus
  `manipulation/test_manipulation_primitives.py` (includes `move_linear_path`).
- **Docs:** `docs/capabilities/manipulation/franka_integration.md`.

---

## 4. The task side (`~/franka_3_setup`, local git repo)

All scripts use the stack through `Dimos.connect()`, so the stack must be running.

### 4.1 Perception: `tabletop.py`

Finds whatever is on the table, without knowing what it is.

- **`world_points(depth, K, T_world_opt)`:** turns depth pixels into 3-D points in
  the world.
- **`fit_table(pts)`:** RANSAC fit of the table plane in that camera's own points,
  so "above the table" is exact for that camera.
- **`above_table(...)`:** keeps the points that are:
  - more than 2 cm (tripod) or 6 mm (wrist) above the plane;
  - inside the visible table;
  - more than 30 cm from either base axis, which excludes cables;
  - within 85 cm reach;
  - not on the taped board (`fixtures.json`).
- **`group(...)`:** puts those points on a 5 mm grid and joins connected cells.
  Each group gets a fitted box: centre, long-side direction (`yaw`), length, width,
  height (95th percentile) and the table height under it. Each box becomes a
  `TableObject`.
- **`look(app, model, cameras)`:** runs the above on the tripod and both wrist
  cameras, each in its own pose. Wrist poses come from the arm's joints through the
  model (`Model.hand`) and the hand-eye calibration.
- **`colour_candidates(...)`:** the fallback for flat things depth misses, such as a
  pen or the paper. It finds pixels whose Lab colour differs from the table's
  median, puts them on the table plane along their camera rays, and groups them
  into blobs.
- **`refresh_scene(app)`:** the tripod re-measure described in §2.
- **`table_outline`**, **`inside`**, **`draw`:** helpers. `Model` is Drake forward
  kinematics of both arms.

### 4.2 The handover skill: `handoff.py`

**`cycle()` runs one episode:**
1. `reset`: clear old faults, open both Hands, both arms to ready.
2. `refresh_scene`.
3. `stage_look`:
   1. The tripod and both wrist cameras find candidates. Candidates more cameras
      agree on come first.
   2. If depth finds nothing, the colour fallback runs.
   3. `look_at` puts the nearer arm's wrist camera 35 cm straight over the best
      candidate and measures it properly.
4. **Checks:**
   - more than 6.8 cm wide: refused, since the Hand opens 8 cm;
   - **thin** (under 2 cm across) or **short** (under 9 cm long): relay instead of a
     mid-air handover.
5. `stage_plan` (no motion):
   1. `handover_search` searches handover poses: object centre 42 cm out from the
      middle and 36 cm up; heading 0/±30/±60°; tilt 0–90°; roll 0/90/270°. Each
      pose is tried with three receiver grips (the object's bottom or either side).
   2. **Each candidate pose must:**
      - have IK for both arms, with the wrist joints within 170° of ready;
      - keep every hand part and camera at least 5 cm above the table;
      - have no collisions;
      - pass the **guard's own check at every step** of the receiver's straight
        approach and the giver's back-out.
   3. It stops after 3 feasible poses or 20 s, and gives up at 30 s. Poses that
      worked before are tried first (`handover_wins.json`).
   4. Then it picks which end of the object to grasp, the one needing the least
      wrist turn into the handover.
6. `stage_pick` (giver):
   1. Above the object, then straight down to the grasp height. That height comes
      from `grasp_pose`: pads at mid-height, fingertips 2 mm above the table, and
      the palm kept above a tall object. `FINGERTIP_REACH` includes the 1.75 mm
      measured on both arms.
   2. Close, and check the fingers stopped at about the object's width.
   3. Lift 12 cm.
7. `stage_present`: the giver moves to the handover pose.
8. `stage_take`:
   1. The receiver lines up 10 cm back, with the object's free half added as a
      planner obstacle.
   2. It comes in straight and closes, and must read about the expected width.
   3. **Only then** the giver opens and backs out straight.
9. `stage_place` / `place_object`: the receiver sets the object down flat
   somewhere free on its own side, chosen by random sampling with checks for the
   board, the table edge, the wrist limits and IK. It opens and goes to ready.
10. `relay` (thin or short objects): the giver sets the object down in the middle,
    the receiver looks at it with its wrist camera, picks it and places it.

**The `Arms` class is the safe wrapper every step goes through:**
- `move`: plan, execute, then check the measured joints are within 1.5° of the goal.
- `along` / `straight`: `move_linear`, then check the TCP moved within 1 cm of the
  intended vector.
- `gripper`: wait until the fingers stop, then check that opening reached open and
  closing really closed.
- `check`: fail on any new guard refusal or new arm fault since before the step.
- `add_table`: adds the tabletop to the planner as a box, so plans can't dip into it.
- `reset` and `recover`: clear faults, lift any arm that stopped low, both to ready.

**`solve(...)`** is the IK used everywhere: least squares from several seed poses,
rejecting solutions that collide or put a hand part into the table.
- `seeds` supplies the starting guesses.
- `bearing` gives the joint-1 angle pointing at a point.
- `camera_optical` gives the wrist camera frame on the Hand.

`loop -n N` runs `cycle` N times and logs each to `handoff_log.jsonl`.

### 4.3 The marker demo: `marker_demo.py` (stopped at the uncap)

**The idea:**
1. The right arm picks a marker by its cap and holds it up.
2. The left arm comes in end-on and grips the body.
3. Pulling apart removes the cap.
4. Draw, recap by retracing the same path, put down.

**Pieces reused later:**
- `find_marker` and `find_paper`, colour-based finders on the table plane;
- `look_with`;
- `smiley(paper)`: the strokes, upright for someone standing in front of the table;
- `jacobian` and `torques`: force from the joint torques, with `tau_J` noise about
  0.03 Nm;
- `push_until`.

**Why it stopped:** the cap needed more pull than either grip held. At 30 N the cap
slid out of the right Hand; at 60 N the body slid out of the left.

### 4.4 The pen demo: `pen_demo.py` (unfinished)

**Roles:** the arm nearer the paper is the **drawer** (left); the other is the
**clicker**.

**`all` runs these stages:**

| Stage | What it does |
|---|---|
| `look` | Both arms to ready, then re-measure the tripod. Find the pen (`find_pen`) and the paper with the tripod, then up close with a wrist camera. Save `pen_wrist.png` with the tip (red), the button (blue) and the grip point (green) marked |
| `relay` | If the pen is more than 62 cm from the drawer's base, the clicker moves it to a spot 42–52 cm from the drawer, and the drawer's camera re-measures it |
| `plan` | No motion. Find a grip and an upright pose where the drawer holds the pen and the clicker can reach the button, inside the guard's limits. Find a pose holding it upright over the paper |
| `pick` | Down over the grip point, close, lift straight up, then a *checked carry* (`carry_up`) to the upright pose |
| `click` | The clicker's camera, 30 cm above, finds the button by colour (`measure_button`). The fingertips move over it, drop to 6 cm above the estimate, step down 4 mm at a time until they feel it (`step_touch`), then push up to 12 mm, stopping when it bottoms out (`push_hold`) |
| `touch` | Checked carry high over the paper, straight down, then 1 mm steps until the tip touches |
| `draw` | The clicker moves to watch the paper (`view_pose`). The drawer draws a test tick, pressing deeper until ink shows (`new_ink`), then the smiley in one `move_linear_path`, checked after |
| `back`, `click`, `putdown` | Click the pen off and put it back where it was |

**Pen geometry:**
- `find_pen`: the pen is the blue blob. Its ends are everything along its line that
  isn't table colour. The tip is the end with silver past the blue.
- `pen_in_hand`, `pick_pose`, `upright`, `tilted`, `tip_of`: where the pen is
  relative to the Hand.

**How touch is sensed:**
- The force on the tool is estimated from the change in measured joint torques
  through the Jacobian. It reads about 0.2 N of noise when **still**, but 10–40 N
  of false force **while moving**, from the arm's own accelerations.
- So `step_touch` reads it only between steps. Each step goes up 8 mm and then down
  1 mm further, because shorter moves don't run.

**Carrying safely (`carry`, `carry_up`, `raised`, `lift_to`):**
1. Plan with the pen in hand.
2. Walk **every point** of the plan through the model with the pen attached
   (`pen_points`).
3. If any point brings the pen or the hand within 3 cm of the table or paper,
   refuse, and move nothing.
4. `carry_up` then lifts 8 cm and tries again.
5. Carrying moves go between poses raised 15 cm, with straight lines at each end.

**Tuned numbers, each with a comment saying why:**
- `TIP_CLEARANCE` (the left grasp aims lower);
- `GRIP_FROM_TIP` (6 cm, with the camera on the button side, so the pen can be
  upright both to draw and to be clicked);
- `PRESS`;
- `CLICK_STROKE` and `CLICK_STOP`.

### 4.5 Calibration and checks

| Script | What it does |
|---|---|
| `handeye.py plan/run/solve/refine [--arm 2]` | Eye-in-hand calibration: plans about 15 views of a board, moves slowly, solves `fr3_hand → camera_link` |
| `bimanual_cal.py plan/run/solve` | Each arm views the two boards; solves arm 1 → arm 2 (planar) and the tripod |
| `remeasure_marker_board.py` | After moving the marker board: the right wrist camera re-measures it and updates `bimanual_cal.json` and `fixtures.json` |
| `checks.py`, `height_check.py` | Hover over known points so you can measure by hand (the source of the 1.75 mm correction) |
| `soak.py` | Many moves on one arm, to reproduce faults |
| `view_cam.py` | Live camera viewer |

---

## 5. Safety layers, outermost first

1. **The planner:** plans are collision-free between both arms, the camera boxes
   and the tabletop box.
2. **The scripts:**
   - every move is verified against the measured joints or TCP;
   - every step compares guard refusals and arm faults before and after;
   - the giver opens only after the receiver holds;
   - held-object carries are checked point by point.
3. **`SeparationGuard`:** drops any joint target that brings the arms within 5 cm,
   or the grippers within 5 mm, whatever sent it.
4. **`FrankaConnection`:**
   - refuses targets on stale readings, jumps and targets far from the arm;
   - faults on drift, stalls and reflexes;
   - needs `clear_errors()` before taking commands again.
5. **The robot:** collision reflexes (20/40 Nm, 20/40 N), joint limits, its own
   speed limits at 25%, and Desk's safety.
6. **You:** the e-stop.

---

## 6. What failed, and the rules it left

The details are in `failure_notes.txt`.

- **Arm faults on the safety bus (FSoE).**
  - The right arm, from the evening of 2026-11-22. The left arm, from 01:03 on
    2026-11-23: four times in 25 minutes, once with nothing in its Hand, and once a
    reflex with nothing nearby.
  - Signature: the robot stops sending readings, then "UDP receive: Timeout", then
    FCI refuses connections.
  - Not caused by our commands. Something the two arms share, or heat after many
    hours, is the suspect. **Ask Franka support.**
- **Planned moves with a held pen hit the table twice.** The planner doesn't model
  what a Hand holds. Rule: carry with straight moves near surfaces and checked
  plans between raised poses.
- **Restarting the stack while holding:** the startup "move" drops the grip force,
  and a re-grasp from a held state can fail and release. Rule: never restart or
  re-grasp while holding.
- **False "COMPLETED"** from dimOS. Rule: verify every move against the measured
  state.
- **Gripper calls freezing the arm's reading thread.** Fixed by running the Hand as
  its own program.
- **Thin objects** (a marker or pen lying flat) need fingertips about 1 mm above the
  table. The arms differ by a couple of millimetres there.
- **Torque-based touch works only when the arm is still**, and moves under about
  5 mm don't execute.

---

## 7. Changes on 2026-11-23

**dimOS (`connection.py`, `manipulation_module.py`):**
- `move_linear_path`: straight-line paths through waypoints without stopping (used to draw).
- A motion the robot ends short of its target, in no stop, can now hold instead of faulting (`hold_on_short_end`, off by default); three within 5 s still fault.
- The Hand is handled before the arm's checks, so a refused arm target can no longer block opening or closing the Hand.

**Handover (`handoff.py`):**
- A move that ends short without a fault is planned again from where the arm stopped (twice at most); a straight move that ends short finishes the rest.
- When choosing which end to grip, the grasp itself must be reachable, not only the pose above it.

**Pen skill (`pen_demo.py`):**
- **Roles:** the arm nearer the paper draws, the other clicks. Today: left draws, right clicks.
- **Which end is the tip:** by placement rule. The pen is put down with its tip toward the tripod (`TIP_TOWARD_TRIPOD`); the silver and shape rules were unreliable from above.
- **Pick:** IK seeds include the last wrist joint turned half a turn; run with `WRIST_MAX_DEG=190`. The grasp height is checked after the descent and corrected, allowing for the arm ending short (never more than 3 mm below the target, and it stops when the fingertips rest on the table).
- **Carrying the pen:** to the paper and back in straight lines at constant orientation, from the clicking pose. Any planned move with the pen in hand is checked point by point first (`carry`), lifting 10 cm and re-planning when refused.
- **Click:** one look from 30 cm, aim corrected by an observed offset (`CLICK_AIM_OFFSET`, 8 mm away from the tripod), fingertips down, one straight press `CLICK_PRESS_BELOW` (10 mm) under the button's estimated top, then lift.
- **Drawing height:** from the model plus `TOUCH_MODEL_OFFSET`, set by eye (`lower_pen.py`): -2 and -6 mm drew too high, -12 mm pressed into a collision stop. The value at the end of the day is -15 mm and is untested. It varies with how the pen sits in the grip.
- **No camera ink check** (`INK_CHECK = False`).
- **Stack settings for the pen:** `--right_arm.grasp_force 70 --left_arm.grasp_force 15 --separationguard.arm_separation_m 0.015 --right_arm.hold_on_short_end true --left_arm.hold_on_short_end true`.

**Hardware:** see `failure_notes.txt`, sections e to i.

# R1 Lite Bring-up Log (living lab notebook)

Verbose, dated evidence trail of the R1 Lite dimos integration. Append-only:
new findings get new dated entries; earlier observations are never rewritten
(if one turns out wrong, a later entry says so and why). At the end of the
integration this gets distilled into a polished README in the style of
[`scripts/r1pro_test/README.md`](../r1pro_test/README.md).

Branch: `krishna/task/r1lite-integration` (stacked on
`mustafa/task/r1pro-coordinator-integration`, tip `9ab0ab318`; main was 204
commits ahead of the shared base at branch time — Mustafa nudged to rebase).

---

## 2026-07-02 — Day 1: discovery, environment, credentials, first recon

### 1. Finding the robot (the hard way, then the surprise)

**Hypothesis:** R1 Lite ships like the R1 Pro — ethernet at `192.168.2.150`
(factory) or no IPv4 at all.

**Tests & findings:**
- Laptop `enp130s0` was `NO-CARRIER` until cable plugged (check:
  `cat /sys/class/net/enp130s0/carrier`).
- Added runtime IPs to laptop without touching the saved "lidar" profile:
  `nmcli device modify enp130s0 +ipv4.addresses 192.168.2.100/24 +ipv4.addresses 192.168.123.100/24`
  (runtime-only — reverts on reconnect; saved profile keeps only 192.168.1.5).
- Pings to `.2.150` / `.123.150`: nothing.
- **IPv6 all-nodes multicast ping** (`ping -6 ff02::1%enp130s0`): one foreign
  responder, `fe80::c68f:8fbd:376b:a2`, MAC `e0:51:d8:1f:04:75` — works
  because every UP interface always has a link-local IPv6 even with zero
  IPv4 config. This is the no-sudo substitute for Mustafa's tcpdump trick.
- Ping sweep of 192.168.{1,2,123}.0/24 + `ip neigh`: `192.168.1.85`
  answered with the SAME MAC → device found. `nmap -sV`: hostname
  **`r1lite`**, OpenSSH 8.9p1 (Ubuntu 22.04), only port 22 open.
- **Gotcha (ARP flux):** laptop had TWO interfaces on 192.168.1.0/24 (wifi
  .110 + eth .5), and `.85` later turned out reachable only via WIFI. The
  robot answered ARP for its *wifi* address out its *ethernet* port.
  Resolved identity definitively by comparing SSH host keys over both paths
  (`ssh-keyscan 192.168.1.85` vs `ssh-keyscan fe80::...%enp130s0`) —
  **identical ed25519 key → same machine on both paths.**

**Conclusion:** direct cable to robot confirmed; robot also on office wifi
at `192.168.1.85` (someone provisioned it — non-factory state); ethernet
port *appeared* to have no IPv4 (see §5 for the correction).

### 2. Laptop environment (docker + py3.10)

- ghcr pulls needed `read:packages` on the gh token
  (`gh auth refresh -h github.com -s read:packages`).
- **Wrong-image trap:** compose file `docker/dev/docker-compose.yaml` uses
  `ghcr.io/dimensionalos/dev:latest` — that's the NON-ROS track (no
  `/opt/ros`). The ROS-track image is **`ghcr.io/dimensionalos/ros-dev:dev`**
  (what `bin/dev` uses). Launched manually:
  `docker run -d --name dimos-dev-r1lite --network host -v <repo>:/app ... ros-dev:dev`
- `/app/.venv` was the host's py3.12 venv (bind mount!) → interpreter
  doesn't exist in container. Rebuilt per Mustafa §12.A:
  `rm -rf .venv && UV_PYTHON=3.10 uv sync --all-extras --no-extra dds --no-extra unitree-dds`
  → `rclpy OK`, `dimos OK`. `.envrc → .envrc.r1pro` symlinked.
- ROS 2 domain sweep 0–101 from container (scanner now at
  `scripts/r1lite_test/domain_scan.py`): **zero foreign participants** —
  robot stack simply not running (confirmed later, §5).

### 3. Credentials saga

**Hypothesis:** Galaxea default `nvidia`/`nvidia` (all their docs say so).

**Tests:** 3 manual + scripted attempts as `nvidia@192.168.1.85` → all
`Permission denied`. Web docs (R1/R1 Lite/R1 Pro) all confirm nvidia/nvidia
as factory default → concluded unit was re-provisioned.

**Resolution:** actual login is user **`r1lite`** (robot's own hostname as
username), password obtained from [fill in: who/where]. Factory docs were
wrong for this unit because it's NOT the documented Orin config (see §5).
Key auth installed via `ssh-copy-id`; laptop `~/.ssh/config` has alias
`r1lite` → `r1lite@192.168.1.85`. Robot's `last` showed a login
2026-05-22 from `192.168.1.105` (the provisioner's machine, presumably).

### 4. Branch & scaffolding

- `krishna/task/r1lite-integration` branched off Mustafa's branch tip,
  tracking it (plain `git pull` follows his pushes). Strategy: stay
  additive, sync often, never edit his r1pro files here.
- `scripts/r1lite_test/` scaffolding ported from `r1pro_test/`:
  `r1lite_config.py` (single source of truth, `TODO(recon)` markers),
  `test_00_recon.py` (new — graph dump), `test_01..04,06` (parameterized,
  arm tests adapt DOF from live feedback), `run_all_tests.py`,
  `domain_scan.py`, `README.md`. All syntax-checked in container.
  Uncommitted pending hardware reconciliation.

### 5. First SSH recon — most prior assumptions overturned

Ran read-only identity/env recon over SSH. Findings:

| Assumed (from R1 Pro) | Actual (R1 Lite unit) |
|---|---|
| Jetson Orin, aarch64 | **x86_64 PC**, Ubuntu 22.04.5 |
| user `nvidia` | user **`r1lite`** |
| eth has no IPv4 | **eth `enp2s0` = `10.42.0.2/24`** (we only probed 192.168.x!) |
| ROS_DOMAIN_ID=41 | **`ROS_DOMAIN_ID=02`** (bashrc; note the string "02") |
| maybe ROS 1 | **ROS 2 Humble installed** — no upgrade needed |

More robot-side facts (from `~/.bashrc`, `ls ~`):
- `ROS_LOCALHOST_ONLY=0`, `ROS_IP=10.42.0.2` → robot pre-configured to
  talk ROS over the ethernet cable; laptop just needs a `10.42.0.x` addr.
  Mustafa's netplan step is NOT needed on this unit.
- Home dir: `can.sh`, `setup_can.sh`, `can.txt`, `first_start`, `lite`,
  `galaxea/install/`, `GalaxeaDataset`, `realtime_tools`, `test`,
  `findrobot_{server,client}.sh`, **`fastdds_unicast.xml`**,
  **`super_client_configuration_file.xml`** (DDS discovery-server/unicast
  configs — check whether launches depend on them before assuming
  multicast discovery works).
- `can0` interface DOWN; no ros/hdas/mobiman processes → stack fully idle.
  Explains the empty domain sweep in §2.

**Open questions carried forward:**
1. What do `first_start`, `lite`, `can.txt`, the findrobot scripts do?
2. Does the stack launch depend on the FastDDS XML configs (unicast /
   discovery server), and do we need matching client config in the container?
3. Topic names/types vs R1 Pro `/hdas/*` + `/motion_target/*` (test_00).
4. Arm/torso DOF (expected 6/4), torso safe home pose.
5. Chassis 3-gate problem present on R1 Lite's chassis node?
6. Camera/lidar inventory.

**Next session plan (agreed division of labor):** Krishna drives all
state-changing robot commands from a Cursor Remote-SSH window ("Console A");
Claude runs read-only SSH recon + laptop/container work. Steps: laptop
`10.42.0.100/24` → read startup scripts → Krishna starts CAN+stack →
`test_00_recon` from container on domain 2 → reconcile `r1lite_config.py`.

### 6. Startup-machinery recon complete (read-only pass over SSH)

**The password mystery, solved for real:** Galaxea's own `findrobot_server.sh`
hardcodes per-model credentials: `R1-LITE) passwd="1"`, `R1-PRO) passwd="nvidia"`.
And `can.sh`/`sessions.sh` do `echo "1" | sudo -S ...` throughout. So **R1 Lite
factory login = user `r1lite` / password `1`** — the unit was never
re-provisioned; the public docs just only cover the Orin models. (The office
wifi join was still done by someone — login 2026-05-22 from 192.168.1.105.)

**Hardware identity** (`/opt/galaxea/body/hardware.json`): R1-LITE with
ARM=A1X, TORSO=T0, ECU=E03, CHASSIS=C1 (+CS1 steering sw), ARM_END=EE0,
HEAD_CAMERA=HC0, WRIST_CAMERA=WC1. Compare R1 Pro: chassis W1 3-wheel swerve.

**DDS mode — multicast confirmed, three ways:**
- `.bashrc`: `ROS_DISCOVERY_SERVER` and `FASTRTPS_DEFAULT_PROFILES_FILE` lines
  are COMMENTED OUT; active: `ROS_DOMAIN_ID=02`, `ROS_LOCALHOST_ONLY=0`,
  `ROS_IP=10.42.0.2`.
- No listener on port 11811 while idle.
- `prestart_prepare.sh` / `r1lite_prestart_prepare.sh` (sourced by every
  module start script) set only paths/setup.bash — no DDS env.
- Caveat: the boot profile DOES start `fastdds discovery -p 11811`
  (ros2_discovery.yaml) but nothing points at it — vestigial. The
  `findrobot_server.sh` script would switch the robot INTO discovery-server
  mode (rewrites bashrc + reboots) — do NOT run it.

**Boot machinery decoded:**
- Entry point: `robot_startup.sh boot <sessions.d-profile-dir>` → tmuxp loads
  every `*.yaml` in the profile dir; `robot_startup.sh kill` = tmux
  kill-server + ros2 daemon stop. Writes profile name to /tmp/start_type.log
  (currently absent → nothing booted since reboot).
- Profiles under `sessions.d/`: ATCStandard + ATCSystem both have
  **`R1LITEBody.d`** (mobiman.yaml identical; teleop/system differ only
  cosmetically). Also R1LITEVRTeleop.d, R1LITEVRATC.d, ATCHostStandard/R1LITET.d.
- **`R1LITEBody.d` contents** (tmux sessions → commands):
  - `hdas`: `start_hdas_r1lite.sh` = CAN bringup (down/up can0, CAN-FD 1M/5M)
    + `ros2 launch HDAS r1lite.py`; second window: head camera + realsense
    wrist cameras.
  - `mobiman`: chassis (`r1_lite_chassis_control_w_o_eepose_launch.py`),
    torso speed control (R1LITE arg), gripper controller (R1LITE),
    ee_pose, jointTrackerdemo_fast, relaxed_ik left+right.
  - `ros_discovery`: vestigial fastdds discovery server.
  - `system`: system_manager + robot_monitor.
  - `teleop`: **`start_r1lite_tabletop_gello_teleop.sh`** — GELLO leader-arm
    teleop; will try to own the arms. Kill this session after boot unless
    the GELLO rig is attached (likely fails w/o hardware, but don't risk it).
  - `tools`: data_collection + monitors.
- `can.txt` = old candump debug capture (the 175KB "dump of shit"); `~/lite/`
  = stereo calibration artifacts (Chinese dirnames); both ignorable.
- Workspace `~/galaxea/install/` includes full OCS2 MPC stack
  (ocs2_mobile_manipulator etc.), livox_ros_driver2, greenwave_monitor —
  richer than expected; relaxed_ik + jointTracker are the arm command paths.
- Quirk: prestart scripts hardcode `/home/nvidia/galaxea/maps` for MAP_DIR —
  copied from Orin models, harmless for us.

**Agreed boot command for step 3 (Krishna, Console A):**
```bash
cd ~/galaxea/install/startup_config/share/startup_config/script
./robot_startup.sh boot ../sessions.d/ATCStandard/R1LITEBody.d
# wait ~30-60s, then: tmux ls  (expect: hdas, mobiman, ros_discovery, system, r1lite_teleop, tools)
tmux kill-session -t r1lite_teleop   # keep GELLO teleop off the arms
# stop everything later with: ./robot_startup.sh kill
```

## 2026-07-03 — Day 2: stack boot + full topic recon (FIRST CONTACT)

**Boot:** Krishna ran `robot_startup.sh boot ../sessions.d/ATCStandard/R1LITEBody.d`
(Console A). Arms/grippers twitched on HDAS init (expected health sign).
6 tmux sessions up (hdas, mobiman, r1lite_teleop, ros_discovery, system,
tools), 31 ROS processes. GELLO teleop session killed after boot.

**Laptop wiring:** `nmcli device modify enp130s0 +ipv4.addresses 10.42.0.100/24`
→ ping 10.42.0.2 at ~0.4ms over cable. Container on ROS_DOMAIN_ID=2 sees
the full graph via plain multicast — NO XML/discovery-server config needed.

**test_00_recon results (132 topics, 29 nodes):**

| Segment | Feedback topic | DOF | Rate |
|---|---|---|---|
| arm left/right | /hdas/feedback_arm_* | **6 each** | 200 Hz |
| torso | /hdas/feedback_torso | **4** | **488 Hz** |
| chassis | /hdas/feedback_chassis | 3 | 200 Hz |
| gripper l/r | /hdas/feedback_gripper_* | **1 each** | 200 Hz |
| aggregate | /joint_states | 25 (?) | 500 Hz |
| hands | /hdas/feedback_hand_* | silent | 0 Hz (not fitted) |

Upper body = 4+6+6 = **16 joints** (R1 Pro: 18). MotorCommandArray slicing
for the connection module: torso 0-3, left 4-9, right 10-15 (+ grippers
handled separately via target_position_gripper_*).

**Command topics confirmed subs>0:** target_joint_state_{arm_left,arm_right,
torso}, target_position_gripper_*, target_speed_chassis, target_speed_torso
(TwistStamped — new vs R1 Pro), brake_mode, chassis_acc_limit. The
`target_joint_state_arm_*` pubs=1 scare resolved: jointTracker holds idle
publishers (0 Hz) — no contention.

**Chassis gate suspects all present:** /controller (hdas_msg/
ControllerSignalStamped, pubs=1 subs=1), brake_mode + chassis_acc_limit
(subs=1 each), /motion_control/chassis_speed (pubs=1). NOTE: R1 Pro's
gatekeeper /cmd_vel topic does NOT exist — command path is
target_speed_chassis directly. Whether the 3-gate unlock ritual is needed
is test_03's job.

**Perception inventory (differs a lot from R1 Pro):** wrist L/R = full
RealSense stacks (color + depth + aligned + pointcloud + compressed).
Head = STEREO RGB pair (left_raw/right_raw compressed) with NO depth topic
(calib/head_left+right camera_info suggest stereo depth is computed
elsewhere/offline). NO chassis cameras. NO lidar topic (livox_ros_driver2
installed but not in R1LITEBody profile). /robot_description published
(pubs=2) → URDF obtainable straight off the topic. TF + tf_static live.

**Extras noticed:** /hdas/feedback_*_arm_wrench (force/torque!), /hdas/bms,
per-ECU version topics, relaxed_ik EE-pose command path, system_manager
manipulation/navi action servers, /vla/prompt_echo (VLA hooks), OCS2 stack
in workspace. R1 Lite is teleop/data-collection oriented out of the box.

**Config reconciled:** r1lite_config.py updated with all verified values;
EXPECTED_TOPICS now 12 topics incl. grippers + IMUs. Remaining TODO(recon):
joint names per topic, torso home pose, chassis gating behavior, head-depth
story, lidar presence.

### Day 2 continued — fill-ins + test_01 PASS

- **Joint names:** HDAS feedback topics carry placeholder names
  (`['left_arm']`, `['torso']`, ...) — ordering by convention, like R1 Pro.
  `/joint_states` (25 joints) gives the real naming: steer_motor_joint1-3 +
  wheel_motor_joint1-3 (**3-wheel swerve**, C1 ≈ W1 family), torso_joint1-3,
  {left,right}_arm_joint1-6, {left,right}_gripper_finger_joint1-2.
- **URDF captured** → `scripts/r1lite_test/r1lite_from_robot.urdf` (44KB,
  33 joint tags). Meshes referenced as
  `package://mobiman/urdf/R1_Lite/meshes/*.STL` → live on robot at
  `~/galaxea/install/mobiman/share/mobiman/urdf/R1_Lite/meshes/` — scp later
  for data/r1_lite_description. Wrist cams = **D405** (same as R1 Pro);
  link naming mirrors R1 Pro conventions.
- **Torso 3-vs-4 discrepancy:** URDF has only torso_joint1-3, but
  /hdas/feedback_torso streams 4 values @ 488 Hz. Suspect coupled/parallel
  lift motors. Command-side expectation TBD from torso start script /
  test_06 — treat torso command DOF as OPEN until verified.
- **"A message was lost!!!" spam** during `ros2 topic echo` on 200-488 Hz
  topics = Python CLI subscriber too slow, benign. Watch for the same
  symptom in the dimos connection later though (Mustafa's sensor-drop
  class of problem).
- **test_01_topic_discovery: PASS** (132 topics visible, all 12 expected
  present). First formal green check — laptop/container/cable/DDS/config
  verified end-to-end. (Restart hygiene notes: container `docker start` +
  3 setup lines; nmcli runtime IP re-add after suspend; robot tmux
  sessions survive laptop reboots.)

### Day 2 continued — tabletop config discovered, resting pose captured

- **Krishna flagged before any motion test: R1 Lite is a TABLETOP bimanual**
  (photo taken) — arms mounted base-up on a flat platform, folded inward,
  grippers facing each other; camera tower between them; torso = parallel-
  linkage lift column; robot TETHERED (charger + ethernet). R1-Pro-style
  "nudge joint 1" is unsafe (horizontal sweep into tower/other arm).
  Motion tests re-ordered: gripper → wrist joint → torso (blocked) →
  chassis (blocked until untethered).
- **Torso launch decoded** (`torso_control_r1_lite.launch.py`): MPC task-space
  controller — v_{x,z,pitch,yaw} limits (0.15/0.15/0.3/0.3) and bounding box
  x∈[±0.45], z∈[0.175,0.72], pitch∈[±1.25], yaw∈[±0.05] (yaw ~locked).
  Internally loads R1's r1_v2_1_0.urdf with robot_model=R1_LITE switch.
  DANGER: if target_joint_state_torso takes task-space values, a naive
  [0,0,0,0] "zero pose" is BELOW z_min → test_06 LOCKED until semantics
  verified. No home pose constant found in the launch file.
- **Resting pose captured (read-only):** arm_left ≈ all-zero, arm_right ≈
  all-zero → **folded tabletop pose IS joint zero**. Torso [−0.002, −0.002,
  0.004, 0.0] → feedback is 4 JOINT angles (not task-space; task z would be
  ≥0.175). Grippers: left 101.8, right 96.0 → **0–100-ish units (percent
  or mm), NOT radians/meters** — differs from R1 Pro catalog (meters).
- Planned first actuation (pending discussion + subscriber probes):
  gripper no-op (target = current) → small close (−15) → back. One DOF,
  cm-scale, clear path.

### Day 2 — FIRST COMMANDED MOTION (gripper) ✅

Command-path probes: target_position_gripper_left consumed by
r1_gripper_controller (+greenwave_monitor watcher); target_joint_state_torso
consumed by **r1_lite_jointTracker_demo_node** (joint-space!) — NOT the MPC.
So two torso paths: joint targets → jointTracker, task-space speeds
(target_speed_torso) → MPC node. Torso joint targets = 4 joint angles
(matching feedback); resting ≈ zeros. control_torso has 2 publishers
(jointTracker + MPC) — command via ONE path only, watch for contention.
All /motion_target subscribers are BEST_EFFORT/VOLATILE QoS.

Gripper sequence (left, empty, hands clear):
1. `--once` publish target=101.8 (no-op): accepted, zero movement. ✓
2. `--once` publish target=85: **NO effect** — controller ignores one-shots.
3. `ros2 topic pub -r 10` streaming target=85: gripper closed smoothly
   101.84 → 87.0, and **stopped the moment the stream stopped** (Ctrl-C at
   ~0.6s, before reaching target). → r1_gripper_controller is a
   follow-the-stream tracker: commands must be CONTINUOUS, dead-man style.
   Same design consequence as R1 Pro: the dimos connection module must
   publish gripper/joint targets every tick, not fire-and-forget.

Chain validated end-to-end: laptop container → DDS multicast over cable →
mobiman gripper controller → HDAS → CAN → motor. First dimos-side actuation
of the R1 Lite.

### Day 2 — ARM COMMAND PATH VALIDATED ✅ (wrist-roll test)

Gripper reopen settled at 100.84 (~1 unit deadband short of 101.8 target —
normal). Then left arm wrist roll (joint 6): streamed target_joint_state_
arm_left at 10 Hz holding joints 1-5 at rest, joint 6 → 0.2 rad. Tracked to
exactly 0.200, other joints unmoved, physical rotation ~11° in place
confirmed; streamed back to 0.001 — home. Same dead-man stream semantics
as the gripper (jointTracker follows while stream lives).

**Hardware validation status:** feedback ✓ (all segments), gripper ✓,
arm joints ✓ — via streamed /motion_target targets, exactly the R1 Pro
pattern minus DOF/unit differences. Remaining: torso (joint path via
jointTracker, tiny delta — planned, not yet run), chassis (BLOCKED until
untethered + clear floor; gating behavior unknown). Every fact needed for
R1LiteConnection is now in hand: 16 upper-body joints (4+6+6), separate
0-100-unit grippers, stream-required command paths, BEST_EFFORT QoS,
domain 2, multicast, topic map reconciled in r1lite_config.py.

### Day 2 — TORSO JOINT-PATH TEST: FAILED SAFE ⚠️ (important negative result)

Precheck: control_torso had NO active publisher (verified on robot via
ros2 topic hz — container can't deserialize hdas_msg custom types; note:
all dimos-facing topics are standard msgs, hdas_msg only on internal
/motion_control layer).

Streamed target_joint_state_torso holding joints 1,2,4 at rest and joint 3
→ +0.05 rad @ 0.3 rad/s. **Robot SHOOK** (motor oscillation), feedback never
tracked (stayed ~0.001). Ctrl-C → shaking stopped instantly (dead-man
confirmed again), torso settled at ~zeros. No damage.

**Diagnosis:** torso is a PARALLELOGRAM lift — joints 1-3 mechanically
coupled. "Move joint 3 alone, hold 1-2" is kinematically unreachable; the
tracker drove motor 3 against the linkage while 1-2 held → oscillation.
Arm test worked because arm joints are serial/independent; torso joints
are not. This is exactly why Galaxea's torso controller is task-space
(x/z/pitch MPC with linkage-aware solver).

**Rules going forward:**
- DO NOT send single-joint deltas to target_joint_state_torso. Any joint-
  space torso command must be a known linkage-consistent tuple (e.g. a
  pose previously read from feedback).
- v1 dimos integration: torso FIXED (mirrors Mustafa deferring R1 Pro
  torso planning). Torso motion later via target_speed_torso task-space
  path (MPC, bounded, linkage-aware) as a separate designed experiment.
- test_06_torso_command.py marked DO-NOT-RUN as written (R1-Pro-style
  joint targets) — needs rewrite to task-space or consistent-tuple form.

Hardware validation final for the session: feedback ✓, gripper ✓, arm ✓,
torso joint-path ✗ (by design of the mechanism, not a bug in our chain),
chassis untested (tethered).

### Day 2 — Official R1 Lite documentation research (web)

Sources: galaxea-dynamics.com product/spec pages, docs.galaxea-ai/dynamics
R1Lite software-introduction + VR-teleop pages (site flaky — content
recovered via search snippets), aifitlab listing.

**Platform identity:** R1 Lite = data-collection/teleop-oriented dual-arm
mobile manipulator, $39,999 base. 23 total DOF = 2×6 (arms) + 3 (torso,
kinematic) + 6 (chassis) + 2 (grippers). 1280×670mm, vertical workspace
0–1.7m. Onboard PC: **Intel i9-12900HK 14-core / 32GB** (matches our
x86_64 finding; the aifitlab "Jetson AGX Orin" claim is wrong for this
model — that's R1/R1 Pro).

**Arms:** Galaxea A1X, 6-DOF, 600mm reach, 4.2 kg each, payload 3 kg
rated @600mm / 5 kg max, ±0.1mm repeatability, max EE speed 5 m/s (!),
spherical wrist.

**Grippers:** Galaxea G1 force-controlled parallel gripper.
**Position range 0–100 CONFIRMED** (VR-teleop config: opening_threshold
80.0/max 100.0, closing 0.0/min 0.0 → 0=closed, 100=open). Our 101.8
reading = fully open w/ slight overtravel. Units resolved.

**Torso (matches our negative result):** official software guide splits
torso control into TWO nodes and warns: torso velocity signals and torso
joint signals "cannot be used together — joint conflicts, safety risk";
**JointTracker starts with disable_torso=true BY DEFAULT** ("users will
not be able to control the torso via target_joint_state_torso").
Our unit shook on joint targets → its boot config evidently enables it
(or partially) — worth checking start_r1_lite_jointTrackerdemo_fast.sh
args someday, but v1 plan stands: torso FIXED, task-space path later
(v_x ±0.1 m/s, v_z ±0.1 m/s, w_pitch/w_yaw 0.3 rad/s of torso_link3 rel
base_link, right-hand rule).

**Chassis:** 3 self-developed steering-wheel (swerve) modules = 6 DOF,
360° omni, modes translation/spin/Ackermann, max 1.5 m/s; control node
takes vector commands x, y, w simultaneously. → our chassis test =
target_speed_chassis TwistStamped with tiny vx, no gatekeeper needed
per docs (R1 Pro's 3-gate saga may be W1/R1Pro-specific — still verify).

**Head:** pure binocular RGB stereo (no depth) ✓. Wrist: D405 depth cams ✓
(WC1 option fitted on our unit).

### Day 2 — CHASSIS MYSTERY SOLVED: RC mode map (no bypass needed!)

Chassis node (`chassis_type: new_W1` — same W1 family as R1 Pro) subscribes
/controller and vetoes software unless the RC grants mode 5. Krishna mapped
the RC empirically (4 switches: sw1/sw4 two-position, sw2/sw3 three-position):

| Switch combo | mode | Meaning |
|---|---|---|
| all switches position 1 | **5** | SOFTWARE/AUTO — external cmds may drive (the value Mustafa's gatekeeper faked!) |
| sw1@2 + sw2@3 | 2 | brake/lock (matches R1 Pro mode-2 braking) |
| sw1@2 + sw2@3 + sw3@mid | **3** | RC MANUAL — sticks drive (RC-driving validated live!) |
| other single flips | 255 | invalid combo |
| RC OFF | 3 | receiver failsafe = "manual, sticks centered" → chassis obediently parked, software vetoed |

Explains everything: all "chassis dead" symptoms (0.3mm/s creep with gates
streamed, RC "not working") were the RC being off/in manual with no stick
input. The R1 Pro bypass (launch remap + fake mode=5) is NOT needed on this
unit — RC ON + all-switches-position-1 = legitimate mode 5, and the RC
remains a live takeover/brake layer (flip sw1 = revoke software control).
NO robot files were modified. Procedure for software chassis control:
RC ON, mode 5 verified via /controller, then stream gates+speed (test_03).

### Day 2 (late) — THE CHASSIS SAGA: unresolved, fully mapped ⏸️

Chronology of the wheels-won't-turn hunt (all hypotheses tested and killed):
1. Software cmds produced 0.3mm/s creep → suspected R1 Pro 3-gate problem.
2. Streamed acc_limit + brake=false + Gate-1 subscriber (4-shell test, then
   test_03 rewritten to do all gates itself): still creep.
3. Discovered RC mode map (see earlier entry); RC manual (mode 3) DROVE the
   robot — motors/VCU proven working at that moment.
4. Mode 5 verified + all gates streamed: STILL creep. Chassis pane logs
   (tmux capture) showed `target_speed_command 0.05` AND
   `last_send_speed_command 0.05` — node accepts targets, ramp reaches
   full speed, publishes motor commands. Every software gate open.
5. /motion_control/control_chassis capture: v_des=[0.05×3] on wheels but
   **`mode: 0`** in every MotorControl msg — suspicious but unconfirmed
   (never captured a working-drive reference for comparison).
6. **Chassis node LATCHES last target forever — NO staleness timeout**
   (kept publishing v_des=0.05 minutes after stream stopped). RULE: always
   end chassis sessions with a zero-stream. (Arm tracker does have
   dead-man behavior; chassis does not!)
7. Later in the same boot, RC manual STOPPED driving too. Eliminated: RC
   (all 4 axes sweep 240-1807, link healthy), e-stop (released, no error),
   VCU faults (w1-w6 all zero), battery (54V, 99.9%!), stack state (fresh
   restart), brake/acc streams during RC manual. NOTHING moves the wheels.
8. My missteps logged honestly: premature "DDS corruption" call (was my
   own `ros2 daemon stop` + short timeouts emptying the CLI cache —
   container discovery direct via rclpy was always fine); proposed
   Mustafa-bypass (remap+fake mode) would NOT have helped (mode gate
   provably open — ramp ran).
9. **Lead for next session:** Galaxea unboxing docs say R1 Lite ships with
   THREE remotes: torso, chassis, and a separate WIRELESS E-STOP remote
   ("software-based switch halting all operations"). We never checked it —
   a pressed/dead e-stop fob (failsafe = STOP) would explain everything
   incl. "worked earlier, died mid-session". CHECK THE E-STOP REMOTE FIRST.
   Also in docs: "Wheel Motor Zero-Point Calibration Instruction" exists;
   support: product@galaxea-dynamics.com.

Session totals: gripper ✅ arm ✅ torso ⚠️(negative result, guarded)
chassis ⏸️(node-level fully validated; wheel-level refusal unresolved).

## 2026-07-09 — Day 3: CHASSIS SOLVED ✅ (test_03 PASS)

Structured fault-tree session (planned before touching hardware). Key
discriminating experiments and results:

1. **No wireless e-stop fob exists on this unit** (only the base button) —
   suspect C eliminated by inventory.
2. **Clean cold boot** (both e-stops clear, RC in manual BEFORE power-on):
   **RC manual drove the robot WITH NO ROS STACK RUNNING** → RC-manual is
   VCU-DIRECT (radio → VCU firmware). The ROS chassis node was never in
   the manual loop.
3. Reference capture during RC driving: /motion_control/control_chassis
   carried ALL ZEROS while wheels turned — confirms bypass conclusively.
4. Arm reference during working software motion (test_04 PASS):
   control_arm_left shows mode: 0 TOO → **mode field acquitted** (0 does
   not mean "don't execute"). Real arm/chassis frame difference: arms carry
   full impedance params (kp/kd/t_ff), chassis frames have empty arrays —
   turned out to be irrelevant.
5. **THE ANSWER: test_03 on a healthy VCU simply PASSED.** Peak measured
   0.0470 m/s, stop to 0.0000, ~5cm roll, RC in mode 5, all gates streamed
   by the script. Nothing was ever wrong with the software recipe.

**Root cause of the entire Day-2 chassis saga: the VCU was latched**
(e-stop pressed at/before a power-on earlier that day). A latched VCU:
- ignores software chassis commands (the eternal 0.3mm/s "creep")
- eventually also killed RC manual mid-session
- survives ROS stack restarts, reports w1-w6 error_code 0, clears ONLY on
  a clean power cycle with e-stops released
Every Day-2 software experiment ran on poisoned hardware; the recipe
(RC ON mode 5 + Gate-1 subscriber + acc_limit + brake=false + streamed
speed) was correct all along.

**Final chassis operating procedure:** cold-boot with e-stops released →
RC ON, all switches position 1 (mode 5) → test_03 or equivalent gate
streams → drive. End every session with a zero-stream (latch!). If wheels
ever refuse both paths: power cycle, don't debug software.

**Day-3 addendum:** test_03 re-run twice more — PASS both (peaks 0.0490 /
0.0447 m/s, stops to 0.0000). Chassis validation is reproducible. Curiosity
left open (academic): a 40s capture of /motion_control/control_chassis
during the passing runs again showed all-zero v_des — either timing missed
the 1s moves, or software wheel commands route below/beside that topic
just like RC manual does. Irrelevant to the integration (our interface is
/motion_target/*, proven working); logged for completeness.
BRING-UP PHASE COMPLETE — next: R1LiteConnection module.

**Day-3 correction (routing curiosity CLOSED):** synchronized rclpy
recording (both topics, domain 2, 3671 msgs each) during two more test_03
PASSes shows /motion_control/control_chassis DOES carry the wheel commands:
v_des ramps 0.03→0.05 on the three drive wheels one second before measured
motion appears. Plumbing is exactly as documented (node → control_chassis
→ HDAS → CAN → VCU); mode:0 + empty kp/kd are normal for velocity-mode
wheels. All earlier "all-zeros during software drive" captures were tooling
failures, NOT hidden routing. The RC-manual-bypasses-ROS conclusion stands
(that capture was valid and received messages fine).
Tooling lesson for the record: non-interactive SSH does not source
~/.bashrc → any remote ros2/rclpy one-liner MUST set ROS_DOMAIN_ID=2
explicitly, and robot-side `ros2` CLI depends on a flaky daemon — prefer
direct rclpy via python3 heredoc for instrumentation.

## 2026-07-09 (later) — R1LiteConnection LIVE: runs 1 & 2 PASS ✅

**R1LiteConnection module + r1lite-coordinator / r1lite-keyboard-teleop
blueprints written** (mirroring R1ProConnection; torso commands dropped,
grippers first-class, chassis dead-man streaming, validated acc limits).
Registered in all_blueprints. Venv had regressed to py3.12 (host Booster
work re-synced it — bind-mount see-saw); rebuilt py3.10.

**Run 1 (coordinator, hardware):** all 25 transports up, feedback found
instantly, coordinator ticked 100Hz streaming hold-position to the arms —
**arms perfectly still for 40s** (bootstrap gate + hold-current-pose
verified on hardware). Fixed a teardown race found in the log (publish
loop vs dying ROS context — loop now joins before courtesy chassis-zero).

**Run 2 (cameras in rerun):** left wrist + right wrist + head streaming
live in the two-tab layout, ~1.2 GiB/10s, smooth. Verified end-to-end:
ROS compressed → connection decode → LCM → rerun bridge → viewer.

**The viewer saga (write-off of ~45 min):**
- `dimos run` is NEVER headless: global_config default viewer="rerun" —
  the bridge always composes. True headless = `--viewer none`.
- Native viewer in container: X11 auth fail; after `xhost +local:` it
  crashed anyway (software-GL/winit BadDrawable — rerun-in-docker is
  unsupported). Container GUIs: don't.
- rerun-web mode: hung worker 0 mid-start (R1LiteConnection never finished
  starting — cohabits worker 0 with the bridge), only grpc :9876 up, :9090
  never served; ugly force-kill shutdowns. Isolated probe of
  serve_web_viewer() worked — suspicion: first-use asset fetch blocking
  worker 0. (My "rerun-web-viewer package missing" theory was WRONG —
  invented package name; assets ship in rerun-sdk.)
- **WINNING SETUP: viewer on the laptop + connect mode.** Host venv is
  container-built (py3.10 shebangs → not executable on host), so:
  `uv tool install rerun-sdk==0.29.2` on the host, `rerun --port 9877`,
  and in the container VIEWER=rerun-connect (global_config reads env
  vars). Host networking makes 127.0.0.1:9877 shared. Clean shutdowns.
- One-command launcher added: `scripts/r1lite_test/run_r1lite.sh`
  (starts host viewer if needed, starts container, runs blueprint with
  VIEWER=rerun-connect).

Remaining for blueprint validation: run 3 = r1lite-keyboard-teleop
(chassis via full dimos chain; RC mode 5 + staging).

**RUN 3: PASS — r1lite-keyboard-teleop drove the robot.** All six keys
functional (W/S fore-aft, A/D rotate, Q/E strafe — swerve crab confirmed),
release = stop (module dead-man), Ctrl-C = clean elegant shutdown incl.
courtesy chassis-zero guard. Launched via run_r1lite.sh one-command flow.
QoS note: drove even with the BEST_EFFORT publishers — the starved
RELIABLE subscriber on target_speed_chassis is a monitor, not the chassis
controller; publishers switched to RELIABLE anyway (serves both kinds).

**RUNTIME LAYER COMPLETE (2026-07-09):** R1LiteConnection + coordinator +
keyboard teleop are hardware-validated end to end — feedback, arms-hold,
cameras-to-viewer, chassis driving. Remaining for the PR: r1_lite
description package (URDF+meshes to LFS) + catalog factory, distilled
docs/README, CI gate, rebase story vs Mustafa's branch/main.

**Untethered driving (question raised, analyzed, parked):** RC manual —
yes anytime (radio→VCU). dimos over office WiFi — expected NO: DDS
discovery needs multicast and the office AP kills it (same IGMP-snooping
behavior that broke Go2 discovery); robot nodes also aren't registered
with any discovery server, so no unicast rendezvous either. Paths if
needed later: (1) 2-min empirical test (unplug cable, ros2 topic list
over WiFi), (2) Galaxea's official WiFi mode = findrobot_server.sh
discovery-server switch (invasive: rewrites robot bashrc + reboots — a
deliberate migration), (3) RECOMMENDED: dedicated travel router bridged
to the robot's ethernet, Go2-style — nothing on the robot changes.
Cameras over WiFi would strain regardless (--viewer none when wireless).

### Day 3 — rerun-web ROOT-CAUSED via py-spy (dimos-core bug, not ours)

Retried `--viewer rerun-web` deliberately (connect-mode workflow untouched).
Reproduced exactly: grpc :9876 up, :9090 never serves, R1LiteConnection
start stalls mid-sequence, shutdown wedges. py-spy dump of the live hung
worker 0 (pid 3920) shows the smoking gun:

- Bridge thread: `bridge.py:329 start` → `rerun/sinks.py:372 serve_grpc`
  — **active+gil**: spinning inside rr.serve_grpc() WITHOUT releasing the
  GIL; never reaches serve_web_viewer (so :9090 never opens).
- Connection thread: `_setup_sensor_streams` → `Thread.start()` →
  `threading.wait` — waiting for its new thread's started-event, which
  can never fire because the GIL is hogged. Every Python thread in the
  worker starves → "R1LiteConnection started" never logs, teardown hangs.

Isolated probe (fresh python, same venv, same calls) returns instantly →
the deadlock is specific to rerun 0.29.2 serve_grpc() inside dimos'
multiprocessing **forkserver** worker children (rust/tokio state vs fork,
classic). Affects ANY robot blueprint using web mode in-container; native
(rr.spawn) and connect (rr.connect_grpc) paths don't hit it.

**Resolution: rerun-web marked known-broken; rerun-connect stays the
workflow (run_r1lite.sh). Evidence package ready for a dimos-core issue:**
repro = any blueprint w/ RerunBridge viewer_mode=web in a worker; py-spy
dump as above; suggested fix directions = call serve_grpc off the worker
hot path / spawn-context workers / rerun version bump.

**rerun-web WORKAROUND VALIDATED (sidecar pattern):** in-container
`rerun --serve-web --port 9877` (headless — no X11 needed) + blueprint in
VIEWER=rerun-connect → browser at
http://127.0.0.1:9090?url=rerun%2Bhttp%3A%2F%2Flocalhost%3A9877%2Fproxy
shows all cameras live; R1LiteConnection starts cleanly (rust server in
its own process = fork can't poison it — confirms the root-cause theory).
Gotchas: plain :9090 without the ?url= param = empty viewer (must use the
full URL the sidecar prints); add --memory-limit 2GB or the sidecar
buffers history until RAM (hit 1GiB in 38s of cameras).
run_r1lite.sh now has --web mode implementing this. Proper core fix
proposal stands: bridge.py web mode should spawn the server subprocess +
connect_grpc instead of in-process serve_grpc.

# Presenter notes: bimanual FR3 on dimOS

One section per slide: what to say, the numbers, likely questions.

---

## Slide 1: Cover

- **Two days of work:**
  1. the FR3 driver in dimOS;
  2. a second FR3 on the same table, calibrated into one shared world;
  3. two scripted skills: handover and pen.
- **Where the code is:**
  - dimOS: `~/dimos-franka`, branch `franka-fr3-adapter`, committed, not pushed.
  - Skills: `~/franka_3_setup`, local git repo.

---

## Slide 2: Franka FR3 in dimOS

**How the driver works** (`dimos/robot/manipulators/franka/connection.py`):
- **Control mode.** It uses libfranka's **async position control** (`AsyncPositionControlHandler`): we send a joint target, and the robot plans its own way there.
  - Targets come at the coordinator's **100 Hz**, not libfranka's usual 1 kHz real-time loop, so **no real-time kernel** is needed.
  - Franka marks this mode experimental.
- **Speed.** Capped at **25%** of each joint's top speed, the value in Franka's example. The planner's model uses the same 25%, so plans and the arm keep pace.
- **Session quirks handled:**
  - Starting a session makes libfranka send an all-zero target, so the driver immediately sends the measured pose.
  - Sessions end by cancelling, never `stop_control()`, which would also send zeros.

**Checks in the driver.** It refuses a joint target when:

| Check | Limit |
|---|---|
| Last reading too old | over 0.1 s |
| Too far from the previous target | over 0.1 s of motion (0.066 rad on joints 1–4) |
| Too far from where the arm is | over 1 s of motion |

It **faults** on:
- **drift:** a joint more than 0.02 rad outside the band between where it was and its target;
- **stall:** standing still more than 0.02 rad from the target for 0.3 s;
- **reflex or outside stop.**

A faulted arm refuses everything until `clear_errors()`, which does what Desk's recovery does.

**Collision thresholds:** 20/40 Nm on the joints, 20/40 N at the tool (lower = contact reported, upper = the robot stops).

**The Hand** (`hand_process.py`):
- **Grasp or move:** closing by more than 5 mm uses `grasp()`, which holds the object with a force (default **20 N**). Anything else uses `move()`, position only, no force.
- **Own process:** pylibfranka's gripper calls hold Python's **GIL** for about 1 s. In the same process that froze the arm's 100 Hz reading thread, readings went stale and the arm faulted. A separate process has its own GIL. It's a subprocess because dimOS modules are daemon processes and can't have `multiprocessing` children.

**Limits:** positions only (no velocity, Cartesian or torque control); no simulation scene.

**Likely questions:**
- *Why async and not the 1 kHz loop?* dimOS's coordinator runs at 100 Hz; the async mode lets the robot interpolate, with no real-time kernel needed.
- *Tested?* About 52 unit tests against a fake pylibfranka, plus hardware runs.

---

## Slide 3: Tabletop bimanual

**The setup:**
- **Right = arm 1** at 10.42.0.230 (USB ethernet adapter). **Left = arm 2** at 10.43.0.53 (built-in port).
- Each has its own control box. Addresses are pinned by MAC, so reboots don't move them.
- The bases are about 78 cm apart, each turned inward.
- **World frame = the right arm's base.** The left base is at (0.160, 0.758) m, turned −22.4°.

**Calibration:**

| Measurement | How | Accuracy |
|---|---|---|
| **Hand-eye** (each wrist camera on its Hand) | About 15 views of a ChArUco board per arm | A few mm |
| **Arm-to-arm** | Both wrist cameras saw the same two taped boards. **Planar fit** (x, y, rotation only), since both bases sit flat on one level table; a full 3-D fit gave a false 2.7° tilt | **4.3 mm RMS**, 5.7 mm worst |
| **Tripod** | Re-measured from the taped board at the start of every run, on a 2× enlarged image, saved for when the board is hidden | Paper centre agrees with the wrist camera within 2 mm |

**One planner** (`tabletop_bimanual.py`):
- one combined robot model: both arms plus the wrist-camera boxes;
- every plan is checked against the other arm;
- planning groups right, left and both.

**SeparationGuard** (`separation_guard.py`), between the coordinator and the arms:
- For each target, it places that arm at the target and the other at its latest target in a Drake model, and measures:
  - arm links and cameras: at least **5 cm** apart;
  - grippers: at least **5 mm**, so handovers work.
- A target that breaks either rule is dropped and the arm stops.
- A target that **widens** a too-small gap still passes, so a stopped arm can back out.
- It protects against teleop or policy commands that skip the planner.

**Cameras:** three RealSense D435i at 640×480: one per wrist, one on a tripod.

**Likely questions:**
- *Why not Franka's bimanual product?* We had two standalone arms. This works anywhere they're clamped; re-calibrate after moving a base or a camera.
- *How accurate overall?* A few millimetres. Fine for the handover, marginal for the pen.

---

## Slide 4: Handover skill (`handoff.py`, perception in `tabletop.py`) [VIDEO]

**Finding the object** (no object model):
1. Each camera fits the table plane in its own depth data (RANSAC).
2. Points standing above the table are kept (over 2 cm for the tripod, 6 mm for the wrist cameras), skipping the bases, the taped board and anything out of reach.
3. They're grouped on a 5 mm grid, and each group gets a fitted box: centre, angle, length, width, height.
4. Candidates more cameras agree on come first. Flat things depth misses are found by colour instead.
5. The nearer arm's wrist camera looks again from 35 cm to measure it properly.

**Grasp:** across the object's narrow side, near the end closer to that arm's base. Fingertips 2 mm above the table; a tall object is gripped higher so the palm stays clear of its top.

**Handover search:**
- **Varied:** heading (0, ±30°, ±60°), tilt (0–90°), roll (0/90/270°), and which face the receiver grips.
- **Required:**
  - IK for both arms;
  - wrists within 170° of ready;
  - hand parts at least 5 cm above the table;
  - no collisions;
  - the guard's check at every step of the receiver's approach and the giver's back-out.
- Stops at 3 feasible poses or 20 s; gives up at 30 s. Remembers which poses have worked before.

**Safety rules in the skill:**
- The giver opens only after the receiver's fingers have closed to about the object's width.
- Every move is verified against the measured joints, because dimOS reports "COMPLETED" even if the arm didn't arrive.

**Relay:** objects under 2 cm thick or under 9 cm long, or when no handover pose works, go via the table: the giver sets the object down in the middle and the receiver picks it up.

**Limits:**
- box-like objects only;
- at most 6.8 cm across, since the Hand opens 8 cm;
- irregular shapes not supported yet.

**Results:**
- **Yesterday:** 8 handovers in both directions plus relays; the search went from about 90 s to 6–10 s.
- **Today:** 5 of 5 successful cycles, about 85–105 s each, including a 9.3 cm-tall box with a different handover pose.

---

## Slide 5: Pen skill (`pen_demo.py`) [VIDEO]

**Stages:**
1. **look:** tripod and wrist cameras find the pen and paper.
2. **relay**, if the pen is out of the drawing arm's reach.
3. **plan.**
4. **pick:** across the barrel, 6 cm from the tip.
5. **click:** the other arm's wrist camera aims at the button, then one straight press.
6. **touch:** slide straight over the paper and come straight down.
7. **draw:** the smiley, 82 waypoints in about 7 s, one continuous path (`move_linear_path`, added to dimOS for this).
8. **back, click off, put down.**

**Roles:** the left arm draws (the paper is on its side) and the right arm clicks. The right arm is the more precise one right now.

**What was hard, and how it's handled now:**

| Problem | Why | Current handling |
|---|---|---|
| **Which end is the tip** | Silver and shape rules were wrong about half the time from above | **Placement rule:** the pen is put down with its tip toward the tripod |
| **Click aim** | The camera put the button 5–7 mm off | A fixed correction found by watching the misses (8 mm away from the tripod) |
| **Drawing height** | The pen sits a few mm differently in the grip on each pick | Set by eye: −2 mm drew 1 cm high; −12 mm tripped the collision stop; currently −10 mm |
| **Touch by force** | The joint-torque force estimate is clean when **still** (0.2 N noise) but reads 10–40 N falsely while moving, and moves under about 5 mm don't run | Stepping replaced by the height set by eye |
| **Held pen invisible to the planner** | The planner doesn't model what a Hand holds; two table hits before the fix | Every carrying move is checked point by point with the pen attached (3 cm clearance), and carries are straight lines |

**Likely questions:**
- *Did it draw a smiley?* It ran end to end several times. The drawing height is the remaining issue: too high, or too much pressure.
- *Why so many manual offsets?* Millimetre precision from hand-written rules plus a few millimetres of calibration error. That's the point of the conclusion.

---

## Slide 6: Safety-link (FSoE) faults

**What FSoE is:** Fail Safe over EtherCAT, the safety bus between each control box and its arm.

**Symptom:**
1. The arm stops sending joint data mid-move.
2. "UDP receive: Timeout".
3. "Connection to FCI refused".
4. Desk shows an FSoE or fieldbus connection error, once with "Joint firmware update failed", once a "Generic joint error" that required self tests.

**Recovery:** Desk reboot, about 1 minute; sometimes it won't acknowledge and needs a full power cycle.

**Count:**
- **Right arm:** about 8, since the evening of 11-22 (its first was after about 2 days of clean running).
- **Left arm:** about 4, starting at 01:03 on 11-23, after a full day clean. Three came within 12 minutes, one with an empty Hand, one a collision stop with nothing nearby.

**Why it isn't our commands:**
- FSoE is internal to the robot; FCI traffic doesn't travel on it.
- It reproduced with no gripper commands at all (a right-arm soak test).
- It happened during ordinary moves.

**Candidate causes:**
- **Shared power:** both control boxes on one circuit or power strip.
- **Arm-to-box cable or connector strain** since both arms were moved to this table (11-22 morning).
- **Heat:** faults cluster 20–40 minutes into sessions and late at night.
- **Firmware.**

**Next:** contact Franka support with `failure_notes.txt` (every time, message and what the arm was doing); put the boxes on separate outlets; reseat and re-route cables; try swapping which box drives which arm.

---

## Slide 7: Motions end early

**Symptom:**
- The robot ends an async motion **1–10° short** of its target, reporting "Unexpected reply to a Move command".
- No reflex, nothing in Desk.
- The left arm on most moves since this morning; the right arm started doing it later.

**Test with the same moves and code on both arms:**

| | Right arm | Left arm |
|---|---|---|
| Ends from the goal | 0.0–0.3° every time | 1.3–3.4° short, 5 of 6 times |
| Lag behind the plan | about 1° | 2–3° |
| Effect of speed | — | None (same at 12%, 25%, 50%) |

**Candidate causes:**
- The async controller ending the motion while the arm still trails the streamed targets (the left lags more).
- Network: the left port showed 13 transmit errors, the right 0. Latency was fine on both.
- Robot-side degradation after last night's FSoE faults.

**Effects:**
- Every move needs a re-plan to finish.
- **Moves under about 5 mm don't run at all**, because no joint moves more than 0.01 rad. So small stepping moves drift.
- Continuous paths jerk: the drawing went up and down.
- Grasps land 5–7 mm high and need a correction.
- Before the fix, the coordinator kept holding the unreachable goal, which also blocked Hand commands.

**Workarounds (committed):**
- **dimOS:** a short ending with no stop now **holds** instead of faulting, with 3 per 5 s still a fault; Hand commands are handled before arm checks.
- **Scripts:** re-plan the rest; height correction that allows for the shortfall.

---

## Slide 8: Conclusion

**The claim:** precision contact tasks (grasping, pushing a button, hovering a pen over paper) shouldn't depend on millimetre-exact calibration plus hand-tuned rules. Ours missed by 5–10 mm repeatedly.

**Proposed split:**
- **Scripted planning and the guard** do the coarse motion and safety: get the object or tool close, keep the arms apart.
- Then hand off to a **policy trained from teleop demonstrations (VLA / WAM)** that closes the loop from the three cameras for the last centimetres. It sees the gap; it doesn't need to know it to the millimetre.
- **Exact calibration only where it's free: in simulation**, for generating teacher data.

**Keep:** the dimOS FR3 driver, the bimanual model and guard (they protect a policy too), and the coarse perception.

**Evidence:**
- **Handover** tolerates centimetres: 5 of 5 today.
- **Pen** needs millimetres: needed manual offsets at every step.

**Likely questions:**
- *Why a policy, not better calibration?* The errors also come from the grip varying on each pick and from hardware drift; closed-loop vision absorbs both.
- *What data?* Teleop demonstrations of the last-cm phases, plus sim-generated teacher data.
- *What's needed next?* Teleop into the coordinator (the guard already covers it), joint-target streaming for a policy, and three-camera recording.

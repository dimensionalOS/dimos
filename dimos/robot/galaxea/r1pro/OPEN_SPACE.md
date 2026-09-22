# R1Pro open-space demo

An interactive classical pick, carry and place demo on a 12 × 12 m floor, with five named platforms and no apartment walls or cabinets. The platforms retain physical collision geometry. GraspGenX proposes grasps; DimOS handles reachability, body positioning, navigation and manipulation. No ACT policy runs.

## Current demo changes (September 18)

`pick_object` now completes after a checked approach, grasp and verified lift,
holding the object. It **does not** preflight or execute a return-to-ready motion.
The agent chooses the next action based on the user's task. `prepare_carry`
exposes compact cargo-safe retraction separately; `return_to_init` still means
the exact startup arms/torso with the base staying here. `go_to` continues to
prepare a carrying posture for navigation.

`move_linear(arm, dx, dy, dz)` and `move_to_pose(arm, x, y, z, roll, pitch, yaw)`
are now agent skills backed by the existing scene-aware classical planner,
not unchecked generic robot commands. Translation is in world metres and
angles are in radians; omitted pose angles retain current values. Both preserve
the base, gripper actuator commands and held objects. They cannot create a new
grasp or release supported cargo. They share cancellation/recovery controls,
stale-state checks, posture/collision/cargo checks, and measured endpoint checks.
`get_scene` includes measured `tcp_poses` for specifying these moves.

The second-pick seed bug was in the neutral-posture override: it copied raw
measurements from the already-loaded arm instead of the bounded measured IK
seed. In session `d94277a3d2b84205bb0f72d3fd3b8ee0`, action 002, the loaded left
wrist read 1.002380214 rad, just outside the conservative planning bound
1.00229 rad but inside its physical 1.01229 rad stop. All 396 non-obstructed,
in-range candidate evaluations aborted before solving; no carry preflight ran.
The neutral seed now starts bounded and changes only the requested arm.
Physical limits are unchanged; invalid/nonfinite measured states are rejected.
Assessment logs now include candidate counts and leading rejection reasons.

Non-driving trajectories run at 2x their old nominal timing, preserving every
checked sample and stop. Maximum unloaded arm/torso speed is 1.2 rad/s; loaded
arms 0.5 rad/s, loaded torso 0.16 rad/s, and grippers 0.1 m/s. Tray transit is up
to 2x faster within coordinator velocity limits; one-second tray holds remain.
Recovery arm speed scale is 0.5 instead of 0.25. Driving, base positioning,
docking speeds and navigation deadlines are unchanged.

Arm/tray execution no longer expires at a fixed wall-clock `duration + 25`
or `2 * duration + 10` deadline while slow physics is still progressing.
Settling budgets use elapsed simulation time. Independent wall-clock watchdogs
still stop a simulation that hasn't advanced for 10 seconds, or a motion with
no measured/commanded/settling progress for 30 seconds. Recovery also uses a
simulation-time settling budget. Endpoint, ownership and contact checks remain.

Per the demo request, **no regression suites or physical replays were run for
these latest changes**; only source syntax/import checks. Earlier validation
results below refer to earlier revisions, not the new speeds or skill wiring.
Restart the blueprint to load the changes; a scene reset does not reload code.

## Run

### Fresh checkout with Nix

From the repository root (the current laptop uses `/home/mustafa/dimos`):

```bash
nix develop --command uv sync --extra manipulation --extra graspgenx
nix develop --command cargo build --locked --release -p dimos-mls-planner --bin mls_planner
nix develop --command .venv/bin/dimos run r1pro-classical-open-space-sim-agent
```

In another terminal, from the same directory:

```bash
nix develop --command .venv/bin/dimos humancli
```

The Nix shell includes Cargo and rustc for the native navigation planner. The
Python dependency lock pins the `dimos-lcm` commit containing `EpisodeStatus`;
the PyPI 0.1.3 package lacks this message and prevents CLI startup.

GraspGenX also requires a working NVIDIA CUDA driver. Check `nvidia-smi` before
launching. On dimensional-0476, the September 18 migration found the proprietary
595 driver loaded for an RTX 5070 Ti Laptop GPU; the kernel reported that this GPU
requires NVIDIA open kernel modules. The system driver must be corrected before
validating GraspGenX. Changing Python dependencies or entering Nix does not fix
the host kernel driver. Both flavors were initially installed; removing the old
`linux-modules-nvidia-595-*` packages (preserving `*-595-open-*`), rebuilding the
initramfs and rebooting selected the open module.

If `nvidia-smi` works but PyTorch reports no driver inside Nix, check that
`libcuda.so.1` is visible there. The shell hook now includes Ubuntu's
`/usr/lib/x86_64-linux-gnu/libcuda.so*` in its NVIDIA-only library links. Enter a
fresh `nix develop` shell after updating the hook. A GPU tensor allocation and
GraspGenX loading of generator epoch 736 and discriminator epoch 1056 passed
on this laptop after this correction.

A seed-5000 headless startup check also passed through MCP `get_scene`, with all
five objects present and no simulation error. Evidence:
`/tmp/r1pro-cuda-startup-20260918-1529/result.json` (`success: true`). This check
issued no manipulation or language-agent commands. Shared-memory resource-tracker
warnings still occur during shutdown and remain a separate cleanup issue.

### Original workstation environment

Close the previous demo with Ctrl-C, then:

```bash
cd /home/mustafa/dimos-wt/r1pro-classical-apartment
.classical-venv/bin/dimos run r1pro-classical-open-space-sim-agent
```

In another terminal:

```bash
cd /home/mustafa/dimos-wt/r1pro-classical-apartment
.classical-venv/bin/dimos humancli
```

The full MuJoCo viewer and Zenoh are defaults. The agent uses the existing provider configuration. No environment exports are needed. Scroll to zoom in on manipulation or labels. The robot starts idle.

| Platform | Height | Color |
| --- | --- | --- |
| `worktable` | 70 cm | Blue |
| `low_bench` | 60 cm | Green |
| `display_table` | 80 cm | Purple |
| `tall_table` | 90 cm | Orange |
| `high_counter` | 85 cm | Red |

A tray remains on the worktable. Each platform starts with one object: the existing cup, bottle, drink carton, glue stick and toy block, with randomized colors, dimensions, assignments and supported positions. Each new launch randomizes the seed; `reset_scene` repeats the current seed.

## Interact

Ask what objects are present before selecting one. For example:

- “Pick the carton with your right hand.”
- “Go to the low bench.”
- “Place the object in your right hand on the low bench.”
- “Pick that carton up again.”
- “Take it to the high counter and place it there.”
- “Pick the cup with your left hand.”
- “Place it on the display table.”

A pick ends with the object held. Navigation preserves holds. Placement verifies support before opening the gripper. A requested hand is preserved; the other hand may keep holding an item. Use `get_surfaces` for exact region names. The room names from the apartment do not apply here.

The tray is a physical free body. `place_object` with region `tray` puts a held item into it wherever it currently rests. `pick_up_tray` docks in front of the resting tray and lifts it with both hands, keeping its contents; both hands must be empty. `go_to` carries a held tray and docks where it would be set down. `put_down_tray` carries the tray to a named platform, lowers it onto a clear footprint near the robot's edge, verifies support, releases and retreats. Nothing can be picked or placed while the tray is held. The tray planner measures the handle spacing from the model; the classical tray is wider than the home demo's, and the older fixed constant closed the fingers 4 cm inboard of the handles.

The tray is carried with its bottom about 85 cm above the floor, and the torso is already at its tallest level posture, so platforms at or above that height (tall_table, high_counter) cannot receive the tray. `put_down_tray` and a loaded `go_to` refuse them up front with the measured heights. The dock beside a platform steps back along the approach heading until the robot and its carried tray are clear of the platform legs; the arms cover the remaining distance. The footprint search also avoids fixtures standing on the platform. In the packaged apartment this leaves only the worktable: the laptop, lamp, journal and camera on the dining table leave no clear 32 by 47 cm footprint for this tray, and the kitchen counter is above the carry height.

This scene uses the same privileged simulation perception and contact checks as the [classical apartment](CLASSICAL_APARTMENT.md). It simplifies navigation geometry; it does not establish arbitrary-object or hardware reliability.

## Verified run

Seed 5000 completed ten consecutive commands through local MCP: right-hand carton pick, low-bench delivery and re-pick, high-counter delivery, navigation to the display table, left-hand cup pick, and delivery to the tall table. Both final objects were physically supported, released, upright and settled. A separate full GLFW viewer startup passed at seed 5001. These checks did not call the language model.

Evidence is saved locally in `recordings/r1pro-classical-open-space/seed-5000/validation.json`, with per-action states and a scene preview in the same directory. The focused suite passed 68 tests; strict type checking passed on ten changed production sources.

## Simulation speed

The desktop viewer receives state updates at 30 Hz; physics retains its 2 ms timestep (500 steps per simulated second), and camera streaming remains on a separate thread at 10 Hz. The shared simulator exposes `viewer_fps` separately from camera `fps`. The classical demo now sends bounded snapshots to a separate viewer process. Slow viewer synchronization drops display frames instead of holding the physics lock. Camera orbit and zoom remain available; use DimOS commands to modify robot state, since native viewer physics edits affect only the display copy. The native UI may redraw faster than the 30 Hz state updates.

The object labeled `cup` is a narrow, hollow, handleless cylinder, not a detailed mug asset. In the profiled open-space run (seed735730399), it was the orange `object_5` on `display_table`; IDs and positions change across seeds. Its walls now sit on its own bottom disk. Previously the disk and all 24 wall segments touched the table, producing 101 cup/table contacts at rest. The corrected geometry produces five, while preserving the hollow cavity, physical grasp surfaces, outer dimensions and total mass. On the saved slow-run scene this reduced raw physics cost from 2.93 ms to 0.34 ms per step. This is a physics benchmark, not an end-to-end action latency guarantee: GraspGenX and reachability planning still take time.

Restart the blueprint to generate the corrected scene; `reset_scene` reuses the existing model. No new environment exports are needed.

### Performance status

The September 17 desktop run exposed a 12.4-second viewer synchronization stall while holding the live physics lock. The snapshot viewer removes that coupling: a real 12-second suspension of only the viewer left physics running at approximately real time, and its 37 focused simulation/IPC tests pass.

After the host reboot the same day, active cores idled at 0.8 to 1.6 GHz with package temperatures near 60 to 70 C, and the seed 5000 three-action sequence (right carton pick, low-bench delivery and placement) passed headless on the snapshot-viewer code in 6.5 minutes of wall time. The earlier 200 MHz readings were a thermal or power fault of the host, not a simulation regression; check clocks and temperatures again before interpreting any slow run.

## Grasp assessment

Both classical demos generate 100 GraspGenX samples per inference attempt and
return at most 100 candidates. Top-K filtering happens after model inference;
reducing only the returned count would not reduce the generated batch size.
This replaces the previous 600/600 settings. Generation can still time out for
reasons other than sample count; this cap is not a latency guarantee.

Ranking GraspGenX proposals runs in a separate worker process that the simulator starts and warms at the first session call, so the kinematics world (about a minute to build) is paid once, before the first pick. Each pick then ranks in roughly 20 to 40 seconds; the skill polls with a cancellable wait and gives up after 150 seconds with a plain reason ("free the other hand or ask again from closer"), killing the worker so a runaway search cannot starve physics. The worker logs to `assessment-worker.log` in the session directory; each request lives in a `classical-assessment-*` directory with its `result.json`. When the free hand is across the body from the target, the ranker tries repositioned stances first.

### Repeated inference / second-pick timeout (September 18)

The 120-second `GraspGenXModule/propose_grasps` timeout was a lost RPC reply,
not slow grasp generation or a reachability search. In session
`283695c31ed8412fac15f49eb06ea736`, action 002 picked the pink block with the left
hand and completed its 27-waypoint automatic retraction. After navigation,
action 004's right-hand purple-carton pick stayed in `generate_grasps` from
16:47:46 to 16:49:46 and never created an assessment request.

Zenoh 1.10 implicitly optimizes messages of at least 3,072 bytes through shared
memory, using 16 MiB arenas. The skills worker had 54,464 KiB locked against a
64 MiB `RLIMIT_MEMLOCK`; mapping another arena for the roughly 23 KiB grasp reply
failed. Its process maps contained 682 mappings of the same sender arena.
Zenoh's failed `mlock` path left those mappings behind while the query finalized
without a reply. DimOS then resubmitted the whole RPC after 50 ms, explaining
the repeated `iteration 1` logs every roughly 120 + 50 ms until the timeout.
There was one user pick request, not repeated agent requests.

The transport policy now limits implicit SHM optimization to `put` messages in
both Python and native session configuration. Streaming publications retain
SHM; RPC queries and replies use ordinary transport payloads and do not require
mapping another arena. This does not raise system memory limits, delete shared
memory files, reduce grasp counts further, or restart the running demo.

Validation: replaying the earlier failed purple-block cloud returned 100 valid
grasps ten times, in 119–124 ms after warmup, including adapter conversion;
pickle round-trips preserved every pose and score. An isolated five-peer,
300-call test with 100-candidate replies lost 177 replies with the old SHM
policy, versus zero with SHM disabled. The implemented `put`-only policy also
delivered all 300 replies with no SHM allocation errors, using the actual DimOS
session-config builder. The fault is the locked-memory allowance, not exhausted
RAM or a full `/dev/shm` filesystem. All demo processes must be restarted to
replace their existing sessions; a scene reset does not do this.
The focused Python service/wire and RPC suites passed 28 tests, and the native
Zenoh suite passed 29 tests. Mypy and Ruff passed. The release `mls_planner`
binary was rebuilt with the matching native transport policy.

## Local regression

### Natural-posture planning (September 18)

The classical planner now uses the neutral-posture Pink task adapted from
`origin/mustafa/task/r1pro-hosted-teleop-demo` (`teleop_ik.py`): upright torso,
elbows bent, stronger torso/shoulder/elbow weights, and a bounded pull toward
the reference. Wrists retain weaker weights for grasp orientation. This is a
**soft QP objective**, not a hard constraint by itself.

`posture_ik.py` also adds hard QP inequalities for this demo: torso pitch within
±20°, shoulder joints 2/3 within ±90°, and elbows at least 10° bent. Mechanical
joint/velocity limits and inactive-joint locks remain enabled. These are a
conservative robot-specific envelope, not a universal ergonomics model.
Endpoint, Cartesian-sweep and RRT-edge checks enforce the same envelope.
Candidate IK seeds only the requested arm from the neutral reference; it does
not reseed the other hand. Ranking includes posture error and compares up to
three feasible base stances per hand within the existing time budget.

Restart the blueprint to load this policy; a scene reset does not reload code.
Offline replay of the saved seed-2012159698 cup assessment found six plans and
rejected the old backward-leaning posture. The selected candidate's staging
path also passed collision/posture checks without modifying the source state.
The focused suite passed 86 tests; mypy passed on all five changed production
sources. This checks kinematic feasibility,
not a successful physical pick. The existing contact failure still needs a
fresh end-to-end run. Tests exercise actual QP constraint enforcement against
a conflicting objective, neutral pull, inactive-joint locks and path checks.

### Earlier automatic post-pick retraction (superseded)

Historical investigation: the latest changes above removed automatic retraction
and the mandatory return preflight. The carrying planner remains available as
an explicit skill and for navigation.

At this earlier stage picks included `retract_to_carry` after the verified lift.
Success required the return motion to finish with both hands retaining cargo;
lifting alone did not complete the action. The torso and empty arm returned
toward the recorded ready joints. Loaded hands return toward their ready TCP
positions while retaining cargo pitch/roll and closed-gripper actuator targets.
The base does not drive back to its original location. A blocked or cancelled
return remains an incomplete pick with recovery required, never a silent
release or a reported success. Navigation reuses the same carry preparation.

This addresses the saved session `35225ac09d0e42388f0df02ef95767e5` sequence:
action 002 reported a successful extended-arm pick, then action 003 failed in
`prepare_carry`. The old carry planner required both hands to preserve their
current wrist orientations at an arbitrary height, including the empty hand.
The revised planner checks cargo-preserving return corridors to the ready pose.
As with the posture-policy change, restart the blueprint to load this behavior.

The ready check requires a folded loaded elbow, the torso and empty arm near
their home joints, and the loaded TCP near its home position. A loaded wrist
does **not** have to match its empty-hand home orientation: that would tip a
side-grasped object. Gravity-axis yaw is allowed; cargo pitch/roll is retained.
Already-ready hands produce an explicit unchanged hold, not another arm move.

Before executing a pick, the ranked shortlist previously had to pass a copied-scene
lift-and-return check. Its per-plan budget is 5 seconds, with 25 seconds for the
shortlist; unchecked candidates are never returned as feasible. This avoids
selecting a grasp that can lift but cannot fold back safely. The measured
post-lift return is replanned with a 20-second search budget; the preflight does
not replace live contact, collision, posture or stale-state checks.
The shortlist reuses the already-checked post-lift snapshots instead of
repeating approach/lift IK for each candidate.
Re-running the saved cup assessment returned three carry-verified plans in
57.2 seconds total: the non-retractable nearest-stance grasp was rejected,
and the selected nearby stance produced the return tested below.

Offline validation of a newly posture-checked cup grasp produced a 28-waypoint
return in about 1.5 seconds of planning. MuJoCo then executed that return over
22.17 simulated seconds while retaining two-pad grip: maximum cargo slip was
0.66 mm, tilt was 1.29 degrees, and endpoint joint error was 0.003 rad. Repeating
carry preparation after the physical replay returned one unchanged waypoint.
This replay starts from a constructed post-lift hold; it is **not** proof of an
end-to-end physical pick. No running demo was reset or commanded for this test.
The focused regression suite passed 139 tests, including cancellation, loss of
either hand's cargo, stale planning snapshots, and intentional placement release.
Mypy passed on all eight affected production files; Ruff checks passed.

### Explicit return to the startup posture

`return_to_init` restores the recorded startup **arm and torso joints**, keeping
the base at its current location and preserving both gripper actuator commands.
It is separate from `reset_scene`: objects and scene progress are not reset.
The agent now has this tool and is instructed to use it for init/home posture
requests. The target already existed in the simulator's session reset joints;
the missing piece was an exposed action, not an "arbitrary" pose definition.

The planner checks the exact endpoint and then a collision/posture/cargo-safe
joint path. It never substitutes the cargo-adapted `carry_posture` target. If
the fixed startup wrist orientation would tip held cargo, the action reports
that reason before motion; placing the object requires a separate user request.
Compact carrying is now a separate `prepare_carry` action, not part of picking.
An occupied tray, ambiguous grasp, unsettled robot, or stale planning snapshot
also prevents execution. Success requires measured arms/torso within 0.02 rad
of the recorded target, unchanged cargo ownership, and base drift within 5 mm
and 0.005 rad. Motion failure or cancellation keeps the recovery guard active.

Offline replay of session `8b4ce7e161b04461813fdac79d33fd4c`, action 009,
confirmed that the held cup is tilted 2.87 degrees but would reach 29.90 degrees
at the exact startup joints. That endpoint is explicitly rejected before RRT;
the source snapshot was not modified. This is not a live motion test.

A separate synthetic empty-handed fixture used the saved model and recorded
startup joints, with a 0.1 rad right-shoulder perturbation. Real RRT returned a
two-waypoint path in 0.0043 seconds; MuJoCo execution reached the startup joints
within 0.000464 rad, retained both gripper commands, and passed scene inventory
validation with negligible base drift. This verifies the safe homing path, not
a return while holding the cup from action 009.
The focused regression set passed 118 tests, including 25 new skill-level
return-to-init cases. Mypy passed on the four changed production files; Ruff
and diff whitespace checks passed.

Restart the blueprint to advertise the new tool, then ask "Return arms and
torso to init; keep the base here." It can also be called directly:

```bash
nix develop --command .venv/bin/dimos mcp call return_to_init
nix develop --command .venv/bin/dimos mcp call wait_for_action --arg seconds=20
```

Repeat `wait_for_action` while the action is running. Neither implementation
nor regression checks reset or command an already-running demo.

### End-to-end commands

The non-agent blueprint is `r1pro-classical-open-space-sim`. A reproducible test through local MCP, without a language-model call:

```bash
.classical-venv/bin/python -m dimos.robot.galaxea.r1pro.demo_classical_apartment \
  --open-space --seed 5000 --output /tmp/my-open-space-run \
  --mcp-port 10026 --zenoh-scout-addr 224.0.0.224:19492 \
  --actions pick:right:object_4 go:right:low_bench place:right:low_bench \
  --viewer --stay-open
```

Tray actions use `tray_pick::` and `tray_place::<platform>`; the arm field stays empty. A full tray sequence is `pick:right:object_4 place:right:tray tray_pick:: go::display_table tray_place::display_table`.

`--agent --say "Pick the carton with your right hand." "Put it in the tray."` runs the language agent instead and sends each sentence the way HumanCLI does, waiting for the agent to go idle and the action to finish. Pass `--model` to use a provider with working credentials. The launching shell must hold that provider's API key; the agent reports `Agent request failed` with the HTTP status otherwise.

Use a fresh output directory and unused ports for each test. The harness saves physical state and per-action outcomes. Interactive launches do not automatically execute this sequence.

GraspGenX loads its pinned checkpoint from the local HuggingFace cache without contacting the hub. If the module never logs its checkpoint paths at startup, the hub lookup is blocking; set `HF_HUB_OFFLINE=1` in the launching shell. On September 17 a half-open IPv6 route to the hub stalled startup indefinitely.

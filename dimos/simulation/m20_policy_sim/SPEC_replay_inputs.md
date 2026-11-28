# Replay and sim inputs taken from vendor data

What every replay and sim test seeds from vendor logs, captures or fits, by category: (a) a sensor or bus input the
live controller also receives (fine); (b) vendor internal state; (c) a constant fitted to the recording; (d) a timing
alignment chosen to match the vendor. (b)-(d) are replay-only inputs: each row says what the live controller does
instead. Line numbers are as of 2026-11-24; R = `dimos/robot/deeprobotics/m20/controller/rust`, D = `src/dock/docking.rs`,
V = `src/dock/vendor.rs`, VL = `src/dock/vendor_log.rs`.

## Status of the six that hid live behaviour

1. The IMU path the policy tests prove (yesense's roll / pitch / yaw): now the live path too (imu_rpy, 45b43cbde2).
2. The joint-state lag the ECAN replay proves: now live (ReplyLag in BusHardware and the MuJoCo sim, 24bdd9cd61).
3. The dock replay's detector-service path: still replay-only (live has no detector process to call).
4. The dock replay's pre-recording undock state: labelled replay-only; the recording's own dock now has to derive the
   start point charge_manager logged (77466ab774).
5. The policy test's teacher-forced last_action: labelled; policy_session runs closed loop.
6. The vendor-log replay's undock goal and navigating flag from the log: labelled replay-only.

## tests/dock_rec_20261124.rs

| File:line | Value | Source | Cat | Live controller instead | Recommendation |
|---|---|---|---|---|---|
| 41 `T0` | Tick 0 = 1795470616.210740, charge_manager's last charge-enable write before the undock | DDS capture | (d) | No such phase. After a confirmed dock the charging loop starts at `AfterCalls::Charging`, D:751-758 (sleep_until = now, next write after charge_wait_ms). After a boot on the dock there are no charging-loop writes at all: D:559 `take_charge(.., false)`. | Label as replay-only input, because the loop's phase is set by a dock before the recording. Add a fresh-start test (see 571). |
| 571 `resume_charging(start_point, (17, 0x3003))` | Phase Charging; start point; restore state 17 and gait 0x3003; `charge_cmd = true`; `charge_checked = true`; charge_sleep/next_write phase (D:642-656) | The `mInitX` and `mInitRobotGaitStatus_ :0x3003` lines in the cm log | (b) | Boot on the dock: self test D:494-560 (Detect 7-12 s, FindPose, Confirm), then Charging with `charge_cmd = false`. start_point stays [0;3] (V:400), so the undock goal is exit_pile_distance_x = 1.3 (V:477-479). Restore defaults are 17 (D:392) and 0x1001 (D:383). After a dock: restore pair from the view at the request (D:922-924); start point from the core (V:576-589). | Vendor: mInit is zeroed in the ctor (0x185240) and set only by a dock in the same process (0x180d80). 0xe0 is set at goToPileAction (0x180c20), with ctor default 0x1001. The value is unknowable to a fresh process, so label as replay-only input. Then add two tests. (1) A fresh-start test: boot lying on the dock with MOS on → self test → Charging with no charge-enable writes → undock. Assert ChargeEnable(0), the back-off to 1.3 m, MotionState(17) then Gait(0x1001). (2) In this replay, assert our derived values at the recorded dock: `status().start_point` = the log's second `mInitX:0.906093,mInitY:-0.001405,mInitYaw:-0.058511` (05:51:12.926), and the restore pair = the MOTION_INFO gait at the dock request. |
| 246-254, 690-709 `start_point()` | The undock goal x, chosen from ±8 f32 ULPs of the logged 6-decimal value as the one with the fewest steers off | The cm log, then **fitted to the expected steers** | (b)+(c) | The core's own `a.best` at the dock (V:588) | Label as replay-only input, and state in the test that it is fitted. Add the start-point assertion above, so the derivation gets checked somewhere. |
| 555 `detector_service = true`; 303-309 and 643-646 `detector_replied()` at reflective_column_node's logged "ServiceCallback 处理完成" times | Waits of 53-83 ms on each Open/CloseMarkProcess | The rc log | (d) | `detector_service = false` (m20_controller.toml:757), so `call_detector` runs `after_calls` at once (D:726). The live path never enters `Phase::DetectorCall`. | Add a fresh-start test: the same replay with `detector_service = false`, and a pose for every captured sweep (as the always-on detector module, detector_module.rs:274, would publish). Assert the same write values in the same order, with each time within the skipped call durations. Until then, the live dock timing and the ReflectorFilter start window after `filter.restart()` (D:963, D:1056) are untested. Live, the filter can take a sweep captured before the restart, which the vendor's freshly opened process never sees (unverified whether this changes the start fix). |
| 235-240, 593-607 pose times | Each found pose is ingested when charge_manager logged taking it ("get Target position" / "mInitYaw" / "Find target success"), otherwise at rc's last line +0.05 s | cm and rc logs | (d) | Docking takes the newest `dock_pose` on the tick it arrives (D:1045-1056) | Label as replay-only input (input arrival time). The write-time asserts (±1 ms, 808-812) depend on it. |
| 455-464 sweep selection | A logged sweep = the last captured sweep < 0.06 s before its step-1 line; only the sweeps the vendor processed (~10 Hz, and none in the 29.7 s gap while its process was closed) | Capture plus rc log | (d) | The detector module processes every front sweep, all the time (detector_module.rs:274). Docking filters every new pose, even while idle (D:1056). | Covered by the detector_service=false test above (feed every sweep). |
| 290 `t_done` window 0.08 s | Publish time = last rc line within 80 ms of step 1 | rc log | (d) | n/a | Label as replay-only input. |
| 244-245, 638-640 obstacles | charge_manager's own "最近障碍物距离为" readings, held until the next one (never cleared) | cm log | (b) | `nearest_obstacle` (src/dock/obstacle.rs) on every front sweep plus the newest rear sweep while open, with the body height from our joints (detector_module.rs:257) | Derive like the vendor: run `nearest_obstacle` on the NOS capture's front and rear Airy sweeps. Assert our readings equal the logged ones at the logged times, and that sweeps with no log line give none. No test compares obstacle.rs with the vendor; it only has unit tests (obstacle.rs:141-190). |
| 150-174, 624-633 battery | rl_deploy's /BATTERY_DATA as it arrived on NOS (present, MOS); `charger_connected: false` hard-coded; `seq` = index | DDS capture | (a), through rl_deploy | `module::charge_reading` from the ECAN battery frames directly (module.rs:772-790). That has no rl_deploy relay or DDS hop, so it arrives earlier. | The confirm-tick asserts (737, 749, ±1 ms) depend on the relay latency. Derive like the vendor: build the readings with `charge_reading` from the AOS ecan.pcapng battery frames, mapped onto the NOS clock. Then assert the confirm tick within the relay latency. |
| 141-146, 574-590 `View` | rl_deploy's /MOTION_INFO (state, gait) as the view. Codes other than 1/4/17 become Passive. `stand_in` and `fault` are always false. | DDS capture | (a) for charge_manager, but internal for us | `View::update(&controller)` (D:126). In our process Docking's writes drive our own Controller. | Label as replay-only input (it isolates the charge_manager layer). The closed loop is covered only by sim_sequence. |
| 241-243 requests | The ChargeEnable service calls (0/1) | cm log | (a) | The operator's dock/undock requests | Fine |
| 116 `T0 + 5e-4` / `T_END` | Window of compared writes | Capture | (d) | n/a | Fine |
| 572 `tick_of` = ceil((t − T0)·1000) | Tick grid anchored at T0 | Chosen | (d) | Ticks from controller start | Label as replay-only input |

Assertions that would still pass with derivation code deleted:

- The restore-pair capture at the dock request (D:922-924): no undock follows the recorded dock.
- The start-point value chosen during the dock (V:580-589): it is never compared, and the undock's goal is seeded.
- The boot self-test path and `charge_cmd = false`: never reached.
- The whole `detector_service = false` path.

---

## tests/ecan_dock_rec_20261124.rs

| File:line | Value | Source | Cat | Live controller instead | Recommendation |
|---|---|---|---|---|---|
| 187-221 `load`: `state = at_previous_send` | Frame k is computed from the newest reply seen **before send k−1**, i.e. the reply to k−2 | Chosen to match rl_deploy | (d) | rt.rs:233-236: `tick(&state)`, then `exchange(cmd)` returns the reply to the command just sent (ecan.rs:359-399, bus.rs:257-275). Frame k uses the reply to k−1, **one tick fresher** than the vendor. | Derive like the vendor: compute command k from the state available before send k−1, or change the replay to the live ordering and show what breaks. Today the bit-exact stand-up and lie-down proof runs on a latency the robot does not run (verify my reading of rt.rs). policy_steps.py uses the same vendor lag (`st[row-1]`, `M20_LAG = 1`). |
| 60-62, 244-247 `CLOCK_BASE_S = 196608` + `phase` | rl_deploy's seconds-since-start (about 2.3 days), stepping every 1/64 s in f32 | Fitted | (b)+(d) | `stamp = started.elapsed()` (ecan.rs:347, bus.rs:228): near zero after a restart, so the f32 spline time is µs-fine (state_machine.rs:738-741) | The vendor derives it the same way (since its own start). Label as replay-only input, and add a fresh-start test: stand-up and lie-down with the stamp starting near 0, asserting the targets equal the f64 spline within 1 ULP. |
| 609-677, 685-692 `fit_phase` | Phase fitted on the stand-up **and** the lie-down windows, then asserted on both | Fitted | (d) | n/a | Fit on the stand-up only and assert on the lie-down, so it is out of sample, as the doc comment at 23-26 claims. |
| 54 `KNOWN_CLOCK_SLIP` | Two ticks excused | Capture | (d) | n/a | Label as replay-only input (justified: rl_deploy's own late send) |
| 58 `DECIDE_NS = 116 µs` | When a charge-enable request counts for a tick | Fitted on the 16 requests | (d) | In-process: a `Write::ChargeEnable` reaches the hardware on the same tick (ecan.rs:332-336) | Label as replay-only input |
| 375-399 `policies()` | Every runner pre-served 40 steps on a synthetic standing state, so the history is full and `last_action` is set | Chosen to mimic 1.5 days of uptime | (b) | Fresh runners hold the warm-up pose for `warmup_steps` (runner.rs:496-523) | Label as replay-only input, and add a fresh-start test: the first rl_control entry after boot holds the default pose for exactly `warmup_steps` steps, then infers. |
| 477-504 RlControl windows | `force(RlControl)`, entered on 0x3001 when the gait is not an entry gait, then a warm-up of `2000 + (dec+1 − i0%dec)%dec` ticks so the 50 Hz phase lands on rl_deploy's first target change in the window | **Phase fitted to the expected output** | (b)+(d) | Cadence from entry: `since_offer` (state_machine.rs:952, 979-994) | Label as replay-only input for the windowed mode. The cadence from entry is derived live only in the stand-up window (LieDown → RlControl at 05:50:27.851615) and in the env-gated whole session. Assert the first policy target-change tick after that entry explicitly. |
| 470-476 LieDown start | `force(LieDown)` with an entry stamp 60 s earlier | Chosen | (b) | Lying is reached by a real lie-down | Fine (it only places the start) |
| 464-465 | quat = identity, accel = (0, 0, 9.81) | Synthetic (no IMU recorded) | substitute | IMU from LCM (module.rs:215) | Fine; rl_control targets are not compared |
| 452, 454 | `set_fault_checks(false)`, `charge.set_rule(false)` | Chosen | (b) | Faults are on; the rule is off while the dock sequence sends requests (ecan.rs:339-343) | Fine |
| 250-269, 272-282 | /MOTION_STATE and /GAIT from rl_deploy.log; charge-enable arrivals | Log and capture | (a) | Operator, and Docking's writes | Fine |
| 533-536, 554 `tick.frame[16]`, `frame[6..12]` | Frame sequence number and source MAC taken from rl_deploy's frame | Capture | (b) counter | `self.seq` from 0, +1 per frame including charge-enable frames (ecan.rs:323, 372, 380); MAC from the socket | Add a fresh-start test: our seq increments by 1 per sent frame across charge-enable bursts, as the capture shows. Today a broken seq counter still passes. |
| 815-821 `before` gaits | The gait in force before each window | rl_deploy.log | (a) | n/a | Fine |

---

## tests/vendor_replay.rs and src/dock/vendor_log.rs (also examples/charge_replay.rs)

| File:line | Value | Source | Cat | Live controller instead | Recommendation |
|---|---|---|---|---|---|
| VL:271-287 (with `start_known` at VL:249) | When the log begins mid-charge, the undock goal is looked up **later in the log** (the first `StartPoint` line after the stand-up) and set before `begin_exit` | cm log, from the future | (b) | start_point [0;3] → exit_pile_distance_x (V:477-479) | Label as replay-only input, but count these cases in `Report` and print them. The fresh-start test above covers the derivation. |
| VL:312-328 poses | charge_manager's own logged pose (x, y, yaw), so the detector, ReflectorFilter and target_pose are skipped | cm log | (a) for the core | Detector → `filter.sweep` → `target_pose` (D:1055-1063) | Fine (a core unit test). The full chain is in dock_rec. |
| VL:321 `navigating` | False exactly on the logged `Failed(NotNavigating)` line | **The expected answer** | (b) | `view.is(RlControl) && on(approach or finetune runner)` (D:1098) | Label as replay-only input. This test cannot catch a broken live `navigating` derivation. |
| VL:264-268, 271-290, 292-295 | `begin_dock(now)`, `begin_exit(now)`, `finetune_started(now)` at the logged lines | cm log | (d) | Docking's phase changes | Label as replay-only input |
| VL:296-299 | `wait_timed_out()` at logged timeouts; `restarted()` at logged restarts | cm log | (d)/(b) | pose_wait_ms + `pose_wait_overhead_us` (D:1079-1091) | Label as replay-only input. dock_rec covers it, with the fitted 216 µs. |
| VL:300-310 | `no_charge()` at the logged retry or exhaustion | cm log | (b) | The ConfirmCharge timeout on battery MOS (Docking) | Label as replay-only input; sim_sequence's no-charge test derives it |
| VL:329-331; vendor_replay.rs:54 `OBSTACLE_HOLD_MS = 210`; charge_replay.rs:34 `210.0` | The vendor's logged obstacle reading, valid for 210 ms | cm log, fitted hold | (b)+(c) | nearest_obstacle on every sweep | Same as the dock_rec obstacle row |
| VL:374-387 ±1 ms retry | A pose that mismatches is retried at now ± 1 ms; the count goes to `r.clock_phase` but **is not asserted** | Chosen | (d) | n/a | Assert `clock_phase` equals a known small count, or print it in the test, so time-limit logic off by 1 ms cannot hide. |
| VL:240 `now = t.floor()` | Log time to ms | Chosen | (d) | n/a | Fine |
| vendor_replay.rs:132-170 reflector filter | Positions ("P") and restarts ("R") from the rc log | rc log | (a) positions / (b) restart times | Restart at Docking's FindTarget (D:963, D:1091) | Label as replay-only input |

---

## tests/policy_20261120.rs and tools/policy_steps.py (`windows`)

| File:line | Value | Source | Cat | Live controller instead | Recommendation |
|---|---|---|---|---|---|
| 187-189 `last_action = implied_action(previous rl_deploy command)` | Teacher-forced every step | rl_deploy's sent targets | (b) | The runner's own previous action (runner.rs:318, 522, 661) | Make it closed loop like policy_session (own last action), or add that as a second assertion. Today an error in the last-action update is re-synced every step and **would pass**. |
| 195 `imu_rpy_deg: Some(yesense rpy)` | Gravity from yesense's roll/pitch/yaw (`vendor_projected_gravity`) | net.pcap | (a) input, but **a path the live controller never takes** | Nothing in `src/` sets `imu_rpy_deg`. The live IMU is an LCM `sensor_msgs/Imu` quaternion (module.rs:215-224), so gravity uses `projected_gravity(quat)` (runner.rs:395-400) and tilt uses `roll_pitch(quat)` (state_machine.rs:520-528). | Derive like the vendor (SPEC "Policy numerics": rl_deploy uses yesense ImuData_ rpy): carry yesense's rpy into `RobotState.imu_rpy_deg` on the live path. Or add a test of the live quat path against the vendor (today only golden.rs checks it, against our own Python at 1e-4). The bit-exact claim currently does not cover the robot. |
| 173-182 prefix | Reset, activate and axes of every earlier step of the runner (command slew and phase) | replay.py plan | (b) | The state machine's entry/switch flags and runner reset (state_machine.rs ~985) | Label as replay-only input (windows start mid-run) |
| 202-204 | The first 5 steps are skipped (history fill) | Chosen | (b) | n/a | Fine; SPEC notes the first step after an entry still differs [U] |
| policy_steps.py:76-84, 88-94 | Step rows = the plan's (where rl_deploy's targets changed); state = `st[row-1]`; IMU = last sample before row−1 | ECAN capture, vendor output | (d) | `since_offer` cadence (state_machine.rs:979-994), plus the latency issue above | Label as replay-only input for alignment, and see the ECAN latency row |
| replay.py:332-352 | Policy-step rows = rows where the vendor's leg targets changed, with a 20-tick cadence in between | Vendor output | (d) | Cadence from entry | Same |

## examples/policy_session.rs and tools/policy_steps.py (`session`)

| File:line | Value | Source | Cat | Live controller instead | Recommendation |
|---|---|---|---|---|---|
| default closed loop (236-264 only with `--teacher`) | Own last action | n/a | n/a | Same as live | Good |
| 219-235 entry/switch; per-step `runner` and `params` | From the plan's segments (rl_deploy's enable frames, replay.py:324-331) and stand-in detection from **the vendor's gains on the bus** (replay.py:361-370), with stand-in params timed from that (372-394) | Vendor output | (b) | The Controller picks the stand-in and params (state_machine.rs ~1000-1025) | Label as replay-only input. ecan_dock_rec's rl_control windows check stand-in choice and gains live; stand-in targets are checked nowhere. |
| policy_steps.py:155-162 `late ≤ 3` cadence | A late step reads its on-cadence tick's reply | Heuristic fit | (d) | n/a | Label as replay-only input |
| policy_steps.py:42 `COMMAND_LEAD_S = 1.5 ms`, 109/143 `NET_OFFSET_S` (0 / −1.7 ms) | Command and IMU clock offsets | Fitted per session | (c)/(d) | n/a | Label as replay-only input |
| 143, 252-253 knobs `M20_IMU_DELAY_US`, `M20_LAG` (1), `M20_IMU_SHIFT` | Alignment knobs | Chosen | (d) | Live lag is one tick fresher (see ECAN) | Same |
| 180-202 accel 20-tick mean, padding missing ticks | Derived as rl_deploy does | n/a | n/a | `AccelAverage` in rl_control ticks (state_machine.rs:977) | Good |
| 278 `imu_rpy_deg` | As in policy_20261120 | | (a), non-live path | Quaternion | Same recommendation |

## examples/fault_replay.rs

| File:line | Value | Source | Cat | Live controller instead | Recommendation |
|---|---|---|---|---|---|
| 113-128 target per row | rl_deploy's state from the plan's segments | Vendor enable frames | (b) | The controller's own state and target | Label as replay-only input (no assertions; it only prints) |
| 118-121 gait | Plan gait at policy-step rows (includes the inferred stand-in) | Vendor output | (b) | The current runner's gait | Same |
| 131-134 tilt from rpy | yesense rpy | net.pcap | (a), non-live path | `roll_pitch(quat)` | Same as the IMU row |

## examples/replay.rs (open loop)

| File:line | Value | Source | Cat | Live controller instead | Recommendation |
|---|---|---|---|---|---|
| 133 | `f32_state_clock = false` | Chosen (wall-clock stamps) | (b) | true on the robot (m20_controller.toml:64) | Label as replay-only input |
| 161-175 | `force(state)`, `set_time(1e9)`, `hold_state(state)` every tick | Plan segments | (b) | The real transitions and start-up gates | Label as replay-only input. **Transition logic could be deleted and this would still "match".** |
| 151-159 | Enable frames written directly | Plan | (b) | The state machine's enable ticks | Same |
| 190-195 | Teacher `last_action` | rl_deploy output | (b) | Own action | Same |
| plan `row`, `state_row`, `reset`, `activate`, `params` | As above | Vendor output | (d)/(b) | | Same |

## tests/reflective_column_replay.rs

| File:line | Value | Source | Cat | Live controller instead | Recommendation |
|---|---|---|---|---|---|
| 275-291 offset | Log − capture clock = median of the best-centre matches | **Fitted on the answer** | (d) | n/a | Label as replay-only input |
| 299-306 pairing | Among sweeps within 0.12 s (`PAIR_WINDOW_S`, 42), the one whose **post centres best match the logged ones** | **Keyed on the expected output** | (d) | Every sweep (detector_module.rs:274) | Pair by time only (dock_rec's rule: last sweep < 0.06 s before step 1) and assert that the pair is the best-centre one. |
| 344-352 | Only 90% of logged sweeps must pair and 80% have both poses; the rest are dropped silently | Chosen | (d) | n/a | Print and bound the unpaired ones |
| 169-177 decoder settings | Vendor driver config.yaml | Vendor config | config | Same settings live (unverified for the module's driver) | Fine |

## tests/sim_sequence.rs

| File:line | Value | Source | Cat | Live controller instead | Recommendation |
|---|---|---|---|---|---|
| 203, 442, 492-496 | Docking starts fresh; start point and restore gait (0x3001) are derived and asserted | n/a | n/a | Same as live | Good: the only fresh dock → undock coverage. Never covers boot on the dock or an undock with no prior dock. |
| 226-227, 294-297, 350-353 `docked_x -= median bias` | The sim's docked point = docked_x_m minus the detector's x bias **measured against ground truth** | Sim truth | (c) | Live has no truth. docked_x_m cancels in charge_manager's x (detector.rs:205, D:267-269). | The battery contact check (311-315) is relative to where the controller aims, so it cannot catch a systematic stop offset. Put the sim's contacts at the dock geometry (world.py) and assert against that. |
| 416 `contact [0.01, 0.01]`; 469 `SIM_DETECTOR_BIAS_M = 0.004` | Success window and tolerance | Two real docks; chosen | (c) | n/a | Label as fitted (the comment already says so) |
| 309-322 battery model; 258-262 lidar noise/seed; Lockstep policy | Sim stand-ins | Chosen | sim | | Fine |

## tests/golden.rs, tests/modes.rs, tests/ecan_capture.rs, tests/common.rs, other examples

| File:line | Value | Cat | Note |
|---|---|---|---|
| golden.rs:51-63 | Inputs and outputs from our own Python reference (quat path, own last action) | none from the vendor | The only test of the live quat-gravity path, and only at 1e-4 / 1e-3 against our port, not the vendor. |
| modes.rs | Pure decode of the capture | (a) | Nothing seeded |
| ecan_capture.rs:81, 116 | `header.seq` and `frame[16]` from rl_deploy's frames | (b) counter | Same seq recommendation as ECAN |
| ecan_capture.rs:132 | Selects a frame after a timestamp | (a) | Fine |
| common.rs | Config and policy dir only | - | Nothing seeded |
| examples/dock_pcap.rs, reply_census.rs, ecan_dump.rs | Decoders and detector only | (a) | Nothing seeded |
| m20_controller.toml:693-697 `docked_x_m = 0.735` | Fitted (circle fit, dock_compare.py) | (c), live | Cancels in charge_manager's x. It only affects published `dock_pose` "out", so its error is invisible to every docking test. |
| m20_controller.toml:751-753 `pose_wait_overhead_us = 216` | Fitted on the 18 logged missed waits | (c), **live code** (D:1083) | It keeps dock_rec's ±1 ms write times (18 × 216 µs ≈ 3.9 ms). The vendor's real overhead is its logging plus stopMove time, which varies. This is a tuning constant in live code: decide whether to keep it, and label it. |

---

## Prioritized: seeds most likely to hide a live bug

1. **The IMU rpy path** (policy_20261120.rs:195, policy_session.rs:278). The bit-exact policy proof uses yesense rpy, but nothing live sets `imu_rpy_deg` (module.rs:215 builds from the quaternion). The robot runs a gravity and tilt computation that has never been compared with the vendor.
2. **Joint-state latency** (ecan_dock_rec.rs:187-221; policy_steps.py `row-1`). The replays prove rl_deploy's "reply before the previous send". The live rt loop uses the reply to the command just sent (rt.rs:233-236), one tick fresher. Verify, then either match the vendor or re-prove.
3. **`detector_service = true`** (dock_rec.rs:555), with only the vendor's processed sweeps. The live path (D:726 immediate, an always-on detector, filter restart at D:963) is never replayed. Dock timing and the ReflectorFilter start fix may differ live.
4. **`resume_charging(start_point, (17, 0x3003))`** (dock_rec.rs:571), plus `start_point()` fitted to the steers. The boot-on-dock → undock path (charge_cmd false, goal 1.3 m, restore 17/0x1001) is untested. The in-recording dock's start point (log 0.906093, −0.001405, −0.058511) and restore pair are derived but never asserted.
5. **Teacher-forced `last_action`** (policy_20261120.rs:187). It re-syncs every step, so last-action bugs pass.
6. **Vendor obstacle readings** (dock_rec.rs:244, VL:329). `nearest_obstacle` has never been compared with charge_manager's readings, and the replay's held reading does not match the live per-sweep publish.
7. **Battery via rl_deploy's /BATTERY_DATA** (dock_rec.rs:150). The live `charge_reading` from the ECAN frames, and its earlier arrival, are untested in the ±1 ms confirm asserts.
8. **ECAN rl_control windows**: warm-up alignment fitted to the first target change (ecan_dock_rec.rs:494-504) and pre-warmed histories (375-399). A broken live cadence phase or warm-up hold still passes in windowed mode. The clock phase is also fit in-sample on the lie-down (685-692).
9. **Answer-keyed inputs and unasserted slack**:
   - VL:321 `navigating` comes from the failure line.
   - VL:271-287 looks up the undock goal from later in the log.
   - VL:374-387 ±1 ms retries are not asserted.
   - reflective_column_replay.rs:299-306 pairs sweeps by the expected centres.
10. **Frame seq and MAC copied from rl_deploy's frames** (ecan_dock_rec.rs:536, 554; ecan_capture.rs:81). A broken seq counter (ecan.rs:372, 380) passes.

Unverified: whether item 2's latency changes outputs measurably; whether an always-on detector changes the ReflectorFilter start fix (item 3); the module's lidar driver settings versus the vendor's.

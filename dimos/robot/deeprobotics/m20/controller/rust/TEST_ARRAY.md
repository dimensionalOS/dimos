# M20 operator test array: vendor stack vs ours

Every operation an operator would try, from every starting state, with what the vendor stack does (the binaries are
the spec, recordings confirm), what ours does, and the test that proves it. Status: same / **differs** / **unknown**
(bold rows need work). Addresses: `bs` = basic_server (AOS /opt/robot/share/basic_server, md5 538e34f2, Ghidra),
`rl` = rl_deploy v1.5.8, `cm` = charge_manager v4.0.30. `sm` = src/state_machine.rs, `mod.py` = ../module.py.
Captures: 11-20 14:16 / 16:36 ECAN sessions, 11-24 dock recording.

## Who decides what

The vendor has three layers; ours has two.

| layer | vendor | ours |
|---|---|---|
| app / nav commands, usage mode, gait gating | basic_server (VelControl, CmdHandle, NavWorker) | M20Controller RPCs (mod.py); **no usage mode** |
| docking | charge_manager (NOS) | src/dock (in the controller), vendor-exact replay |
| state machine, policies, faults | rl_deploy | sm, runner.rs, policy.rs |

basic_server facts every row below leans on:
- SetMotionState(1) (bs 0x2622d0, the app's stand): send stand, wait up to 5 s for it, then RLControl (0x11),
  SendGait(0x1001), setControlUsageMode(0). Every app stand ends on 0x1001 in REGULAR mode.
- setControlUsageMode (bs 0x261ba8), only on a change: 0 REGULAR (OOA, LIO off), 1 (lidar on, OOA / LIO / height map
  off), 2 NAVIGATION (OOA, terrain classifier, LIO, height map on). Modes 0 and 2 also SwitchGait(0x1001).
- Charger status (bs DDSWorker::ChannelChargerStatusCallback 0x220c60, on a change of charge_manager's status): state 0
  -> mode 0 unless a nav task runs; 1 or 3 -> mode 1; 2 -> wake from sleep.
- SwitchGait(g) (bs 0x261640): nothing if MotionInfo already reports g; unknown codes refused; else SendGait and wait
  3 s for MotionInfo to report it, 3 tries.
- App gait (bs cmdHandle_Gait 0x18b430): 0x2001-0x2004 need the authorization code (0xe007); while a nav task runs
  only 0x3xxx is accepted (0xe008).
- Velocity: app Steer (bs sendVel 0x261e50) is clamped per axis, x / y negated in reverse direction, and reaches
  /STEER only in mode 0; RealSteer, NavCmd and the nav gait service only in modes 1 / 2; charge_manager's
  /CHARGE_CMD steer only in modes 1 / 2.
- Nav task start (bs NavWorker::robotStart): mode 1, StandUp(0x1001) if lying, RLControl, SwitchGait(0x3002), mode 1;
  each waypoint then switches to its own gait (NavWorker::handle_task).

rl_deploy facts: entry gaits 0x1001 / 0x3001 / 0x3004 / 0xf003, others -> 0x1001; the 0x1001 entry runs the
0xf001 stand-in (3 s first time, 1.5 s later, + 0.5 s); a switch to 0x3001 runs a 1.5 s stand-in; a gait switch
(update_policy_ptr) happens only while |vx| + |vy| + |wz| < 0.01, with OnExit / OnEnter action handover; faults per
SafeController (SPEC "State machine").

## Starting states

S1 fresh power-on lying (StartDamping / Idle) · S2 docked charging · S3 docked not charging · S4 standing in each
gait (0x1001, 0x1002, 0x1003, 0x1004, 0x2001-4, 0x3001, 0x3002, 0x3003, 0x3004, 0x3005, 0xf001, 0xf002) ·
S5 walking (any gait, cmd_vel nonzero) · S6 damping / fault · S7 e-stop pressed.

## The array

| # | operation | from | vendor (evidence) | ours (where) | status | test |
|---|---|---|---|---|---|---|
| 1 | stand (blueprint standup()) | M20Connection.standup(): Stand(1) <=12 s, RL(17) <=5 s, wait 0x1001 (bs SetMotionState resets gait + mode 0), usage mode 1 (x3), 0x1001 then terrain gait (0x3003) each confirmed, hold 3 s, command_ready <=15 s; False on failure (connection.py 330-384); StandUp itself = IK 0.12 -> 0.38 m (capture 14:16 rows 27203-30200) | stand_up bit for bit (sm vendor_stand_up); standup() = the same sequence (mod.py standup, pending commit); bs stand -> RL + 0x1001 + mode 0 (basic_server.rs observe) | same (pending commit) | basic_server a_stand_then_enters_rl_control_on_basic_in_regular_mode; ecan_dock_rec_20261124 |
| 2 | stand before 2 s up | S1 | refused until 2 s after the interface came up (rl IdleState) | passive.min_s_before_stand (sm 552) | same | sim_sequence |
| 3 | stand | S2 / S3 | charge_manager undock path, not a plain stand: see 30 | the same RPC as 1, no dock awareness | **unknown** | TODO |
| 4 | stand | S4 / S5 | M20Connection: already RL -> mode 1, 0x1001, terrain gait; bs SetMotionState(1) while in RL times out (no change) | standup() skips the stand when RlControl; bs layer: a Stand while in RL waits 5 s for StandUp, times out | same (pending commit) | a_stand_that_never_comes_times_out |
| 5 | stand | S6 | JointDamping -> Idle after 2 s or on another target, then stands; under a fault it stays | same, plus blocked target until a different one (sm 541) | same | state_machine tests |
| 6 | sit / lie down | S4 | LieDown 2 s cubic to IK 0.03, 2 s damping, then zero; refused during a stand-in | sm 836, refused during stand-in (sm 582) | same | ecan_dock_rec_20261124 (lie-down) |
| 7 | lie down | S5 | bs LieDown = SetMotionState(4) only; rl goes to LieDown at once, velocity ignored | liedown() zeroes cmd_vel first, then lie_down | same (zeroing is extra but harmless) | TODO walking lie-down sim |
| 8 | lie down | S1 / S6 | Idle stays (no LieDown from Idle) | same (sm 552) | same | state_machine |
| 9 | damping (motion 2, soft e-stop) | any | JointDamping kd 4 / wheel 1, 2 s, then Idle | same (sm 552) | same | modes.rs |
| 10 | passive (motion 0) | S4 | **unknown** whether rl RLControl -> Idle directly; ours goes LieDown first | sm 582 (Passive = LieDown from rl_control) | **unknown** | TODO from rl fsm.c |
| 11 | gait 0x1001 -> 0x1002 / 0x1003 / 0x1004 | S4 | rl update_policy_ptr 0x179960: switches only while normalized |vx|+|vy|+|wz| < 0.01; a gait sent during the stand-in is lost (Run writes 0xf001 over the slot) | same (sm rl_control target_gait, switch_below_speed; pending commit) | same (pending commit) | a_gait_switch_waits_until_the_command_is_still, a_gait_sent_during_the_stand_in_is_lost_and_a_resend_switches |
| 12 | gait 0x1001 -> 0x3002 navi_flat | S4 | done by every nav task (bs robotStart); rl handover, no stand-in | handover same; sim: navi_flat falls from basic_flat's stance | **unknown** | sim "switch basic_flat -> navi_flat" known fail; TODO on capture |
| 13 | gait 0x1001 -> 0x3003 / 0x3005 | S4 | direct switch | same | same | standup_rpc_navi_stair |
| 14 | gait -> 0x3001 navi_charge | S4 | 1.5 s 0xf001 stand-in every switch, kp / kd x1.3 for 29 steps | same (toml stand_in, gain_scale) | same | tests/policy_20261120.rs |
| 15 | gait -> 0x3004 finetune | S4 | direct; pose inputs from /STEER | same | same | dock_rec_20261124 |
| 16 | gait -> 0xf001 / 0xf002 six-axis, step | S4 | direct; /STEER axes become pose inputs | same (runner params) | same | golden, sim stand |
| 17 | gait -> 0xf003 align body posture | S1 | bs aligningBodyPosture: only from Idle, RLControl + 0xf003, wait acc_z > 9 and height > const, then 0x1001; 7 s timeout -> damping | **not implemented** (rl 0xf003 not re-implemented) | **differs** | TODO |
| 18 | gait 0x2001-4 extre | S4 | needs authorization code (bs 0xe007) | accepted without one | **differs** | TODO |
| 19 | any non-0x3xxx gait during a nav task | S5 | refused 0xe008 | no nav tasks in ours | **differs** (no nav task) | n/a until nav tasks |
| 20 | unknown gait code | S4 | bs refuses (log, no send); rl would use basic_flat | controller runs fallback runner | **differs** (bs refusal) | TODO |
| 21 | same gait again | S4 | bs no-op | no-op (cmd.gait unchanged) | same | - |
| 22 | gait while lying | S1 | bs sends it; rl keeps it for the next rl_control entry (entry rule) | kept in UserCommand, entry rule applies (sm 603) | same | modes.rs |
| 23 | usage mode 0 / 1 / 2 | any | bs setControlUsageMode: on change; 0 and 2 -> SwitchGait(0x1001); sensors on / off | usage_mode input + basic_server.rs; sensors not switched (dimos runs its own) | same for gait / gating (pending commit); sensors **differ** (deliberate) | basic_server tests |
| 24 | cmd_vel walk, each gait | S4 | /REAL_STEER m/s / per-gait divisor, clipped; /STEER normalized; 150 ms timeout zeroes | same divisors (toml real_steer_divisor), timeout 0.155 s (rt.rs 228) | same | policy_20261120, golden |
| 25 | cmd_vel above max | S5 | normalized clipped to +-1 (rl), app Steer clamped (bs sendVel) | axes clipped in runner | same for rl, **differs** for the bs clamp / reverse direction | TODO |
| 26 | cmd_vel ramp | S5 | limit_cmd_vel per-step slew (basic_flat 0.025 / 0.025 / 0.04, stair 0.04 / 0.04 / 0.08) | same (toml command_slew) | same | golden |
| 27 | cmd_vel in the wrong usage mode | S5 | NavCmd / RealSteer / charge steer only in modes 1 / 2 | zeroed outside 1 / 2 (basic_server.rs out) | same (pending commit) | velocity_reaches_the_controller_only_in_modes_one_and_two |
| 28 | stop (zero cmd_vel) | S5 | policy brakes; a pending gait switch happens once still | same (still-rule) | same (pending commit) | a_gait_switch_waits_until_the_command_is_still |
| 29 | dock | S4 | cm goToPileAction: records gait + state, NaviCharge, staging / fine point, FinetuneStand, lie, charge check, retries | vendor-exact port (src/dock/vendor.rs), steer goes straight to the controller | same sequence; **differs**: bs forwards cm steer only in modes 1 / 2 | vendor_replay (100 docks / 94 undocks), dock_rec_20261124, sim dock tests |
| 30 | undock | S2 / S3 | cm exitPileAction: stand, back off, restore the recorded gait; bs charger state 0 -> mode 0 -> SwitchGait(0x1001); M20Connection then _stand() to the terrain gait | docking restores; bs layer: state 0 -> mode 0 -> 0x1001; **no M20Connection _stand() after (parity rank 7)** | **differs** (blueprint level) | charger_states_drive_the_mode_and_the_undock_ends_on_basic |
| 31 | undock ends in | S2 | 0x1001 basic_flat, regular mode, no automatic nav gait | 0x1001, no auto switch | same | TODO assert in sim |
| 32 | cancel dock | S5 (docking) | cm cancel (charge cmd 0) | cancel_dock() (docking.rs 881) | same | docking tests |
| 33 | stand / walk command during a dock | S5 | any motion / gait command cancels cm's sequence (?) | motion_seq change cancels (docking.rs) | **unknown** (vendor) | TODO from cm |
| 34 | charge loss while docked | S2 | cm charging loop: no charge -> err 0x1007 / retry; bs mode 0 on state 0 | charging loop port | **unknown** (whole path) | TODO |
| 35 | started on the dock | S2 / S3 | cm PowerOnSelfTest, idle 3 s charge check -> charging | ported (89becf2c16) | same | docking tests |
| 36 | tilt / fall | S4 / S5 | SafeController tilt limits by target and gait (20 / 45 / 45-65 / 90 deg) | same (sm 175) | same | tilt_limits_are_rl_deploy_s, fault_replay |
| 37 | joint out of range | S5 | 31 readings > 0.24 rad past the range | same (sm 254) | same | joint_checks_count_like_rl_deploy |
| 38 | driver fault / overheat | any | status word != 1; motor 120 / driver 100 deg C latched | same (sm 204) | same | driver_faults_are_rl_deploy_s |
| 39 | overheat soft fallback | S5 | motor >= 100 / driver >= 90, still, > 70 C for 1 s -> 0xf001 stand | **not implemented** | **differs** | TODO |
| 40 | fault recovery | S6 | stays in damping while the fault holds; target reset to 2 | stays; requires a different request | same | state_machine |
| 41 | e-stop press | any | **unknown**: remote e-stop -> motion 2? hardware button cuts joint power -> driver status fault | driver status fault only | **unknown** | TODO record |
| 42 | e-stop release | S7 | **unknown** (needs a new stand?) | needs a new stand | **unknown** | TODO record |
| 43 | IMU stale / NaN | S5 | status bits only, no fault | same | same | fault_replay |
| 44 | missed motor replies | S5 | no fault | no fault (5aa0a98c2d) | same | reply_census |
| 45 | inference failure | S5 | Ort throws: process exits (dog drops) | fault -> JointDamping | **differs** (safer, deliberate) | unknown_forced_runner_does_not_panic |
| 46 | restart our stack while standing | S4 | rl_deploy is a service: the blueprint stopping leaves the robot standing | M20Walker=controller: the module lies the robot down, then exits (7a3d455252); M20Walker=service: the m20-controller unit keeps holding (rust/service, pending commit, not installed) | same with the service; lie-down otherwise | rt shutdown_tests |
| 47 | restart while docked | S2 | cm PowerOnSelfTest resumes charging | resume_charging (docking.rs 642) | same | docking tests |
| 48 | sleep / wake (app) | S1 | bs PowerManagement: setSleepManual lies down, setPowerState wakes with 0x1001 | **none** | **differs** | TODO |
| 49 | reset joint zero (app) | S1 | bs cmdHandle_ResetJointsZero | **none** | **differs** | out of scope? ask |
| 50 | nav task (app) | S1 / S4 | robotStart: mode 1, stand if lying, 0x3002, waypoints with their gaits | **none** (dimos nav instead) | **differs** | TODO RPC for the nav start |
| 51 | direction (reverse) | S5 | bs setDirection: x / y negated for app Steer | **none** | **differs** | TODO |
| 52 | status query | any | m20tool status (MotionInfo, gait, usage mode, battery, errors) | controller_report topic + status() RPC (852a453e0c, 1d05cd36b4), usage_mode (pending commit) | same | status.rs tests, test_what_the_native_process_publishes_reaches_the_python_side |

## Blueprint parity (M20Connection vs M20Controller, scratchpad blueprint_parity.md)

The stack's reference is the vendor-mode blueprint (M20Connection + bridge + basic_server). Ranks there:
1 standup blocking / False, 2 stand after a fault, 3 refuse standup / liedown / terrain while docked, 13 terrain
confirm: done (pending commit). 4 module stop: lie-down safety net (7a3d455252), service (pending). 12 still-rule:
done (pending). 7 / 8 undock and failed dock end with M20Connection's _stand(), 9 dock() / undock() block and raise, 10 goal cancel
around docking (on dock start, and every 1 s while velocity arrives during docking / on the dock), 14 NaN zeroing,
0.4 s timeout, 2 / 1 / 2 clamp, 15 command_ready at 10 Hz: done (pending commit). **Open: 5, 6, 11 (dock agent:
docking.rs), 15's /HES_STATUS e-stop input (no e-stop source in our stack yet), 18 charge_status() type, 19 move(),
21 odom, 22 motion_state / gait_state topics.**

## Order of fixes

1. Row 1 / 4: standup() ends on 0x1001 like the app stand; a separate nav-start RPC does robotStart (row 50).
2. Rows 23, 27, 29, 30, 18, 20: port basic_server's usage mode, charger-driven mode changes and gating.
3. Rows 11, 12, 28: rl_deploy's still-rule for gait switches; find why basic_flat -> navi_flat falls in our sim.
4. Rows 17, 39, 48, 51: the remaining features. Rows marked unknown need the vendor recording or more decompile.

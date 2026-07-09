# R1 Lite Integration — Bring-up Test Suite (scaffolding)

Robot-side ROS 2 validation for the Galaxea R1 Lite, ported from
[`scripts/r1pro_test/`](../r1pro_test/) (Mustafa's R1 Pro work). These
scripts talk **plain ROS 2** to the robot — no dimos — so we can prove the
robot obeys commands before layering the dimos connection Module on top.

> **Status: SCAFFOLDING.** Every topic name, joint count, domain ID, and
> pose in [`r1lite_config.py`](r1lite_config.py) is inherited from the R1
> Pro and marked `TODO(recon)`. Nothing here has run against R1 Lite
> hardware yet. **Run `test_00_recon.py` first and reconcile the config.**

## Known R1 Lite differences from R1 Pro (verify during recon)

| Aspect | R1 Pro | R1 Lite (expected) |
|---|---|---|
| Arm DOF | 7 per arm | **6 per arm** (Galaxea spec) |
| Upper-body joints | 18 (4+7+7) | likely **16** (4+6+6) — confirm |
| ROS | ROS 2 Humble | docs ship ROS1 **and** ROS2 tracks — confirm which is installed |
| Chassis 3-gate problem | yes | unknown — test_03 assumes it may differ |
| ROS_DOMAIN_ID | 41 | unknown — run `domain_scan.py` |

## Order of operations

```bash
# 0. Find the robot's domain (robot stack must be running)
python3 scripts/r1lite_test/domain_scan.py

# 1. Recon: dump topics/nodes/rates/joint-counts; then edit r1lite_config.py
export ROS_DOMAIN_ID=<from step 0>
python3 scripts/r1lite_test/test_00_recon.py

# 2. Once config is reconciled, run the graded suite (single DDS session)
python3 scripts/r1lite_test/run_all_tests.py --skip-chassis --skip-arm  # read-only first
python3 scripts/r1lite_test/run_all_tests.py                            # full, robot moves
```

All commands run **inside the dev container** (`ghcr.io/dimensionalos/ros-dev:dev`,
`--network host`) with `source /opt/ros/humble/setup.bash` and the py3.10
venv active — same environment as the R1 Pro bring-up.

## Files

| File | Purpose |
|---|---|
| `r1lite_config.py` | Single source of truth for topics/DOF/domain — **the file to edit after recon** |
| `domain_scan.py` | Sweep ROS_DOMAIN_ID 0-101 to find the robot |
| `test_00_recon.py` | Read-only graph dump: topics, nodes, rates, joint counts |
| `test_01_topic_discovery.py` | Assert expected topics are visible |
| `test_02_read_arm_feedback.py` | Read left-arm JointState feedback |
| `test_03_chassis_command.py` | Drive chassis briefly (robot moves) |
| `test_04_arm_joint_command.py` | Move one arm joint and return (robot moves) |
| `test_06_torso_command.py` | Move torso to home pose and back (robot moves) |
| `run_all_tests.py` | Graded runner, one rclpy session, safety confirms between steps |

## Open questions the recon must close

1. ROS 1 or ROS 2 on this unit? (If ROS 1: apply Galaxea's ROS2 upgrade first.)
2. Real topic names + message types vs the R1 Pro `/hdas/*` + `/motion_target/*` set.
3. Arm/torso DOF and safe torso home pose (from the robot's startup scripts).
4. Does the chassis need a gatekeeper / have the 3-gate problem?
5. RMW implementation + FastDDS version.

# Plan Review: m20-rosnav-migration

**Generated:** 2026-03-13
**Reviewers:** Forward (spec→plan), Reverse (plan→spec), Context (plan→codebase)

---

## Summary

| Category | P0 | P1 | P2 | Total |
|----------|----|----|----|----|
| Coverage Gaps | 2 | 3 | 3 | 8 |
| Codebase Misalignment | 0 | 3 | 1 | 4 |
| Consistency Issues | 1 | 2 | 2 | 5 |
| Scope Creep | 0 | 0 | 0 | 0 |
| **Total** | **3** | **8** | **6** | **17** |

---

## P0 Findings (Must Fix)

### 1. Open Questions 1-6 must be resolved before implementation

- **Category:** Coverage Gap
- **Found by:** Forward
- **What:** The spec explicitly states Open Questions 1-6 must be resolved "before implementation." The plan starts Phase 1 (blueprint/config creation) without resolving OQ1 (rsdriver DDS QoS), OQ2 (FASTLIO2 fork dual-lidar support), OQ3 (IMU message type), OQ4 (Python 3.10 on aarch64). These are binary blockers — if FASTLIO2 fork only supports single-lidar, or IMU uses drdds custom type, the entire pipeline fails at integration time.
- **Action:** Update plan — add Phase 0 (Discovery) with SSH-to-robot tasks resolving OQ 1-6 before any implementation begins.
- **Recommendation:** Add 4 tasks: (1) SSH to NOS, run `ros2 topic info -v /lidar_points /IMU` to get QoS and message types, (2) Review robosense_fast_lio fork source for dual-lidar merged input, (3) Test `uv python install 3.10` on NOS aarch64, (4) Estimate nav container image size from existing builds.

### 2. drdds PYTHONPATH not wired for host dimos process

- **Category:** Coverage Gap
- **Found by:** Forward
- **What:** The spec requires drdds-ros2-msgs on PYTHONPATH for `/NAV_CMD` publishing. Task 2.1 rebuilds drdds for Python 3.10 but no task ensures PYTHONPATH is set when `launch_nos.py` runs via `deploy.sh start`. The robot will accept goals but silently fail to send velocity commands.
- **Action:** Update plan — add PYTHONPATH configuration to Task 2.1 (setup) and Task 2.3 (start).
- **Recommendation:** In Task 2.1, install drdds into the venv's site-packages (or add to PYTHONPATH in the venv activation script). In Task 2.3, verify `python -c "import drdds"` succeeds before launching.

### 3. Velocity translation pipeline not verified [CORROBORATED]

- **Category:** Consistency Issue
- **Found by:** Forward + Context (corroborated)
- **What:** The coverage matrix dismisses spec §1.3 (ROSNav Bridge Adaptations) as "no changes needed." Forward review found no task verifies the cmd_vel→NavCmd pipeline works end-to-end. Context review confirmed `_on_cmd_vel` exists at M20Connection line 397 and routes to `M20VelocityController.set_twist()`, so the code IS there — but no plan task verifies: (a) `_on_cmd_vel` is wired as an `In[Twist]` stream port, (b) it works with `enable_ros=False`, (c) NavCmd actually reaches the robot.
- **Action:** Update plan — add verification task in Phase 1 or Phase 4, and correct coverage matrix entry.
- **Recommendation:** Add acceptance criterion to Task 1.2: "Verify M20Connection.cmd_vel: In[Twist] is wired by autoconnect to ROSNav.cmd_vel: Out[Twist]." Add acceptance criterion to Task 4.3: "Verify /NAV_CMD DDS topic receives velocity commands during autonomous navigation."

---

## P1 Findings (Should Fix)

### 4. docker_restart_policy duplicate flag [CORROBORATED]

- **Category:** Codebase Misalignment
- **Found by:** Context
- **What:** `DockerModuleConfig` already has `docker_restart_policy: str = "on-failure:3"` as a native field with the desired default. The plan says to add `"--restart=on-failure:3"` to `docker_extra_args`, which would produce a duplicate `--restart` flag in the docker run command.
- **Action:** Update plan — remove `--restart` from `docker_extra_args` in Task 1.1. The default is already correct; no override needed.

### 5. all_blueprints.py is auto-generated, not manually edited

- **Category:** Codebase Misalignment
- **Found by:** Context
- **What:** File header says "This file is auto-generated. Run `pytest dimos/robot/test_all_blueprints_generation.py` to regenerate." Task 1.3 tells the developer to manually add an entry. This will be overwritten on the next generation run.
- **Action:** Update plan — Task 1.3 should say "Run `pytest dimos/robot/test_all_blueprints_generation.py` to regenerate the registry" instead of manually editing.

### 6. entrypoint.sh modification missing from plan

- **Category:** Codebase Misalignment
- **Found by:** Context
- **What:** The context document explicitly flagged `dimos/robot/deeprobotics/m20/docker/entrypoint.sh` as requiring modification. It currently waits for `/ODOM`, checks `/ALIGNED_POINTS`, and restarts lio_perception — all wrong for ROSNav mode. The plan's Modified Files table and task list omit this file entirely.
- **Action:** Update plan — add entrypoint.sh to Phase 2 as a task (adapt for host-side use: keep `/IMU` wait, replace `/ALIGNED_POINTS` with `/lidar_points`, remove lio_perception restart).

### 7. Phase 2/3 gate criteria absent from plan

- **Category:** Coverage Gap
- **Found by:** Forward
- **What:** The spec defines explicit Phase 2 gate (2-week arise_slam spike with go/no-go) and Phase 3 gate (relocalization validated, map persistence, stair-climbing gait tested). The plan says "separate plans" but doesn't document these gates or reference where they'll be planned.
- **Action:** Update plan — add a "Phase Gates" section referencing the spec's stated criteria and timeboxes.

### 8. SLAM divergence user-facing state reporting not implemented

- **Category:** Coverage Gap
- **Found by:** Forward
- **What:** Spec requires dimos viewer to show "Localization Degraded" (high uncertainty) and "Localization Lost" (no valid pose). No plan task implements or verifies host-side covariance monitoring or state propagation.
- **Action:** Update plan — add Task to Phase 4 (or as acceptance criterion on Task 4.2) to verify SLAM state is reported to dimos viewer. Alternatively, explicitly defer to Phase 2 with rationale.

### 9. Network loss error handling absent from plan

- **Category:** Coverage Gap
- **Found by:** Forward
- **What:** The spec has an entire error handling section for NOS-to-AOS/GOS network loss (DDS reconnect, FASTLIO2 reconvergence). No plan task addresses this. Coverage matrix has no row for it.
- **Action:** Update plan — add to Phase 4 integration test or add coverage matrix entry noting this is tested during end-to-end validation (Task 4.3 implicitly covers some of this).

### 10. M20Connection ROS removal: spec says "remove", plan uses flags

- **Category:** Consistency Issue
- **Found by:** Reverse
- **What:** Spec §1.4 says "Remove the ROS path entirely (no more rclpy in M20Connection)." Plan keeps M20Connection unchanged and suppresses via `enable_ros=False, enable_lidar=False`. This leaves dead code and doesn't match the spec's intent of a clean architectural boundary.
- **Action:** Accept as-is with documentation — the flag approach avoids breaking `m20_minimal` blueprint which still uses the ROS path for teleop mode. Document this deviation explicitly in the plan.

### 11. Blueprint file path deviates from spec

- **Category:** Consistency Issue
- **Found by:** Reverse
- **What:** Spec §1.5 says `dimos/robot/deeprobotics/m20/blueprints/m20_rosnav.py`. Plan creates `blueprints/rosnav/m20_rosnav.py`. The subdirectory pattern matches codebase convention (existing `basic/`, `agentic/`, `smart/` subdirs).
- **Action:** Accept as-is — plan follows codebase convention over spec's stated path. Document deviation.

---

## P2 Findings (Consider)

### 12. IMU topic message type unverified before integration
- **Found by:** Forward. OQ3 asks if `/IMU` uses `sensor_msgs/Imu` or drdds custom. No pre-work task.

### 13. enable_lidar=False narrative is misleading
- **Found by:** Context. Plan says flags "suppress outputs" — actually only skips CycloneDDS init. Port declarations remain. Blueprint's `.remappings()` is still required (and IS correctly included).

### 14. --legacy flag maintenance burden
- **Found by:** Reverse. Spec doesn't prescribe this; maintaining dual code paths adds complexity. Consider just documenting the git commit for rollback instead.

### 15. Undocumented env vars in M20ROSNavConfig
- **Found by:** Reverse. `USE_ROUTE_PLANNER`, `MODE`, `USE_RVIZ` not in spec. Reasonable container config but represents undocumented implementation knowledge.

### 16. No existing M20 blueprints in all_blueprints.py
- **Found by:** Context. Plan says "place alphabetically after existing M20 entries" but no M20 entries exist.

### 17. OOM monitoring >80% warning not implemented
- **Found by:** Forward. Spec says "Warning at >80% usage." Plan checks post-hoc in Task 4.3 but doesn't implement the warning mechanism.

---

## Coverage Summary

**Forward (Spec→Plan):**
- Fully covered: 15 sections
- Partially covered: 6 sections
- Not covered: 5 sections (Phase 2/3 deferred, network loss, SLAM user feedback)

**Reverse (Plan→Spec):**
- Spec-backed: 14 tasks
- Spec-implied: 3 tasks
- Infrastructure: 4 tasks
- Scope creep: 0
- Gold-plating: 0

**Context Alignment:**
- Aligned: 11 decisions
- Contradicts: 2 decisions (restart policy, all_blueprints edit method)
- Narrative errors: 2 (enable_lidar description, ROSNav proxy description)

**Coverage Matrix Accuracy:** 14/19 rows correct; 3 inaccurate; 2 missing rows

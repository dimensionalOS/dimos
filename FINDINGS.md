# Hybrid Teleop on Alfred: findings

Integration branch `mustafa/integration/hybrid-teleop-alfred` = main + `mb/feat/alfred-full-stack` + the
operator-ui stack (hold, page, estop, cloud, target). One line per issue. Severity: **blocker** (stops the
demo), **fix** (wrong or missing, worked around here), **nice** (worth doing, not in the way). PR names refer to
the proposed breakdown at the end.

| # | What broke or was missing | Where | Severity | PR |
|---|---|---|---|---|
| 1 | Main deleted the point-cloud self-filter (#4612, replaced by `UrdfManagerPointClearing`) that Alfred's perception builds on; git merged it silently and Alfred stacks failed to import. Restored the 4 files here. | dimos `dimos/manipulation/planning/utils/*self_filter*.py`, `alfred_full_perception.py` | blocker | A |
| 2 | Alfred used the early port of `UrdfManager` (`self_filter`, `robot_description` topic); main's has neither. Call site adapted; the Controller reads both protocols. | dimos `alfred_full.py` `alfred_urdf_manager()` | fix | A |
| 3 | `pick_object`: main's grasp contact check wants the arm before grasps exist, Alfred picks the arm after (nearest to the object). Merged: contact check runs once the arm is chosen. On Alfred it is skipped anyway: the scene is in odom, the grasps in base_link ("no scene point cloud in the grasps' frame"). | dimos `dimos/manipulation/pick_and_place_module.py` | fix | A |
| 4 | An Alfred-side scene-registration test leaked its module thread; main's thread-leak check fails it. Test now stops the module. | dimos `test_object_scene_registration_temporal.py` | nice | A |
| 5 | The branch name `mustafa/integration/...` fails the branch-name hook ("integration" is not an allowed type); commits here use `SKIP=branch-name`. | dimos `bin/hooks/branch_name_check` | nice | none |
| 6 | operator-ui-hold's task card makes `operator_hold` require an `operator_hold_command` port: a plain `ControlCoordinator` with `operator_hold_task()` now fails at start (worked on main). | dimos `dimos/control/tasks/operator_hold_task/_registry.py` | fix | B |
| 7 | `OperatorHoldCoordinator` lived in an xArm blueprint file; any other robot importing it pulled in the xArm sim. Moved to `dimos/control/operator_hold_coordinator.py`. | dimos `hybrid_teleop_sim.py` | fix | B |
| 8 | A Desktop app cannot read `operator_hold_status` / `operator_estop_status`: they are pickled Python dataclasses on zenoh. Added `get_operator_status()` (RPC, plain JSON) and `OperatorHandoff.operator_status` (JSON in a `std_msgs.String`). | dimos `operator_hold_coordinator.py`, `operator_handoff.py` | fix | C, D |
| 9 | `ControlCoordinator.set_estop` latches only teleop/twist/gripper tasks; arm trajectories, the base's velocity and planar-trajectory tasks and the arm hold ignore it. An app e-stop must be latch + hold + cancel: added the `estop(bool)` RPC doing exactly the stream e-stop's steps. | dimos `dimos/control/coordinator.py`, `operator_hold_coordinator.py` | fix | C |
| 10 | The lift bypasses the hold and the e-stop: `PillarConnection.home` / `set_position` (RPCs and agent skills) talk to the Nano directly. The app's lift jog and Home still move it during a hold, so does an agent's `set_position`, and a homing run cannot be frozen at all (set_position is refused while homing). App buttons are disabled during hold/e-stop here; the agent skills are still open. | dimos `dimos/robot/diy/alfred/pillar_connection.py`; app lift card | fix | F |
| 11 | The gripper task keeps its target through a hold and finishes it after Acknowledge, so not everything "stays put until told". Arm and base trajectories abort cleanly; base teleop goes idle after 0.2 s. | dimos `dimos/control/tasks/gripper_task/gripper_task.py` | nice | B |
| 12 | `OperatorCommandsModule` (operator-ui-target) duplicates the app's scan / click / pick, is stream-only (web SDK), and its pick acknowledges a hold by itself, which conflicts with "Acknowledge is the operator's, explicit". Not used on Alfred. | dimos `dimos/manipulation/operator_assist/operator_commands_module.py` | nice | none (drop) |
| 13 | No `OPENAI_API_KEY` on the Orin (`.env` or env), so no LLM agent can run there. Bench stacks add McpClient only when the key is set; `request_operator` is reachable over MCP (:9990) either way. | Orin `~/dimos/.env` | blocker (agent path only) | E |
| 14 | `cancel_trajectory` cancels only the coordinator's one joint-trajectory task (arms + lift); a whole-body plan's base half stops only because the hold preempts it. | dimos `dimos/control/coordinator.py` | nice | C |
| 15 | Alfred's right wrist motor (`openarm_right_joint5`, CAN id 5 on can3) is dead (pinned 129 °C sensor, found Oct 26): hardware picks are left-arm only. | Alfred hardware | blocker (right arm) | none |

# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""House demo startup is lazy and cancellation prevents subsequent motion."""

from concurrent.futures import CancelledError
import json
from pathlib import Path

import numpy as np
import pytest

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.robot.galaxea.r1pro.home_blueprint import r1pro_home_sim
from dimos.robot.galaxea.r1pro.home_coordinator import R1ProHomeCoordinator
from dimos.robot.galaxea.r1pro.home_sim import R1ProHomeSim
from dimos.robot.galaxea.r1pro.navigation_base import PlanarVelocityServo
from dimos.robot.galaxea.r1pro.navigation_blueprint import build_r1pro_packing_navigation
from dimos.robot.galaxea.r1pro.navigation_delivery import follow_navigation_path
from dimos.robot.galaxea.r1pro.packing_run import PackingRunConfig, run_packing_sequence


def test_home_blueprint_resolves_without_compiling_a_scene(mocker, tmp_path):
    compile_scene = mocker.patch("dimos.robot.galaxea.r1pro.grasping_blueprint.GraspingTask")
    blueprint = build_r1pro_packing_navigation(
        scene_path=tmp_path / "not-yet-built.xml",
        artifact=str(tmp_path / "policy"),
        simulator=R1ProHomeSim,
        coordinator_type=R1ProHomeCoordinator,
        prepare_scene_on_build=True,
    )
    assert len(blueprint.blueprints) == 5
    compile_scene.assert_not_called()
    # Exercise the same parser as `dimos run`, including current main's typed globals.
    parsed = BlueprintConfigParser(r1pro_home_sim).parse(environ={})
    assert parsed.global_config["transport"] == "zenoh"
    assert parsed.global_config["simulation"] == "mujoco"


def test_full_tray_stops_without_starting_act(mocker, tmp_path):
    sim, policy, control = mocker.Mock(), mocker.Mock(), mocker.Mock()
    sim.packing_state.return_value = {"ready_for_pick": True, "success": False}
    sim.packing_order.return_value = [0]
    sim.select_bottle.return_value = {"selected": False, "reason": "tray_full"}
    report = run_packing_sequence(
        control,
        policy,
        sim,
        PackingRunConfig(artifact=Path("policy"), output=tmp_path),
        {},
    )
    assert report["completion_reason"] == "tray_full"
    assert not report["success"]
    policy.start_rollout.assert_not_called()
    assert json.loads((tmp_path / "result.json").read_text())["completion_reason"] == "tray_full"


def test_cancelled_navigation_does_not_start_the_task(mocker):
    pause = mocker.Mock(side_effect=CancelledError("stopped"))
    sim, control = mocker.Mock(), mocker.Mock()
    with pytest.raises(CancelledError):
        follow_navigation_path(control, sim, [[0, 0, 0], [1, 0, 0]], "carry", {}, pause=pause)
    control.task_invoke.assert_not_called()


def test_base_hold_still_runs_if_task_cancellation_fails(mocker):
    sim, control = mocker.Mock(), mocker.Mock()
    sim.task_state.return_value = {
        "base_pose": [0, 0, 0],
        "sim_time": 1,
        "obstacles": ["cabinet"],
        "tray": {"support_geoms": []},
    }
    control.task_invoke.side_effect = [None, True, RuntimeError("cancel RPC failed")]
    with pytest.raises(RuntimeError, match="cancel RPC failed"):
        follow_navigation_path(control, sim, [[0, 0, 0], [1, 0, 0]], "carry", {"stages": []})
    sim.stop_navigation_base.assert_called_once_with()


def test_faster_servo_obeys_acceleration_and_diagonal_speed_caps():
    pose = np.zeros(3)
    servo = PlanarVelocityServo(pose, max_speed=0.6, max_accel=0.6)
    previous = np.zeros(3)
    for tick in range(150):
        now = tick * 0.02
        servo.command_twist(np.array([1.0, 1.0, 0.0]), now)
        pose = servo.step(pose, 0.02, now)
        assert np.linalg.norm(servo.velocity[:2] - previous[:2]) <= 0.012000001
        assert np.linalg.norm(servo.velocity[:2]) <= 0.600000001
        previous = servo.velocity.copy()
    assert np.linalg.norm(servo.velocity[:2]) == pytest.approx(0.6)
    servo.stop(pose)
    assert servo.step(pose, 0.02, 4) == pytest.approx(pose)

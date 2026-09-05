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

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any

import numpy as np
import pytest

from dimos.control.task import CoordinatorState, JointStateSnapshot
from dimos.control.tasks.microduck_policy_task.microduck_policy_task import (
    ACTION_LEN,
    OBS_LEN,
    MicroDuckPolicyTask,
    MicroDuckPolicyTaskConfig,
)
from dimos.hardware.whole_body.spec import IMUState
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.robot.pollen.microduck.config import MICRODUCK_HOME, MICRODUCK_JOINTS


@dataclass
class _Node:
    name: str
    shape: list[int]
    type: str = "tensor(float)"


class _FakeSession:
    def __init__(self, policy: str, value: float) -> None:
        self.policy = policy
        self.value = value
        self.inputs: list[np.ndarray[Any, np.dtype[np.float32]]] = []

    def get_inputs(self) -> list[_Node]:
        return [_Node("observation", [1, OBS_LEN])]

    def get_outputs(self) -> list[_Node]:
        return [_Node("action", [1, ACTION_LEN])]

    def run(self, outputs: list[str], feeds: dict[str, Any]) -> list[np.ndarray[Any, Any]]:
        assert outputs == ["action"]
        observation = np.asarray(feeds["observation"], dtype=np.float32)
        self.inputs.append(observation.copy())
        return [np.full((1, ACTION_LEN), self.value, dtype=np.float32)]


@pytest.fixture
def policy_dir(tmp_path: Path) -> Path:
    policies = [
        {"file": "alpha_walking.onnx", "kind": "perpetual"},
        {"file": "alpha_stand.onnx", "kind": "perpetual"},
        {
            "file": "alpha_sitstand.onnx",
            "name": "sitstand",
            "kind": "scripted",
            "unwind_s": 1.0,
        },
        {
            "file": "alpha_ground_pick.onnx",
            "name": "ground_pick",
            "kind": "episodic",
            "duration_s": 2.8,
            "command": {"encoding": "phase", "period_s": 4.0, "end_phase": 0.7},
        },
        {"file": "roulade.onnx", "kind": "episodic", "duration_s": 1.0, "chain": True},
        {
            "file": "ball_kick_left.onnx",
            "name": "kick_left",
            "kind": "episodic",
            "duration_s": 0.5,
        },
        {
            "file": "ball_kick_right.onnx",
            "name": "kick_right",
            "kind": "episodic",
            "duration_s": 0.5,
        },
        {
            "file": "roller.onnx",
            "kind": "perpetual",
            "mode": "roller",
            "action_scale": 0.8,
        },
    ]
    manifest = {
        "schema_version": 2,
        "model_api": 1,
        "obs_len": OBS_LEN,
        "action_len": ACTION_LEN,
        "robot": {"model": "microduck", "control_hz": 50},
        "policies": policies,
    }
    (tmp_path / "manifest.json").write_text(json.dumps(manifest), encoding="utf-8")
    for item in policies:
        if item.get("mode") != "roller":
            (tmp_path / str(item["file"])).write_bytes(b"fake ONNX")
    return tmp_path


@pytest.fixture
def task_and_sessions(
    policy_dir: Path,
) -> tuple[MicroDuckPolicyTask, dict[str, _FakeSession]]:
    values = {
        "alpha_walking": 0.2,
        "alpha_stand": 0.1,
        "alpha_sitstand": 0.3,
        "alpha_ground_pick": 0.4,
        "roulade": 0.5,
        "ball_kick_left": 0.6,
        "ball_kick_right": 0.7,
    }
    sessions: dict[str, _FakeSession] = {}

    def factory(path: Path, _providers: list[str]) -> _FakeSession:
        session = _FakeSession(path.stem, values[path.stem])
        sessions[path.stem] = session
        return session

    task = MicroDuckPolicyTask(
        "microduck_policy",
        MicroDuckPolicyTaskConfig(
            policy_dir=policy_dir,
            joint_names=list(MICRODUCK_JOINTS),
            session_factory=factory,
        ),
    )
    task.start()
    return task, sessions


def _state(t_now: float = 1.0, dt: float = 0.02) -> CoordinatorState:
    return CoordinatorState(
        joints=JointStateSnapshot(
            joint_positions=dict(zip(MICRODUCK_JOINTS, MICRODUCK_HOME, strict=True)),
            joint_velocities={
                name: float(index) / 10.0 for index, name in enumerate(MICRODUCK_JOINTS)
            },
            joint_efforts={name: 0.0 for name in MICRODUCK_JOINTS},
        ),
        imu={
            "microduck": IMUState(
                quaternion=(1.0, 0.0, 0.0, 0.0),
                gyroscope=(1.0, 2.0, 3.0),
            )
        },
        t_now=t_now,
        dt=dt,
    )


def _twist(vx: float, vy: float = 0.0, yaw_rate: float = 0.0) -> Twist:
    message = Twist()
    message.linear.x = vx
    message.linear.y = vy
    message.angular.z = yaw_rate
    return message


def test_policy_set_warms_every_model_and_reports_manifest_skills(
    task_and_sessions: tuple[MicroDuckPolicyTask, dict[str, _FakeSession]],
) -> None:
    task, sessions = task_and_sessions

    assert set(sessions) == {
        "alpha_walking",
        "alpha_stand",
        "alpha_sitstand",
        "alpha_ground_pick",
        "roulade",
        "ball_kick_left",
        "ball_kick_right",
    }
    assert all(len(session.inputs) == 1 for session in sessions.values())
    assert task.list_skills() == [
        {"name": "ground_pick", "duration_s": 2.8, "chainable": False, "required_mode": "walk"},
        {"name": "kick_left", "duration_s": 0.5, "chainable": False, "required_mode": "walk"},
        {"name": "kick_right", "duration_s": 0.5, "chainable": False, "required_mode": "walk"},
        {"name": "roulade", "duration_s": 1.0, "chainable": True, "required_mode": "walk"},
    ]


def test_observation_action_scale_and_filters_are_shared_across_policy_switches(
    task_and_sessions: tuple[MicroDuckPolicyTask, dict[str, _FakeSession]],
) -> None:
    task, sessions = task_and_sessions
    state = _state()

    stand_output = task.compute(state)
    assert stand_output is not None
    stand_observation = sessions["alpha_stand"].inputs[-1][0]
    np.testing.assert_allclose(stand_observation[0:3], [1.0, 2.0, 3.0])
    np.testing.assert_allclose(stand_observation[3:6], [0.0, 0.0, -1.0])
    np.testing.assert_allclose(stand_observation[6:20], np.zeros(ACTION_LEN))
    np.testing.assert_allclose(stand_observation[20:34], np.arange(ACTION_LEN) / 10.0)
    np.testing.assert_allclose(stand_observation[34:48], np.zeros(ACTION_LEN))
    np.testing.assert_allclose(stand_observation[48:61], np.zeros(13))
    np.testing.assert_allclose(stand_output.positions, np.asarray(MICRODUCK_HOME) + 0.1, atol=1e-7)

    assert task.on_twist_command(_twist(0.4), 1.01)
    state.t_now = 1.02
    walk_output = task.compute(state)

    assert walk_output is not None
    walk_observation = sessions["alpha_walking"].inputs[-1][0]
    np.testing.assert_allclose(walk_observation[34:48], np.full(ACTION_LEN, 0.1))
    np.testing.assert_allclose(walk_observation[48:51], [0.08, 0.0, 0.0], rtol=1e-6)
    np.testing.assert_allclose(walk_observation[51:61], np.zeros(10))
    expected = np.asarray(MICRODUCK_HOME) + 0.18
    expected[0:5] = np.asarray(MICRODUCK_HOME[0:5]) + 0.156
    expected[5:9] = np.asarray(MICRODUCK_HOME[5:9]) + 0.14
    expected[9:14] = np.asarray(MICRODUCK_HOME[9:14]) + 0.156
    np.testing.assert_allclose(walk_output.positions, expected, rtol=1e-6)
    assert task.get_status()["current_policy"] == "walk"


def test_deadman_returns_to_stand_and_clears_applied_twist(
    task_and_sessions: tuple[MicroDuckPolicyTask, dict[str, _FakeSession]],
) -> None:
    task, _ = task_and_sessions
    state = _state()
    assert task.on_twist_command(_twist(0.4), 1.0)
    task.compute(state)
    state.t_now = 1.6

    output = task.compute(state)

    assert output is not None
    status = task.get_status()
    assert status["current_policy"] == "stand"
    assert status["applied_twist"] == {"vx": 0.0, "vy": 0.0, "yaw_rate": 0.0}
    assert status["command_age_s"] == pytest.approx(0.6)


@pytest.mark.parametrize(
    ("skill", "duration"),
    [("ground_pick", 2.8), ("kick_left", 0.5), ("kick_right", 0.5), ("roulade", 1.0)],
)
def test_each_one_shot_runs_for_its_manifest_window_then_hands_back_to_stand(
    task_and_sessions: tuple[MicroDuckPolicyTask, dict[str, _FakeSession]],
    skill: str,
    duration: float,
) -> None:
    task, _ = task_and_sessions
    state = _state(dt=0.1)
    assert task.run_skill(skill) == {"accepted": True, "reason": None}

    output = task.compute(state)
    assert output is not None
    assert task.get_status()["current_policy"] == skill
    assert task.get_status()["busy"]

    for _ in range(round(duration / state.dt) + 1):
        state.t_now += state.dt
        task.compute(state)
    state.t_now += state.dt
    task.compute(state)

    status = task.get_status()
    assert status["current_policy"] == "stand"
    assert status["busy"] is False


def test_posture_head_body_estop_and_reset_contract(
    task_and_sessions: tuple[MicroDuckPolicyTask, dict[str, _FakeSession]],
) -> None:
    task, _ = task_and_sessions
    state = _state(dt=0.5)

    assert task.set_head_pose(0.1, 0.2, 0.3, 0.1)["accepted"]
    assert not task.set_head_pose(0.1, 0.2, 1.5, 0.1)["accepted"]
    assert task.set_body_pose(z=-0.01, roll=0.1, pitch=-0.1)["accepted"]
    assert task.set_posture("sit")["accepted"] is False
    assert task.set_body_pose(active=False)["accepted"]
    assert task.set_posture("sit") == {"accepted": True, "reason": None}
    assert task.set_posture("sit") == {"accepted": True, "reason": None}
    task.compute(state)
    assert task.get_status()["current_policy"] == "sit"
    assert task.set_posture("stand")["accepted"]
    task.compute(state)
    assert task.get_status()["current_policy"] == "rise"

    task.set_estop(True)
    status = task.get_status()
    assert status["estopped"] is True
    assert status["armed"] is False
    assert status["busy"] is False
    task.start()
    assert task.get_status()["estopped"] is True
    assert task.get_status()["armed"] is False
    task.set_estop(False)
    assert task.arm()["accepted"]
    assert task.reset_runtime_state(reactivate=True)
    assert task.get_status()["armed"] is True


def test_incomplete_state_emits_nothing_and_runtime_failure_disarms(
    task_and_sessions: tuple[MicroDuckPolicyTask, dict[str, _FakeSession]],
) -> None:
    task, sessions = task_and_sessions
    incomplete = _state()
    incomplete.joints.joint_positions.pop(MICRODUCK_JOINTS[-1])

    assert task.compute(incomplete) is None

    sessions["alpha_stand"].value = float("nan")
    assert task.compute(_state()) is None
    status = task.get_status()
    assert status["armed"] is False
    assert "non-finite" in status["last_error"]


def test_look_at_reports_reachable_and_clamped_targets(
    task_and_sessions: tuple[MicroDuckPolicyTask, dict[str, _FakeSession]],
) -> None:
    task, _ = task_and_sessions

    reachable = task.look_at(1.0, 0.2, -0.1)
    behind = task.look_at(-1.0, 0.0, 0.0)

    assert reachable["accepted"] is True
    assert reachable["clamped"] is False
    assert behind["accepted"] is True
    assert behind["clamped"] is True
    assert abs(behind["head"]["head_yaw"]) == pytest.approx(1.4)

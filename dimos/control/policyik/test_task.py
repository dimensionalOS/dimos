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

"""Regression checks for the portable policy's temporal and command contract."""

from collections.abc import Iterator
import hashlib
import json
from pathlib import Path

import numpy as np
import pytest

from dimos.control.policyik.controller import PERIOD_NS
from dimos.control.policyik.task import PolicyIKTask, PolicyIKTaskConfig
from dimos.control.task import CoordinatorState, JointStateSnapshot


@pytest.fixture
def task(tmp_path: Path) -> Iterator[PolicyIKTask]:
    names = [f"joint{i}" for i in range(18)]
    xml = ['<robot name="test"><link name="base"/>']
    for i, name in enumerate(names):
        parent = "base" if i == 0 else f"link{i - 1}"
        xml.append(
            f'<link name="link{i}"/><joint name="{name}" type="revolute">'
            f'<parent link="{parent}"/><child link="link{i}"/>'
            '<origin xyz="0 0 .02"/><axis xyz="0 0 1"/>'
            '<limit lower="-2" upper="2" effort="10" velocity="3"/></joint>'
        )
    (tmp_path / "robot.urdf").write_text("".join(xml) + "</robot>")
    np.savez(
        tmp_path / "actor.npz",
        mean=np.zeros(124),
        variance=np.ones(124),
        weight0=np.zeros((256, 124)),
        bias0=np.zeros(256),
        weight1=np.zeros((256, 256)),
        bias1=np.zeros(256),
        weight2=np.zeros((18, 256)),
        bias2=np.ones(18),
    )
    manifest = dict(
        joints=names,
        tools=["link8", "link17"],
        frame="base",
        actor_sha256=hashlib.sha256((tmp_path / "actor.npz").read_bytes()).hexdigest(),
        urdf_sha256=hashlib.sha256((tmp_path / "robot.urdf").read_bytes()).hexdigest(),
    )
    (tmp_path / "manifest.json").write_text(json.dumps(manifest))
    value = PolicyIKTask("paired", PolicyIKTaskConfig(tmp_path))
    value.start()
    yield value
    value.stop()


def state(stamp: float) -> CoordinatorState:
    return CoordinatorState(
        joints=JointStateSnapshot(
            joint_positions={f"joint{i}": 0.0 for i in range(18)},
            joint_velocities={f"joint{i}": 0.0 for i in range(18)},
            timestamp=stamp,
        ),
        t_now=stamp,
        dt=PERIOD_NS / 1e9,
    )


def goals(x: float) -> list[list[float]]:
    return [[x, 0, 0.18, 1, 0, 0, 0], [x, 0, 0.36, 1, 0, 0, 0]]


def test_codec_nanosecond_roundoff_does_not_drop_control_ticks(task: PolicyIKTask) -> None:
    for i in range(600):
        stamp = (i * PERIOD_NS - i % 2) / 1e9
        assert task.compute(state(stamp)) is not None
    assert task.status()["inferences"] == 600


def test_goal_changes_preserve_accumulated_reference(task: PolicyIKTask) -> None:
    task.set_goals(goals(0.2), "base")
    first = task.compute(state(0))
    task.set_goals(goals(0.4), "base")
    second = task.compute(state(PERIOD_NS / 1e9))
    assert first is not None and second is not None
    np.testing.assert_allclose(first.positions, [0.0625] * 18)
    np.testing.assert_allclose(second.positions, [0.125] * 18)


def test_stale_and_duplicate_feedback_never_advances_policy(task: PolicyIKTask) -> None:
    sample = state(0)
    assert task.compute(sample) is not None
    assert task.compute(sample) is None
    sample.t_now = 1
    assert task.compute(sample) is None
    assert task.status()["inferences"] == 1


def test_partial_preemption_stops_whole_coordinated_controller(task: PolicyIKTask) -> None:
    assert task.compute(state(0)) is not None
    task.on_preempted("other", frozenset(["joint0"]))
    assert not task.is_active()
    assert task.compute(state(PERIOD_NS / 1e9)) is None
    task.set_goals(goals(0.2), "base")
    output = task.compute(state(2 * PERIOD_NS / 1e9))
    assert output is not None
    np.testing.assert_allclose(output.positions, [0.0625] * 18)


def test_estop_cannot_resume_old_accumulator(task: PolicyIKTask) -> None:
    assert task.compute(state(0)) is not None
    task.set_estop(True)
    assert task.compute(state(PERIOD_NS / 1e9)) is None
    task.set_estop(False)
    assert not task.is_active()
    task.set_goals(goals(0.2), "base")
    output = task.compute(state(2 * PERIOD_NS / 1e9))
    assert output is not None
    np.testing.assert_allclose(output.positions, [0.0625] * 18)


@pytest.mark.parametrize(
    "frame,values",
    [
        ("world", goals(0.2)),
        ("base", [[0, 0, 0, 0, 0, 0, 0]] * 2),
        ("base", [[float("nan"), 0, 0, 1, 0, 0, 0]] * 2),
    ],
)
def test_invalid_goal_rejected(task: PolicyIKTask, frame: str, values: list[list[float]]) -> None:
    with pytest.raises(ValueError):
        task.set_goals(values, frame)

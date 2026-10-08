# Copyright 2025-2026 Dimensional Inc.
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

from __future__ import annotations

import xml.etree.ElementTree as ET

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from dimos.control.task import CoordinatorState, JointStateSnapshot
from dimos.control.tasks.sew_teleop_task.task import SewTeleopTask
from dimos.robot.galaxea.r1pro.sew_model import R1SewModel
from dimos.teleop.webxr.body_tracking import BodyTrackingSnapshot
from dimos.teleop.webxr.controller_types import Buttons


@pytest.fixture
def task(tmp_path):
    root = ET.Element("robot", name="generated_test_arm")
    ET.SubElement(root, "link", name="base_link")

    def joint(name, parent, child, axis, xyz="0 0 .2", kind="revolute"):
        ET.SubElement(root, "link", name=child)
        j = ET.SubElement(root, "joint", name=name, type=kind)
        ET.SubElement(j, "parent", link=parent)
        ET.SubElement(j, "child", link=child)
        ET.SubElement(j, "origin", xyz=xyz, rpy="0 0 0")
        ET.SubElement(j, "axis", xyz=axis)
        ET.SubElement(j, "limit", lower="-1.4", upper="1.4", effort="1", velocity="1")

    for i, a in enumerate(["0 1 0", "0 1 0", "0 -1 0", "0 0 1"], 1):
        joint(
            f"torso_joint{i}", "base_link" if i == 1 else f"torso_link{i - 1}", f"torso_link{i}", a
        )
    for side, y in [("left", 0.2), ("right", -0.2)]:
        joint(f"{side}_base", "torso_link4", f"{side}_arm_base_link", "0 0 1", f"0 {y} .2", "fixed")
        for i, a in enumerate(["0 1 0", "1 0 0", "0 0 1", "0 1 0", "0 0 1", "0 1 0", "1 0 0"], 1):
            joint(
                f"{side}_arm_joint{i}",
                f"{side}_arm_base_link" if i == 1 else f"{side}_arm_link{i - 1}",
                f"{side}_arm_link{i}",
                a,
                "0 0 -.1",
            )
    p = tmp_path / "model.urdf"
    ET.ElementTree(root).write(p)
    return SewTeleopTask("test", R1SewModel(p))


def snapshot(task, q, t):
    c = np.array([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
    hand_axes = np.column_stack([[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]])
    center = np.array([0.0, 0.0, 1.5])
    joints = {}

    def add(name, p, r=np.eye(3)):
        joints[name] = {
            "position": (c.T @ p).tolist(),
            "orientation": Rotation.from_matrix(c.T @ r).as_quat().tolist(),
        }

    add("hips", center + np.array([0.0, 0.0, -0.5]))
    for i, side in enumerate(["left", "right"]):
        u, l, h = task.model.solvers[side].features(q[4 + 7 * i : 11 + 7 * i])
        shoulder = center + np.array([0.0, 0.2 if i == 0 else -0.2, 0.0])
        add(f"{side}-arm-upper", shoulder)
        add(f"{side}-arm-lower", shoulder + u * 0.3)
        add(f"{side}-hand-wrist", shoulder + u * 0.3 + l * 0.3)
        add(f"{side}-hand-palm", shoulder + u * 0.3 + l * 0.3, h @ hand_axes.T)
    return BodyTrackingSnapshot(
        type="body_tracking_snapshot", capture_time_s=t, frame_id="local-floor", joints=joints
    )


def tick(task, q, t, pressed=True):
    task.on_body_tracking(snapshot(task, q, t), t)
    task.on_teleop_buttons(Buttons((1 << 1) | (1 << 9) if pressed else 0), t)
    return task.compute(
        CoordinatorState(
            joints=JointStateSnapshot(joint_positions=dict(zip(task.names, q, strict=True))),
            t_now=t,
            dt=0.01,
        )
    )


def test_gate_emits_nothing_then_active_and_loss_requires_release(task):
    q = np.zeros(18)
    q[[7, 14]] = -0.4
    assert tick(task, q, 1.0, False) is None
    assert tick(task, q, 1.01) is None
    assert task.state == "ALIGNING"
    assert tick(task, q, 1.2) is None
    result = tick(task, q, 1.4)
    assert result is not None
    np.testing.assert_allclose(result.positions, q, atol=1e-7)
    state = CoordinatorState(
        joints=JointStateSnapshot(joint_positions=dict(zip(task.names, q, strict=True))),
        t_now=2.0,
        dt=0.01,
    )
    assert task.compute(state) is None
    assert task.state == "WAIT_RELEASE"
    assert tick(task, q, 2.1) is None
    assert task.state == "WAIT_RELEASE"
    assert tick(task, q, 2.2, False) is None
    assert tick(task, q, 2.3) is None
    assert task.state == "ALIGNING"


def test_missing_one_arm_disarms_whole_group(task):
    q = np.zeros(18)
    q[[7, 14]] = -0.4
    tick(task, q, 1.0, False)
    tick(task, q, 1.1)
    tick(task, q, 1.5)
    bad = snapshot(task, q, 1.6)
    del bad.joints["left-arm-lower"]
    task.on_body_tracking(bad, 1.6)
    assert (
        task.compute(
            CoordinatorState(
                joints=JointStateSnapshot(joint_positions=dict(zip(task.names, q, strict=True))),
                t_now=1.6,
                dt=0.01,
            )
        )
        is None
    )
    assert task.state == "WAIT_RELEASE"


def test_misalignment_does_not_move_robot(task):
    q = np.zeros(18)
    q[[7, 14]] = -0.4
    tick(task, q, 1.0, False)
    operator = q.copy()
    operator[4] = 0.5
    task.on_body_tracking(snapshot(task, operator, 1.1), 1.1)
    task.on_teleop_buttons(Buttons((1 << 1) | (1 << 9)), 1.1)
    state = CoordinatorState(
        joints=JointStateSnapshot(joint_positions=dict(zip(task.names, q, strict=True))),
        t_now=1.1,
        dt=0.01,
    )
    assert task.compute(state) is None
    assert task.state == "ALIGNING"
    assert task.status["ready"] is False


def test_active_command_is_bounded_for_all_arm_joints(task):
    q = np.zeros(18)
    q[[7, 14]] = -0.4
    tick(task, q, 1.0, False)
    tick(task, q, 1.1)
    assert tick(task, q, 1.5) is not None
    operator = q.copy()
    operator[4] = 0.2
    task.on_body_tracking(snapshot(task, operator, 1.6), 1.6)
    task.on_teleop_buttons(Buttons((1 << 1) | (1 << 9)), 1.6)
    result = task.compute(
        CoordinatorState(
            joints=JointStateSnapshot(joint_positions=dict(zip(task.names, q, strict=True))),
            t_now=1.6,
            dt=0.01,
        )
    )
    assert result is not None
    assert 0 < np.max(np.abs(np.asarray(result.positions) - q)) <= 0.00500001


def test_changed_geometry_is_rejected(task, tmp_path):
    task.model.joints["left_arm_joint2"].find("axis").set("xyz", "0 1 0")
    p = tmp_path / "changed.urdf"
    ET.ElementTree(task.model.root).write(p)
    with pytest.raises(ValueError, match="Unsupported SEW geometry"):
        R1SewModel(p)


def test_nonmonotonic_frame_and_stalled_tick_disarm(task):
    q = np.zeros(18)
    q[[7, 14]] = -0.4
    tick(task, q, 1.0, False)
    tick(task, q, 1.1)
    assert tick(task, q, 1.5) is not None
    assert not task.on_body_tracking(snapshot(task, q, 1.5), 1.6)
    assert task.state == "WAIT_RELEASE"
    state = CoordinatorState(
        joints=JointStateSnapshot(joint_positions=dict(zip(task.names, q, strict=True))),
        t_now=1.6,
        dt=0.3,
    )
    assert task.compute(state) is None
    assert task.status["reason"] == "Invalid or stalled control timestep"

# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

import xml.etree.ElementTree as ET

import pytest

from dimos.control.task import CoordinatorState, JointStateSnapshot
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.robot.galaxea.r1pro.config import R1PRO_MODEL_PATH
from dimos.robot.galaxea.r1pro.torso_height_task import (
    TORSO_LIFT_JOINTS,
    R1ProTorsoHeightTask,
    TorsoHeightParams,
    torso_xz,
)


def state(q, now):
    return CoordinatorState(
        joints=JointStateSnapshot(joint_positions=dict(zip(TORSO_LIFT_JOINTS, q, strict=True))),
        t_now=now,
        dt=0.01,
    )


def test_lift_curve_matches_urdf_and_respects_joint_limits_and_rates():
    urdf = ET.parse(R1PRO_MODEL_PATH.resolve()).getroot()
    joints = [urdf.findall(f"joint[@name='torso_joint{i}']")[0] for i in (1, 2, 3)]
    assert all(j.findall("axis")[0].attrib["xyz"] == "0 1 0" for j in joints[:2])
    assert all(j.findall("origin")[0].attrib["xyz"] == "0 0 0.35" for j in joints[1:])
    limits = [
        (float(j.findall("limit")[0].attrib["lower"]), float(j.findall("limit")[0].attrib["upper"]))
        for j in joints[:2]
    ]
    task = R1ProTorsoHeightTask("lift", 20, TorsoHeightParams(), limits)
    q = [0.0, 0.0]
    for i in range(300):
        now = i * 0.01
        task.on_lift_rate(Float32(data=-0.03), now)
        out = task.compute(state(q, now))
        assert out is not None
        target = out.positions
        assert target is not None
        assert max(abs(a - b) for a, b in zip(target, q, strict=True)) <= 0.0015 + 1e-12
        assert all(lo <= p <= hi for p, (lo, hi) in zip(target, limits, strict=True))
        q = target
    x, height = torso_xz(q)
    assert x == pytest.approx(0.0, abs=1e-10)
    assert 0.45 < height < 0.7
    assert task.claim().joints == frozenset(TORSO_LIFT_JOINTS)


def test_timeout_preemption_and_estop_discard_the_lift_target():
    task = R1ProTorsoHeightTask("lift", 20, TorsoHeightParams(), [(-1.2, 1.8), (-2.6, 2.6)])
    q = [0.2, -0.4]
    task.on_lift_rate(Float32(data=-0.03), 1.0)
    moving = task.compute(state(q, 1.0))
    assert moving is not None and moving.positions != q
    stopped = task.compute(state(q, 1.3))
    assert stopped is not None and stopped.positions == q
    assert task.compute(state(q, 1.4)) is None
    task.on_lift_rate(Float32(data=-0.03), 1.4)
    task.on_preempted("hold", frozenset(TORSO_LIFT_JOINTS))
    assert task.compute(state(q, 1.4)) is None
    task.set_estop(True)
    assert not task.on_lift_rate(Float32(data=-0.03), 1.5)
    task.set_estop(False)
    assert task.compute(state(q, 1.6)) is None

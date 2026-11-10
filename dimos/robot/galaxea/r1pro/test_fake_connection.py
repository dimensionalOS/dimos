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

"""Tests for ``FakeR1ProConnection``: the in-memory R1 Pro stand-in.

No ROS 2, no robot, no vendor stack — the module runs entirely in-process
on ``LocalTransport`` ports. The feedback loop is real: commands integrate
into simulated state and come back on ``joint_state``.
"""

from __future__ import annotations

import sys
import time
from typing import Any

import pytest

from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.galaxea.r1pro._connection_test_fakes import LocalTransport
from dimos.robot.galaxea.r1pro.connection import (
    R1PRO_BASE_JOINTS,
    R1PRO_UPPER_BODY_JOINTS,
)
from dimos.robot.galaxea.r1pro.fake_connection import (
    FakeR1ProConnection,
)
from dimos.utils.testing.waiting import wait_until

DEADMAN_S = 5.0


class Rig:
    """One ``FakeR1ProConnection`` with every port on an in-process transport."""

    def __init__(self, **config: Any) -> None:
        self.module = FakeR1ProConnection(**{"deadman_timeout_s": DEADMAN_S, **config})
        self.ports = {
            name: LocalTransport() for name in {**self.module.inputs, **self.module.outputs}
        }
        for name, transport in self.ports.items():
            self.module.set_transport(name, transport)  # type: ignore[arg-type]
        self.feedback: list[JointState] = []
        self.ports["joint_state"].subscribe(self.feedback.append)

    def __enter__(self) -> Rig:
        self.module.start()
        return self

    def __exit__(self, *exc: Any) -> None:
        self.module.stop()

    def move(self, positions: dict[str, float]) -> None:
        self.ports["position_command"].publish(
            JointState(name=list(positions), position=list(positions.values()))
        )

    def drive(self, vx: float, vy: float, wz: float) -> None:
        self.ports["velocity_command"].publish(
            JointState(name=R1PRO_BASE_JOINTS, velocity=[vx, vy, wz])
        )

    def latest_body(self) -> JointState | None:
        """The newest 18-joint feedback frame, if one arrived."""
        return next((m for m in reversed(self.feedback) if len(m.name) == 18), None)


def test_no_ros_or_vendor_imports_needed() -> None:
    """The fake path never pulls in the vendor/ROS modules."""
    assert "rclpy" not in sys.modules
    assert "hdas_msg" not in sys.modules


def test_position_command_moves_feedback_to_target() -> None:
    """A full 18-joint command integrates into the feedback at tracking speed."""
    target = [0.05] * 18
    with Rig() as rig:
        rig.move(dict(zip(R1PRO_UPPER_BODY_JOINTS, target, strict=True)))
        wait_until(
            lambda: (m := rig.latest_body()) is not None
            and all(abs(p - t) < 1e-3 for p, t in zip(m.position, target, strict=True)),
            timeout=5.0,
            interval=0.05,
        )
        m = rig.latest_body()
        assert m is not None and m.name == R1PRO_UPPER_BODY_JOINTS


def test_partial_position_command_is_refused() -> None:
    with Rig() as rig:
        rig.move({R1PRO_UPPER_BODY_JOINTS[0]: 0.1})
        # A partial frame passes validation, then write() refuses it and the
        # module records the failure and halts.
        wait_until(
            lambda: rig.module.status().last_error is not None,
            timeout=2.0,
            interval=0.01,
        )
        error = rig.module.status().last_error
        assert error is not None and "18" in error


def test_grippers_are_independent_and_report_in_metres() -> None:
    def left_opening(rig: Rig) -> float | None:
        for m in reversed(rig.feedback):
            if "r1pro/left_gripper" in m.name:
                return m.position[m.name.index("r1pro/left_gripper")]
        return None

    with Rig() as rig:
        rig.move({"r1pro/left_gripper": 0.06})
        wait_until(
            lambda: (opening := left_opening(rig)) is not None and abs(opening - 0.06) < 1e-3,
            timeout=5.0,
            interval=0.05,
        )
        # The right jaw was never told anything and reports its zero opening.
        right = [m for m in rig.feedback if "r1pro/right_gripper" in m.name]
        assert right
        assert all(
            m.position[m.name.index("r1pro/right_gripper")] == pytest.approx(0.0) for m in right
        )
        with rig.module._lock:
            assert rig.module._gripper_target["right"] is None


def test_gripper_command_out_of_range_is_refused() -> None:
    """The vendor opening range (0-100 mm) is enforced by the limits table."""
    with Rig() as rig:
        rig.move({"r1pro/left_gripper": 0.5})
        wait_until(
            lambda: rig.module.status().last_rejection is not None,
            timeout=2.0,
            interval=0.01,
        )
        with rig.module._lock:
            assert rig.module._gripper_target["left"] is None


def test_chassis_speed_outside_joint_limits_is_rejected() -> None:
    with Rig() as rig:
        rig.drive(5.0, 0.0, 0.0)
        assert rig.module.status().last_rejection is not None
        assert rig.module._speed.linear.x == 0.0


def test_deadman_halts_when_commands_stop() -> None:
    with Rig(deadman_timeout_s=0.2) as rig:
        rig.drive(0.5, 0.0, 0.0)
        wait_until(lambda: rig.module._speed.linear.x > 0, timeout=2.0, interval=0.01)
        wait_until(
            lambda: rig.module.status().deadman_fired,
            timeout=5.0,
            interval=0.05,
        )
        assert rig.module._speed.linear.x == 0.0


def test_nonzero_yaw_world_command_and_feedback_use_canonical_joints() -> None:
    import math

    with Rig() as rig:
        with rig.module._lock:
            rig.module._base_pose = [0.0, 0.0, math.pi / 2]
        rig.drive(0.2, 0.0, 0.0)
        wait_until(lambda: rig.module.base_state()["x"] > 0.01, timeout=2.0, interval=0.01)
        state = rig.module.base_state()
        assert state["vx"] == pytest.approx(0.0, abs=1e-12)
        assert state["vy"] == pytest.approx(-0.2)
        assert state["y"] == pytest.approx(0.0, abs=1e-12)
        planar = next(m for m in reversed(rig.feedback) if m.name == R1PRO_BASE_JOINTS)
        assert list(planar.velocity) == pytest.approx([0.2, 0.0, 0.0])
        assert planar.position[2] == pytest.approx(math.pi / 2)
        assert "cmd_vel" not in rig.module.inputs
        assert "velocity_command" in rig.module.inputs


def test_halt_holds_arms_and_leaves_grippers() -> None:
    """halt zeroes the chassis and holds the arms; the jaws keep their target
    (there is no verified vendor cancel — same as the real module)."""
    with Rig() as rig:
        target = [0.05] * 18
        rig.move(dict(zip(R1PRO_UPPER_BODY_JOINTS, target, strict=True)))
        rig.move({"r1pro/right_gripper": 0.04})
        rig.drive(0.3, 0.0, 0.0)
        wait_until(lambda: rig.module._speed.linear.x > 0, timeout=2.0, interval=0.01)
        rig.module.halt()
        assert rig.module._speed.linear.x == 0.0
        with rig.module._lock:
            assert rig.module._target_q == list(rig.module._q)
            assert rig.module._gripper_target["right"] == 0.04


def test_feedback_can_be_suppressed_and_resumed() -> None:
    with Rig() as rig:
        wait_until(lambda: len(rig.feedback) > 0, timeout=2.0, interval=0.01)
        rig.module.set_feedback_enabled(False)
        rig.module.set_gripper_feedback_enabled("left", False)
        time.sleep(0.1)
        count = len(rig.feedback)
        time.sleep(0.2)
        # No body frames while suppressed; right jaw still reports.
        recent = rig.feedback[count:]
        assert all(len(m.name) != 18 for m in recent)
        assert all("r1pro/left_gripper" not in m.name for m in recent)
        assert any("r1pro/right_gripper" in m.name for m in recent)
        rig.module.set_feedback_enabled(True)
        rig.module.set_gripper_feedback_enabled("left", True)
        wait_until(
            lambda: any(len(m.name) == 18 for m in rig.feedback[count:])
            and any("r1pro/left_gripper" in m.name for m in rig.feedback[count:]),
            timeout=2.0,
            interval=0.01,
        )

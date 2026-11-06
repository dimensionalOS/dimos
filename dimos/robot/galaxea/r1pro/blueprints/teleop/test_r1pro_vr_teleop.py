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

"""Tests for the R1 Pro Quest VR teleop blueprint.

The chain under test is the real one: a live ``ArmTeleopModule`` fed
programmatic Quest packets (``_on_pose_bytes``/``_on_joy_bytes``, the same
decoders the websocket routes call) → the coordinator's ``teleop_ik`` and
gripper tasks → the per-robot contract ports → ``FakeR1ProConnection`` →
``joint_state`` feedback back into the coordinator. No ROS 2, no robot, no
vendor stack, no browser. The actual Quest browser session remains to be
verified by a human on a headset.
"""

from __future__ import annotations

import socket
import sys
import time
from typing import Any

import pytest

from dimos.control.tasks.teleop_ik_task.teleop_ik_task import TeleopIKTask
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.Joy import Joy
from dimos.robot.galaxea.r1pro._connection_test_fakes import LocalTransport
from dimos.robot.galaxea.r1pro.blueprints.teleop.r1pro_vr_teleop import (
    R1PRO_VR_TARGET_FRAMES,
    R1PRO_VR_TASK_NAME,
    R1ProVrCoordinator,
    _vr_tasks,
    make_r1pro_vr_teleop,
)
from dimos.robot.galaxea.r1pro.config import R1PRO_COORDINATOR_MODEL
from dimos.robot.galaxea.r1pro.connection import (
    R1PRO_GRIPPER_JOINTS,
    R1PRO_UPPER_BODY_JOINTS,
    R1ProConnection,
)
from dimos.robot.galaxea.r1pro.fake_connection import FakeR1ProConnection
from dimos.teleop.webxr.extensions import ArmTeleopModule
from dimos.utils.testing.waiting import wait_until


def _free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def _pose_bytes(hand: str, x: float, y: float, z: float) -> bytes:
    return PoseStamped(
        ts=time.time(),
        frame_id=hand,
        position=Vector3(x, y, z),
        orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
    ).lcm_encode()


def _joy_bytes(hand: str, *, trigger: float, grip: float) -> bytes:
    # axes: [thumbstick_x, thumbstick_y, trigger_analog, grip_analog]
    return Joy(
        ts=time.time(),
        frame_id=hand,
        axes=[0.0, 0.0, trigger, grip],
        buttons=[0, 0, 0, 0, 0, 0],
    ).lcm_encode()


class Chain:
    """ArmTeleopModule → R1ProVrCoordinator → FakeR1ProConnection, in-process.

    The streams the blueprint wires share one ``LocalTransport`` per logical
    name, so messages flow exactly as the blueprint routes them; every other
    port gets its own transport.
    """

    #: logical stream -> ((module, out port), (module, in port))
    _SHARED: dict[str, tuple[tuple[str, str], tuple[str, str]]] = {
        "left_cartesian_command": (
            ("teleop", "left_controller_output"),
            ("coordinator", "left_cartesian_command"),
        ),
        "right_cartesian_command": (
            ("teleop", "right_controller_output"),
            ("coordinator", "right_cartesian_command"),
        ),
        "teleop_buttons": (
            ("teleop", "teleop_buttons"),
            ("coordinator", "teleop_buttons"),
        ),
        "left_gripper_command": (
            ("teleop", "left_gripper_command"),
            ("coordinator", "left_gripper_command"),
        ),
        "right_gripper_command": (
            ("teleop", "right_gripper_command"),
            ("coordinator", "right_gripper_command"),
        ),
        "position_command": (
            ("coordinator", "r1pro_position_command"),
            ("fake", "position_command"),
        ),
        "joint_state": (
            ("fake", "joint_state"),
            ("coordinator", "r1pro_joint_state"),
        ),
    }

    def __init__(self, *, input_timeout_s: float = 1.0, deadman_s: float = 5.0) -> None:
        self.teleop = ArmTeleopModule(server_port=_free_port(), input_timeout_s=input_timeout_s)
        self.coordinator = R1ProVrCoordinator(
            tick_rate=100,
            model=R1PRO_COORDINATOR_MODEL,
            command_only_joints=list(R1PRO_GRIPPER_JOINTS),
            tasks=_vr_tasks(),
        )
        self.fake = FakeR1ProConnection(deadman_timeout_s=deadman_s)
        modules = {"teleop": self.teleop, "coordinator": self.coordinator, "fake": self.fake}

        self.stream: dict[str, LocalTransport] = {}
        wired: set[tuple[int, str]] = set()
        for name, (producer, consumer) in self._SHARED.items():
            transport = LocalTransport()
            self.stream[name] = transport
            for mod_key, port in (producer, consumer):
                modules[mod_key].set_transport(port, transport)  # type: ignore[arg-type]
                wired.add((id(modules[mod_key]), port))
        for module in modules.values():
            for port in {**module.inputs, **module.outputs}:
                if (id(module), port) not in wired:
                    transport = LocalTransport()
                    module.set_transport(port, transport)  # type: ignore[arg-type]
                    if module is self.fake and port == "cmd_vel":
                        # Nothing in this stack may ever publish here.
                        self.cmd_vel_transport = transport

    def __enter__(self) -> Chain:
        self.teleop.start()
        self.coordinator.start()
        self.fake.start()
        return self

    def __exit__(self, *exc: Any) -> None:
        self.fake.stop()
        self.coordinator.stop()
        self.teleop.stop()

    # Programmatic Quest input (the websocket decoders' own entry points).

    def send_pose(self, hand: str, x: float, y: float, z: float) -> None:
        self.teleop._on_pose_bytes(_pose_bytes(hand, x, y, z))

    def send_joy(self, hand: str, *, trigger: float, grip: float) -> None:
        self.teleop._on_joy_bytes(_joy_bytes(hand, trigger=trigger, grip=grip))

    def hold(self, *, trigger_left: float = 1.0, trigger_right: float = 1.0) -> None:
        """Both controllers present, both grips held (the engage condition).

        Triggers default to 1.0, i.e. a fully-closed command (opening 0), so
        the jaws stay put unless a test drives them.
        """
        for hand in ("left", "right"):
            self.send_pose(hand, 0.3, 0.1 if hand == "left" else -0.1, 1.2)
        self.send_joy("left", trigger=trigger_left, grip=1.0)
        self.send_joy("right", trigger=trigger_right, grip=1.0)

    def release(self) -> None:
        self.send_joy("left", trigger=1.0, grip=0.0)
        self.send_joy("right", trigger=1.0, grip=0.0)

    @property
    def task(self) -> TeleopIKTask:
        task = self.coordinator._tasks[R1PRO_VR_TASK_NAME]
        assert isinstance(task, TeleopIKTask)
        return task

    def body_q(self) -> list[float]:
        with self.fake._lock:
            return list(self.fake._q)

    def gripper_target(self, side: str) -> float | None:
        with self.fake._lock:
            return self.fake._gripper_target[side]

    def gripper_opening(self, side: str) -> float:
        with self.fake._lock:
            return self.fake._gripper_measured[side]


# Assembly / backend switching


def test_fake_backend_is_the_default_and_uses_no_ros() -> None:
    blueprint = make_r1pro_vr_teleop()
    modules = {atom.module for atom in blueprint.blueprints}
    assert FakeR1ProConnection in modules
    assert R1ProConnection not in modules
    # Building the fake stack never pulled in ROS or vendor modules.
    assert "rclpy" not in sys.modules
    assert "hdas_msg" not in sys.modules


def test_registered_blueprint_is_the_fake_backend() -> None:
    from dimos.robot.galaxea.r1pro.blueprints.teleop.r1pro_vr_teleop import r1pro_vr_teleop

    modules = {atom.module for atom in r1pro_vr_teleop.blueprints}
    assert FakeR1ProConnection in modules
    assert R1ProConnection not in modules


def test_real_backend_assembles_r1pro_connection_without_starting_it() -> None:
    """The real path is assembly-checked only: never built, never started."""
    blueprint = make_r1pro_vr_teleop(backend="real", boot_vendor_stack=False)
    modules = {atom.module for atom in blueprint.blueprints}
    assert R1ProConnection in modules
    assert FakeR1ProConnection not in modules


def test_unknown_backend_is_rejected() -> None:
    with pytest.raises(ValueError, match="unknown backend"):
        make_r1pro_vr_teleop(backend="sim")  # type: ignore[arg-type]


def _twist_wiring(blueprint: Any) -> tuple[set[str], set[str]]:
    """Twist stream names after remapping, split into producers and consumers."""
    produced: set[str] = set()
    consumed: set[str] = set()
    for atom in blueprint.active_blueprints:
        for conn in atom.streams:
            if conn.type is not Twist:
                continue
            name = blueprint.remapping_map.get((atom.name, conn.name), conn.name)
            assert isinstance(name, str)
            if conn.direction in ("out", "inout"):
                produced.add(name)
            if conn.direction in ("in", "inout"):
                consumed.add(name)
    return produced, consumed


def test_no_cmd_vel_source_anywhere_in_the_stack() -> None:
    """The stack cannot produce a base command: no producer, no route, no task."""
    for backend in ("fake", "real"):
        blueprint = make_r1pro_vr_teleop(backend=backend, boot_vendor_stack=False)
        produced, consumed = _twist_wiring(blueprint)
        # The connection's cmd_vel input and the coordinator's inherited
        # twist_command port exist (same contract as the real stack), but no
        # module produces either, so neither can ever carry a base command.
        assert "cmd_vel" not in produced, produced
        assert "twist_command" not in produced, produced
        # The viewer websocket's keyboard output is the only possible Twist
        # producer, and it is not routed into the base input of either module.
        assert produced <= {"tele_cmd_vel"}, produced
        assert "tele_cmd_vel" not in consumed, consumed
        assert "cmd_vel" in consumed
        # No explicit bus override either.
        assert {name for (name, msg_type) in blueprint.transport_map if msg_type is Twist} == set()
    # The coordinator declares no <robot>_cmd_vel port.
    coordinator = R1ProVrCoordinator(
        model=R1PRO_COORDINATOR_MODEL,
        command_only_joints=list(R1PRO_GRIPPER_JOINTS),
        tasks=[],
    )
    try:
        assert "r1pro_cmd_vel" not in coordinator.outputs
    finally:
        coordinator.stop()
    # No task claims the chassis axes.
    for task in _vr_tasks():
        assert all("/base/" not in joint for joint in task.joint_names)


def test_ik_model_is_torso_plus_both_arms_grippers_excluded() -> None:
    teleop = next(t for t in _vr_tasks() if t.type == "teleop_ik")
    # All 18 upper-body joints, torso included, in the vendor layout order.
    assert teleop.joint_names == R1PRO_UPPER_BODY_JOINTS
    assert len(teleop.joint_names) == 18
    # The jaws never enter the IK set or any trajectory.
    assert not any("gripper" in joint for joint in teleop.joint_names)
    trajectory = next(t for t in _vr_tasks() if t.type == "trajectory")
    assert not any("gripper" in joint for joint in trajectory.joint_names)
    # One task, both hands, on the URDF's gripper links.
    bindings = teleop.params["bindings"]
    assert {(b["hand"], b["target_frame"]) for b in bindings} == {
        ("left", R1PRO_VR_TARGET_FRAMES["left"]),
        ("right", R1PRO_VR_TARGET_FRAMES["right"]),
    }
    model_config = teleop.params["robot_model"]
    assert set(model_config.joint_names) == set(R1PRO_UPPER_BODY_JOINTS)
    # Both jaws are driven by their own feedback-gated task, each scoped to
    # its own hand's trigger stream by port binding (never the shared
    # broadcast port).
    grippers = [t for t in _vr_tasks() if t.type == "gripper"]
    assert {t.joint_names[0] for t in grippers} == set(R1PRO_GRIPPER_JOINTS)
    for task in grippers:
        assert task.params["require_feedback"] is True
        side = "left" if "left" in task.joint_names[0] else "right"
        assert task.stream_bind == {"gripper_command": f"{side}_gripper_command"}


# The live chain (fake backend, real modules, in-process transports)


def test_no_input_no_motion() -> None:
    """An idle stack never moves the simulated robot: the coordinator holds
    the reported zeros and the jaws are never tasked."""
    with Chain() as chain:
        time.sleep(0.5)
        assert chain.body_q() == [0.0] * 18
        assert chain.gripper_target("left") is None
        assert chain.gripper_target("right") is None
        # The only commands are holds at the measured zeros.
        for msg in chain.stream["position_command"].sent:
            assert all(p == pytest.approx(0.0) for p in msg.position)
        assert chain.cmd_vel_transport.sent == []


def test_quest_input_drives_upper_body_and_feedback() -> None:
    """Grip both hands, move one controller: the IK solution reaches the fake
    and comes back as joint_state feedback."""
    with Chain() as chain:
        chain.hold()
        wait_until(lambda: chain.task.is_active(), timeout=5.0, interval=0.02)
        # Move the left controller +0.15 m in x from its captured reference.
        chain.send_pose("left", 0.45, 0.1, 1.2)
        wait_until(
            lambda: any(abs(q) > 1e-3 for q in chain.body_q()),
            timeout=5.0,
            interval=0.05,
        )
        # The torso is in the same solve, not pinned: the layout is torso 4,
        # left 7, right 7, and moving the controller off-centre loads it.
        assert any(abs(q) > 1e-3 for q in chain.body_q()[:4]), (
            f"torso did not participate in IK: {chain.body_q()[:4]}"
        )
        # Feedback carries the motion back on the contract names.
        wait_until(
            lambda: any(
                len(m.name) == 18 and any(abs(p) > 1e-3 for p in m.position)
                for m in chain.stream["joint_state"].sent
            ),
            timeout=2.0,
            interval=0.02,
        )
        # The chassis was never asked for anything.
        assert chain.stream["position_command"].sent, "coordinator never commanded"
        assert chain.cmd_vel_transport.sent == []
        last = chain.stream["position_command"].sent[-1]
        assert isinstance(last, JointState)
        # Commands carry the 18 upper-body joints; a jaw rides along only
        # while its task drives it.
        assert set(last.name) <= set(R1PRO_UPPER_BODY_JOINTS) | set(R1PRO_GRIPPER_JOINTS)


def test_triggers_drive_each_jaw_independently() -> None:
    with Chain() as chain:
        # The unscoped broadcast port has no route to either jaw.
        assert "gripper_command" not in chain.coordinator._routes
        # Left trigger released (open command), right held (closed command).
        chain.hold(trigger_left=0.0, trigger_right=1.0)
        wait_until(lambda: chain.task.is_active(), timeout=5.0, interval=0.02)
        wait_until(
            lambda: chain.gripper_target("left") == pytest.approx(0.1),
            timeout=5.0,
            interval=0.02,
        )
        wait_until(
            lambda: chain.gripper_opening("left") > 0.05,
            timeout=5.0,
            interval=0.05,
        )
        assert chain.gripper_target("right") is None or chain.gripper_opening(
            "right"
        ) == pytest.approx(0.0)
        assert chain.gripper_opening("right") == pytest.approx(0.0)


def test_release_disengages_and_holds_position() -> None:
    with Chain() as chain:
        chain.hold()
        wait_until(lambda: chain.task.is_active(), timeout=5.0, interval=0.02)
        chain.send_pose("left", 0.45, 0.1, 1.2)
        wait_until(lambda: any(abs(q) > 1e-3 for q in chain.body_q()), timeout=5.0, interval=0.05)
        chain.release()
        wait_until(lambda: not chain.task.is_active(), timeout=5.0, interval=0.02)
        held = chain.body_q()
        time.sleep(0.4)
        assert chain.body_q() == pytest.approx(held, abs=1e-9)


def test_stale_input_disengages_the_session() -> None:
    with Chain(input_timeout_s=0.3) as chain:
        chain.hold()
        wait_until(lambda: chain.task.is_active(), timeout=5.0, interval=0.02)
        # Input stops arriving; the teleop module expires it and publishes the
        # safe (released) button state, which ends the task's session.
        wait_until(lambda: not chain.task.is_active(), timeout=5.0, interval=0.05)


def test_estop_clears_the_session_and_stops_commands() -> None:
    with Chain(deadman_s=0.5) as chain:
        chain.hold()
        wait_until(lambda: chain.task.is_active(), timeout=5.0, interval=0.02)
        chain.send_pose("left", 0.45, 0.1, 1.2)
        wait_until(lambda: any(abs(q) > 1e-3 for q in chain.body_q()), timeout=5.0, interval=0.05)
        assert chain.coordinator.set_estop(True)
        assert not chain.task.is_active()
        sent_at_estop = len(chain.stream["position_command"].sent)
        time.sleep(0.3)
        # While estopped the coordinator sends the connection nothing.
        assert len(chain.stream["position_command"].sent) == sent_at_estop
        # The deadman then halts the stand-in.
        wait_until(lambda: chain.fake.status().deadman_fired, timeout=5.0, interval=0.05)
        # Clearing with the grips released does not resurrect the session or
        # any old intent: the task waits for a fresh grip capture.
        chain.release()
        assert chain.coordinator.set_estop(False)
        time.sleep(0.3)
        assert not chain.task.is_active()
        held = chain.body_q()
        # A fresh engagement re-captures both references, so the controller
        # pose that moved the arm pre-estop produces no jump: zero relative
        # motion until the operator actually moves.
        chain.hold()
        wait_until(lambda: chain.task.is_active(), timeout=5.0, interval=0.02)
        time.sleep(0.3)
        # Tolerance covers the IK command-filter's re-convergence transient
        # (~1e-3 rad); the pre-estop intent would jump orders of magnitude more.
        assert chain.body_q() == pytest.approx(held, abs=1e-2)


def test_gripper_target_latched_during_outage_replays_on_recovery() -> None:
    """EXPOSURE: the framework's current gripper semantics, made visible.

    A jaw whose feedback drops out loses its *emitted* target (never replayed).
    But a target latched *during* the outage was never sent, so it is held and
    goes out automatically on the first tick whose feedback covers the jaw
    again — no new operator input required. This is the coordinator/task's
    documented behavior (see GripperControlTaskConfig.require_feedback); it
    has NOT been independently approved as queueing semantics, and a software
    stop is neither a vendor cancel nor a physical E-STOP.
    """
    with Chain() as chain:
        chain.hold(trigger_left=1.0)  # jaw closed
        wait_until(lambda: chain.task.is_active(), timeout=5.0, interval=0.02)
        # Drive an opening and wait for it to reach the fake.
        chain.send_joy("left", trigger=0.5, grip=1.0)  # opening 0.5 -> 0.05 m
        wait_until(
            lambda: chain.gripper_target("left") == pytest.approx(0.05),
            timeout=5.0,
            interval=0.02,
        )
        # The jaw's feedback drops out; past the reading timeout the
        # coordinator's snapshot no longer covers the jaw.
        chain.fake.set_gripper_feedback_enabled("left", False)
        time.sleep(0.5)
        # A new intent is latched during the outage. It must NOT be sent
        # while the jaw's feedback is missing.
        chain.send_joy("left", trigger=0.2, grip=1.0)  # opening 0.8 -> 0.08 m
        time.sleep(0.3)
        assert chain.gripper_target("left") == pytest.approx(0.05)
        # Feedback returns: the latched intent replays with no new input.
        chain.fake.set_gripper_feedback_enabled("left", True)
        wait_until(
            lambda: chain.gripper_target("left") == pytest.approx(0.08),
            timeout=5.0,
            interval=0.02,
        )

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

The chain under test is the real one: a live ``ArmTeleopModule``
fed programmatic Quest packets (``_on_pose_bytes``/``_on_joy_bytes``, the same
decoders the websocket routes call) → the coordinator's ``teleop_ik``,
gripper and ``vel_chassis`` tasks → the per-robot contract ports →
``FakeR1ProConnection`` → ``joint_state`` feedback (and the simulated
chassis pose) back into the coordinator. No ROS 2, no robot, no vendor
stack, no browser. The actual Quest browser session remains to be verified
by a human on a headset.
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
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.robot.galaxea.r1pro._connection_test_fakes import LocalTransport
from dimos.robot.galaxea.r1pro.blueprints.teleop.r1pro_vr_teleop import (
    R1PRO_VR_CHASSIS_TASK_NAME,
    R1PRO_VR_START_POSE,
    R1PRO_VR_TARGET_FRAMES,
    R1PRO_VR_TASK_NAME,
    R1ProVrCoordinator,
    _vr_tasks,
    make_r1pro_vr_teleop,
)
from dimos.robot.galaxea.r1pro.config import (
    R1PRO_HOME_POSE,
    R1PRO_PLANAR_BASE,
    make_r1pro_model_config,
)
from dimos.robot.galaxea.r1pro.connection import (
    R1PRO_BASE_JOINTS,
    R1PRO_GRIPPER_JOINTS,
    R1PRO_UPPER_BODY_JOINTS,
    R1ProConnection,
)
from dimos.robot.galaxea.r1pro.fake_connection import FakeR1ProConnection, FakeR1ProConnectionConfig
from dimos.robot.galaxea.r1pro.joylo import BASE_FRAME
from dimos.robot.galaxea.r1pro.motion import motion_groups
from dimos.robot.galaxea.r1pro.teleop_coordinator import R1ProTeleopCoordinator, shared_teleop_tasks
from dimos.robot.galaxea.r1pro.torso_height_task import R1ProTorsoHeightTask
from dimos.teleop.joystick import JoystickTeleopMapper
from dimos.teleop.webxr.controller_types import Buttons
from dimos.teleop.webxr.extensions import ArmTeleopModule
from dimos.teleop.webxr.module import WebXRTeleopModule
from dimos.utils.testing.waiting import wait_until


@pytest.fixture(autouse=True)
def no_http_server(mocker):
    # These are decoder/control tests. HTTPS startup has its own module tests.
    mocker.patch.object(WebXRTeleopModule, "_start_server")
    mocker.patch.object(WebXRTeleopModule, "_stop_server")


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


def _joy_bytes(
    hand: str,
    *,
    trigger: float,
    grip: float,
    stick_x: float = 0.0,
    stick_y: float = 0.0,
    enable: bool = False,
) -> bytes:
    # axes: [thumbstick_x, thumbstick_y, trigger_analog, grip_analog]
    return Joy(
        ts=time.time(),
        frame_id=hand,
        axes=[stick_x, stick_y, trigger, grip],
        buttons=[0, 0, 0, int(enable), 0, 0],
    ).lcm_encode()


class Chain:
    """ArmTeleopModule → R1ProVrCoordinator → FakeR1ProConnection.

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
        # The thumbstick Twist: teleop → coordinator's twist bus → the
        # vel_chassis task's arbitration → the coordinator's base port → the
        # connection's cmd_vel.
        "left_joy": (("teleop", "left_joy"), ("joystick", "left_joy")),
        "right_joy": (("teleop", "right_joy"), ("joystick", "right_joy")),
        "lift_rate": (("joystick", "lift_rate"), ("coordinator", "lift_rate")),
        "twist_command": (("joystick", "base_twist"), ("coordinator", "twist_command")),
        "r1pro_velocity_command": (
            ("coordinator", "r1pro_velocity_command"),
            ("fake", "velocity_command"),
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

    def __init__(
        self,
        *,
        input_timeout_s: float = 1.0,
        deadman_s: float = 5.0,
        enable_button: int | None = None,
        coordinator: R1ProVrCoordinator | R1ProTeleopCoordinator | None = None,
    ) -> None:
        #: Full stick deflection is the chassis' base_range, as in the blueprint.
        vx, vy, wz = R1PRO_PLANAR_BASE.velocity_limits
        self.teleop = ArmTeleopModule(
            server_port=_free_port(),
            input_timeout_s=input_timeout_s,
        )
        self.joystick = JoystickTeleopMapper(
            base_linear_speed=min(vx, vy),
            base_angular_speed=wz,
            enable_button=enable_button,
        )
        self.coordinator = coordinator or R1ProVrCoordinator(
            tick_rate=100,
            connections=[FakeR1ProConnectionConfig()],
            tasks=_vr_tasks(),
        )
        self.fake = FakeR1ProConnection(deadman_timeout_s=deadman_s)
        modules = {
            "teleop": self.teleop,
            "joystick": self.joystick,
            "coordinator": self.coordinator,
            "fake": self.fake,
        }

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
                    module.set_transport(port, LocalTransport())  # type: ignore[arg-type]

    def __enter__(self) -> Chain:
        try:
            self.joystick.start()
            self.teleop.start()
            self.coordinator.start()
            self.fake.start()
            wait_until(
                lambda: self.coordinator._connections["r1pro"].ready_for_control(), timeout=5.0
            )
        except BaseException:
            self.__exit__()
            raise
        return self

    def __exit__(self, *exc: Any) -> None:
        self.joystick.stop()
        self.fake.stop()
        self.coordinator.stop()
        self.teleop.stop()

    # Programmatic Quest input (the websocket decoders' own entry points).

    def send_pose(self, hand: str, x: float, y: float, z: float) -> None:
        self.teleop._on_pose_bytes(_pose_bytes(hand, x, y, z))

    def send_joy(self, hand: str, *, trigger: float, grip: float) -> None:
        self.teleop._on_joy_bytes(_joy_bytes(hand, trigger=trigger, grip=grip))

    def send_sticks(
        self,
        *,
        left_x: float = 0.0,
        left_y: float = 0.0,
        right_x: float = 0.0,
        right_y: float = 0.0,
        enable: bool = False,
        left_trigger: float = 1.0,
        right_trigger: float = 1.0,
    ) -> None:
        """Both controllers present with the given sticks (grips released)."""
        self.teleop._on_joy_bytes(
            _joy_bytes("left", trigger=left_trigger, grip=0.0, stick_x=left_x, stick_y=left_y)
        )
        self.teleop._on_joy_bytes(
            _joy_bytes(
                "right",
                trigger=right_trigger,
                grip=0.0,
                stick_x=right_x,
                stick_y=right_y,
                enable=enable,
            )
        )

    def hold(self, *, trigger_left: float = 1.0, trigger_right: float = 1.0) -> None:
        """Both controllers present, both grips held (the IK engage condition).

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

    def base_state(self) -> dict[str, float]:
        return self.fake.base_state()

    def chassis_speeds(self) -> list[tuple[float, float, float]]:
        """Every (vx, vy, wz) the coordinator sent the connection, oldest first."""
        return [tuple(msg.velocity) for msg in self.stream["r1pro_velocity_command"].sent]


class SharedChain(Chain):
    _SHARED = {
        **Chain._SHARED,
        **{
            f"{side}_gripper_command": (
                ("teleop", f"{side}_gripper_command"),
                ("coordinator", f"webxr_{side}_gripper"),
            )
            for side in ("left", "right")
        },
        "twist_command": (("joystick", "base_twist"), ("coordinator", "webxr_twist_command")),
    }


@pytest.fixture
def shared_chain(mocker):
    coordinator = R1ProTeleopCoordinator(
        connections=[FakeR1ProConnectionConfig()],
        tasks=shared_teleop_tasks(dict.fromkeys(R1PRO_UPPER_BODY_JOINTS, 1.0)),
        motion_limits=motion_groups(),
    )
    mocker.patch.object(coordinator, "_leader", mocker.Mock(), create=True)
    with SharedChain(coordinator=coordinator) as chain:
        yield chain


def test_arm_home_reaches_preset_holds_torso_and_requires_explicit_reenable(shared_chain):
    chain = shared_chain
    coordinator = chain.coordinator
    joint = "r1pro/left_arm_joint4"
    home = R1PRO_HOME_POSE["left_arm_joint4"]
    wait_until(lambda: joint in coordinator.get_joint_positions(), timeout=2)
    coordinator._dispatch("leader_joint_command", JointState(name=[joint], position=[home - 0.2]))
    wait_until(lambda: coordinator.get_joint_positions()[joint] < home - 0.15, timeout=3)
    torso = {n: q for n, q in coordinator.get_joint_positions().items() if "torso_joint" in n}

    result = coordinator.home_arms()
    assert result["tracking_paused"] is True
    # Incoming device goals must not replace the recovery trajectory.
    coordinator._dispatch("leader_joint_command", JointState(name=[joint], position=[home - 0.5]))
    wait_until(lambda: coordinator.get_arm_home_status()["state"] == "reached", timeout=5)
    assert coordinator.get_joint_positions()[joint] == pytest.approx(home, abs=0.03)
    assert {n: coordinator.get_joint_positions()[n] for n in torso} == pytest.approx(torso)
    assert coordinator.get_arm_home_status()["tracking_paused"] is True

    # Re-selecting even the SAME input is an explicit new enable after homing.
    coordinator.select_teleop_source("leader")
    coordinator._dispatch("leader_joint_command", JointState(name=[joint], position=[home - 0.2]))
    wait_until(lambda: coordinator.get_joint_positions()[joint] < home - 0.1, timeout=3)
    assert coordinator.get_arm_home_status()["tracking_paused"] is False


def test_cancel_arm_home_does_not_resume_old_teleop(shared_chain):
    coordinator = shared_chain.coordinator
    wait_until(lambda: len(coordinator.get_joint_positions()) >= 18, timeout=2)
    coordinator.home_arms()
    result = coordinator.cancel_arm_home()
    assert result["state"] == "cancelled"
    assert result["tracking_paused"] is True
    assert coordinator._trajectory_task is not None
    assert coordinator._trajectory_task.is_active() is False
    coordinator._dispatch(
        "leader_joint_command", JointState(name=["r1pro/left_arm_joint4"], position=[-1.2])
    )
    assert coordinator._trajectory_task.is_active() is False


def test_arm_home_refuses_missing_feedback_and_estop(shared_chain, mocker):
    coordinator = shared_chain.coordinator
    mocker.patch.object(coordinator, "get_joint_positions", return_value={})
    with pytest.raises(RuntimeError, match="feedback"):
        coordinator.home_arms()
    coordinator.set_estop(True)
    with pytest.raises(RuntimeError, match="emergency-stopped"):
        coordinator.home_arms()
    coordinator._leader.pause.assert_not_called()


def test_shared_stack_handover_clears_intent_and_requires_released_webxr_clutches(shared_chain):
    chain = shared_chain
    coordinator = chain.coordinator
    coordinator._dispatch(
        "leader_joint_command",
        JointState(name=list(R1PRO_UPPER_BODY_JOINTS), position=chain.body_q()),
    )
    coordinator._dispatch("joylo_twist_command", Twist(linear=[0.2, 0, 0]))
    wait_until(lambda: chain.base_state()["vx"] > 0, timeout=2)
    chain.hold()
    coordinator.select_teleop_source("webxr")
    coordinator._leader.pause.assert_called_once_with()
    wait_until(lambda: chain.base_state()["vx"] == 0, timeout=2)
    held = Buttons()
    held.left_grip = held.right_grip = held.right_thumbstick = True
    coordinator._dispatch("teleop_buttons", held)
    coordinator._dispatch("webxr_twist_command", Twist(linear=[0.3, 0, 0]))
    coordinator._dispatch("joylo_twist_command", Twist(linear=[0.2, 0, 0]))
    assert coordinator.get_teleop_source() == {"source": "webxr", "webxr_ready": False}
    assert chain.base_state()["vx"] == 0
    assert not chain.task.is_active()

    chain.release()
    wait_until(lambda: coordinator.get_teleop_source()["webxr_ready"], timeout=2)
    chain.hold(trigger_left=0.0)
    wait_until(lambda: chain.task.is_active(), timeout=2)
    wait_until(lambda: chain.gripper_opening("left") > 0.01, timeout=2)
    # The inactive leader cannot close the jaw or start driving.
    coordinator._dispatch("left_gripper_command", Float32(data=0.0))
    coordinator._dispatch("joylo_twist_command", Twist(linear=[0.2, 0, 0]))
    assert chain.base_state()["vx"] == 0
    assert coordinator._tasks["left_gripper"]._target == [0.1]

    def body_moves():
        chain.send_sticks(left_y=-0.5, right_y=0.5, enable=True)
        return chain.base_state()["vx"] > 0 and abs(chain.body_q()[1]) > 1e-4

    wait_until(body_moves, timeout=3, interval=0.02)
    chain.send_sticks(enable=False)
    wait_until(lambda: chain.base_state()["vx"] == 0, timeout=2)
    assert coordinator.get_teleop_source()["source"] == "webxr"

    coordinator.select_teleop_source("leader")
    assert not chain.task.is_active()
    assert coordinator._tasks["left_gripper"]._target is None
    coordinator._dispatch("webxr_twist_command", Twist(linear=[0.3, 0, 0]))
    assert not coordinator._tasks["vel_chassis"].is_active()
    coordinator._dispatch("left_gripper_command", Float32(data=1.0))
    assert coordinator._tasks["left_gripper"]._target is None
    coordinator._dispatch(
        "leader_joint_command",
        JointState(name=list(R1PRO_UPPER_BODY_JOINTS), position=chain.body_q()),
    )
    coordinator._dispatch("joylo_twist_command", Twist(linear=[0.1, 0, 0]))
    wait_until(lambda: chain.base_state()["vx"] == pytest.approx(0.1), timeout=2)


def test_calibration_hold_blocks_all_tasks_and_release_requires_fresh_clutch(shared_chain):
    chain = shared_chain
    coordinator = chain.coordinator
    coordinator.select_teleop_source("webxr")
    chain.release()
    wait_until(lambda: coordinator.get_teleop_source()["webxr_ready"], timeout=2)
    chain.hold()
    wait_until(lambda: chain.task.is_active(), timeout=2)
    coordinator.hold_for_arm_calibration(True)
    coordinator.set_estop(True)
    coordinator.set_estop(False)
    chain.hold()
    coordinator._dispatch("webxr_twist_command", Twist(linear=[0.3, 0, 0]))
    assert not chain.task.is_active()
    assert not coordinator._tasks["vel_chassis"].is_active()
    assert not coordinator.get_teleop_source()["webxr_ready"]
    coordinator.hold_for_arm_calibration(False)
    chain.hold()
    assert not chain.task.is_active()
    chain.release()
    wait_until(lambda: coordinator.get_teleop_source()["webxr_ready"], timeout=2)
    chain.hold()
    wait_until(lambda: chain.task.is_active(), timeout=2)


def test_shared_stack_webxr_ik_uses_2025_model_and_records_limited_actions(shared_chain):
    chain = shared_chain
    coordinator = chain.coordinator
    coordinator.select_teleop_source("webxr")
    chain.release()
    wait_until(lambda: coordinator.get_teleop_source()["webxr_ready"], timeout=2)
    chain.hold()
    # Wait for the real solver to capture references before moving a controller.
    wait_until(
        lambda: all(hand.robot_reference is not None for hand in chain.task._hands.values()),
        timeout=3,
    )
    chain.send_pose("left", 0.35, 0.1, 1.2)
    wait_until(lambda: any(abs(q) > 1e-4 for q in chain.body_q()[4:11]), timeout=3)
    commands = coordinator.coordinator_joint_command.transport.sent
    assert any(any(abs(p) > 1e-4 for p in msg.position) for msg in commands)
    model = chain.task._teleop_config.robot_model.model.load()
    assert model.get_joint("r1pro/torso_joint2").origin_xyz == (0.0, 0.0, 0.4)
    assert model.get_joint("r1pro/torso_joint3").origin_xyz == (0.0, 0.0001005, 0.3)
    assert coordinator.get_motion_limits()["arm_rate"]["rate"] == 0.75
    chain.release()
    wait_until(lambda: not chain.task.is_active(), timeout=2)
    assert coordinator.get_teleop_source()["source"] == "webxr"


def test_hands_source_takes_the_arms_from_joylo_and_keeps_its_jaws_and_base(shared_chain):
    chain = shared_chain
    coordinator = chain.coordinator
    coordinator.select_teleop_source("hands")
    coordinator._leader.set_arm_source.assert_called_once_with("hands")
    assert coordinator.get_teleop_source()["source"] == "hands"
    held, released = Buttons(), Buttons()
    held.left_grip = held.right_grip = True
    assert make_r1pro_model_config().base_link == BASE_FRAME
    frames = list(R1PRO_VR_TARGET_FRAMES.values())

    def grippers() -> dict[str, PoseStamped]:
        """Where the grippers are, in the robot's base frame."""
        state = JointState(name=list(R1PRO_UPPER_BODY_JOINTS), position=chain.body_q())
        poses: dict[str, PoseStamped] = chain.task._solver.frame_poses(state, frames)
        return poses

    home = grippers()

    def hands(forward: float = 0.0, buttons: Buttons = held) -> None:
        """What JoyLo sends at each tick while it follows the hands: where each gripper goes."""
        coordinator._dispatch("hand_buttons", buttons)
        for stream, frame, dx in zip(
            ("hand_left_pose", "hand_right_pose"), frames, (forward, 0.0), strict=True
        ):
            p, q = home[frame].position, home[frame].orientation
            coordinator._dispatch(
                stream,
                PoseStamped(
                    ts=time.time(),
                    frame_id=BASE_FRAME,
                    position=[p.x + dx, p.y, p.z],
                    orientation=[q.x, q.y, q.z, q.w],
                ),
            )

    # The leader arms' joint targets and the headset module's own inputs are not this source's.
    coordinator._dispatch(
        "leader_joint_command",
        JointState(name=list(R1PRO_UPPER_BODY_JOINTS), position=[0.3] * 18),
    )
    coordinator._dispatch("teleop_buttons", held)
    coordinator._dispatch("left_gripper_command", Float32(data=1.0))
    assert not coordinator._trajectory_task.is_active() and not chain.task.is_active()
    assert coordinator._tasks["left_gripper"]._target is None

    def following() -> bool:
        hands()
        return bool(chain.task.is_active())

    wait_until(following, timeout=3, interval=0.02)
    # Each gripper is told to be where it is: nothing moves, whatever the first pose was.
    for _ in range(10):
        hands()
        time.sleep(0.02)
    for frame in frames:
        assert grippers()[frame].position.to_list() == pytest.approx(
            home[frame].position.to_list(), abs=0.01
        )
    assert chain.body_q()[:4] == pytest.approx([0.0] * 4, abs=1e-6)

    # The left gripper is told to be 5 cm further forward: it goes there, and the right one stays.
    def left_gripper_arrives() -> bool:
        hands(forward=0.05)
        left, right = (grippers()[frame].position for frame in frames)
        return (
            abs(left.x - home[frames[0]].position.x - 0.05) < 0.01
            and abs(right.x - home[frames[1]].position.x) < 0.01
        )

    wait_until(left_gripper_arrives, timeout=5, interval=0.02)
    assert chain.body_q()[:4] == pytest.approx([0.0] * 4, abs=1e-6)
    coordinator._dispatch(
        "leader_joint_command",
        JointState(name=["r1pro/torso_joint3"], position=[-0.05]),
    )
    wait_until(lambda: abs(chain.body_q()[2] + 0.05) < 0.01, timeout=3)
    # JoyLo still closes a jaw and drives the base.
    coordinator._dispatch("left_gripper_command", Float32(data=0.0))
    assert coordinator._tasks["left_gripper"]._target == [0.0]

    def drives() -> bool:
        hands(forward=0.05)
        coordinator._dispatch("joylo_twist_command", Twist(linear=[0.1, 0, 0]))
        return chain.base_state()["vx"] == pytest.approx(0.1)

    wait_until(drives, timeout=3, interval=0.02)
    # JoyLo paused: its deadman ends the session.
    hands(buttons=released)
    wait_until(lambda: not chain.task.is_active(), timeout=2)
    coordinator.select_teleop_source("leader")
    assert coordinator._leader.set_arm_source.call_args.args == ("leader",)
    hands()
    assert not chain.task.is_active()


def test_source_selection_does_not_clear_estop_or_replay_old_leader_packets(shared_chain):
    coordinator = shared_chain.coordinator
    old = JointState(name=list(R1PRO_UPPER_BODY_JOINTS), position=[0.1] * 18)
    coordinator.set_estop(True)
    coordinator.select_teleop_source("webxr")
    assert coordinator._estopped and coordinator._tasks_estopped
    coordinator.select_teleop_source("leader")
    coordinator.set_estop(False)
    coordinator._dispatch("leader_joint_command", old)
    assert not coordinator._trajectory_task.is_active()
    with pytest.raises(ValueError, match="teleop source"):
        coordinator.select_teleop_source("unknown")


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


def test_thumbstick_twist_is_the_only_chassis_command_path(monkeypatch: pytest.MonkeyPatch) -> None:
    """One producer of a base Twist (the teleop sticks), one robot base port.

    The stack's only Twist streams are the coordinator's twist bus and the
    R1 Pro's ``cmd_vel`` port: the thumbstick publishes the former, the
    ``vel_chassis`` task arbitrates it onto the latter, and nothing else can
    inject a base speed.
    """
    from dimos.core.global_config import global_config

    # Match the served viser stack, not an optional rerun bridge.
    monkeypatch.setattr(global_config, "viewer", "none")
    for backend in ("fake", "real"):
        blueprint = make_r1pro_vr_teleop(backend=backend, boot_vendor_stack=False)
        connection_cls = FakeR1ProConnection if backend == "fake" else R1ProConnection
        produced, consumed = _twist_wiring(blueprint)
        assert produced == {"twist_command"}, produced
        assert consumed == {"twist_command"}, consumed
        teleop_atom = next(a for a in blueprint.blueprints if a.module is JoystickTeleopMapper)
        connection_atom = next(a for a in blueprint.blueprints if a.module is connection_cls)
        # The teleop module's sticks produce the bus twist; the connection's
        # cmd_vel input is the per-robot base port, fed by the coordinator.
        assert blueprint.remapping_map[(teleop_atom.name, "base_twist")] == "twist_command"
        assert (
            blueprint.remapping_map[(connection_atom.name, "velocity_command")]
            == "r1pro_velocity_command"
        )
        # No explicit bus override either.
        assert {name for (name, msg_type) in blueprint.transport_map if msg_type is Twist} == set()

    coordinator = R1ProVrCoordinator(
        connections=[FakeR1ProConnectionConfig()],
        tasks=[],
    )
    try:
        assert "r1pro_velocity_command" in coordinator.outputs
        assert "twist_command" in coordinator.inputs
    finally:
        coordinator.stop()

    # Exactly one task drives the chassis, in velocity mode, and its axes are
    # the base port's own; nothing else claims a base axis.
    chassis = [task for task in _vr_tasks() if set(task.joint_names) & set(R1PRO_BASE_JOINTS)]
    assert [task.name for task in chassis] == [R1PRO_VR_CHASSIS_TASK_NAME]
    assert chassis[0].type == "twist_velocity"
    assert chassis[0].joint_names == R1PRO_BASE_JOINTS
    assert chassis[0].params == {
        "axes": ["linear.x", "linear.y", "angular.z"],
        "timeout": 0.2,
    }


def test_blueprint_teleop_scaling_matches_the_base_range() -> None:
    """Full stick deflection is the chassis' limit, so teleop cannot overshoot."""
    vx, vy, wz = R1PRO_PLANAR_BASE.velocity_limits
    for backend in ("fake", "real"):
        blueprint = make_r1pro_vr_teleop(backend=backend, boot_vendor_stack=False)
        atom = next(a for a in blueprint.blueprints if a.module is JoystickTeleopMapper)
        assert atom.kwargs["base_linear_speed"] == pytest.approx(min(vx, vy, 0.3))
        assert atom.kwargs["base_angular_speed"] == pytest.approx(min(wz, 0.5))


def test_ik_steers_only_arms_and_rejects_automatic_torso_motion() -> None:
    teleop = next(t for t in _vr_tasks() if t.type == "teleop_ik")
    arms = [j for j in R1PRO_UPPER_BODY_JOINTS if "_arm_joint" in j]
    assert teleop.joint_names == arms
    with pytest.raises(ValueError, match="explicit controls"):
        make_r1pro_vr_teleop(full_torso=True)
    with pytest.raises(ValueError, match="explicit controls"):
        _vr_tasks(full_torso=True)


def test_model_home_pose_is_the_forward_reaching_start() -> None:
    """One preset: the model's home_joints and the fake's start pose agree."""
    model = make_r1pro_model_config()
    assert model.home_joints is not None
    by_name = dict(zip(model.joint_names, model.home_joints, strict=True))
    for joint, value in R1PRO_VR_START_POSE.items():
        assert by_name[joint] == pytest.approx(value)
    # Arms out front, not the stowed all-zero pose.
    assert any(abs(by_name[j]) > 0.2 for j in by_name if "_arm_joint" in j)
    assert all(by_name[j] == 0.0 for j in by_name if "/torso_joint" in j)


def test_ik_model_is_torso_plus_both_arms_grippers_excluded() -> None:
    teleop = next(t for t in _vr_tasks() if t.type == "teleop_ik")
    # The jaws and the chassis never enter the IK set or any trajectory.
    assert not any("gripper" in joint or "/base/" in joint for joint in teleop.joint_names)
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
        assert task.params == {}
        side = "left" if "left" in task.joint_names[0] else "right"
        assert task.stream_bind == {"gripper_command": f"{side}_gripper_command"}


# The live chain (fake backend, real modules, in-process transports)


def test_no_input_no_motion() -> None:
    """An idle stack never moves the simulated robot: the coordinator holds
    the reported zeros, the jaws are never tasked, and the base stays still."""
    with Chain() as chain:
        time.sleep(0.5)
        assert chain.body_q() == [0.0] * 18
        assert chain.gripper_target("left") is None
        assert chain.gripper_target("right") is None
        # The only commands are holds at the measured zeros.
        for msg in chain.stream["position_command"].sent:
            assert all(p == pytest.approx(0.0) for p in msg.position)
        # No thumbstick has ever deflected, so the base was only ever told zero.
        assert chain.chassis_speeds()
        assert all(speeds == (0.0, 0.0, 0.0) for speeds in chain.chassis_speeds())
        assert chain.base_state()["x"] == pytest.approx(0.0)
        assert chain.base_state()["vx"] == pytest.approx(0.0)


def test_quest_input_drives_upper_body_and_feedback() -> None:
    """Grip both hands, move one controller: the IK solution reaches the fake
    and comes back as joint_state feedback."""
    with Chain() as chain:
        chain.hold()
        wait_until(lambda: chain.task.is_active(), timeout=5.0, interval=0.02)
        # Move the left controller +0.15 m in x from its captured reference.
        chain.send_pose("left", 0.45, 0.1, 1.2)
        wait_until(
            lambda: any(abs(q) > 1e-3 for q in chain.body_q()[4:11]),
            timeout=5.0,
            interval=0.05,
        )
        # Hand motion changes the arms without borrowing any waist joints.
        assert chain.body_q()[:4] == pytest.approx([0.0] * 4, abs=1e-6)
        # Feedback carries the motion back on the contract names.
        wait_until(
            lambda: any(
                len(m.name) == 18 and any(abs(p) > 1e-3 for p in m.position)
                for m in chain.stream["joint_state"].sent
            ),
            timeout=2.0,
            interval=0.02,
        )
        # The chassis was never asked for anything but zero speed.
        assert chain.stream["position_command"].sent, "coordinator never commanded"
        assert all(speeds == (0.0, 0.0, 0.0) for speeds in chain.chassis_speeds())
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

        def settled() -> bool:
            with chain.fake._lock:
                return chain.fake._q == chain.fake._target_q

        # Inactive stops new IK targets; the fake still tracks the last one.
        wait_until(settled, timeout=5.0, interval=0.02)
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
        frames = list(R1PRO_VR_TARGET_FRAMES.values())
        reference = chain.task._solver.frame_poses(
            JointState(name=list(R1PRO_UPPER_BODY_JOINTS), position=held), frames
        )
        # Re-engagement captures the current grippers. The redundant elbows may
        # relax toward their preferred posture, but old hand targets must not replay.
        chain.coordinator.task_invoke("operator_hold", "acknowledge")
        chain.hold()
        wait_until(lambda: chain.task.is_active(), timeout=5.0, interval=0.02)
        first_command = len(chain.stream["position_command"].sent)
        wait_until(
            lambda: len(chain.stream["position_command"].sent) > first_command + 20, timeout=3
        )
        actual = chain.task._solver.frame_poses(
            JointState(name=list(R1PRO_UPPER_BODY_JOINTS), position=chain.body_q()), frames
        )
        for frame in frames:
            assert actual[frame].position.to_list() == pytest.approx(
                reference[frame].position.to_list(), abs=0.01
            )
        assert chain.body_q()[:4] == pytest.approx(held[:4], abs=1e-6)


def test_gripper_outage_is_gated_by_the_connection_without_replay() -> None:
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
        # Recovery alone does not reauthorize the blocked target.
        chain.fake.set_gripper_feedback_enabled("left", True)
        time.sleep(0.15)
        assert chain.gripper_target("left") == pytest.approx(0.05)
        chain.send_joy("left", trigger=0.7, grip=1.0)
        wait_until(
            lambda: chain.gripper_target("left") == pytest.approx(0.03),
            timeout=5.0,
            interval=0.02,
        )


# The thumbstick chassis chain (fake backend, real modules, in-process)


def test_thumbstick_drives_the_chassis_through_the_velocity_task() -> None:
    """Left stick forward → Twist on the bus → vel_chassis → cmd_vel → the fake.

    The base actually moves in the simulated model, so this is the whole
    chain, not a conversion function.
    """
    with Chain() as chain:
        chain.send_sticks(left_y=-1.0)  # pushed fully forward
        wait_until(
            lambda: chain.base_state()["vx"] == pytest.approx(1.0), timeout=5.0, interval=0.02
        )
        wait_until(lambda: chain.base_state()["x"] > 0.05, timeout=5.0, interval=0.02)
        # The twist that drove it came from the teleop module's sticks...
        published = chain.stream["twist_command"].sent
        assert any(twist.linear.x == pytest.approx(1.0) for twist in published)
        # ...was arbitrated by the vel_chassis velocity task...
        chassis_task = chain.coordinator._tasks[R1PRO_VR_CHASSIS_TASK_NAME]
        assert chassis_task.is_active()
        # ...and reached the connection's base port within the base range.
        speeds = chain.chassis_speeds()
        assert max(vx for vx, _, _ in speeds) == pytest.approx(1.0)
        assert all(abs(vy) == 0.0 and abs(wz) == 0.0 for _, vy, wz in speeds)
        # Arms and jaws were never tasked by moving the base.
        assert chain.gripper_target("left") is None
        assert chain.body_q() == [0.0] * 18


def test_right_stick_turns_the_chassis() -> None:
    with Chain() as chain:
        chain.send_sticks(right_x=-1.0)  # pushed fully right → +yaw
        wait_until(
            lambda: chain.base_state()["wz"] == pytest.approx(2.0), timeout=5.0, interval=0.02
        )
        wait_until(lambda: abs(chain.base_state()["yaw"]) > 0.02, timeout=5.0, interval=0.02)
        assert chain.base_state()["vx"] == pytest.approx(0.0)
        assert max(wz for _, _, wz in chain.chassis_speeds()) == pytest.approx(2.0)


def test_thumbstick_outside_physical_range_is_rejected() -> None:
    with Chain() as chain:
        chain.send_sticks(left_y=-5.0, right_x=5.0)
        time.sleep(0.15)
        assert chain.base_state()["vx"] == 0.0
        assert chain.base_state()["wz"] == 0.0
        assert not any(abs(v) > 1.0 for v, _, _ in chain.chassis_speeds())


def test_centred_sticks_send_zero_speed_not_the_last_one() -> None:
    with Chain() as chain:
        chain.send_sticks(left_y=-1.0)
        wait_until(
            lambda: chain.base_state()["vx"] == pytest.approx(1.0), timeout=5.0, interval=0.02
        )
        chain.send_sticks()  # sticks centred, no deflection
        wait_until(
            lambda: chain.base_state()["vx"] == pytest.approx(0.0), timeout=5.0, interval=0.02
        )
        # Zero speed is what went out on the wire, not a held last value.
        assert chain.chassis_speeds()[-1] == (0.0, 0.0, 0.0)
        assert chain.stream["twist_command"].sent[-1].linear.x == pytest.approx(0.0)
        stopped = chain.base_state()["x"]
        time.sleep(0.2)
        assert chain.base_state()["x"] == pytest.approx(stopped, abs=1e-6)


def test_arm_and_jaw_commands_never_reach_the_chassis() -> None:
    with Chain() as chain:
        chain.hold()
        wait_until(lambda: chain.task.is_active(), timeout=5.0, interval=0.02)
        chain.send_pose("left", 0.45, 0.1, 1.2)
        wait_until(lambda: any(abs(q) > 1e-3 for q in chain.body_q()), timeout=5.0, interval=0.05)
        # The arms moved, the sticks never left centre: the base did not.
        assert all(speeds == (0.0, 0.0, 0.0) for speeds in chain.chassis_speeds())
        assert chain.base_state()["x"] == pytest.approx(0.0)
        assert chain.base_state()["vx"] == pytest.approx(0.0)


def test_joy_timeout_zeroes_the_chassis_instead_of_holding_speed() -> None:
    """No new Joy frames → body intent expires before the connection deadman."""
    with Chain(deadman_s=5.0) as chain:
        chain.send_sticks(left_y=-1.0)
        wait_until(
            lambda: chain.base_state()["vx"] == pytest.approx(1.0), timeout=5.0, interval=0.02
        )
        # No further Joy packets: the mapper must not rejuvenate cached input.
        wait_until(
            lambda: chain.base_state()["vx"] == pytest.approx(0.0), timeout=5.0, interval=0.02
        )
        assert chain.chassis_speeds()[-1] == (0.0, 0.0, 0.0)
        # The deadman has not fired: the zero came from the task's timeout.
        assert not chain.fake.status().deadman_fired


def test_position_hold_is_scoped_to_upper_body_joints() -> None:
    with Chain() as chain:
        chain.coordinator.task_invoke("operator_hold", "request", {"route": "manual"})
        hold = chain.coordinator._tasks["operator_hold"]
        wait_until(lambda: bool(hold.claim().joints), timeout=5.0)
        claim = hold.claim()
        assert claim.joints == frozenset(R1PRO_UPPER_BODY_JOINTS)
        assert not set(R1PRO_BASE_JOINTS + R1PRO_GRIPPER_JOINTS) & claim.joints
        # Position hold does not redefine velocity-task authorization.
        chain.send_sticks(left_y=-1.0)
        wait_until(lambda: chain.base_state()["vx"] == 1.0, timeout=5.0)


def test_estop_stops_the_chassis_and_clearing_it_does_not_resume_motion() -> None:
    with Chain(deadman_s=0.5) as chain:
        chain.send_sticks(left_y=-1.0)
        wait_until(
            lambda: chain.base_state()["vx"] == pytest.approx(1.0), timeout=5.0, interval=0.02
        )
        assert chain.coordinator.set_estop(True)
        sent_at_estop = len(chain.chassis_speeds())
        time.sleep(0.3)
        # While estopped the coordinator sends the connection nothing.
        assert len(chain.chassis_speeds()) == sent_at_estop
        wait_until(lambda: chain.fake.status().deadman_fired, timeout=5.0, interval=0.05)
        assert chain.base_state()["vx"] == pytest.approx(0.0)
        # Clearing E-stop alone must not replay the old velocity. Position
        # hold does not inhibit fresh velocity commands after the clear.
        chain.send_sticks()
        assert chain.coordinator.set_estop(False)
        time.sleep(0.1)
        assert chain.base_state()["vx"] == pytest.approx(0.0)
        chain.send_sticks(left_y=-1.0)
        wait_until(lambda: chain.base_state()["vx"] == pytest.approx(1.0), timeout=5.0)


@pytest.mark.parametrize("stop", ["clutch", "timeout", "estop"])
def test_body_sticks_move_base_and_lower_torso_without_ik_joint_overlap(stop: str) -> None:
    with Chain(enable_button=3, deadman_s=0.5) as chain:
        # One actual Joy event enables driving and requests lower height.
        chain.send_sticks(left_y=-0.3, right_y=1.0, enable=True)
        wait_until(
            lambda: chain.body_q()[0] > 0.001 and chain.base_state()["x"] > 0.001,
            timeout=5.0,
            interval=0.01,
        )
        assert chain.body_q()[1] < 0.0
        assert chain.body_q()[2:] == [0.0] * 16
        lift = chain.coordinator._tasks["torso_height"]
        assert isinstance(lift, R1ProTorsoHeightTask)
        assert not lift.claim().joints & chain.task.claim().joints
        if stop == "clutch":
            chain.send_sticks(enable=False)
        elif stop == "estop":
            assert chain.coordinator.set_estop(True)
        # No further input in the timeout case: neither module may replay Joy.
        wait_until(lambda: chain.base_state()["vx"] == 0.0, timeout=5.0)
        wait_until(lambda: lift._target is None, timeout=5.0)

        def settled() -> bool:
            with chain.fake._lock:
                return chain.fake._q == chain.fake._target_q

        wait_until(settled, timeout=5.0)
        if stop == "estop":
            assert chain.coordinator.set_estop(False)
        stopped = chain.body_q()
        time.sleep(0.1)
        assert chain.body_q() == stopped
        assert chain.base_state()["vx"] == 0.0
        assert chain.joystick.config.enable_button == 3

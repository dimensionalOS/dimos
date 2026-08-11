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

"""Deterministic tests for the R1 Lite quest teleop module and blueprints.

No headset, no ROS, no threads: the module is constructed with rpc
disabled and the event loop factory stubbed, and Out streams are replaced
per test.
"""

from __future__ import annotations

import base64
import json
from pathlib import Path
import sys
import time
import types
from typing import Any
import xml.etree.ElementTree as ET

import numpy as np
import pinocchio
import pytest

from dimos.control.coordinator import ControlCoordinator
import dimos.core.module as module_mod
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.planning.kinematics.pinocchio_ik import PinocchioIK
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Joy import Joy
from dimos.protocol.rpc.spec import RPCSpec
from dimos.robot.galaxea.r1lite import config as cfg
from dimos.robot.galaxea.r1lite.blueprints.basic.r1lite_quest_teleop import (
    r1lite_quest_teleop,
    r1lite_quest_teleop_sim,
)
from dimos.robot.galaxea.r1lite.quest_module import R1LiteQuestTeleopModule
from dimos.teleop.quest.quest_types import Hand, QuestControllerState, ThumbstickState

_FIXTURE = Path(__file__).parent / "testdata" / "quest_replay_fixture.jsonl"


class _NoRpc(RPCSpec):
    """Module.__init__ treats a ValueError from the rpc factory as rpc disabled."""

    def __init__(self, **kw: Any) -> None:
        raise ValueError("rpc disabled for unit tests")


@pytest.fixture(autouse=True)
def _no_background_threads(monkeypatch: Any) -> None:
    monkeypatch.setattr(module_mod, "get_loop", lambda: (types.SimpleNamespace(), None))


class _FakeOut:
    def __init__(self) -> None:
        self.msgs: list[Any] = []

    def publish(self, msg: Any) -> None:
        self.msgs.append(msg)


def _module(**config_kwargs: Any) -> R1LiteQuestTeleopModule:
    m = R1LiteQuestTeleopModule(rpc_transport=_NoRpc, **config_kwargs)
    for stream in (
        "cmd_vel",
        "gripper_left_command",
        "gripper_right_command",
        "teleop_buttons",
        "left_controller_output",
        "right_controller_output",
    ):
        setattr(m, stream, _FakeOut())
    return m


def _controller(
    *,
    is_left: bool = True,
    stick_x: float = 0.0,
    stick_y: float = 0.0,
    stick_press: bool = False,
    trigger: float = 0.0,
    primary: bool = False,
) -> QuestControllerState:
    return QuestControllerState(
        is_left=is_left,
        trigger=trigger,
        primary=primary,
        thumbstick_press=stick_press,
        thumbstick=ThumbstickState(x=stick_x, y=stick_y),
    )


def _mark_fresh(m: R1LiteQuestTeleopModule, *hands: Hand) -> float:
    now = time.monotonic()
    for hand in hands:
        m._joy_rx_ts[hand] = now
    return now


def _coordinator_tasks(blueprint: Any) -> list[Any]:
    kwargs = next(atom.kwargs for atom in blueprint.blueprints if atom.module is ControlCoordinator)
    return list(kwargs["tasks"])


# Chassis and grippers


def test_left_stick_maps_to_linear_velocity() -> None:
    m = _module()
    now = _mark_fresh(m, Hand.LEFT)
    twist = m._chassis_twist(_controller(stick_y=-1.0, stick_x=0.5), None, now)
    assert twist.linear.x == pytest.approx(0.2)
    assert twist.linear.y == pytest.approx(-0.1)
    assert twist.angular.z == 0.0


def test_stale_controller_contributes_zero() -> None:
    m = _module()
    now = _mark_fresh(m, Hand.RIGHT)
    m._joy_rx_ts[Hand.LEFT] = now - 10.0
    twist = m._chassis_twist(
        _controller(stick_y=-1.0),
        _controller(is_left=False, stick_x=1.0),
        now,
    )
    assert twist.linear.x == 0.0
    assert twist.angular.z == pytest.approx(-0.4)


def test_any_thumbstick_press_zeros_twist() -> None:
    m = _module()
    now = _mark_fresh(m, Hand.LEFT, Hand.RIGHT)
    twist = m._chassis_twist(
        _controller(stick_y=-1.0),
        _controller(is_left=False, stick_x=1.0, stick_press=True),
        now,
    )
    assert (twist.linear.x, twist.linear.y, twist.angular.z) == (0.0, 0.0, 0.0)


def test_gripper_command_maps_trigger_to_percent() -> None:
    m = _module()
    assert m._gripper_command(0.0).position[0] == pytest.approx(100.0)
    assert m._gripper_command(1.0).position[0] == pytest.approx(0.0)


def test_gripper_streams_only_while_engaged_and_fresh() -> None:
    m = _module()
    left = _controller(trigger=1.0, primary=True)
    _mark_fresh(m, Hand.LEFT)
    m._publish_button_state(left, None)
    assert m.gripper_left_command.msgs == []
    m._is_engaged[Hand.LEFT] = True
    m._publish_button_state(left, None)
    assert len(m.gripper_left_command.msgs) == 1
    m._joy_rx_ts[Hand.LEFT] = time.monotonic() - 10.0
    m._publish_button_state(left, None)
    assert len(m.gripper_left_command.msgs) == 1


def test_publish_tick_always_streams_twist() -> None:
    m = _module()
    m._publish_button_state(None, None)
    assert len(m.cmd_vel.msgs) == 1
    assert len(m.teleop_buttons.msgs) == 1


# Hand-delta mapping


def test_position_deadband_zeroes_small_deltas() -> None:
    m = _module(position_deadband_m=0.02)
    m._is_engaged[Hand.LEFT] = True
    m._initial_poses[Hand.LEFT] = PoseStamped()
    m._current_poses[Hand.LEFT] = PoseStamped(position=[0.01, 0.01, 0.0])
    out = m._get_output_pose(Hand.LEFT)
    assert (out.position.x, out.position.y, out.position.z) == (0.0, 0.0, 0.0)


def test_position_deadband_is_soft_and_applies_before_gain() -> None:
    m = _module(position_deadband_m=0.02, motion_gain=2.0)
    m._is_engaged[Hand.LEFT] = True
    m._initial_poses[Hand.LEFT] = PoseStamped()
    m._current_poses[Hand.LEFT] = PoseStamped(position=[0.05, 0.0, 0.0])
    out = m._get_output_pose(Hand.LEFT)
    assert out.position.x == pytest.approx((0.05 - 0.02) * 2.0)
    assert out.position.y == 0.0


def test_motion_gain_scales_position_delta_only() -> None:
    m = _module(motion_gain=1.3)
    m._is_engaged[Hand.LEFT] = True
    m._initial_poses[Hand.LEFT] = PoseStamped()
    m._current_poses[Hand.LEFT] = PoseStamped(position=[0.10, 0.0, 0.20])
    out = m._get_output_pose(Hand.LEFT)
    assert out.position.x == pytest.approx(0.13)
    assert out.position.z == pytest.approx(0.26)


def test_local_rotation_uses_hand_frame_delta() -> None:
    m = _module(local_rotation=True)
    m._is_engaged[Hand.LEFT] = True
    initial = PoseStamped(orientation=Quaternion.from_euler(Vector3(0.0, 0.0, 1.0)))
    current = PoseStamped(
        orientation=initial.orientation * Quaternion.from_euler(Vector3(0.5, 0.0, 0.0))
    )
    m._initial_poses[Hand.LEFT] = initial
    m._current_poses[Hand.LEFT] = current
    out = m._get_output_pose(Hand.LEFT)
    expected = initial.orientation.inverse() * current.orientation
    got = out.orientation
    dot = sum(
        a * b
        for a, b in zip(
            (got.x, got.y, got.z, got.w),
            (expected.x, expected.y, expected.z, expected.w),
            strict=False,
        )
    )
    assert abs(abs(dot) - 1.0) < 1e-6


# Blueprint contracts


def test_teleop_tasks_use_arm_slices_and_pink() -> None:
    for blueprint in (r1lite_quest_teleop, r1lite_quest_teleop_sim):
        tasks = {t.name: t for t in _coordinator_tasks(blueprint)}
        left = tasks["teleop_left_arm"]
        right = tasks["teleop_right_arm"]
        assert left.type == right.type == "teleop_ik"
        assert left.priority == right.priority == 20
        assert left.joint_names == cfg.LEFT_ARM_JOINTS
        assert right.joint_names == cfg.RIGHT_ARM_JOINTS
        for task, urdf_names in (
            (left, cfg.LEFT_ARM_URDF_JOINTS),
            (right, cfg.RIGHT_ARM_URDF_JOINTS),
        ):
            control_ik = task.params["control_ik"]
            robot_model = control_ik["robot_model"]
            assert robot_model.joint_names == urdf_names
            assert robot_model.get_coordinator_joint_names() == task.joint_names
            # The controlled point is the grasp-center URDF frame
            # (formerly the task-level tool_offset_m).
            assert robot_model.end_effector_link.endswith("_arm_grasp_center")
            assert control_ik["orientation_cost"] == 0.5
            assert control_ik["joint_centering_cost"] == 1e-2
            assert control_ik["max_velocity"] == 1.1
            # Folded boot pose: measured up to 1.74 deg below the vendor
            # URDF lower bound; the tolerance must cover it.
            assert control_ik["seed_limit_tolerance"] >= 0.04
            assert task.params["timeout"] == 1.5


def test_sim_planner_models_have_matching_trajectory_tasks() -> None:
    tasks = {task.name: task for task in _coordinator_tasks(r1lite_quest_teleop_sim)}
    planner_kwargs = next(
        atom.kwargs
        for atom in r1lite_quest_teleop_sim.blueprints
        if atom.module is ManipulationModule
    )

    for robot in planner_kwargs["robots"]:
        task_name = robot.coordinator_task_name
        assert task_name is not None
        task = tasks[task_name]
        assert task.type == "trajectory"
        assert task.joint_names == robot.get_coordinator_joint_names()


def test_sim_planner_models_render_grippers_and_target_their_eef() -> None:
    planner_kwargs = next(
        atom.kwargs
        for atom in r1lite_quest_teleop_sim.blueprints
        if atom.module is ManipulationModule
    )

    for robot in planner_kwargs["robots"]:
        assert robot.model_path == cfg.R1LITE_VISER_ARM_MODEL
        assert robot.end_effector_link == "r1lite_gripper_tip"


def test_viser_eef_is_five_centimeters_forward_of_gripper_eef() -> None:
    root = ET.parse(cfg.R1LITE_VISER_ARM_MODEL).getroot()
    joint = root.find("./joint[@name='r1lite_gripper_tip_joint']")

    assert joint is not None
    assert joint.find("parent").attrib["link"] == "gripper_eef_link"
    assert joint.find("origin").attrib["xyz"] == "0.05 0 0"


def test_module_rotation_pairing_and_no_default_recording() -> None:
    for blueprint in (r1lite_quest_teleop, r1lite_quest_teleop_sim):
        kwargs = next(
            atom.kwargs for atom in blueprint.blueprints if atom.module is R1LiteQuestTeleopModule
        )
        # World-frame deltas: the merged task applies rotations
        # world-frame, so the module must publish them world-frame too.
        assert kwargs["local_rotation"] is False
        # Recording is opt-in per session, never a blueprint default.
        assert kwargs.get("record_path", "") == ""  # recording stays opt-in


def test_hardware_blueprint_teleop_overrides() -> None:
    from dimos.robot.galaxea.r1lite.connection import R1LiteConnection

    kwargs = next(
        atom.kwargs for atom in r1lite_quest_teleop.blueprints if atom.module is R1LiteConnection
    )
    assert kwargs["tracking_speed"] == 3.5
    assert kwargs["enable_cameras"] is False


# Shipped arm models


@pytest.mark.parametrize("model", [cfg.R1LITE_LEFT_ARM_MODEL, cfg.R1LITE_RIGHT_ARM_MODEL])
def test_arm_model_is_the_a1x_chain(model: Path) -> None:
    # Pin the vendor-derived chain: 6 revolute joints, the A1X axis
    # pattern, the 300 mm forearm, and the vendor limit convention.
    root = ET.parse(model).getroot()
    joints = [j for j in root.findall("joint") if j.get("type") == "revolute"]
    assert len(joints) == 6
    axes = [j.find("axis").get("xyz") for j in joints]
    assert axes == ["0 0 1", "0 1 0", "0 1 0", "0 1 0", "0 0 1", "1 0 0"]
    j3_x = float(joints[2].find("origin").get("xyz").split()[0])
    assert j3_x == pytest.approx(-0.3)
    limit1 = joints[0].find("limit")
    assert float(limit1.get("upper")) == pytest.approx(2.8623467075)


@pytest.mark.parametrize("model", [cfg.R1LITE_LEFT_ARM_MODEL, cfg.R1LITE_RIGHT_ARM_MODEL])
def test_arm_model_header_names_provenance(model: Path) -> None:
    text = model.read_text()
    assert "userguide-galaxea/URDF" in text
    assert "2e5d31e1784481a34d178006c0d0e18e0a84a82a" in text


# Offline chase characterization (the wedge class). This models the chase
# algorithm on a DLS solver to characterize the wedge failure mode; the
# production-path evidence is test_replay_fixture_drives_production_pipeline.


def test_chase_algorithm_characterization_folded_home_teleport() -> None:
    # From the folded home, small cartesian targets need large joint
    # motion, and pose-stream gaps teleport the target. The chase window
    # recentered on the EE, bounded steps, and the 45 degree backstop must
    # reach a far target with zero rejections.
    ik = PinocchioIK.from_model_path(cfg.R1LITE_LEFT_ARM_MODEL, ee_joint_id=6)
    q0 = np.zeros(6)
    ee0 = ik.forward_kinematics(q0)
    target = pinocchio.SE3(ee0.rotation, ee0.translation + np.array([0.25, 0.05, 0.15]))

    hard = np.deg2rad(45.0)
    step = np.deg2rad(1.5)
    win_t, win_r = 0.08, np.deg2rad(20.0)

    def windowed(q: Any, scale: float) -> pinocchio.SE3:
        ee = ik.forward_kinematics(q)
        off = target.translation - ee.translation
        dist = np.linalg.norm(off)
        max_t = win_t * scale
        pos = target.translation if dist <= max_t else ee.translation + off * (max_t / dist)
        w = pinocchio.log3(ee.rotation.T @ target.rotation)
        angle = np.linalg.norm(w)
        max_r = win_r * scale
        rot = (
            target.rotation if angle <= max_r else ee.rotation @ pinocchio.exp3(w * (max_r / angle))
        )
        return pinocchio.SE3(rot, pos)

    q = q0
    ticks = 0
    while np.linalg.norm(ik.forward_kinematics(q).translation - target.translation) >= 0.005:
        ticks += 1
        assert ticks <= 600, "teleported target not reached in 600 ticks"
        # Models production's ordering: full window first, quartered backoff
        # if the solution trips the branch-flip gate. Zero final rejections.
        accepted = None
        for scale in (1.0, 0.25):
            q_sol, _, _ = ik.solve(windowed(q, scale), q)
            if np.max(np.abs(q_sol - q)) < hard:
                accepted = q_sol
                break
        assert accepted is not None, "rejection at the smallest window"
        q = q + np.clip(accepted - q, -step, step)


# Recorder and replay fixture


def test_recorder_writes_replayable_frames(tmp_path: Path) -> None:
    record = tmp_path / "session.jsonl"
    m = _module(record_path=str(record))
    frames = [b"\x01\x02fingerprint-a", b"\x03\x04fingerprint-b"]
    for frame in frames:
        m._record_frame(frame)
    m._close_recorder()
    entries = [json.loads(line) for line in record.read_text().splitlines()]
    assert [base64.b64decode(e["data"]) for e in entries] == frames
    assert entries[1]["t"] >= entries[0]["t"]


def test_recorder_off_by_default() -> None:
    m = _module()
    m._record_frame(b"frame")
    m._close_recorder()
    assert m._record_file is None
    assert m._record_count == 0


def test_replay_fixture_is_sanitized_and_loadable() -> None:
    scripts = str(Path(__file__).resolve().parents[4] / "scripts" / "r1lite_test")
    sys.path.insert(0, scripts)
    try:
        from replay_quest_stream import load_frames
    finally:
        sys.path.remove(scripts)
    frames = load_frames(_FIXTURE, 0.0, float("inf"))
    assert len(frames) == 90
    assert frames[0][0] == 0.0
    engage_count = 0
    prev = 0
    for _, data in frames:
        msg: Any
        try:
            msg = Joy.lcm_decode(data)
        except Exception:
            msg = PoseStamped.lcm_decode(data)
            continue
        primary = msg.buttons[4] if len(msg.buttons) > 4 else 0
        if primary and not prev:
            engage_count += 1
        prev = primary
    assert engage_count == 1


# Command bounding and validation (module layer)


def test_unknown_joy_frame_id_rejected() -> None:
    m = _module()
    raw = Joy(frame_id="middle", axes=[0.0] * 4, buttons=[0] * 7).lcm_encode()
    m._on_joy_bytes(raw)
    assert m._controllers[Hand.LEFT] is None
    assert m._controllers[Hand.RIGHT] is None


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), -float("inf")])
def test_non_finite_stick_contributes_zero(bad: float) -> None:
    m = _module()
    now = _mark_fresh(m, Hand.LEFT)
    twist = m._chassis_twist(_controller(stick_y=bad, stick_x=bad), None, now)
    assert (twist.linear.x, twist.linear.y) == (0.0, 0.0)


def test_extreme_stick_clamps_to_configured_speed() -> None:
    m = _module()
    now = _mark_fresh(m, Hand.LEFT, Hand.RIGHT)
    twist = m._chassis_twist(
        _controller(stick_y=-100.0),
        _controller(is_left=False, stick_x=100.0),
        now,
    )
    assert twist.linear.x == pytest.approx(m.config.linear_speed)
    assert twist.angular.z == pytest.approx(-m.config.angular_speed)


# Fail-closed engagement (stream loss)


def _feed_fresh(m: R1LiteQuestTeleopModule, hand: Hand, primary: bool) -> None:
    now = time.monotonic()
    m._controllers[hand] = _controller(is_left=(hand is Hand.LEFT), primary=primary)
    m._joy_rx_ts[hand] = now
    m._pose_rx_ts[hand] = now
    m._current_poses[hand] = PoseStamped()


def test_stale_joy_while_engaged_forces_release() -> None:
    m = _module()
    _feed_fresh(m, Hand.LEFT, primary=True)
    with m._lock:
        m._handle_engage()
    assert m._is_engaged[Hand.LEFT]
    m._joy_rx_ts[Hand.LEFT] -= 10.0
    with m._lock:
        m._handle_engage()
    assert not m._is_engaged[Hand.LEFT]
    assert m._controllers[Hand.LEFT] is None
    assert m._require_release[Hand.LEFT]


def test_stale_pose_stream_alone_forces_release() -> None:
    m = _module()
    _feed_fresh(m, Hand.LEFT, primary=True)
    with m._lock:
        m._handle_engage()
    assert m._is_engaged[Hand.LEFT]
    m._pose_rx_ts[Hand.LEFT] -= 10.0
    with m._lock:
        m._handle_engage()
    assert not m._is_engaged[Hand.LEFT]


def test_recovery_with_button_still_held_stays_released() -> None:
    m = _module()
    _feed_fresh(m, Hand.LEFT, primary=True)
    with m._lock:
        m._handle_engage()
    m._joy_rx_ts[Hand.LEFT] -= 10.0
    with m._lock:
        m._handle_engage()
    # Stream recovers with the primary button STILL held: must not re-engage.
    _feed_fresh(m, Hand.LEFT, primary=True)
    with m._lock:
        m._handle_engage()
    assert not m._is_engaged[Hand.LEFT]
    # Observed release, then a fresh press: engagement works again.
    _feed_fresh(m, Hand.LEFT, primary=False)
    with m._lock:
        m._handle_engage()
    _feed_fresh(m, Hand.LEFT, primary=True)
    with m._lock:
        m._handle_engage()
    assert m._is_engaged[Hand.LEFT]


# Full vendor-chain pinning

# Every rpy is zero: the A1X chain encodes orientation entirely through
# joint axes and translations, in the vendor publication and the on-robot
# capture alike. A nonzero rpy appearing is a chain change and must fail.
_EXPECTED_CHAIN = {
    "left": [
        ("left_arm_joint1", "0 0 0.08605", "0 0 0", "0 0 1", -2.86234670748, 2.86234670748),
        ("left_arm_joint2", "0 0.03075 0.04925", "0 0 0", "0 1 0", 0.01745329252, 3.12414670748),
        ("left_arm_joint3", "-0.3 0.00025004 0", "0 0 0", "0 1 0", -3.29864670748, -0.01745329252),
        (
            "left_arm_joint4",
            "0.1747 0.00049739 0.075485",
            "0 0 0",
            "0 1 0",
            -1.55334670748,
            1.55334670748,
        ),
        (
            "left_arm_joint5",
            "0.08 -0.031498 0.0405",
            "0 0 0",
            "0 0 1",
            -1.55334670748,
            1.55334670748,
        ),
        ("left_arm_joint6", "0.022503 0 -0.0405", "0 0 0", "1 0 0", -2.86234670748, 2.86234670748),
    ],
    "right": [
        ("right_arm_joint1", "0 0 0.08605", "0 0 0", "0 0 1", -2.86234670748, 2.86234670748),
        ("right_arm_joint2", "0 0.03075 0.04925", "0 0 0", "0 1 0", 0.01745329252, 3.12414670748),
        ("right_arm_joint3", "-0.3 0.00025004 0", "0 0 0", "0 1 0", -3.29864670748, -0.01745329252),
        (
            "right_arm_joint4",
            "0.1747 0.00049739 0.075485",
            "0 0 0",
            "0 1 0",
            -1.55334670748,
            1.55334670748,
        ),
        (
            "right_arm_joint5",
            "0.08 -0.031498 0.0405",
            "0 0 0",
            "0 0 1",
            -1.55334670748,
            1.55334670748,
        ),
        ("right_arm_joint6", "0.022503 0 -0.0405", "0 0 0", "1 0 0", -2.86234670748, 2.86234670748),
    ],
}


@pytest.mark.parametrize("side", ["left", "right"])
def test_every_joint_origin_axis_and_limit_is_pinned(side: str) -> None:
    # The complete deterministic chain comparison: every origin xyz and rpy,
    # axis, and limit of both shipped models, pinned to the values verified
    # against the vendor publication and the on-robot capture.
    model = cfg.R1LITE_LEFT_ARM_MODEL if side == "left" else cfg.R1LITE_RIGHT_ARM_MODEL
    root = ET.parse(model).getroot()
    actual = []
    for j in root.findall("joint"):
        if j.get("type") != "revolute":
            continue
        lim = j.find("limit")
        origin = j.find("origin")
        actual.append(
            (
                j.get("name"),
                origin.get("xyz"),
                origin.get("rpy", "0 0 0"),
                j.find("axis").get("xyz"),
                float(lim.get("lower")),
                float(lim.get("upper")),
            )
        )
    expected = _EXPECTED_CHAIN[side]
    assert len(actual) == len(expected) == 6
    for got, want in zip(actual, expected, strict=True):
        assert got[0] == want[0]
        assert got[1] == want[1]
        assert got[2] == want[2]
        assert got[3] == want[3]
        assert got[4] == pytest.approx(want[4])
        assert got[5] == pytest.approx(want[5])


# Production replay: fixture frames through the module into production tasks


class _FakeJoints:
    def __init__(self, positions: dict[str, float]) -> None:
        self._positions = positions

    def get_position(self, name: str) -> float | None:
        return self._positions.get(name)


def _task_limits(task: Any) -> tuple[Any, Any]:
    """Controlled-joint position bounds from the solver's runtime model."""
    rt = task._ik._runtime
    lower = np.array([rt.model.lowerPositionLimit[i] for i in rt.mapping.q_indices])
    upper = np.array([rt.model.upperPositionLimit[i] for i in rt.mapping.q_indices])
    return lower, upper


def test_replay_fixture_drives_production_pipeline() -> None:
    from dimos_lcm.geometry_msgs import PoseStamped as LCMPoseStamped
    from dimos_lcm.sensor_msgs import Joy as LCMJoy

    from dimos.control.tasks.teleop_task.teleop_task import create_task
    from dimos.robot.galaxea.r1lite.blueprints.basic.r1lite_quest_teleop import _teleop_tasks

    left_cfg = next(t for t in _teleop_tasks() if t.name == "teleop_left_arm")
    task = create_task(left_cfg, None)
    task.start()

    m = _module(
        task_names={"left": "teleop_left_arm", "right": "teleop_right_arm"},
        local_rotation=True,
        position_deadband_m=0.02,
        motion_gain=1.3,
    )
    fp_pose = LCMPoseStamped._get_packed_fingerprint()
    fp_joy = LCMJoy._get_packed_fingerprint()

    # The merged solver owns limits and rate: per-tick deltas are bounded
    # by the velocity clamp, and positions stay inside the model bounds.
    step_limit = task._config.control_ik.max_velocity * 0.02 + 1e-9
    lower, upper = _task_limits(task)
    q_start = (lower + upper) / 2.0
    positions = dict(zip(cfg.LEFT_ARM_JOINTS, q_start.tolist(), strict=True))
    tool_start = task._ik.forward_kinematics(q_start)

    engaged_seen = False
    commands = 0
    t_now = 0.0
    prev_q = q_start
    frames = [json.loads(line) for line in _FIXTURE.read_text().splitlines()]
    frames.append({"t": 0.0, "wall": 0.0, "data": base64.b64encode(b"garbagegarbage").decode()})
    for frame in frames:
        data = base64.b64decode(frame["data"])
        fingerprint = data[:8]
        if fingerprint == fp_joy:
            m._on_joy_bytes(data)
        elif fingerprint == fp_pose:
            m._on_pose_bytes(data)
        else:
            continue  # unknown fingerprint: dropped, exactly like the server

        t_now += 0.02
        with m._lock:
            m._handle_engage()
            if m._should_publish(Hand.LEFT):
                pose = m._get_output_pose(Hand.LEFT)
                if pose is not None:
                    m._publish_msg(Hand.LEFT, pose)
            m._publish_button_state(m._controllers.get(Hand.LEFT), m._controllers.get(Hand.RIGHT))
        task.on_teleop_buttons(m.teleop_buttons.msgs[-1], t_now)
        for routed in m.left_controller_output.msgs:
            assert routed.frame_id == "teleop_left_arm"
            task.on_cartesian_command(routed, t_now)
        m.left_controller_output.msgs.clear()

        state = types.SimpleNamespace(t_now=t_now, dt=0.02, joints=_FakeJoints(positions))
        out = task.compute(state)
        if out is None:
            continue
        commands += 1
        engaged_seen = True
        assert out.joint_names == cfg.LEFT_ARM_JOINTS
        q_new = np.array(out.positions)
        assert np.all(np.isfinite(q_new))
        assert np.all(q_new >= lower - 1e-9)
        assert np.all(q_new <= upper + 1e-9)
        assert np.max(np.abs(q_new - prev_q)) <= step_limit
        prev_q = q_new
        positions = dict(zip(cfg.LEFT_ARM_JOINTS, out.positions, strict=True))

    assert engaged_seen
    assert commands > 10
    # The fixture moves the hand; the production solver moved the tool.
    tool_end = task._ik.forward_kinematics(prev_q)
    assert float(np.linalg.norm(tool_end.translation - tool_start.translation)) > 0.005
    # After the release frames at the fixture tail, the task is inert.
    state = types.SimpleNamespace(t_now=t_now + 0.02, dt=0.02, joints=_FakeJoints(positions))
    assert task.compute(state) is None


def test_folded_boot_pose_seed_solves() -> None:
    # The robot boots with joint 2 measured up to 1.74 deg BELOW the
    # vendor URDF lower bound (three hardware sessions, 2026-07-28). The
    # old solver hard-failed and the arm froze until dragged past the
    # limit; the blueprint's seed_limit_tolerance must cover this depth
    # so the first engage of every session works.
    from dimos.control.tasks.teleop_task.teleop_task import create_task
    from dimos.robot.galaxea.r1lite.blueprints.basic.r1lite_quest_teleop import _teleop_tasks
    from dimos.teleop.quest.quest_types import Buttons

    left_cfg = next(t for t in _teleop_tasks() if t.name == "teleop_left_arm")
    task = create_task(left_cfg, None)
    task.start()
    lower, _upper = _task_limits(task)
    q_boot = lower.copy()
    q_boot[1] -= np.deg2rad(1.74)
    positions = dict(zip(left_cfg.joint_names, q_boot.tolist(), strict=True))

    engage = Buttons()
    engage.left_primary = True
    assert task.on_teleop_buttons(engage, 1.0)
    pose = PoseStamped(position=[0.02, 0.0, 0.02], frame_id="teleop_left_arm")
    assert task.on_cartesian_command(pose, t_now=1.0)
    state = types.SimpleNamespace(t_now=1.02, dt=0.01, joints=_FakeJoints(positions))
    out = task.compute(state)
    assert out is not None, "arm frozen at folded boot pose (seed tolerance regression)"
    q_new = np.array(out.positions)
    assert np.all(q_new >= lower - 1e-9)
    assert np.max(np.abs(q_new - q_boot)) <= np.deg2rad(5.0)


def test_engaged_task_times_out_when_stream_stops() -> None:
    from dimos.control.tasks.teleop_task.teleop_task import create_task
    from dimos.robot.galaxea.r1lite.blueprints.basic.r1lite_quest_teleop import _teleop_tasks
    from dimos.teleop.quest.quest_types import Buttons

    left_cfg = next(t for t in _teleop_tasks() if t.name == "teleop_left_arm")
    task = create_task(left_cfg, None)
    task.start()
    # The merged task only accepts pose deltas while its engage button is
    # held (left hand -> left_primary).
    engage = Buttons()
    engage.left_primary = True
    assert task.on_teleop_buttons(engage, 1.0)
    positions = {name: 0.1 for name in cfg.LEFT_ARM_JOINTS}
    pose = PoseStamped(position=[0.01, 0.0, 0.0], frame_id="teleop_left_arm")
    assert task.on_cartesian_command(pose, t_now=1.0)
    state = types.SimpleNamespace(t_now=1.02, dt=0.02, joints=_FakeJoints(positions))
    assert task.compute(state) is not None
    # No further commands: past the configured timeout the task goes inert.
    late = types.SimpleNamespace(t_now=1.02 + 2.0, dt=0.02, joints=_FakeJoints(positions))
    assert task.compute(late) is None


# Malformed Quest input (non-finite and invalid values at ingress)


def _joy_frame(
    frame_id: str,
    *,
    stick_x: float = 0.0,
    stick_y: float = 0.0,
    trigger: float = 0.0,
    grip: float = 0.0,
    primary: bool = False,
) -> bytes:
    return Joy(
        ts=1.0,
        frame_id=frame_id,
        axes=[stick_x, stick_y, trigger, grip],
        buttons=[0, 0, 0, 0, int(primary), 0, 0],
    ).lcm_encode()


def _pose_frame(
    frame_id: str,
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
    q: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0),
) -> bytes:
    return PoseStamped(
        ts=1.0,
        frame_id=frame_id,
        position=Vector3(x=x, y=y, z=z),
        orientation=Quaternion(q[0], q[1], q[2], q[3]),
    ).lcm_encode()


@pytest.mark.parametrize(
    "frame",
    [
        _joy_frame("left", trigger=float("nan")),
        _joy_frame("left", grip=float("inf")),
        _joy_frame("left", stick_x=float("nan")),
        _joy_frame("left", stick_y=float("-inf")),
    ],
    ids=["nan_trigger", "inf_grip", "nan_stick_x", "neg_inf_stick_y"],
)
def test_non_finite_joy_axes_rejected_without_refreshing_freshness(frame: bytes) -> None:
    m = _module()
    m._on_joy_bytes(frame)
    assert m._controllers.get(Hand.LEFT) is None
    assert m._joy_rx_ts[Hand.LEFT] == 0.0


@pytest.mark.parametrize(
    "frame",
    [
        _pose_frame("left", x=float("nan")),
        _pose_frame("left", z=float("inf")),
        _pose_frame("left", q=(float("nan"), 0.0, 0.0, 1.0)),
        _pose_frame("left", q=(0.0, 0.0, 0.0, 0.0)),
        _pose_frame("head"),
    ],
    ids=["nan_position", "inf_position", "nan_quaternion", "zero_quaternion", "unknown_frame"],
)
def test_malformed_pose_rejected_without_refreshing_freshness(frame: bytes) -> None:
    m = _module()
    m._on_pose_bytes(frame)
    assert m._current_poses.get(Hand.LEFT) is None
    assert m._pose_rx_ts[Hand.LEFT] == 0.0


def test_valid_frames_still_cached_and_fresh() -> None:
    m = _module()
    m._on_joy_bytes(_joy_frame("left", trigger=0.5, primary=True))
    m._on_pose_bytes(_pose_frame("left", x=0.1))
    assert m._controllers[Hand.LEFT] is not None
    assert m._controllers[Hand.LEFT].trigger == pytest.approx(0.5)
    assert m._joy_rx_ts[Hand.LEFT] > 0.0
    assert m._current_poses[Hand.LEFT] is not None
    assert m._pose_rx_ts[Hand.LEFT] > 0.0


def test_non_finite_trigger_never_publishes_gripper_command() -> None:
    # Defense in depth: even if a non-finite trigger reached the cache, the
    # gripper publisher must drop it, not convert it into a valid command.
    m = _module()
    assert m._gripper_command(float("nan")) is None
    _feed_fresh(m, Hand.LEFT, primary=True)
    with m._lock:
        m._handle_engage()
    m._controllers[Hand.LEFT] = _controller(is_left=True, primary=True, trigger=float("nan"))
    with m._lock:
        m._publish_button_state(m._controllers[Hand.LEFT], None)
    assert m.gripper_left_command.msgs == []


def test_malformed_stream_storm_forces_stale_release() -> None:
    # A stream stuck on malformed frames must not keep itself fresh: with
    # only bad frames arriving, the engaged hand takes the stale release.
    m = _module()
    _feed_fresh(m, Hand.LEFT, primary=True)
    with m._lock:
        m._handle_engage()
    assert m._is_engaged[Hand.LEFT]
    m._joy_rx_ts[Hand.LEFT] -= 10.0
    m._pose_rx_ts[Hand.LEFT] -= 10.0
    for _ in range(3):
        m._on_joy_bytes(_joy_frame("left", trigger=float("nan"), primary=True))
        m._on_pose_bytes(_pose_frame("left", x=float("nan")))
    assert m._joy_rx_ts[Hand.LEFT] < time.monotonic() - 5.0
    with m._lock:
        m._handle_engage()
    assert not m._is_engaged[Hand.LEFT]
    assert m._require_release[Hand.LEFT]


# Stale release end to end: module stream loss through the production task


def test_stale_stream_release_reaches_production_task() -> None:
    from dimos.control.tasks.teleop_task.teleop_task import create_task
    from dimos.robot.galaxea.r1lite.blueprints.basic.r1lite_quest_teleop import _teleop_tasks

    left_cfg = next(t for t in _teleop_tasks() if t.name == "teleop_left_arm")
    task = create_task(left_cfg, None)
    task.start()

    m = _module(
        task_names={"left": "teleop_left_arm", "right": "teleop_right_arm"},
        local_rotation=True,
    )
    lower, upper = _task_limits(task)
    q_start = (lower + upper) / 2.0
    positions = dict(zip(cfg.LEFT_ARM_JOINTS, q_start.tolist(), strict=True))

    def tick(t_now: float) -> Any:
        with m._lock:
            m._handle_engage()
            if m._should_publish(Hand.LEFT):
                pose = m._get_output_pose(Hand.LEFT)
                if pose is not None:
                    m._publish_msg(Hand.LEFT, pose)
            m._publish_button_state(m._controllers.get(Hand.LEFT), m._controllers.get(Hand.RIGHT))
        task.on_teleop_buttons(m.teleop_buttons.msgs[-1], t_now)
        for routed in m.left_controller_output.msgs:
            task.on_cartesian_command(routed, t_now)
        m.left_controller_output.msgs.clear()
        state = types.SimpleNamespace(t_now=t_now, dt=0.02, joints=_FakeJoints(positions))
        return task.compute(state)

    # An active production command is established through the real path.
    m._on_pose_bytes(_pose_frame("left"))
    m._on_joy_bytes(_joy_frame("left", primary=True))
    tick(0.02)
    m._on_pose_bytes(_pose_frame("left", x=0.05))
    out = tick(0.04)
    assert out is not None
    positions = dict(zip(cfg.LEFT_ARM_JOINTS, out.positions, strict=True))

    # Both streams die mid-motion. The module tick publishes the released
    # button state, the task routes it, and the task goes inert.
    with m._lock:
        m._joy_rx_ts[Hand.LEFT] -= 10.0
        m._pose_rx_ts[Hand.LEFT] -= 10.0
    assert tick(0.06) is None
    assert not m._is_engaged[Hand.LEFT]
    assert tick(0.08) is None

    # The stream recovers with the primary STILL held: no engagement, no
    # command, end to end.
    m._on_joy_bytes(_joy_frame("left", primary=True))
    m._on_pose_bytes(_pose_frame("left", x=0.08))
    assert tick(0.10) is None
    assert not m._is_engaged[Hand.LEFT]

    # Observed release, fresh press: commands flow again.
    m._on_joy_bytes(_joy_frame("left", primary=False))
    assert tick(0.12) is None
    m._on_joy_bytes(_joy_frame("left", primary=True))
    m._on_pose_bytes(_pose_frame("left", x=0.08))
    tick(0.14)
    m._on_pose_bytes(_pose_frame("left", x=0.11))
    assert tick(0.16) is not None
    assert m._is_engaged[Hand.LEFT]

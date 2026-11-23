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

"""The tabletop-clamped FR3 pair's shared model and the guard that keeps the arms apart."""

from __future__ import annotations

from collections.abc import Iterator
import math
import platform
import sys
from typing import Any

import numpy as np
import pytest

if sys.platform == "linux" and platform.machine() == "aarch64":
    pytest.importorskip("pydrake", reason="Drake is unavailable in the Linux ARM test environment")

from scipy.spatial.transform import Rotation

from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.franka.separation_guard import SeparationGuard
from dimos.robot.manipulators.franka.tabletop_bimanual import (
    LEFT_BASE_RPY,
    LEFT_BASE_XYZ,
    SIDES,
    TABLETOP_BIMANUAL_READY,
    arm_joint_names,
    finger_joint,
    link,
    make_tabletop_bimanual_model_config,
    scene_camera_transform,
    wrist_camera_link,
    wrist_camera_transform,
)

READY = TABLETOP_BIMANUAL_READY[:7]
# Each arm swung 0.5 rad toward the other and reaching forward and down: the arms
# overlap in the middle of the table.
RIGHT_IN = [0.5, 0.6, 0.0, -1.2, 0.0, 1.571, 0.785]
LEFT_IN = [-0.5, 0.6, 0.0, -1.2, 0.0, 1.571, 0.785]
# A handover over the middle of the table, the two grippers face to face, level,
# fingers opening sideways, their TCPs 4 cm apart (solved by IK): the grippers are
# 21 mm apart and the rest of the arms (wrists, cameras) 5.7 cm.
HANDOVER_RIGHT = [0.284, 0.186, -0.082, -2.464, 1.943, 1.748, 1.247]
HANDOVER_LEFT = [-0.272, 0.186, 0.047, -2.464, -1.94, 1.753, 0.326]
# The right gripper 1 cm further in (TCPs 3 cm apart): grippers 11 mm, but the rest
# of the arms 4.7 cm.
HANDOVER_RIGHT_CLOSER = [0.296, 0.195, -0.071, -2.45, 1.963, 1.761, 1.247]


@pytest.fixture(scope="module")
def world() -> Iterator[Any]:
    from dimos.manipulation.planning.spec.validation import prepare_robot_model
    from dimos.manipulation.planning.world.drake_world import DrakeWorld

    config = make_tabletop_bimanual_model_config()
    w = DrakeWorld(enable_viz=False)
    w.load_model(prepare_robot_model(config))
    w.finalize()
    yield w, config


def _pose(world: Any, right: list[float], left: list[float]) -> Any:
    w, config = world
    ctx = w.get_live_context()
    w.set_joint_state(ctx, JointState(name=config.joint_names, position=[*right, *left]))
    return ctx


def _arm(side: str) -> list[str]:
    return [link(side, f"link{i}") for i in range(8)] + [link(side, "hand")]


def test_each_arm_has_its_own_group_and_joints() -> None:
    config = make_tabletop_bimanual_model_config()
    groups = {g.name: g for g in config.planning_groups}
    assert set(groups) == {"right", "left", "both"}
    assert list(groups["left"].joint_names) == arm_joint_names("left")
    assert groups["left"].tip_link == "left/fr3_hand_tcp"
    assert config.joint_names == [*arm_joint_names("right"), *arm_joint_names("left")]
    assert finger_joint("left") == "left/fr3_finger_joint1"


def test_the_left_base_sits_where_it_was_measured(world: Any) -> None:
    w, _ = world
    T = np.asarray(w.get_link_pose(_pose(world, READY, READY), "left/base"))
    assert np.allclose(T[:3, 3], LEFT_BASE_XYZ, atol=1e-6)
    expected = Rotation.from_euler("xyz", LEFT_BASE_RPY).as_matrix()
    assert np.allclose(T[:3, :3], expected, atol=1e-6)
    # About 78 cm apart, each turned in: the left arm faces back toward the right.
    assert 0.75 < math.hypot(*LEFT_BASE_XYZ[:2]) < 0.80


def test_the_arms_are_well_apart_at_ready(world: Any) -> None:
    w, _ = world
    ctx = _pose(world, READY, READY)
    assert w.is_collision_free(ctx)
    assert w.get_min_distance_between(ctx, _arm("right"), _arm("left")) > 0.3


def test_the_world_sees_the_arms_collide_with_each_other(world: Any) -> None:
    w, _ = world
    ctx = _pose(world, RIGHT_IN, LEFT_IN)
    assert not w.is_collision_free(ctx)
    assert w.get_min_distance_between(ctx, _arm("right"), _arm("left")) < 0.0
    # Each arm alone reaching in is fine; it is the pair that collides.
    assert w.is_collision_free(_pose(world, RIGHT_IN, READY))
    assert w.is_collision_free(_pose(world, READY, LEFT_IN))


def test_each_wrist_camera_is_in_the_collision_world(world: Any) -> None:
    w, _ = world
    ctx = _pose(world, READY, READY)
    assert w.is_collision_free(ctx)  # Its bracket on its own hand is not a collision.
    for side in SIDES:
        camera = np.asarray(w.get_link_pose(ctx, wrist_camera_link(side)))[:3, 3]
        tcp = np.asarray(w.get_link_pose(ctx, link(side, "hand_tcp")))[:3, 3]
        # Beside the hand, above the fingertips.
        assert 0.03 < np.linalg.norm(camera[:2] - tcp[:2]) < 0.09
        assert 0.0 < camera[2] - tcp[2] < 0.08


def test_cameras_hang_off_each_hand_and_the_world() -> None:
    for side in SIDES:
        t = wrist_camera_transform(side)
        assert (t.frame_id, t.child_frame_id) == (f"{side}/fr3_hand", f"{side}_wrist_camera_link")
    scene = scene_camera_transform()
    assert (scene.frame_id, scene.child_frame_id) == ("world", "scene_camera_link")
    # Over the table, looking down at it.
    assert 1.0 < scene.translation.z < 1.6
    q = scene.rotation
    lens = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()[:, 0]
    assert lens[2] < -0.5


# ---------------------------------------------------------------- guard


class Recorder:
    def __init__(self) -> None:
        self.sent: list[JointState] = []

    def broadcast(self, selfstream: Any, value: Any) -> None:
        self.sent.append(value)

    def publish(self, value: Any) -> None:
        self.sent.append(value)

    def subscribe(self, cb: Any, selfstream: Any = None) -> Any:
        return lambda: None

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass


@pytest.fixture(scope="module")
def guard() -> Iterator[tuple[SeparationGuard, dict[str, Recorder]]]:
    g = SeparationGuard()
    out = {"right": Recorder(), "left": Recorder()}
    g.set_transport("right_command", out["right"])  # type: ignore[arg-type]
    g.set_transport("left_command", out["left"])  # type: ignore[arg-type]
    g._load_world()
    yield g, out


def _command(side: str, q: list[float], finger: float | None = None) -> JointState:
    names, position = list(arm_joint_names(side)), list(q)
    if finger is not None:
        names.append(finger_joint(side))
        position.append(finger)
    return JointState(name=names, position=position)


def _reset(
    g: SeparationGuard, out: dict[str, Recorder], right: list[float], left: list[float]
) -> None:
    g._target = {"right": None, "left": None}
    g._on_state("right", _command("right", right))
    g._on_state("left", _command("left", left))
    for r in out.values():
        r.sent.clear()


def test_guard_passes_targets_that_keep_the_arms_apart(guard: Any) -> None:
    g, out = guard
    _reset(g, out, READY, READY)
    g._on_request("right", _command("right", RIGHT_IN, finger=0.04))
    assert len(out["right"].sent) == 1
    assert g.status()["arm_clearance_m"] > g.config.arm_separation_m


def test_guard_lets_the_grippers_meet_for_a_handover(guard: Any) -> None:
    g, out = guard
    _reset(g, out, READY, READY)
    g._on_request("left", _command("left", HANDOVER_LEFT))
    g._on_request("right", _command("right", HANDOVER_RIGHT))
    assert len(out["left"].sent) == 1
    assert len(out["right"].sent) == 1
    status = g.status()
    assert status["gripper_clearance_m"] < g.config.arm_separation_m  # Closer than arms may be.
    assert status["arm_clearance_m"] >= g.config.arm_separation_m


def test_guard_keeps_the_wrists_and_cameras_apart_in_a_handover(guard: Any) -> None:
    g, out = guard
    _reset(g, out, READY, READY)
    g._on_request("left", _command("left", HANDOVER_LEFT))
    g._on_request("right", _command("right", HANDOVER_RIGHT_CLOSER))
    assert out["right"].sent == []
    assert g.status()["gripper_clearance_m"] > g.config.gripper_separation_m
    assert g.status()["arm_clearance_m"] < g.config.arm_separation_m


def test_guard_holds_an_arm_that_would_hit_the_other(guard: Any) -> None:
    g, out = guard
    _reset(g, out, READY, READY)
    g._on_request("left", _command("left", LEFT_IN))  # Fine alone: the left reaches in.
    g._on_request("right", _command("right", RIGHT_IN))  # Now the right would hit it.
    assert len(out["left"].sent) == 1
    assert out["right"].sent == []
    assert g.status()["refused"]["right"] >= 1


def test_guard_judges_against_the_other_arms_target_not_its_reading(guard: Any) -> None:
    g, out = guard
    # The left arm still reads ready, but has been sent in: the right must not follow.
    _reset(g, out, READY, READY)
    g._on_request("left", _command("left", LEFT_IN))
    g._on_request("right", _command("right", RIGHT_IN))
    assert out["right"].sent == []


def _reaching(j1: float) -> list[float]:
    """The right arm reaching in like RIGHT_IN, swung to ``j1``; against LEFT_IN the gap
    is about 2.5 cm at j1 -0.05 and 5.6 cm at -0.1, and the arms overlap from about 0."""
    return [j1, 0.6, 0.0, -1.2, 0.0, 1.571, 0.785]


def test_guard_lets_an_arm_back_out_of_a_too_small_gap(guard: Any) -> None:
    g, out = guard
    _reset(g, out, _reaching(-0.05), LEFT_IN)  # 2.5 cm apart, say pushed there by hand
    g._on_request("right", _command("right", _reaching(-0.02)))  # closer: held
    assert out["right"].sent == []
    g._on_request("right", _command("right", _reaching(-0.08)))  # wider, still < 5 cm: passes
    assert len(out["right"].sent) == 1


def test_guard_trusts_only_a_clear_target_out_of_an_overlap(guard: Any) -> None:
    g, out = guard
    # Overlapping in the model: a shallower overlap is not trusted to be moving apart.
    _reset(g, out, RIGHT_IN, LEFT_IN)
    g._on_request("right", _command("right", _reaching(0.6)))
    assert out["right"].sent == []
    g._on_request("right", _command("right", READY))
    assert len(out["right"].sent) == 1


def test_guard_passes_a_hand_only_command(guard: Any) -> None:
    g, out = guard
    _reset(g, out, RIGHT_IN, LEFT_IN)
    g._on_request("right", JointState(name=[finger_joint("right")], position=[0.0]))
    assert len(out["right"].sent) == 1


def test_guard_holds_both_arms_until_it_knows_where_the_other_is(guard: Any) -> None:
    g, out = guard
    g._target = {"right": None, "left": None}
    g._measured = {"right": list(READY), "left": None}
    for r in out.values():
        r.sent.clear()
    g._on_request("right", _command("right", READY))
    assert out["right"].sent == []


def test_calibration_loads_from_a_file_and_defaults_to_dimensionals(tmp_path, monkeypatch) -> None:
    from dimos.robot.manipulators.franka.tabletop_bimanual import (
        CALIBRATION_ENV,
        DIMENSIONAL_CALIBRATION,
        load_calibration,
    )

    monkeypatch.delenv(CALIBRATION_ENV, raising=False)
    assert load_calibration() == DIMENSIONAL_CALIBRATION
    other = DIMENSIONAL_CALIBRATION.model_copy(deep=True)
    other.left_base.xyz = (0.0, 0.9, 0.0)
    path = tmp_path / "cal.json"
    path.write_text(other.model_dump_json())
    assert load_calibration(path).left_base.xyz == (0.0, 0.9, 0.0)
    monkeypatch.setenv(CALIBRATION_ENV, str(path))
    assert load_calibration().left_base.xyz == (0.0, 0.9, 0.0)

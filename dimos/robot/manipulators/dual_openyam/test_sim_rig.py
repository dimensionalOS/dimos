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

"""The simulated rig end to end: its physics running in real time, with no DimOS transport."""

from collections.abc import Iterator
from pathlib import Path

import cv2
import numpy as np
import pytest

from dimos.robot.manipulators.dual_openyam.blueprints.grasp import (
    DUAL_OPENYAM_CAMERA_TRANSFORM,
    DUAL_OPENYAM_TABLE_TOP_Z,
)
from dimos.robot.manipulators.dual_openyam.config import (
    DUAL_OPENYAM,
    DUAL_OPENYAM_HOME_JOINTS,
)
from dimos.robot.manipulators.dual_openyam.sim_rig import DualOpenYamSimRig, SimCameraFrame
from dimos.robot.manipulators.dual_openyam.sim_scene import (
    OVERHEAD_CAMERA,
    WORKCELL_OBJECTS,
    WRIST_CAMERAS,
)
from dimos.utils.testing.waiting import wait_until

pytestmark = pytest.mark.self_hosted

ARM_JOINTS = DUAL_OPENYAM.arm_joint_names
# A posture away from home for both arms, within every joint's range.
REACHING = [0.4, 0.8, 1.3, 0.3, -0.3, 0.5, -0.4, 0.8, 1.3, -0.3, 0.3, -0.5]


def positions(rig: DualOpenYamSimRig) -> dict[str, float]:
    return dict(zip(rig.joint_names, rig.read().position, strict=True))


def arm_error(rig: DualOpenYamSimRig, target: list[float]) -> float:
    """The largest distance of any arm joint from its target, rad."""
    now = positions(rig)
    return max(abs(now[name] - goal) for name, goal in zip(ARM_JOINTS, target, strict=True))


@pytest.fixture
def rig() -> Iterator[DualOpenYamSimRig]:
    """The rig's physics alone: no camera, so no OpenGL context is needed."""
    rig = DualOpenYamSimRig(DUAL_OPENYAM, camera_size=None)
    rig.connect()
    yield rig
    rig.disconnect()


@pytest.fixture
def frames(mujoco_gl: None) -> Iterator[list[dict[str, SimCameraFrame]]]:
    """Every set of frames a rig with cameras renders, as they arrive."""
    rendered: list[dict[str, SimCameraFrame]] = []
    rig = DualOpenYamSimRig(DUAL_OPENYAM, on_frames=lambda new: rendered.append(dict(new)))
    rig.connect()
    yield rendered
    rig.disconnect()


def test_starts_at_home_with_jaws_closed_and_stays_there(rig: DualOpenYamSimRig) -> None:
    assert rig.joint_names == DUAL_OPENYAM.joint_names

    wait_until(lambda: rig.sim_time > 1.0, timeout=10.0)

    assert arm_error(rig, DUAL_OPENYAM_HOME_JOINTS) < 1e-3
    now = positions(rig)
    assert now["left_gripper"] == pytest.approx(0.0, abs=1e-3)
    assert now["right_gripper"] == pytest.approx(0.0, abs=1e-3)


def test_joint_state_follows_a_position_command_on_both_arms(rig: DualOpenYamSimRig) -> None:
    rig.command(dict(zip(ARM_JOINTS, REACHING, strict=True)))
    wait_until(lambda: arm_error(rig, REACHING) < 2e-3, timeout=10.0)

    reading = rig.read()
    assert np.abs(reading.velocity).max() < 0.2
    # The elbows carry the forearms: gravity compensation shows as effort.
    efforts = dict(zip(rig.joint_names, reading.effort, strict=True))
    assert abs(efforts["left_joint3"]) > 1.0
    assert abs(efforts["right_joint3"]) > 1.0

    rig.command({"left_joint1": 0.0})
    wait_until(lambda: abs(positions(rig)["left_joint1"]) < 0.01, timeout=10.0)
    # A command for one joint leaves the others at their targets.
    assert positions(rig)["right_joint1"] == pytest.approx(REACHING[6], abs=0.01)


def test_both_jaws_open_and_close(rig: DualOpenYamSimRig) -> None:
    jaws = ("left_gripper", "right_gripper")

    rig.command({"left_gripper": 1.0, "right_gripper": 0.5})
    wait_until(
        lambda: [positions(rig)[jaw] for jaw in jaws] == pytest.approx([1.0, 0.5], abs=0.01),
        timeout=10.0,
    )

    rig.command(dict.fromkeys(jaws, 0.0))
    wait_until(
        lambda: [positions(rig)[jaw] for jaw in jaws] == pytest.approx([0.0, 0.0], abs=0.01),
        timeout=10.0,
    )
    assert arm_error(rig, DUAL_OPENYAM_HOME_JOINTS) < 1e-3


def test_objects_stay_on_the_table_while_the_arms_move(rig: DualOpenYamSimRig) -> None:
    resting = {name: rig.body_position(name) for name in WORKCELL_OBJECTS}

    rig.command(dict(zip(ARM_JOINTS, REACHING, strict=True)))
    wait_until(lambda: arm_error(rig, REACHING) < 0.01, timeout=10.0)

    for name, start in resting.items():
        now = rig.body_position(name)
        assert now == pytest.approx(start, abs=1e-3)
        # No body frame is below the table top, give or take the contact's softness.
        assert now[2] > DUAL_OPENYAM_TABLE_TOP_Z - 1e-3


def test_hold_stops_an_arm_on_its_way(rig: DualOpenYamSimRig) -> None:
    # Swing the left arm outwards about its base, away from the other arm.
    rig.command({"left_joint1": 1.5})
    wait_until(lambda: positions(rig)["left_joint1"] > 0.3, timeout=10.0, interval=0.01)

    rig.hold()
    wait_until(lambda: np.abs(rig.read().velocity).max() < 0.01, timeout=10.0)

    assert 0.3 < positions(rig)["left_joint1"] < 1.3


def test_rejects_unknown_joints_and_reads_nothing_once_disconnected(
    rig: DualOpenYamSimRig,
) -> None:
    with pytest.raises(ValueError, match="unknown Dual OpenYAM joints"):
        rig.command({"left_joint7": 0.0})

    rig.disconnect()

    with pytest.raises(RuntimeError, match="not connected"):
        rig.read()
    with pytest.raises(RuntimeError, match="not connected"):
        rig.command({"left_joint1": 0.0})


def test_every_camera_renders_the_scene(
    frames: list[dict[str, SimCameraFrame]], tmp_path: Path
) -> None:
    wait_until(lambda: len(frames) >= 3, timeout=20.0)
    newest = frames[-1]

    assert set(newest) == {OVERHEAD_CAMERA, *WRIST_CAMERAS.values()}
    for name, frame in newest.items():
        assert frame.color.shape == (480, 640, 3)
        assert frame.color.dtype == np.uint8
        # A rendered scene has many shades; a blank or failed render has one.
        assert len(np.unique(frame.color.reshape(-1, 3), axis=0)) > 100
        saved = tmp_path / f"{name}.png"
        cv2.imwrite(str(saved), cv2.cvtColor(frame.color, cv2.COLOR_RGB2BGR))
        assert cv2.imread(str(saved)).shape == (480, 640, 3)

    overhead = newest[OVERHEAD_CAMERA]
    mount = DUAL_OPENYAM_CAMERA_TRANSFORM.translation.to_numpy()
    assert overhead.position == pytest.approx(mount, abs=1e-6)
    assert overhead.depth is not None
    assert overhead.depth.shape == (480, 640)
    # It looks down at the table from 0.45 m: the nearest thing it sees is the
    # foot of an arm or the plate they stand on, the farthest the wall behind
    # the table.
    assert 0.2 < overhead.depth.min() < 0.45
    assert 0.6 < overhead.depth.max() < 1.0
    # The white table fills the middle of the picture and the red cup stands left of it.
    assert overhead.color[300, 320].min() > 180
    red, green, blue = overhead.color[150:190, 70:110].reshape(-1, 3).mean(axis=0)
    assert red > 2 * green and red > 2 * blue
    for name in WRIST_CAMERAS.values():
        assert newest[name].depth is None
    # Frames keep coming, each of a later moment.
    assert frames[-1][OVERHEAD_CAMERA].stamp > frames[0][OVERHEAD_CAMERA].stamp

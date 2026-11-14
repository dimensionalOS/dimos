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

"""The self-cleared scene cloud, without cameras or detectors."""

from __future__ import annotations

import asyncio
from collections.abc import Iterator
import threading
from types import SimpleNamespace

import numpy as np
from numpy.typing import NDArray
import pytest
from pytest_mock import MockerFixture

from dimos.manipulation.manipulation_localize import ManipulationLocalize
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.protocol.tf.tf import TF
from dimos.robot.manipulators.dual_openyam.blueprints.grasp import (
    DUAL_OPENYAM_TABLE_TOP_Z,
    dual_openyam_self_filter_model,
)
from dimos.robot.manipulators.dual_openyam.scene_registration import (
    SelfClearedManipulationLocalize,
)

TS = 1000.0
# The coordinator reports the connection's joints under the robot name "arm".
ARM_JOINTS = [f"arm/{side}_joint{i}" for side in ("left", "right") for i in range(1, 7)]
REST = dict.fromkeys(ARM_JOINTS, 0.0)
# The right arm's shoulder and elbow each lifted a radian off the folded rest pose.
RAISED = {**REST, "arm/right_joint2": 1.0, "arm/right_joint3": 1.0}
# Inside the right base and right link 1, which stand where they are in both poses.
BASE = np.array([[0.0, -0.222, 0.02], [0.0, -0.222, 0.09], [0.02, -0.21, 0.105]])
# Within 5 mm of the right arm's gripper end at REST, over 5 cm from the arm at RAISED.
ARM_AT_REST = np.array([[0.12, -0.20, 0.14], [0.14, -0.22, 0.18], [0.16, -0.20, 0.12]])
# Within 5 mm of the right arm's gripper end at RAISED, over 5 cm from the arm at REST.
ARM_WHEN_RAISED = np.array([[0.26, -0.20, 0.38], [0.28, -0.20, 0.40], [0.32, -0.20, 0.36]])
# On the table, clear of both arms.
BLOCK = np.array([[0.3, 0.1, 0.0], [0.3, 0.1, 0.02]])
# The model's root on tf as the planner publishes it: an edge from the world frame.
ROOT_AT_ORIGIN = Transform(frame_id="world", child_frame_id="dual_openyam_base", ts=TS)


def _table() -> NDArray[np.float64]:
    """Table points 2 cm apart, clear of both arms."""
    x, y = np.meshgrid(np.arange(0.1, 0.5, 0.02), np.arange(-0.3, 0.3, 0.02))
    return np.stack([x.ravel(), y.ravel(), np.full(x.size, DUAL_OPENYAM_TABLE_TOP_Z)], axis=1)


def _view(ts: float, *parts: NDArray[np.float64], root: Transform = ROOT_AT_ORIGIN) -> PointCloud2:
    """One depth view in the world frame, captured at ``ts``: the table plus ``parts``.

    The points are given in the model's root frame, which stands in the world at ``root``.
    """
    points = np.concatenate([_table(), *parts])
    matrix = root.to_matrix()
    return PointCloud2.from_numpy(
        (points @ matrix[:3, :3].T + matrix[:3, 3]).astype(np.float32),
        frame_id="world",
        timestamp=ts,
    )


def _rows(cloud: PointCloud2 | NDArray[np.float64]) -> list[tuple[float, ...]]:
    """Points to the tenth of a millimetre, in a fixed order."""
    points = cloud.points_f32() if isinstance(cloud, PointCloud2) else cloud
    return sorted(tuple(round(float(v), 4) for v in point) for point in points)


@pytest.fixture
def module(mocker: MockerFixture) -> Iterator[SelfClearedManipulationLocalize]:
    """The module started with its ports and the detector's models stubbed."""
    m = SelfClearedManipulationLocalize(model=dual_openyam_self_filter_model(), padding_m=0.02)
    for port in ("tf", "color_image", "depth_image", "camera_info", "coordinator_joint_state"):
        mocker.patch.object(getattr(m, port), "subscribe", return_value=mocker.Mock())
    mocker.patch("dimos.perception.localize.module.DanDetector", autospec=True)
    m.start()
    yield m
    m.stop()


def _joints_at(
    module: SelfClearedManipulationLocalize, joints: dict[float, dict[str, float]]
) -> None:
    """Hand the module the joint state the coordinator sent at each time."""
    for ts, positions in joints.items():
        state = JointState(name=list(positions), position=list(positions.values()), ts=ts)
        asyncio.run(module.handle_coordinator_joint_state(state))


def _remember(
    module: SelfClearedManipulationLocalize,
    mocker: MockerFixture,
    views: list[PointCloud2],
    root: Transform | None = ROOT_AT_ORIGIN,
) -> None:
    """A ready module whose memory holds one embedded frame per view, and tf holds ``root``."""
    module._ready = threading.Event()
    module._ready.set()
    module.index = mocker.Mock()
    module.index.get_time_range.return_value = (TS - 100.0, views[-1].ts)
    module.index.time_range.return_value = [SimpleNamespace(ts=view.ts) for view in views]
    by_time = {view.ts: view for view in views}
    module.rig = mocker.Mock(cameras={"camera_color_optical_frame": None}, world_frame="world")
    module.rig.backdrop.side_effect = lambda ts, frame, trunc: by_time[ts]
    module.rig.tf = TF()
    if root is not None:
        module.rig.tf.receive_transform(root)


def test_scene_keeps_the_table_and_drops_the_arm_base(
    module: SelfClearedManipulationLocalize, mocker: MockerFixture
) -> None:
    _joints_at(module, {TS: REST})
    _remember(module, mocker, [_view(TS, BASE)])

    cleared = module.scene_cloud()

    # Two of the base points are over 1 cm from the collision mesh: they go with the
    # 2 cm padding the module was given, and would stay with the default 1 cm.
    assert cleared is not None and cleared.frame_id == "world" and cleared.ts == TS
    assert _rows(cleared) == _rows(_table())


def test_the_robot_is_cleared_where_tf_puts_the_model_root(
    module: SelfClearedManipulationLocalize, mocker: MockerFixture
) -> None:
    # The root a quarter turn about Z and 0.5 m, -0.2 m, 0.1 m from the world origin.
    root = Transform(
        translation=Vector3(0.5, -0.2, 0.1),
        rotation=Quaternion(0.0, 0.0, np.sqrt(0.5), np.sqrt(0.5)),
        frame_id="world",
        child_frame_id="dual_openyam_base",
        ts=TS,
    )
    _joints_at(module, {TS: REST})
    _remember(module, mocker, [_view(TS, BASE, root=root)], root)

    cleared = module.scene_cloud()

    assert cleared is not None
    matrix = root.to_matrix()
    assert _rows(cleared) == _rows(_table() @ matrix[:3, :3].T + matrix[:3, 3])

    # With no root on tf the robot cannot be placed in the view.
    _remember(module, mocker, [_view(TS, BASE, root=root)], root=None)
    assert module.scene_cloud() is None


def test_each_view_loses_the_robot_where_it_stood_at_that_capture(
    module: SelfClearedManipulationLocalize, mocker: MockerFixture
) -> None:
    _joints_at(module, {TS: REST, TS + 2.0: RAISED})

    # The arm at rest in the first view and raised in the second: neither image stays.
    _remember(module, mocker, [_view(TS, ARM_AT_REST), _view(TS + 2.0, ARM_WHEN_RAISED)])
    cleared = module.scene_cloud()
    assert cleared is not None and cleared.ts == TS + 2.0
    assert _rows(cleared) == _rows(_table())

    # The same points seen while the arm stood elsewhere are the scene's, and stay.
    _remember(module, mocker, [_view(TS, ARM_WHEN_RAISED), _view(TS + 2.0, ARM_AT_REST)])
    kept = module.scene_cloud()
    assert kept is not None
    assert _rows(kept) == _rows(np.concatenate([_table(), ARM_AT_REST, ARM_WHEN_RAISED]))


def test_views_whose_robot_pose_is_unknown_are_left_out(
    module: SelfClearedManipulationLocalize, mocker: MockerFixture
) -> None:
    # The joint state at the third view lacks the left arm.
    right_only = {name: 0.0 for name in ARM_JOINTS if "right" in name}
    _joints_at(module, {TS: REST, TS + 20.0: right_only})
    known = _view(TS, BASE)
    # 5 s from the nearest joint state, past the 0.5 s it may be off by.
    stateless = _view(TS + 5.0, BASE, BLOCK)
    partial = _view(TS + 20.0, BASE, BLOCK)

    _remember(module, mocker, [known, stateless, partial])
    cleared = module.scene_cloud()
    assert cleared is not None and cleared.ts == TS
    assert _rows(cleared) == _rows(_table())

    # No view left: no scene, rather than one with the robot in it.
    _remember(module, mocker, [stateless, partial])
    assert module.scene_cloud() is None


def test_scene_is_none_before_ready_or_without_depth(
    module: SelfClearedManipulationLocalize, mocker: MockerFixture
) -> None:
    _joints_at(module, {TS: REST})
    _remember(module, mocker, [_view(TS, BASE)])
    module._ready.clear()
    assert module.scene_cloud() is None

    module._ready.set()
    module.config.geometry = "lidar"
    assert module.scene_cloud() is None


def test_a_scene_the_robot_is_not_in_is_the_one_localize_gives(
    module: SelfClearedManipulationLocalize, mocker: MockerFixture
) -> None:
    """The window and the merge are those of the method this one overrides."""
    _joints_at(module, {TS: REST, TS + 1.0: RAISED, TS + 2.0: REST})
    _remember(module, mocker, [_view(TS, BLOCK), _view(TS + 1.0), _view(TS + 2.0, BLOCK)])

    uncleared = ManipulationLocalize.scene_cloud(module, start=-3.0, duration=2.5)
    window = module.index.time_range.call_args.args
    cleared = module.scene_cloud(start=-3.0, duration=2.5)

    assert window == (TS - 1.0, TS + 1.5)
    assert module.index.time_range.call_args.args == window
    assert uncleared is not None and cleared is not None
    assert (cleared.frame_id, cleared.ts) == (uncleared.frame_id, uncleared.ts)
    assert _rows(cleared) == _rows(uncleared)

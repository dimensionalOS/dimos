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

from types import SimpleNamespace

import numpy as np
import pytest

from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.perception.experimental.object_scene_registration import ObjectSceneRegistrationModule
from dimos.robot.manipulators.dual_openyam.blueprints.grasp import (
    DUAL_OPENYAM_TABLE_TOP_Z,
    dual_openyam_self_filter_model,
)
from dimos.robot.manipulators.dual_openyam.scene_registration import (
    SelfClearedSceneRegistrationConfig,
    SelfClearedSceneRegistrationModule,
)
from dimos.robot.roboplan_body_filter import RoboPlanBodyFilter
from dimos.types.timestamped import TimestampedBufferCollection

TS = 1000.0
ARM_JOINTS = [f"{side}_joint{i}" for side in ("left", "right") for i in range(1, 7)]


def _scene() -> PointCloud2:
    """Table points plus a cluster on the right arm's base at rest."""
    x, y = np.meshgrid(np.arange(0.1, 0.5, 0.02), np.arange(-0.3, 0.3, 0.02))
    table = np.stack([x.ravel(), y.ravel(), np.full(x.size, DUAL_OPENYAM_TABLE_TOP_Z)], axis=1)
    # Inside the right base and right link 1 at rest.
    base = np.array([[0.0, -0.222, 0.02], [0.0, -0.222, 0.09], [0.02, -0.21, 0.105]])
    return PointCloud2.from_numpy(
        np.concatenate([table, base]).astype(np.float32), frame_id="world", timestamp=TS
    )


@pytest.fixture
def module(monkeypatch: pytest.MonkeyPatch) -> SelfClearedSceneRegistrationModule:
    m = SelfClearedSceneRegistrationModule.__new__(SelfClearedSceneRegistrationModule)
    m.config = SelfClearedSceneRegistrationConfig(
        model=dual_openyam_self_filter_model(), padding_m=0.02
    )
    m._states = TimestampedBufferCollection[JointState](600.0)
    m._states.add(JointState(name=ARM_JOINTS, position=[0.0] * 12, ts=TS))
    m._root_link = "dual_openyam_base"
    m._body_filter = RoboPlanBodyFilter(m.config.model.load(), 0.02)
    identity = Transform(
        translation=Vector3(0, 0, 0),
        rotation=Quaternion(0, 0, 0, 1),
        frame_id="dual_openyam_base",
        child_frame_id="world",
    )
    monkeypatch.setattr(
        SelfClearedSceneRegistrationModule,
        "tfbuffer",
        property(lambda self: SimpleNamespace(get=lambda *a, **k: identity)),
    )
    monkeypatch.setattr(
        ObjectSceneRegistrationModule, "get_full_scene_pointcloud", lambda self, *a, **k: _scene()
    )
    return m


def test_scene_keeps_the_table_and_drops_the_arm_base(
    module: SelfClearedSceneRegistrationModule,
) -> None:
    raw = _scene().as_numpy()[0]
    cleared = module.get_full_scene_pointcloud()
    assert cleared is not None and cleared.frame_id == "world"
    points = cleared.as_numpy()[0]
    assert len(points) == len(raw) - 3
    assert np.allclose(points[:, 2], DUAL_OPENYAM_TABLE_TOP_Z, atol=1e-6)


def test_scan_time_state_serves_a_cloud_read_at_pick_time(
    module: SelfClearedSceneRegistrationModule,
) -> None:
    # The arm moved after the scan; the cloud still clears with the state from the scan.
    module._states.add(JointState(name=ARM_JOINTS, position=[0.8] * 12, ts=TS + 25.0))
    cleared = module.get_full_scene_pointcloud()
    assert cleared is not None and len(cleared.as_numpy()[0]) == len(_scene().as_numpy()[0]) - 3


def test_scene_is_withheld_without_a_state_near_the_capture(
    module: SelfClearedSceneRegistrationModule,
) -> None:
    module._states = TimestampedBufferCollection[JointState](600.0)
    module._states.add(JointState(name=ARM_JOINTS, position=[0.0] * 12, ts=TS - 5.0))
    assert module.get_full_scene_pointcloud() is None


def test_module_is_the_registration_module_with_a_joint_state_input() -> None:
    assert issubclass(SelfClearedSceneRegistrationModule, ObjectSceneRegistrationModule)
    assert "coordinator_joint_state" in SelfClearedSceneRegistrationModule.__annotations__

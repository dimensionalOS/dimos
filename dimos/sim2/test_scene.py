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

from __future__ import annotations

from typing import Any

import pytest

from dimos.core.hardware_module import Mounts
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.robot.assets.model import RobotModel, UsdModel
from dimos.sim2.scene import SimScene, SimSceneConfig

pytest.importorskip("pxr")

_ROOM = """<robot name="room">
  <link name="room"/><link name="desk"/><link name="aisle"/><link name="spot"/>
  <joint name="desk" type="fixed"><parent link="room"/><child link="desk"/><origin xyz="1 0 0"/></joint>
  <joint name="aisle" type="fixed"><parent link="room"/><child link="aisle"/><origin xyz="0 2 0"/></joint>
  <joint name="spot" type="fixed"><parent link="aisle"/><child link="spot"/><origin xyz="0 1 0"/></joint>
</robot>"""


class _RoomConfig(SimSceneConfig):
    usd: UsdModel = UsdModel.from_robot_model(RobotModel.from_urdf_string(_ROOM))


class _Room(SimScene):
    config: _RoomConfig
    mounts = Mounts("room/room/aisle/spot")


def test_a_scene_publishes_only_the_path_to_its_mounts() -> None:
    room = _Room()
    try:
        edges = {(t.frame_id, t.child_frame_id) for t in room.static_transforms()}
    finally:
        room.stop()

    assert edges == {
        ("room", "room/room"),
        ("room/room", "room/room/aisle"),
        ("room/room/aisle", "room/room/aisle/spot"),
    }


def test_a_scene_works_its_edges_out_once_and_restamps_them(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    room = _Room()
    try:
        worked_out = []
        namespaced = room._usd_edges_as_namespaced_transforms

        def working_out(*args: Any) -> list[Transform]:
            worked_out.append(1)
            return namespaced(*args)

        monkeypatch.setattr(room, "_usd_edges_as_namespaced_transforms", working_out)
        stamps = [room.static_transforms(ts=float(tick)) for tick in range(3)]
    finally:
        room.stop()

    assert len(worked_out) == 1
    assert [{t.ts for t in edges} for edges in stamps] == [{0.0}, {1.0}, {2.0}]

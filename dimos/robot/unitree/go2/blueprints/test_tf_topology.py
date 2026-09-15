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

"""Every Go2 blueprint publishes a tf tree, not a tf graph.

tf gives each frame one parent. Two publishers writing the same child frame make
the buffer a graph instead, and a lookup then resolves by hop count rather than by
which source is authoritative, so the wrong odometry can win silently. Each
blueprint keeps the invariant a different way: nav_3d turns GO2Connection's tf off
so the static mount tree owns base_link, and the static tree is rooted at
mid360_link so it never writes the frame PointLio owns.
"""

import pytest

from dimos.core.coordination.blueprints import Blueprint
from dimos.hardware.sensors.lidar.pointlio.module import PointLio
from dimos.mapping.relocalization.module import RelocalizationModule
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_mid360_record import (
    unitree_go2_mid360_record,
)
from dimos.robot.unitree.go2.blueprints.navigation.unitree_go2_nav_3d import (
    unitree_go2_nav_3d,
    unitree_go2_nav_3d_relocalization,
)
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.robot.unitree.go2.go2_mid360_static_transforms import (
    Go2Mid360StaticTf,
    mount_transforms,
)

BLUEPRINTS = [unitree_go2_nav_3d, unitree_go2_nav_3d_relocalization, unitree_go2_mid360_record]


def _tf_children_by_publisher(blueprint: Blueprint) -> dict[str, set[str]]:
    """Child frames each tf publisher the blueprint actually enables will write."""
    odom = PoseStamped(ts=1.0, frame_id="go2_odom")
    children: dict[str, set[str]] = {}
    for atom in blueprint.blueprints:
        if atom.module is GO2Connection and atom.kwargs.get("publish_tf", True):
            children["GO2Connection"] = {t.child_frame_id for t in GO2Connection._odom_to_tf(odom)}
        if atom.module is Go2Mid360StaticTf:
            children["Go2Mid360StaticTf"] = {t.child_frame_id for t in mount_transforms()}
        if atom.module is PointLio:
            sensor_frame = atom.kwargs.get("sensor_frame_id", "mid360_link")
            children["PointLio"] = {sensor_frame}
        if issubclass(atom.module, RelocalizationModule):
            children[atom.module.__name__] = {atom.kwargs.get("map_frame", "map")}
    return children


@pytest.mark.parametrize("blueprint", BLUEPRINTS)
def test_no_frame_has_two_tf_parents(blueprint: Blueprint) -> None:
    by_publisher = _tf_children_by_publisher(blueprint)
    assert by_publisher, "blueprint publishes no tf, so this asserts nothing"
    claimed: set[str] = set()
    for publisher, frames in by_publisher.items():
        clash = claimed & frames
        assert not clash, f"{publisher} also writes {sorted(clash)}"
        claimed |= frames


def test_static_tree_does_not_write_the_pointlio_frame() -> None:
    """Rooting the mount tree at mid360_link is what keeps it off PointLio's edge."""
    assert "mid360_link" not in {t.child_frame_id for t in mount_transforms()}

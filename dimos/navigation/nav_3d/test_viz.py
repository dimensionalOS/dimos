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

import pickle

import pytest

from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.Path import Path
from dimos.navigation.nav_3d import viz

rr = pytest.importorskip("rerun")


def test_goal_placeholder_is_not_drawn() -> None:
    assert viz.render_goal(PointStamped(x=float("nan"), y=0.0, z=0.0)) is None
    assert viz.render_goal(PointStamped(x=1.0, y=2.0, z=0.0)) is not None


def test_empty_path_keeps_the_last_one_drawn() -> None:
    assert viz.render_path(Path(poses=[])) is None
    assert viz.render_path(Path(poses=[PoseStamped(), PoseStamped()])) is not None


def test_bridge_config_pickles_for_the_workers() -> None:
    static = pickle.loads(pickle.dumps(viz.nav_static(0.7, 0.3, 0.3, 0.1)))
    overrides = pickle.loads(pickle.dumps(viz.nav_visual_override(2.0, 0.08, 0.1)))
    assert len(static["world/robot_body"](rr)) == 2
    assert callable(overrides["world/surface_map"])


def test_overrides_cover_the_maps_and_the_planner_entities() -> None:
    off = viz.nav_visual_override(0.0, 0.08, 0.1)
    on = viz.nav_visual_override(2.0, 0.08, 0.1)
    assert off["world/surface_map"] is None and off["world/nodes"] is None
    assert callable(on["world/surface_map"]) and callable(on["world/node_edges"])
    assert callable(on["world/global_map"]) and callable(off["world/path"])

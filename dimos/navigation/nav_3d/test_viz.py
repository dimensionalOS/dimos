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

import numpy as np
import rerun as rr

from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.Path import Path
from dimos.navigation.nav_3d import viz


def test_goal_placeholder_is_not_drawn() -> None:
    assert viz.render_goal(PointStamped(x=float("nan"), y=0.0, z=0.0)) is None
    goal = viz.render_goal(PointStamped(x=1.0, y=2.0, z=0.0))
    assert goal is not None
    assert goal.positions.as_arrow_array().to_pylist() == [[1.0, 2.0, 0.0]]


def test_empty_path_keeps_the_last_one_drawn() -> None:
    assert viz.render_path(Path(poses=[])) is None
    path = viz.render_path(Path(poses=[PoseStamped(1.0, 0.0, 0.0), PoseStamped(2.0, 0.0, 0.0)]))
    assert path is not None
    lift = viz.PATH_Z_LIFT
    np.testing.assert_allclose(
        path.strips.as_arrow_array().to_pylist(), [[[1, 0, lift], [2, 0, lift]]], atol=1e-6
    )


def test_bridge_config_pickles_for_the_workers() -> None:
    static = pickle.loads(pickle.dumps(viz.nav_static(0.7, 0.3, 0.3, 0.1)))
    overrides = pickle.loads(pickle.dumps(viz.nav_visual_override(2.0, 0.08, 0.1)))
    assert len(static["world/robot_body"](rr)) == 2
    assert callable(overrides["world/surface_map"])

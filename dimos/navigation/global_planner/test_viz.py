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

from functools import partial
import pickle

import numpy as np
import rerun as rr

from dimos.mapping.ray_tracing.viz import render_map_region
from dimos.messages.dimos_msgs.msg import PointCloudRegion
from dimos.messages.geometry_msgs.msg import Point, PointStamped, Pose, PoseStamped
from dimos.messages.nav_msgs.msg import Path
from dimos.messages.std_msgs.msg import Header
from dimos.msgs.geometry import quaternion_from_euler
from dimos.msgs.pointcloud import pointcloud_from_xyz
from dimos.msgs.time import time_from_nanoseconds
from dimos.navigation.global_planner import viz
from dimos.navigation.global_planner.mls_planner.viz import render_surface_region
from dimos.visualization.rerun.bridge import is_keyed_by_seq


def test_goal_placeholder_is_not_drawn() -> None:
    assert (
        viz.render_goal(
            PointStamped(
                header=Header(frame_id="odom", stamp=time_from_nanoseconds(0)),
                point=Point(x=float("nan"), y=0.0, z=0.0),
            )
        )
        is None
    )
    goal = viz.render_goal(
        PointStamped(
            header=Header(frame_id="odom", stamp=time_from_nanoseconds(0)),
            point=Point(x=1.0, y=2.0, z=0.0),
        )
    )
    assert goal is not None
    assert goal.positions.as_arrow_array().to_pylist() == [[1.0, 2.0, 0.0]]


def test_empty_path_keeps_the_last_one_drawn() -> None:
    assert (
        viz.render_path(
            Path(header=Header(frame_id="odom", stamp=time_from_nanoseconds(0)), poses=[])
        )
        is None
    )
    path = viz.render_path(
        Path(
            header=Header(frame_id="odom", stamp=time_from_nanoseconds(0)),
            poses=[
                PoseStamped(
                    header=Header(frame_id="odom", stamp=time_from_nanoseconds(0)),
                    pose=Pose(
                        position=Point(x=x, y=0, z=0), orientation=quaternion_from_euler(0, 0, 0)
                    ),
                )
                for x in [1.0, 2.0]
            ],
        )
    )
    assert path is not None
    lift = viz.PATH_Z_LIFT
    np.testing.assert_allclose(
        path.strips.as_arrow_array().to_pylist(), [[[1, 0, lift], [2, 0, lift]]], atol=1e-6
    )


def test_keyed_renderers_keep_their_flag_through_the_pickle_round_trip() -> None:
    plain = pickle.loads(pickle.dumps(render_map_region))
    bound = pickle.loads(pickle.dumps(partial(render_map_region, voxel_size=0.1)))
    assert is_keyed_by_seq(plain) and is_keyed_by_seq(bound)
    assert not is_keyed_by_seq(viz.render_path)
    assert not is_keyed_by_seq(pickle.loads(pickle.dumps(partial(viz.render_path))))


def test_bridge_config_pickles_for_the_workers() -> None:
    static = pickle.loads(pickle.dumps(viz.nav_static(0.7, 0.3, 0.3, 0.1)))
    overrides = pickle.loads(pickle.dumps(viz.nav_visual_override(2.0, 0.08, 0.1)))
    assert len(static["world/robot_body"](rr)) == 2

    surface = overrides["world/surface_map"]
    assert surface.func is render_surface_region and surface.keywords["voxel_size"] == 0.08
    regions = overrides["world/map_regions"]
    assert regions.func is render_map_region
    assert regions.keywords == {"voxel_size": 0.08, "height_range": viz.HEIGHT_RANGE}
    header = Header(frame_id="map", stamp=time_from_nanoseconds(0))
    region = PointCloudRegion(
        header=header,
        region_id=(4 << 16) | 7,
        cloud=pointcloud_from_xyz(np.array([[1.0, 1.0, 0.0]], dtype=np.float32), header=header),
    )
    (path, _, static_entry) = regions(region)[0]
    assert path == "world/map_regions/4_7" and static_entry

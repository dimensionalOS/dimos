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

from dimos_generated.dimos_msgs.msg import RegionPointCloud2
from dimos_generated.geometry_msgs.msg import Point, PointStamped, Pose, PoseStamped, Quaternion
from dimos_generated.nav_msgs.msg import Path
from dimos_generated.std_msgs.msg import Header
import numpy as np
import rerun as rr

from dimos.mapping.ray_tracing.viz import render_map_region
from dimos.msgs.pointcloud import pointcloud_from_xyz
from dimos.msgs.time import time_from_seconds
from dimos.navigation.global_planner import viz
from dimos.navigation.global_planner.mls_planner.viz import render_surface_region
from dimos.visualization.rerun.bridge import is_keyed_by_region


def test_goal_placeholder_is_not_drawn() -> None:
    assert (
        viz.render_goal(
            PointStamped(
                header=Header(stamp=time_from_seconds(0.0), frame_id=""),
                point=Point(x=float("nan"), y=0.0, z=0.0),
            )
        )
        is None
    )
    goal = viz.render_goal(
        PointStamped(
            header=Header(stamp=time_from_seconds(0.0), frame_id=""),
            point=Point(x=1.0, y=2.0, z=0.0),
        )
    )
    assert goal is not None
    assert goal.positions.as_arrow_array().to_pylist() == [[1.0, 2.0, 0.0]]


def test_empty_path_keeps_the_last_one_drawn() -> None:
    assert (
        viz.render_path(Path(header=Header(stamp=time_from_seconds(0.0), frame_id=""), poses=[]))
        is None
    )
    path = viz.render_path(
        Path(
            header=Header(stamp=time_from_seconds(0.0), frame_id=""),
            poses=[
                PoseStamped(
                    header=Header(stamp=time_from_seconds(0.0), frame_id=""),
                    pose=Pose(
                        position=Point(x=1.0, y=0.0, z=0.0),
                        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                    ),
                ),
                PoseStamped(
                    header=Header(stamp=time_from_seconds(0.0), frame_id=""),
                    pose=Pose(
                        position=Point(x=2.0, y=0.0, z=0.0),
                        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                    ),
                ),
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
    assert is_keyed_by_region(plain) and is_keyed_by_region(bound)
    assert not is_keyed_by_region(viz.render_path)
    assert not is_keyed_by_region(pickle.loads(pickle.dumps(partial(viz.render_path))))


def test_bridge_config_pickles_for_the_workers() -> None:
    static = pickle.loads(pickle.dumps(viz.nav_static(0.7, 0.3, 0.3, 0.1)))
    overrides = pickle.loads(pickle.dumps(viz.nav_visual_override(2.0, 0.08, 0.1)))
    assert len(static["world/robot_body"](rr)) == 2

    surface = overrides["world/surface_map"]
    assert surface.func is render_surface_region and surface.keywords["voxel_size"] == 0.08
    regions = overrides["world/map_regions"]
    assert regions.func is render_map_region
    assert regions.keywords == {"voxel_size": 0.08, "height_range": viz.HEIGHT_RANGE}
    region = pointcloud_from_xyz(
        np.array([[1.0, 1.0, 0.0]], dtype=np.float32),
        header=Header(stamp=time_from_seconds(0.0), frame_id=""),
    )
    region = RegionPointCloud2(region_id=(4 << 16) | 7, cloud=region)
    (path, _, static_entry) = regions(region)[0]
    assert path == "world/map_regions/4_7" and static_entry

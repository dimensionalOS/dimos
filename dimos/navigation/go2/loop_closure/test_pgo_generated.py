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

import math
import sys
from types import SimpleNamespace
from unittest.mock import MagicMock

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from dimos_generated.std_msgs.msg import Header
from dimos_message_build.registry import decode as cdr_decode, encode as cdr_encode
import numpy as np
import pytest

from dimos.navigation.go2.loop_closure.pgo import Keyframe, PGOConfig, PoseGraph, _KeyPose, _PGOState
from dimos.memory.type.observation import Observation
from dimos.msgs.pointcloud import pointcloud_from_xyz, pointcloud_xyz
from dimos.msgs.time import time_from_nanoseconds, time_from_seconds


def test_generated_observation_pose_correction_applies_rotation_after_translation():
    local = TransformStamped(
        header=Header(frame_id="world_raw", stamp=time_from_seconds(1.0)),
        child_frame_id="body",
        transform=Transform(
            translation=Vector3(x=0.0, y=0.0, z=0.0),
            rotation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
    )
    optimized = TransformStamped(
        header=Header(frame_id="world_corrected", stamp=time_from_seconds(1.0)),
        child_frame_id="body",
        transform=Transform(
            translation=Vector3(x=5.0, y=0.0, z=0.0),
            rotation=Quaternion(z=math.sqrt(0.5), w=math.sqrt(0.5), x=0.0, y=0.0),
        ),
    )
    graph = PoseGraph(keyframes=(Keyframe(ts=1.0, local=local, optimized=optimized),))
    source = Observation(id=1, ts=1.0, _data="payload", pose=(1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0))
    corrected = next(graph(iter([source])))
    assert corrected.data == "payload" and corrected.ts == source.ts
    np.testing.assert_allclose(
        corrected.pose_tuple, (3.0, 1.0, 0.0, 0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)), atol=1e-9
    )
    assert source.pose_tuple == (1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0)


def test_pose_graph_retains_poseless_observations_without_constructing_correction():
    source = Observation(id=1, ts=1.0, _data="payload")
    assert list(PoseGraph()(iter([source]))) == [source]


def test_pgo_submap_places_generated_body_clouds_before_merging(monkeypatch):
    class MatrixPose:
        def __init__(self, x):
            self.value = np.eye(4)
            self.value[0, 3] = x

        def matrix(self):
            return self.value

    solver = SimpleNamespace(
        **{
            name: MagicMock(name=name)
            for name in (
                "Pose3",
                "ISAM2Params",
                "ISAM2",
                "NonlinearFactorGraph",
                "Values",
            )
        }
    )
    monkeypatch.setitem(sys.modules, "gtsam", solver)
    state = _PGOState(PGOConfig(submap_resolution=0.2))
    solver.ISAM2Params.return_value.setRelinearizeThreshold.assert_called_once_with(0.01)
    assert solver.ISAM2Params.return_value.relinearizeSkip == 1
    clouds = [
        pointcloud_from_xyz(
            np.array([[1.0, 0.0, 0.0]]), header=Header(frame_id="body", stamp=time_from_seconds(ts))
        )
        for ts in (1.0, 2.0)
    ]
    state._key_poses = [
        _KeyPose(local=MatrixPose(0), optimized=MatrixPose(offset), timestamp=ts, body_cloud=cloud)
        for cloud, offset, ts in zip(clouds, (2.0, 4.0), (1.0, 2.0), strict=True)
    ]
    before = [cdr_encode(cloud) for cloud in clouds]
    result = state._get_submap(0, 1)
    np.testing.assert_allclose(pointcloud_xyz(result), [[3.0, 0.0, 0.0], [5.0, 0.0, 0.0]])
    assert result.header.frame_id == "world_corrected"
    assert result.header.stamp == time_from_seconds(2.0)
    assert [cdr_encode(cloud) for cloud in clouds] == before


@pytest.mark.parametrize(
    "query, expected_x", [(-5.0, 0.0), (1.0, 0.0), (6.0, 5.0), (11.0, 10.0), (100.0, 10.0)]
)
def test_generated_correction_interpolates_and_clips_without_solver(query, expected_x):
    keyframes = []
    for ts, x in ((1.0, 0.0), (11.0, 10.0)):
        local = TransformStamped(
            header=Header(frame_id="world_raw", stamp=time_from_seconds(ts)),
            child_frame_id="body",
            transform=Transform(
                translation=Vector3(x=0.0, y=0.0, z=0.0),
                rotation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
        )
        optimized = TransformStamped(
            header=Header(frame_id="world_corrected", stamp=time_from_seconds(ts)),
            child_frame_id="body",
            transform=Transform(
                translation=Vector3(x=x, y=0.0, z=0.0),
                rotation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
        )
        keyframes.append(Keyframe(ts=ts, local=local, optimized=optimized))
    graph = PoseGraph(keyframes=tuple(keyframes))
    value = graph.correction_at(query)
    assert value.header.frame_id == "world_corrected" and value.child_frame_id == "world_raw"
    assert value.header.stamp == time_from_seconds(query)
    assert value.transform.translation.x == pytest.approx(expected_x)
    assert cdr_decode(cdr_encode(value), TransformStamped) == value


def test_generated_correct_preserves_exact_source_stamp_and_rejects_bad_frame():
    local = TransformStamped(
        header=Header(frame_id="world_raw", stamp=Time(sec=0, nanosec=0)),
        child_frame_id="body",
        transform=Transform(
            translation=Vector3(x=0.0, y=0.0, z=0.0),
            rotation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
    )
    optimized = TransformStamped(
        header=Header(frame_id="world_corrected", stamp=Time(sec=0, nanosec=0)),
        child_frame_id="body",
        transform=Transform(
            translation=Vector3(x=5.0, y=0.0, z=0.0),
            rotation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
    )
    graph = PoseGraph(keyframes=(Keyframe(ts=1.0, local=local, optimized=optimized),))
    source = TransformStamped(
        header=Header(frame_id="world_raw", stamp=time_from_nanoseconds(1700000000123456789)),
        child_frame_id="camera",
        transform=Transform(
            translation=Vector3(x=2.0, y=0.0, z=0.0),
            rotation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
    )
    result = graph.correct(source)
    assert result.header.stamp == source.header.stamp
    assert result.header.frame_id == "world_corrected" and result.child_frame_id == "camera"
    assert result.transform.translation.x == 7.0 and source.transform.translation.x == 2.0
    source.header.frame_id = "unrelated"
    with pytest.raises(ValueError, match="Cannot compose frames"):
        graph.correct(source)

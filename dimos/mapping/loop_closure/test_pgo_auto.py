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

from dimos_generated.geometry_msgs.msg import Transform, TransformStamped, Vector3
from dimos_generated.std_msgs.msg import Header
import numpy as np
import pytest

from dimos.mapping.loop_closure.pgo_auto import (
    FRAME_BODY,
    FRAME_WORLD_CORRECTED,
    FRAME_WORLD_RAW,
    PGO,
    Keyframe,
    apply_corrections,
    keyframes_to_corrections,
    make_interpolator,
)
from dimos.memory.store.memory import MemoryStore
from dimos.memory.type.observation import Observation
from dimos.msgs.geometry import quaternion_from_euler
from dimos.msgs.pointcloud import pointcloud_from_xyz
from dimos.msgs.time import time_from_seconds, to_seconds


def _pose(x: float, frame: str, ts: float) -> TransformStamped:
    return TransformStamped(
        header=Header(frame_id=frame, stamp=time_from_seconds(ts)),
        child_frame_id=FRAME_BODY,
        transform=Transform(translation=Vector3(x=x), rotation=quaternion_from_euler(0, 0, 0)),
    )


def test_generated_corrections_preserve_frames_time_and_nearest_neighbor() -> None:
    with MemoryStore() as store:
        keyframes = store.stream("keyframes", Keyframe)
        keyframes.append(
            Keyframe(0.0, _pose(1, FRAME_WORLD_RAW, 0), _pose(3, FRAME_WORLD_CORRECTED, 0)), ts=0
        )
        keyframes.append(
            Keyframe(2.0, _pose(2, FRAME_WORLD_RAW, 2), _pose(6, FRAME_WORLD_CORRECTED, 2)), ts=2
        )
        corrections = keyframes_to_corrections(keyframes)
        lookup = make_interpolator(corrections)
        for ts, expected in [(-1, 2), (0.9, 2), (1.1, 4), (3, 4)]:
            correction = TransformStamped.decode(lookup(ts).encode())
            assert correction.transform.translation.x == expected
            assert correction.header.frame_id == FRAME_WORLD_CORRECTED
            assert correction.child_frame_id == FRAME_WORLD_RAW
            assert to_seconds(correction.header.stamp) == ts
        stream = store.stream("values", str)
        stream.append("payload", ts=0, pose=(1, 0, 0, 0, 0, 0, 1))
        stream.append("no pose", ts=2)
        output = list(apply_corrections(stream, corrections))
        assert output[0].data == "payload"
        assert output[0].pose_tuple == (3, 0, 0, 0, 0, 0, 1)
        assert output[0].ts == 0
        assert output[1].pose is None
        assert stream.first().pose_tuple == (1, 0, 0, 0, 0, 0, 1)


def test_empty_pgo_has_generated_identity_correction() -> None:
    pytest.importorskip("gtsam")
    graph = next(PGO()(iter(()))).data
    assert not graph.keyframes and not graph.loops
    correction = graph.correction_at(0)
    assert to_seconds(correction.header.stamp) == 0
    assert correction.transform.rotation.w == 1
    result = graph.correct(_pose(5, FRAME_WORLD_RAW, 0))
    assert result.transform.translation.x == 5
    assert result.header.frame_id == FRAME_WORLD_CORRECTED


def test_pgo_consumes_generated_cloud_without_mutating_input() -> None:
    pytest.importorskip("gtsam")
    cloud = pointcloud_from_xyz(
        np.array([[1.0, 0, 0], [1, 1, 0], [1, 0, 1]]),
        header=Header(frame_id=FRAME_WORLD_RAW, stamp=time_from_seconds(12.25)),
    )
    before = cloud.encode()
    quaternion = quaternion_from_euler(0, 0, 0.1)
    observation = Observation(
        ts=12.25,
        _data=cloud,
        pose=(1, 0, 0, quaternion.x, quaternion.y, quaternion.z, quaternion.w),
    )
    graph = next(PGO()(iter([observation]))).data
    assert len(graph.keyframes) == 1
    assert graph.keyframes[0].ts == 12.25
    assert graph.keyframes[0].local.header.frame_id == FRAME_WORLD_RAW
    correction = graph.correction_at(12.25)
    assert correction.transform.translation.x == pytest.approx(0, abs=1e-6)
    assert correction.transform.rotation.w == pytest.approx(1)
    assert cloud.encode() == before

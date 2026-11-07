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

import pytest

from dimos.autoresearch.truth_scene import TruthObject, TruthSceneModule
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.robot.manipulators.damiao._damiao_test_helpers import LocalTransport


def _transform(name: str, x: float, y: float, z: float) -> Transform:
    return Transform(Vector3(x, y, z), Quaternion(), "world", name, ts=5.0)


def test_scan_scene_answers_world_boxes_for_the_named_bodies() -> None:
    module = TruthSceneModule(
        objects={
            "cube": TruthObject(size=(0.04, 0.04, 0.04)),
            "bowl": TruthObject(size=(0.10, 0.10, 0.04), centre=(0.0, 0.0, 0.02)),
        }
    )
    poses: LocalTransport[TFMessage] = LocalTransport()
    module.body_poses.transport = poses
    module.start()
    try:
        assert module.scan_scene(["cube"]).detections_length == 0

        poses.publish(
            TFMessage(
                _transform("cube", 0.26, 0.11, 0.02),
                _transform("bowl", 0.36, 0.0, 0.0),
                _transform("cylinder", 0.26, -0.11, 0.04),
            )
        )
        found = module.scan_scene(["red cube", "the bowl", "spoon"])

        assert found.header.frame_id == "world" and found.ts == 5.0
        assert [d.id for d in found.detections] == ["cube", "bowl"]
        assert [d.results[0].hypothesis.class_id for d in found.detections] == ["cube", "bowl"]
        cube, bowl = found.detections
        assert (cube.bbox.center.position.x, cube.bbox.center.position.z) == (0.26, 0.02)
        assert (cube.bbox.size.x, cube.bbox.size.y, cube.bbox.size.z) == (0.04, 0.04, 0.04)
        assert bowl.bbox.center.position.z == pytest.approx(0.02)
        assert bowl.bbox.size.z == 0.04
        # Without words every body with a box answers; the cylinder has none.
        assert [d.id for d in module.scan_scene(None).detections] == ["cube", "bowl"]
        assert module.get_object_pointcloud_by_name("cube") is None
    finally:
        module.stop()

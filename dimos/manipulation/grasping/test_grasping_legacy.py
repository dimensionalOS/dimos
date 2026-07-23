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

import inspect
from types import SimpleNamespace
from typing import Any, cast, get_type_hints

from dimos.core.stream import Out
from dimos.manipulation.grasping.grasp_gen_spec import GraspGenSpec
from dimos.manipulation.grasping.grasping import GraspingModule
from dimos.manipulation.grasping.legacy_grasp_gen_spec import LegacyGraspGenSpec
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseArray import PoseArray
from dimos.msgs.std_msgs.Header import Header


class _Scene:
    def __init__(self, object_cloud: object, scene_cloud: object) -> None:
        self.object_cloud = object_cloud
        self.scene_cloud = scene_cloud
        self.scene_exclusion: str | None = "unset"

    def get_object_pointcloud_by_name(self, name: str) -> object:
        return self.object_cloud

    def get_object_pointcloud_by_object_id(self, object_id: str) -> object:
        return self.object_cloud

    def get_full_scene_pointcloud(
        self,
        exclude_object_id: str | None = None,
        depth_trunc: float = 2,
        voxel_size: float = 0.01,
    ) -> object:
        self.scene_exclusion = exclude_object_id
        return self.scene_cloud


class _Backend:
    def __init__(self, result: PoseArray | None) -> None:
        self.result = result
        self.calls: list[tuple[object, object | None]] = []

    def generate_grasps(
        self, pointcloud: object, scene_pointcloud: object | None = None
    ) -> PoseArray | None:
        self.calls.append((pointcloud, scene_pointcloud))
        return self.result


def _module(backend: _Backend, scene: _Scene) -> tuple[GraspingModule, list[PoseArray]]:
    module = GraspingModule.__new__(GraspingModule)
    published: list[PoseArray] = []
    untyped_module = cast("Any", module)
    untyped_module._grasp_gen = backend
    untyped_module._scene_registration = scene
    untyped_module.grasps = SimpleNamespace(publish=published.append)
    untyped_module._format_grasp_result = lambda grasps, object_name: "generated"
    return module, published


def test_legacy_shapes_and_lean_spec() -> None:
    assert list(inspect.signature(GraspGenSpec.propose_grasps).parameters) == [
        "self",
        "object_pointcloud",
    ]
    assert not hasattr(GraspGenSpec, "generate_grasps")
    assert list(inspect.signature(LegacyGraspGenSpec.generate_grasps).parameters) == [
        "self",
        "pointcloud",
        "scene_pointcloud",
    ]
    assert get_type_hints(GraspingModule)["grasps"] == Out[PoseArray]


def test_default_collision_filter_fetches_scene_and_publishes() -> None:
    object_cloud, scene_cloud = object(), object()
    result = PoseArray(cast("Any", Header)(1.0, "base"), [cast("Any", Pose)(0, 0, 0)])
    backend = _Backend(result)
    scene = _Scene(object_cloud, scene_cloud)
    module, published = _module(backend, scene)

    assert module.generate_grasps(object_id="object-id") == "generated"
    assert backend.calls == [(object_cloud, scene_cloud)]
    assert scene.scene_exclusion == "object-id"
    assert published == [result]


def test_collision_filter_false_passes_no_scene_and_none_is_not_published() -> None:
    object_cloud, scene_cloud = object(), object()
    backend = _Backend(None)
    scene = _Scene(object_cloud, scene_cloud)
    module, published = _module(backend, scene)

    assert module.generate_grasps(filter_collisions=False) == "No grasps generated for 'object'"
    assert backend.calls == [(object_cloud, None)]
    assert scene.scene_exclusion == "unset"
    assert published == []

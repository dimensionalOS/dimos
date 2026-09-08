# Copyright 2025-2026 Dimensional Inc.
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

from collections.abc import Callable, Iterator
from types import SimpleNamespace
from typing import Any
from unittest.mock import MagicMock

import pytest

from dimos.memory.store.memory import MemoryStore
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.perception.experimental.object_scene_registration import (
    ObjectSceneRegistrationModule,
)
from dimos.perception.memory.types import Localization, LocalizePolicy

ModuleFactory = Callable[..., ObjectSceneRegistrationModule]


def _localization(
    *,
    reason: str | None = None,
    frame_id: str = "world",
    cloud_frame: str = "world",
    last_seen: float = 99.0,
) -> Localization:
    cloud = MagicMock(spec=PointCloud2)
    cloud.frame_id = cloud_frame
    return Localization(
        instance_id="query-0",
        semantic_score=0.9,
        identity_score=0.5,
        ambiguity_margin=1.0,
        position_world_xyz=(0.1, 0.2, 0.3),
        orientation_world_xyzw=(0.0, 0.0, 0.0, 1.0),
        frame_id=frame_id,
        support=None,
        pose_timestamp=last_seen,
        geometry_timestamp=last_seen,
        last_seen_timestamp=last_seen,
        point_cloud=cloud,
        cloud_mode="latest_visible",
        coverage=0.5,
        n_views=2,
        reason=reason,
    )


@pytest.fixture
def module_factory() -> Iterator[ModuleFactory]:
    modules: list[ObjectSceneRegistrationModule] = []

    def create(**kwargs: Any) -> ObjectSceneRegistrationModule:
        module = ObjectSceneRegistrationModule(**kwargs)
        modules.append(module)
        return module

    yield create

    for module in reversed(modules):
        module.stop()


def _running_module(
    results: list[Localization | None], module_factory: ModuleFactory
) -> tuple[ObjectSceneRegistrationModule, MagicMock]:
    module = module_factory()
    store = MagicMock()
    store.streams.camera_info.exists.return_value = True
    store.streams.color_image.exists.return_value = True
    store.streams.color_image.last.return_value = SimpleNamespace(ts=100.0)
    detector = MagicMock()
    detector.localize.return_value = results
    snapshot = MagicMock()
    snapshot.exists.return_value = True
    index = MagicMock()
    index.time_range.return_value.materialize.return_value = snapshot
    module._store = store
    module._detector = detector
    module._index = index
    return module, detector


def test_localize_objects_batches_and_rejects_ambiguous_results(
    module_factory: ModuleFactory,
) -> None:
    accepted = _localization()
    ambiguous = _localization(reason="ambiguous_between_coexisting_candidates")
    module, detector = _running_module([accepted, ambiguous, None], module_factory)

    result = module.localize_objects(["cup", "bowl", "fork"])

    assert result == [accepted, None, None]
    detector.localize.assert_called_once_with(
        module._store,
        ["cup", "bowl", "fork"],
        index=module._index.time_range.return_value.materialize.return_value,
        require_pose=True,
        world_frame="world",
        optical_frame="camera_color_optical_frame",
        tf_tolerance=0.12,
        policy=LocalizePolicy(),
    )


@pytest.mark.parametrize("prompts", [[], [""], ["cup", " cup "]])
def test_localize_objects_rejects_invalid_prompts(
    prompts: list[str], module_factory: ModuleFactory
) -> None:
    module, _ = _running_module([], module_factory)

    with pytest.raises(ValueError):
        module.localize_objects(prompts)


def test_localize_objects_requires_ready_index(module_factory: ModuleFactory) -> None:
    module = module_factory()

    with pytest.raises(RuntimeError, match="not running"):
        module.localize_objects(["cup"])


def test_localize_objects_rejects_invalid_batch_shape(module_factory: ModuleFactory) -> None:
    module, detector = _running_module([], module_factory)
    detector.localize.return_value = _localization()

    with pytest.raises(RuntimeError, match="invalid batch result"):
        module.localize_objects(["cup"])


@pytest.mark.parametrize(
    ("localization", "message"),
    [
        (_localization(frame_id="camera"), "Localization frame mismatch"),
        (_localization(cloud_frame="camera"), "point cloud frame mismatch"),
    ],
)
def test_localize_objects_rejects_frame_mismatch(
    localization: Localization, message: str, module_factory: ModuleFactory
) -> None:
    module, _ = _running_module([localization], module_factory)

    with pytest.raises(RuntimeError, match=message):
        module.localize_objects(["cup"])


def test_localize_objects_drops_stale_results(module_factory: ModuleFactory) -> None:
    module, _ = _running_module([_localization(last_seen=60.0)], module_factory)

    assert module.localize_objects(["cup"]) == [None]


def test_record_tf_splits_messages_for_timestamped_lookup(
    module_factory: ModuleFactory,
) -> None:
    module = module_factory(max_tf_observations=2)
    store = MemoryStore(max_size=10)
    module._store = store
    module._create_streams(store)
    first = Transform(ts=1.0, frame_id="world", child_frame_id="link7")
    second = Transform(ts=2.0, frame_id="link7", child_frame_id="camera_link")
    third = Transform(ts=3.0, frame_id="camera_link", child_frame_id="camera_color_frame")

    module._record_tf(TFMessage(first, second, third))

    observations = store.streams.tf.to_list()
    assert [observation.ts for observation in observations] == [2.0, 3.0]
    assert [len(observation.data.transforms) for observation in observations] == [1, 1]


def test_non_world_target_is_rejected_before_model_start(module_factory: ModuleFactory) -> None:
    module = module_factory(target_frame="base_link")

    with pytest.raises(ValueError, match="target_frame='world'"):
        module.start()

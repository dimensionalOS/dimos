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

import time
from typing import TYPE_CHECKING

import pytest

from dimos.models.vl.moondream import MoondreamVlModel
from dimos.msgs.sensor_msgs.Image import Image

if TYPE_CHECKING:
    from dimos.models.vl.base import VlModel


@pytest.mark.parametrize(
    "model_class,model_name",
    [
        (MoondreamVlModel, "Moondream"),
    ],
)
def test_vlm_query_batch(model_class: "type[VlModel]", model_name: str) -> None:
    """Test query_batch optimization - multiple images, same query."""
    from dimos.utils.testing.legacy_pickle import LegacyPickleStore

    # Load 5 frames at 1-second intervals using LegacyPickleStore
    replay = LegacyPickleStore[Image]("unitree_go2_office_walk2/video")
    images = [replay.find_closest_seek(i).to_rgb() for i in range(0, 10, 2)]

    print(f"\nTesting {model_name} query_batch with {len(images)} images")

    model: VlModel = model_class()
    model.start()

    query = "Describe this image in a short sentence"

    # Sequential queries (print as they come in)
    print("\nSequential queries:")
    sequential_results = []
    start_time = time.time()
    for i, img in enumerate(images):
        result = model.query(img, query)
        sequential_results.append(result)
        print(f"  [{i}] {result[:120]}...")
    sequential_time = time.time() - start_time
    print(f"  Time: {sequential_time:.3f}s")

    # Batched queries (pre-encode all images)
    print("\nBatched queries (query_batch):")
    start_time = time.time()
    batch_results = model.query_batch(images, query)
    batch_time = time.time() - start_time
    for i, result in enumerate(batch_results):
        print(f"  [{i}] {result[:120]}...")
    print(f"  Time: {batch_time:.3f}s")

    speedup_pct = (sequential_time - batch_time) / sequential_time * 100
    print(f"\nSpeedup: {speedup_pct:.1f}%")

    # Verify results are valid strings
    assert len(batch_results) == len(images)
    assert all(isinstance(r, str) and len(r) > 0 for r in batch_results)

    model.stop()

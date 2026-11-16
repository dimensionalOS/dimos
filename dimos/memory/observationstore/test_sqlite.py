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

import warnings

import numpy as np
import pytest

from dimos.models.embedding.base import Embedding


@pytest.mark.parametrize(
    ("operation", "expected"),
    [
        pytest.param(lambda q, e: q.count(), 0, id="count"),
        pytest.param(lambda q, e: q.to_list(), [], id="to_list"),
        pytest.param(lambda q, e: q.search(e).to_list(), [], id="search"),
    ],
)
def test_near_warns_when_stream_has_no_attached_poses(sqlite_store, operation, expected):
    stream = sqlite_store.stream("lidar", float)
    embedding = Embedding(vector=np.array([1.0, 0.0], dtype=np.float32))
    stream.append(1.0, ts=1.0, embedding=embedding)
    # A different posed stream must not hide missing poses on this stream.
    sqlite_store.stream("posed", float).append(1.0, ts=1.0, pose=(1, 2, 3))

    with pytest.warns(UserWarning, match="no attached poses.*pose-fill.*--target lidar"):
        assert operation(stream.near((1, 2, 3), 1.0), embedding) == expected


def test_near_does_not_warn_for_empty_spatial_match(sqlite_store):
    stream = sqlite_store.stream("lidar", float)
    stream.append(1.0, ts=1.0, pose=(1, 2, 3))

    with warnings.catch_warnings(record=True) as emitted:
        warnings.simplefilter("always")
        assert stream.near((10, 20, 30), 1.0).to_list() == []
    assert not emitted


def test_near_does_not_warn_for_empty_stream(sqlite_store):
    stream = sqlite_store.stream("lidar", float)

    with warnings.catch_warnings(record=True) as emitted:
        warnings.simplefilter("always")
        assert stream.near((1, 2, 3), 1.0).count() == 0
    assert not emitted

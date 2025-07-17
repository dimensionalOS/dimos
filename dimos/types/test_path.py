# Copyright 2025 Dimensional Inc.
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

import numpy as np
import pytest

from dimos.msgs.geometry_msgs import Vector3
from dimos.types.path import Path


@pytest.fixture
def path():
    return Path([(1, 2, 3), (4, 5, 6), (7, 8, 9)])


@pytest.fixture
def empty_path():
    return Path()


def test_init(path):
    assert path.length() == 10.392304845413264
    assert len(path) == 3
    assert np.array_equal(path[1], [4.0, 5.0, 6.0])


def test_init_empty():
    empty = Path()
    assert len(empty) == 0
    assert empty.length() == 0.0


def test_init_Vector3():
    points = map((lambda p: Vector3(p)), [[1, 2], [3, 4], [5, 6]])
    path = Path(points)
    print(path)


def test_init_numpy_array():
    points = np.array([[1, 2], [3, 4], [5, 6]])
    path = Path(points)
    assert len(path) == 3
    assert np.array_equal(path[0], [1.0, 2.0])


def test_add_path(path):
    path2 = Path([(10, 11, 12)])
    result = path + path2
    assert len(result) == 4
    assert np.array_equal(result[3], [10.0, 11.0, 12.0])


def test_add_point(path):
    result = path + (10, 11, 12)
    assert len(result) == 4
    assert np.array_equal(result[3], [10.0, 11.0, 12.0])


def test_append(path):
    original_len = len(path)
    path.append((10, 11, 12))
    assert len(path) == original_len + 1
    assert np.array_equal(path[-1], [10.0, 11.0, 12.0])


def test_extend(path):
    path2 = Path([(10, 11, 12), (13, 14, 15)])
    original_len = len(path)
    path.extend(path2)
    assert len(path) == original_len + 2
    assert np.array_equal(path[-1], [13.0, 14.0, 15.0])


def test_insert(path):
    path.insert(1, (10, 11, 12))
    assert len(path) == 4
    assert np.array_equal(path[1], [10.0, 11.0, 12.0])
    assert np.array_equal(path[2], [4.0, 5.0, 6.0])  # Original point shifted


def test_remove(path):
    removed = path.remove(1)
    assert len(path) == 2
    assert np.array_equal(removed, [4.0, 5.0, 6.0])
    assert np.array_equal(path[1], [7.0, 8.0, 9.0])  # Next pointhey ca shifted down


def test_clear(path):
    path.clear()
    assert len(path) == 0


def test_resample(path):
    resampled = path.resample(2.0)
    assert len(resampled) >= 2
    # Resampling can create more points than original * 2 for complex paths
    assert len(resampled) > 0


def test_simplify(path):
    simplified = path.simplify(0.1)
    assert len(simplified) <= len(path)
    assert len(simplified) >= 2


def test_smooth(path):
    smoothed = path.smooth(0.5, 1)
    assert len(smoothed) == len(path)
    assert np.array_equal(smoothed[0], path[0])  # First point unchanged
    assert np.array_equal(smoothed[-1], path[-1])  # Last point unchanged


def test_nearest_point_index(path):
    idx = path.nearest_point_index((4, 5, 6))
    assert idx == 1

    idx = path.nearest_point_index((1, 2, 3))
    assert idx == 0


def test_nearest_point_index_empty():
    empty = Path()
    with pytest.raises(ValueError):
        empty.nearest_point_index((1, 2, 3))


def test_reverse(path):
    reversed_path = path.reverse()
    assert len(reversed_path) == len(path)
    assert np.array_equal(reversed_path[0], path[-1])
    assert np.array_equal(reversed_path[-1], path[0])


def test_getitem_slice(path):
    slice_path = path[1:3]
    assert isinstance(slice_path, Path)
    assert len(slice_path) == 2
    assert np.array_equal(slice_path[0], [4.0, 5.0, 6.0])


def test_get_vector(path):
    vector = path.get_vector(1)
    assert isinstance(vector, Vector3)
    assert vector == Vector3([4.0, 5.0, 6.0])


def test_head_tail_last(path):
    head = path.head()
    assert isinstance(head, Vector3)
    assert head == Vector3([1.0, 2.0, 3.0])

    last = path.last()
    assert isinstance(last, Vector3)
    assert last == Vector3([7.0, 8.0, 9.0])

    tail = path.tail()
    assert isinstance(tail, Path)
    assert len(tail) == 2
    assert np.array_equal(tail[0], [4.0, 5.0, 6.0])


def test_head_tail_last_empty():
    empty = Path()
    assert empty.head() is None
    assert empty.last() is None
    assert empty.tail() is None


def test_iter(path):
    arrays = list(path)
    assert len(arrays) == 3
    assert all(isinstance(arr, np.ndarray) for arr in arrays)
    assert np.array_equal(arrays[0], [1.0, 2.0, 3.0])


def test_vectors(path):
    vectors = list(path.vectors())
    assert len(vectors) == 3
    assert all(isinstance(v, Vector3) for v in vectors)
    assert vectors[0] == Vector3([1.0, 2.0, 3.0])


def test_repr(path):
    repr_str = repr(path)
    assert "Path" in repr_str
    assert "3 Points" in repr_str


def test_ipush(path):
    new_path = path.ipush((10, 11, 12))
    assert len(new_path) == 4
    assert len(path) == 3  # Original unchanged
    assert np.array_equal(new_path[-1], [10.0, 11.0, 12.0])


def test_iclip_tail(path):
    clipped = path.iclip_tail(2)
    assert len(clipped) == 2
    assert np.array_equal(clipped[0], [4.0, 5.0, 6.0])
    assert np.array_equal(clipped[1], [7.0, 8.0, 9.0])


def test_iclip_tail_negative():
    path = Path([(1, 2, 3)])
    with pytest.raises(ValueError):
        path.iclip_tail(-1)


def test_serialize(path):
    serialized = path.serialize()
    assert isinstance(serialized, dict)
    assert serialized["type"] == "path"
    assert serialized["points"] == [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]]


def test_as_vectors(path):
    vectors = path.as_vectors()
    assert len(vectors) == 3
    assert all(isinstance(v, Vector3) for v in vectors)
    assert vectors[0] == Vector3([1.0, 2.0, 3.0])


def test_points_property(path):
    points = path.points
    assert isinstance(points, np.ndarray)
    assert points.shape == (3, 3)
    assert np.array_equal(points[0], [1.0, 2.0, 3.0])


# def test_lcm_encode_decode(path):
#    print(path.lcm_encode())

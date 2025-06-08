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

from dimos.types.path import Path
from dimos.types.vector import Vector
import numpy as np
import pytest


def test_path_initialization():
    # Test empty path
    empty_path = Path()
    assert len(empty_path) == 0

    # Test with None
    none_path = Path(None)
    assert len(none_path) == 0

    # Test with Vector objects
    vectors = [Vector(1, 2), Vector(3, 4), Vector(5, 6)]
    path_vectors = Path(vectors)
    assert len(path_vectors) == 3
    assert np.array_equal(path_vectors[0], np.array([1, 2]))
    assert np.array_equal(path_vectors[2], np.array([5, 6]))

    # Test with tuples
    tuples = [(1, 2), (3, 4), (5, 6)]
    path_tuples = Path(tuples)
    assert len(path_tuples) == 3
    assert np.array_equal(path_tuples[0], np.array([1, 2]))

    # Test with numpy array (2D)
    np_array = np.array([[1, 2], [3, 4], [5, 6]])
    path_numpy = Path(np_array)
    assert len(path_numpy) == 3
    assert np.array_equal(path_numpy[0], np.array([1, 2]))

    # Test with 3D numpy array (2D -> multiple 3D points)
    np_array_3d = np.array([[1, 2, 3], [4, 5, 6]])
    path_3d = Path(np_array_3d)
    assert len(path_3d) == 2
    assert np.array_equal(path_3d[0], np.array([1, 2, 3]))
    assert np.array_equal(path_3d[1], np.array([4, 5, 6]))

    # Test with 1D numpy array (single point)
    single_point = np.array([1, 2, 3])
    path_single = Path(single_point)
    assert len(path_single) == 1
    assert np.array_equal(path_single[0], np.array([1, 2, 3]))

    # Test with empty 1D numpy array
    empty_array = np.array([])
    path_empty_array = Path(empty_array)
    assert len(path_empty_array) == 0


def test_path_properties():
    path = Path([(1, 2), (3, 4), (5, 6)])

    # Test points property
    points = path.points
    assert isinstance(points, np.ndarray)
    assert points.shape == (3, 2)

    # Test as_vectors
    vectors = path.as_vectors()
    assert len(vectors) == 3
    assert all(isinstance(v, Vector) for v in vectors)
    assert vectors[0] == Vector(1, 2)


def test_path_serialization():
    path = Path([(1, 2), (3, 4)])
    serialized = path.serialize()
    assert serialized["type"] == "path"
    assert serialized["points"] == [[1.0, 2.0], [3.0, 4.0]]


def test_path_manipulation():
    path = Path()

    # Test append
    path.append(Vector(1, 2))
    assert len(path) == 1
    assert np.array_equal(path[0], np.array([1, 2]))

    path.append((3, 4))
    assert len(path) == 2
    assert np.array_equal(path[1], np.array([3, 4]))

    # Test extend with list
    path.extend([(5, 6), (7, 8)])
    assert len(path) == 4
    assert np.array_equal(path[3], np.array([7, 8]))

    # Test extend with another Path
    other_path = Path([(9, 10), (11, 12)])
    path.extend(other_path)
    assert len(path) == 6
    assert np.array_equal(path[5], np.array([11, 12]))

    # Test insert
    path.insert(2, (2.5, 3.5))
    assert len(path) == 7
    assert np.array_equal(path[2], np.array([2.5, 3.5]))

    # Test remove
    removed = path.remove(2)
    assert len(path) == 6
    assert np.array_equal(removed, np.array([2.5, 3.5]))

    # Test clear
    path.clear()
    assert len(path) == 0


def test_path_indexing_and_slicing():
    path = Path([(1, 2), (3, 4), (5, 6), (7, 8)])

    # Test indexing
    assert np.array_equal(path[0], np.array([1, 2]))
    assert np.array_equal(path[-1], np.array([7, 8]))

    # Test slicing
    sub_path = path[1:3]
    assert isinstance(sub_path, Path)
    assert len(sub_path) == 2
    assert np.array_equal(sub_path[0], np.array([3, 4]))

    # Test get_vector
    vector = path.get_vector(1)
    assert isinstance(vector, Vector)
    assert vector == Vector(3, 4)


def test_path_head_tail_last():
    path = Path([(1, 2), (3, 4), (5, 6)])

    # Test head
    head = path.head()
    assert isinstance(head, Vector)
    assert head == Vector(1, 2)

    # Test last
    last = path.last()
    assert isinstance(last, Vector)
    assert last == Vector(5, 6)

    # Test tail
    tail = path.tail()
    assert isinstance(tail, Path)
    assert len(tail) == 2
    assert tail.head() == Vector(3, 4)

    # Test empty path
    empty_path = Path()
    assert empty_path.head() is None
    assert empty_path.last() is None
    assert empty_path.tail() is None

    # Test single point path
    single_path = Path([(1, 2)])
    assert single_path.tail() is None


def test_path_length():
    # Test empty path
    empty_path = Path()
    assert empty_path.length() == 0.0

    # Test single point
    single_path = Path([(1, 2)])
    assert single_path.length() == 0.0

    # Test simple path
    path = Path([(0, 0), (3, 4), (6, 8)])  # 5 + 5 = 10
    assert abs(path.length() - 10.0) < 1e-10


def test_path_resample():
    # Create a simple path
    path = Path([(0, 0), (10, 0), (20, 0)])

    # Resample with 5-unit spacing
    resampled = path.resample(5.0)
    assert len(resampled) >= 4  # Should have points at 0, 5, 10, 15, 20

    # Test edge cases
    empty_path = Path()
    assert len(empty_path.resample(1.0)) == 0

    single_path = Path([(1, 2)])
    resampled_single = single_path.resample(1.0)
    assert len(resampled_single) == 1


def test_path_simplify():
    # Create a path with redundant points
    path = Path([(0, 0), (1, 0.1), (2, 0), (3, 0.1), (4, 0)])

    # Simplify with tolerance
    simplified = path.simplify(0.2)
    assert len(simplified) < len(path)

    # Test edge cases
    empty_path = Path()
    assert len(empty_path.simplify(1.0)) == 0

    single_path = Path([(1, 2)])
    simplified_single = single_path.simplify(1.0)
    assert len(simplified_single) == 1


def test_path_smooth():
    # Create a jagged path
    path = Path([(0, 0), (1, 2), (2, 0), (3, 2), (4, 0)])

    # Smooth the path
    smoothed = path.smooth(weight=0.5, iterations=1)
    assert len(smoothed) == len(path)

    # First and last points should remain unchanged
    assert np.array_equal(smoothed[0], path[0])
    assert np.array_equal(smoothed[-1], path[-1])

    # Test edge cases
    empty_path = Path()
    assert len(empty_path.smooth()) == 0

    single_path = Path([(1, 2)])
    smoothed_single = single_path.smooth()
    assert len(smoothed_single) == 1


def test_path_nearest_point():
    path = Path([(0, 0), (1, 0), (2, 0), (3, 0)])

    # Test nearest point
    nearest_idx = path.nearest_point_index((1.3, 0.2))
    assert nearest_idx == 1  # Should be closest to (1, 0)

    nearest_idx = path.nearest_point_index(Vector(2.7, 0.1))
    assert nearest_idx == 3  # Should be closest to (3, 0)

    # Test with empty path
    empty_path = Path()
    with pytest.raises(ValueError):
        empty_path.nearest_point_index((1, 2))


def test_path_reverse():
    path = Path([(1, 2), (3, 4), (5, 6)])
    reversed_path = path.reverse()

    assert len(reversed_path) == len(path)
    assert np.array_equal(reversed_path[0], path[-1])
    assert np.array_equal(reversed_path[-1], path[0])


def test_path_iteration():
    path = Path([(1, 2), (3, 4), (5, 6)])

    points = list(path)
    assert len(points) == 3
    assert all(isinstance(p, np.ndarray) for p in points)
    assert np.array_equal(points[0], np.array([1, 2]))


def test_path_immutable_operations():
    path = Path([(1, 2), (3, 4)])

    # Test ipush
    new_path = path.ipush((5, 6))
    assert len(path) == 2  # Original unchanged
    assert len(new_path) == 3
    assert np.array_equal(new_path[-1], np.array([5, 6]))

    # Test iclip_tail
    long_path = Path([(1, 2), (3, 4), (5, 6), (7, 8), (9, 10)])
    clipped = long_path.iclip_tail(3)
    assert len(clipped) == 3
    assert np.array_equal(clipped[0], np.array([5, 6]))

    # Test with max_len = 0
    empty_clipped = long_path.iclip_tail(0)
    assert len(empty_clipped) == 0

    # Test with negative max_len
    with pytest.raises(ValueError):
        long_path.iclip_tail(-1)


def test_path_addition():
    path = Path([(1, 2), (3, 4)])
    new_path = path + (5, 6)

    assert len(path) == 2  # Original unchanged
    assert len(new_path) == 3
    assert np.array_equal(new_path[-1], np.array([5, 6]))


def test_path_repr():
    path = Path([(1, 2), (3, 4), (5, 6)])
    repr_str = repr(path)
    assert "Path" in repr_str
    assert "3 Points" in repr_str


def test_path_edge_cases():
    # Test with very small numbers
    path = Path([(1e-10, 1e-10), (2e-10, 2e-10)])
    assert len(path) == 2

    # Test with large numbers
    path = Path([(1e10, 1e10), (2e10, 2e10)])
    assert len(path) == 2

    # Test with negative numbers
    path = Path([(-1, -2), (-3, -4)])
    assert len(path) == 2
    assert np.array_equal(path[0], np.array([-1, -2]))


def test_path_3d():
    # Test with 3D points
    path_3d = Path([(1, 2, 3), (4, 5, 6), (7, 8, 9)])
    assert len(path_3d) == 3
    assert path_3d.points.shape == (3, 3)

    # Test operations on 3D path
    length = path_3d.length()
    assert length > 0

    nearest_idx = path_3d.nearest_point_index((2, 3, 4))
    assert isinstance(nearest_idx, int)

    # Test 3D numpy array initialization and operations
    np_3d_path = Path(np.array([[0, 0, 0], [3, 4, 0]]))  # 3-4-5 triangle in 3D
    assert len(np_3d_path) == 2
    assert abs(np_3d_path.length() - 5.0) < 1e-10

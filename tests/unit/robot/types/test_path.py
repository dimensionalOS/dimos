import pytest
import numpy as np
from dimos.types.path import Path
from dimos.types.vector import Vector


class TestPathInit:
    def test_init_empty(self):
        path = Path()
        assert len(path) == 0
        assert path.points.shape == (0, 0)

    def test_init_with_vectors(self):
        points = [Vector(1, 2), Vector(3, 4), Vector(5, 6)]
        path = Path(points)
        assert len(path) == 3
        assert np.array_equal(path.points, np.array([[1, 2], [3, 4], [5, 6]]))

    def test_init_with_tuples(self):
        points = [(1, 2), (3, 4), (5, 6)]
        path = Path(points)
        assert len(path) == 3
        assert np.array_equal(path.points, np.array([[1, 2], [3, 4], [5, 6]]))

    def test_init_with_numpy_array(self):
        points = np.array([[1, 2], [3, 4], [5, 6]])
        path = Path(points)
        assert len(path) == 3
        assert np.array_equal(path.points, points)


class TestPathProperties:
    def test_as_vectors(self):
        path = Path([(1, 2), (3, 4), (5, 6)])
        vectors = path.as_vectors()
        assert len(vectors) == 3
        assert all(isinstance(v, Vector) for v in vectors)
        assert vectors[0] == Vector(1, 2)
        assert vectors[1] == Vector(3, 4)
        assert vectors[2] == Vector(5, 6)


class TestPathModification:
    def test_append(self):
        path = Path([(1, 2), (3, 4)])
        path.append(Vector(5, 6))
        assert len(path) == 3
        assert np.array_equal(path.points, np.array([[1, 2], [3, 4], [5, 6]]))

        # Append to empty path
        empty_path = Path()
        empty_path.append(Vector(1, 2))
        assert len(empty_path) == 1
        assert np.array_equal(empty_path.points, np.array([[1, 2]]))

    def test_extend(self):
        path1 = Path([(1, 2), (3, 4)])
        path2 = Path([(5, 6), (7, 8)])

        # Extend with a list of points
        path1.extend([(5, 6), (7, 8)])
        assert len(path1) == 4
        assert np.array_equal(path1.points, np.array([[1, 2], [3, 4], [5, 6], [7, 8]]))

        # Extend with another Path
        path3 = Path([(1, 2)])
        path3.extend(path2)
        assert len(path3) == 3
        assert np.array_equal(path3.points, np.array([[1, 2], [5, 6], [7, 8]]))

        # Extend empty path
        empty_path = Path()
        empty_path.extend(path2)
        assert len(empty_path) == 2
        assert np.array_equal(empty_path.points, np.array([[5, 6], [7, 8]]))

    def test_insert(self):
        path = Path([(1, 2), (5, 6)])
        path.insert(1, Vector(3, 4))
        assert len(path) == 3
        assert np.array_equal(path.points, np.array([[1, 2], [3, 4], [5, 6]]))

        # Insert to empty path
        empty_path = Path()
        empty_path.insert(0, Vector(1, 2))
        assert len(empty_path) == 1
        assert np.array_equal(empty_path.points, np.array([[1, 2]]))

    def test_remove(self):
        path = Path([(1, 2), (3, 4), (5, 6)])
        removed = path.remove(1)
        assert len(path) == 2
        assert np.array_equal(path.points, np.array([[1, 2], [5, 6]]))
        assert np.array_equal(removed, np.array([3, 4]))

    def test_clear(self):
        path = Path([(1, 2), (3, 4), (5, 6)])
        path.clear()
        assert len(path) == 0
        assert path.points.shape[0] == 0


class TestPathAlgorithms:
    def test_length(self):
        # Empty path
        empty_path = Path()
        assert empty_path.length() == 0.0

        # Single point
        single_point_path = Path([(1, 2)])
        assert single_point_path.length() == 0.0

        # Simple path
        path = Path([(0, 0), (3, 0), (3, 4)])
        assert path.length() == 3 + 4 == 7.0

    def test_resample(self):
        # Test with point spacing = 1.0
        path = Path([(0, 0), (3, 0)])  # Horizontal line of length 3
        resampled = path.resample(1.0)
        assert len(resampled) == 4  # Should have points at 0,0 - 1,0 - 2,0 - 3,0
        assert np.allclose(resampled.points, np.array([[0, 0], [1, 0], [2, 0], [3, 0]]))

        # Test with empty path
        empty_path = Path()
        resampled_empty = empty_path.resample(1.0)
        assert len(resampled_empty) == 0

        # Test with invalid spacing
        path = Path([(0, 0), (1, 0)])
        invalid_resampled = path.resample(-1.0)
        assert len(invalid_resampled) == 2
        assert np.array_equal(invalid_resampled.points, path.points)

    def test_simplify(self):
        # Path with collinear points
        path = Path([(0, 0), (1, 0), (2, 0), (3, 0)])
        simplified = path.simplify(0.1)
        assert len(simplified) == 2
        assert np.allclose(simplified.points, np.array([[0, 0], [3, 0]]))

        path = Path([(0, 0), (0.5, 0.01), (1.0, 0), (1.5, 0), (2, 0)])
        simplified = path.simplify(0.1)
        # Should at least simplify some of the points
        assert len(simplified) < len(path)
        # But always keep endpoints
        assert np.array_equal(simplified.points[0], np.array([0, 0]))
        assert np.array_equal(simplified.points[-1], np.array([2, 0]))

        # Test with empty path
        empty_path = Path()
        simplified_empty = empty_path.simplify(0.1)
        assert len(simplified_empty) == 0

    def test_smooth(self):
        # Create a zigzag path
        path = Path([(0, 0), (1, 1), (2, 0), (3, 1)])

        # Smooth with weight = 0.5, 1 iteration
        smoothed = path.smooth(0.5, 1)
        assert len(smoothed) == 4
        # The middle points should be closer to the average of their neighbors
        assert smoothed.points[1][1] < 1.0  # Middle points should be smoothed
        assert smoothed.points[2][1] > 0.0

        # First and last points should remain unchanged
        assert np.array_equal(smoothed.points[0], np.array([0, 0]))
        assert np.array_equal(smoothed.points[-1], np.array([3, 1]))

        # Test with invalid parameters
        invalid_weight = path.smooth(-0.1, 1)
        assert np.array_equal(invalid_weight.points, path.points)

        invalid_iterations = path.smooth(0.5, 0)
        assert np.array_equal(invalid_iterations.points, path.points)

    def test_nearest_point_index(self):
        path = Path([(0, 0), (1, 1), (2, 2)])

        # Test exact match
        assert path.nearest_point_index(Vector(1, 1)) == 1

        # Test closest match
        assert path.nearest_point_index(Vector(0.6, 0.6)) == 1
        assert path.nearest_point_index(Vector(0.4, 0.4)) == 0

        # Test with empty path
        empty_path = Path()
        with pytest.raises(ValueError):
            empty_path.nearest_point_index(Vector(0, 0))

    def test_reverse(self):
        path = Path([(0, 0), (1, 1), (2, 2)])
        reversed_path = path.reverse()

        assert len(reversed_path) == 3
        assert np.array_equal(reversed_path.points, np.array([[2, 2], [1, 1], [0, 0]]))

        # Original path should remain unchanged
        assert np.array_equal(path.points, np.array([[0, 0], [1, 1], [2, 2]]))


class TestPathAccessors:
    def test_getitem(self):
        path = Path([(0, 0), (1, 1), (2, 2), (3, 3)])

        # Get single point
        point = path[1]
        assert np.array_equal(point, np.array([1, 1]))

        # Get slice
        slice_path = path[1:3]
        assert isinstance(slice_path, Path)
        assert len(slice_path) == 2
        assert np.array_equal(slice_path.points, np.array([[1, 1], [2, 2]]))

    def test_get_vector(self):
        path = Path([(0, 0), (1, 1), (2, 2)])
        vector = path.get_vector(1)
        assert isinstance(vector, Vector)
        assert vector == Vector(1, 1)

    def test_head_last(self):
        path = Path([(0, 0), (1, 1), (2, 2)])

        # Test head
        head = path.head()
        assert isinstance(head, Vector)
        assert head == Vector(0, 0)

        # Test last
        last = path.last()
        assert isinstance(last, Vector)
        assert last == Vector(2, 2)

        # Test with empty path
        empty_path = Path()
        assert empty_path.head() is None
        assert empty_path.last() is None

    def test_tail(self):
        path = Path([(0, 0), (1, 1), (2, 2)])
        tail = path.tail()

        assert isinstance(tail, Path)
        assert len(tail) == 2
        assert np.array_equal(tail.points, np.array([[1, 1], [2, 2]]))

        # Test with short path
        short_path = Path([(0, 0)])
        assert short_path.tail() is None

        # Test with empty path
        empty_path = Path()
        assert empty_path.tail() is None

    def test_iter(self):
        path = Path([(0, 0), (1, 1), (2, 2)])
        points = list(path)

        assert len(points) == 3
        assert np.array_equal(points[0], np.array([0, 0]))
        assert np.array_equal(points[1], np.array([1, 1]))
        assert np.array_equal(points[2], np.array([2, 2]))


class TestPathOperations:
    def test_ipush(self):
        path = Path([(0, 0), (1, 1)])
        new_path = path.ipush(Vector(2, 2))

        # Original path should remain unchanged
        assert len(path) == 2

        # New path should have the additional point
        assert len(new_path) == 3
        assert np.array_equal(new_path.points, np.array([[0, 0], [1, 1], [2, 2]]))

        # Test with empty path
        empty_path = Path()
        new_path = empty_path.ipush(Vector(0, 0))
        assert len(new_path) == 1
        assert np.array_equal(new_path.points, np.array([[0, 0]]))

    def test_iclip_tail(self):
        path = Path([(0, 0), (1, 1), (2, 2), (3, 3)])

        # Clip to last 2 points
        clipped = path.iclip_tail(2)
        assert len(clipped) == 2
        assert np.array_equal(clipped.points, np.array([[2, 2], [3, 3]]))

        # Original path should remain unchanged
        assert len(path) == 4

        # Clip with max_len > path length
        full_clipped = path.iclip_tail(10)
        assert len(full_clipped) == 4
        assert np.array_equal(full_clipped.points, path.points)

        # Test max_len = 0 (now should raise an error)
        with pytest.raises(ValueError):
            path.iclip_tail(0)
            
        # Test negative max_len
        with pytest.raises(ValueError):
            path.iclip_tail(-1)

    def test_add_operator(self):
        path = Path([(0, 0), (1, 1)])
        new_path = path + Vector(2, 2)

        assert len(new_path) == 3
        assert np.array_equal(new_path.points, np.array([[0, 0], [1, 1], [2, 2]]))


class TestPathSerialization:
    def test_serialize(self):
        path = Path([(0, 0), (1, 1), (2, 2)])
        serialized = path.serialize()

        assert serialized["type"] == "path"
        assert serialized["points"] == [[0.0, 0.0], [1.0, 1.0], [2.0, 2.0]]

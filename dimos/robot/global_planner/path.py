import numpy as np
from typing import List, Union, Tuple, Iterator, TypeVar
from dimos.robot.global_planner.vector import Vector

T = TypeVar("T", bound="Path")


class Path:
    """A class representing a path as a sequence of points."""

    def __init__(
        self,
        points: Union[
            List[Vector], List[np.ndarray], List[Tuple], np.ndarray, None
        ] = None,
    ):
        """Initialize a path from a list of points.

        Args:
            points: List of Vector objects, numpy arrays, tuples, or a 2D numpy array where each row is a point.
                   If None, creates an empty path.

        Examples:
            Path([Vector(1, 2), Vector(3, 4)])  # from Vector objects
            Path([(1, 2), (3, 4)])              # from tuples
            Path(np.array([[1, 2], [3, 4]]))    # from 2D numpy array
        """
        if points is None:
            self._points = np.zeros((0, 0), dtype=float)
            return

        if isinstance(points, np.ndarray) and points.ndim == 2:
            # If already a 2D numpy array, use it directly
            self._points = points.astype(float)
        else:
            # Convert various input types to numpy array
            converted = []
            for p in points:
                if isinstance(p, Vector):
                    converted.append(p.data)
                else:
                    converted.append(p)
            self._points = np.array(converted, dtype=float)

    @property
    def points(self) -> np.ndarray:
        """Get the path points as a numpy array."""
        return self._points

    def as_vectors(self) -> List[Vector]:
        """Get the path points as Vector objects."""
        return [Vector(p) for p in self._points]

    def append(self, point: Union[Vector, np.ndarray, Tuple]) -> None:
        """Append a point to the path.

        Args:
            point: A Vector, numpy array, or tuple representing a point
        """
        if isinstance(point, Vector):
            point_data = point.data
        else:
            point_data = np.array(point, dtype=float)

        if len(self._points) == 0:
            # If empty, create with correct dimensionality
            self._points = np.array([point_data])
        else:
            self._points = np.vstack((self._points, point_data))

    def extend(
        self, points: Union[List[Vector], List[np.ndarray], List[Tuple], "Path"]
    ) -> None:
        """Extend the path with more points.

        Args:
            points: List of points or another Path object
        """
        if isinstance(points, Path):
            if len(self._points) == 0:
                self._points = points.points.copy()
            else:
                self._points = np.vstack((self._points, points.points))
        else:
            for point in points:
                self.append(point)

    def insert(self, index: int, point: Union[Vector, np.ndarray, Tuple]) -> None:
        """Insert a point at a specific index.

        Args:
            index: The index at which to insert the point
            point: A Vector, numpy array, or tuple representing a point
        """
        if isinstance(point, Vector):
            point_data = point.data
        else:
            point_data = np.array(point, dtype=float)

        if len(self._points) == 0:
            self._points = np.array([point_data])
        else:
            self._points = np.insert(self._points, index, point_data, axis=0)

    def remove(self, index: int) -> np.ndarray:
        """Remove and return the point at the given index.

        Args:
            index: The index of the point to remove

        Returns:
            The removed point as a numpy array
        """
        point = self._points[index].copy()
        self._points = np.delete(self._points, index, axis=0)
        return point

    def clear(self) -> None:
        """Remove all points from the path."""
        self._points = np.zeros(
            (0, self._points.shape[1] if len(self._points) > 0 else 0), dtype=float
        )

    def length(self) -> float:
        """Calculate the total length of the path.

        Returns:
            The sum of the distances between consecutive points
        """
        if len(self._points) < 2:
            return 0.0

        # Efficient vector calculation of consecutive point distances
        diff = self._points[1:] - self._points[:-1]
        segment_lengths = np.sqrt(np.sum(diff * diff, axis=1))
        return float(np.sum(segment_lengths))

    def resample(self: T, point_spacing: float) -> T:
        """Resample the path with approximately uniform spacing between points.

        Args:
            point_spacing: The desired distance between consecutive points

        Returns:
            A new Path object with resampled points
        """
        if len(self._points) < 2 or point_spacing <= 0:
            return self.__class__(self._points.copy())

        resampled_points = [self._points[0].copy()]
        accumulated_distance = 0.0

        for i in range(1, len(self._points)):
            current_point = self._points[i]
            prev_point = self._points[i - 1]
            segment_vector = current_point - prev_point
            segment_length = np.linalg.norm(segment_vector)

            if segment_length < 1e-10:
                continue

            direction = segment_vector / segment_length

            # Add points along this segment until we reach the end
            while accumulated_distance + segment_length >= point_spacing:
                # How far along this segment the next point should be
                dist_along_segment = point_spacing - accumulated_distance
                if dist_along_segment < 0:
                    break

                # Create the new point
                new_point = prev_point + direction * dist_along_segment
                resampled_points.append(new_point)

                # Update for next iteration
                accumulated_distance = 0
                segment_length -= dist_along_segment
                prev_point = new_point

            # Update the accumulated distance for the next segment
            accumulated_distance += segment_length

        # Add the last point if it's not already there
        if len(self._points) > 1:
            last_point = self._points[-1]
            if not np.array_equal(resampled_points[-1], last_point):
                resampled_points.append(last_point.copy())

        return self.__class__(np.array(resampled_points))

    def simplify(self: T, tolerance: float) -> T:
        """Simplify the path using the Ramer-Douglas-Peucker algorithm.

        Args:
            tolerance: The maximum distance a point can deviate from the simplified path

        Returns:
            A new simplified Path object
        """
        if len(self._points) <= 2:
            return self.__class__(self._points.copy())

        # Implementation of Ramer-Douglas-Peucker algorithm
        def rdp(points, epsilon, start, end):
            if end <= start + 1:
                return [start]

            # Find point with max distance from line
            line_vec = points[end] - points[start]
            line_length = np.linalg.norm(line_vec)

            if line_length < 1e-10:  # If start and end points are the same
                # Distance from next point to start point
                max_dist = np.linalg.norm(points[start + 1] - points[start])
                max_idx = start + 1
            else:
                max_dist = 0
                max_idx = start

                for i in range(start + 1, end):
                    # Distance from point to line
                    p_vec = points[i] - points[start]

                    # Project p_vec onto line_vec
                    proj_scalar = np.dot(p_vec, line_vec) / (line_length * line_length)
                    proj = points[start] + proj_scalar * line_vec

                    # Calculate perpendicular distance
                    dist = np.linalg.norm(points[i] - proj)

                    if dist > max_dist:
                        max_dist = dist
                        max_idx = i

            # Recursive call
            result = []
            if max_dist > epsilon:
                result_left = rdp(points, epsilon, start, max_idx)
                result_right = rdp(points, epsilon, max_idx, end)
                result = result_left + result_right[1:]
            else:
                result = [start, end]

            return result

        indices = rdp(self._points, tolerance, 0, len(self._points) - 1)
        indices.append(len(self._points) - 1)  # Make sure the last point is included
        indices = sorted(set(indices))  # Remove duplicates and sort

        return self.__class__(self._points[indices])

    def smooth(self: T, weight: float = 0.5, iterations: int = 1) -> T:
        """Smooth the path using a moving average filter.

        Args:
            weight: How much to weight the neighboring points (0-1)
            iterations: Number of smoothing passes to apply

        Returns:
            A new smoothed Path object
        """
        if len(self._points) <= 2 or weight <= 0 or iterations <= 0:
            return self.__class__(self._points.copy())

        smoothed_points = self._points.copy()

        for _ in range(iterations):
            new_points = np.zeros_like(smoothed_points)
            new_points[0] = smoothed_points[0]  # Keep first point unchanged

            # Apply weighted average to middle points
            for i in range(1, len(smoothed_points) - 1):
                neighbor_avg = 0.5 * (smoothed_points[i - 1] + smoothed_points[i + 1])
                new_points[i] = (1 - weight) * smoothed_points[
                    i
                ] + weight * neighbor_avg

            new_points[-1] = smoothed_points[-1]  # Keep last point unchanged
            smoothed_points = new_points

        return self.__class__(smoothed_points)

    def nearest_point_index(self, point: Union[Vector, np.ndarray, Tuple]) -> int:
        """Find the index of the closest point on the path to the given point.

        Args:
            point: The reference point

        Returns:
            Index of the closest point on the path
        """
        if len(self._points) == 0:
            raise ValueError("Cannot find nearest point in an empty path")

        if isinstance(point, Vector):
            point_data = point.data
        else:
            point_data = np.array(point, dtype=float)

        # Calculate squared distances to all points
        diff = self._points - point_data
        sq_distances = np.sum(diff * diff, axis=1)

        # Return index of minimum distance
        return int(np.argmin(sq_distances))

    def reverse(self: T) -> T:
        """Reverse the path direction.

        Returns:
            A new Path object with points in reverse order
        """
        return self.__class__(self._points[::-1].copy())

    def __len__(self) -> int:
        """Return the number of points in the path."""
        return len(self._points)

    def __getitem__(self, idx) -> Union[np.ndarray, "Path"]:
        """Get a point or slice of points from the path."""
        if isinstance(idx, slice):
            return self.__class__(self._points[idx])
        return self._points[idx].copy()

    def get_vector(self, idx: int) -> Vector:
        """Get a point at the given index as a Vector object."""
        return Vector(self._points[idx])

    def head(self) -> Vector:
        """Get the first point in the path as a Vector object."""
        if len(self._points) == 0:
            return None
        return Vector(self._points[0])

    def tail(self) -> "Path":
        """Get all points except the first point as a new Path object."""
        if len(self._points) <= 1:
            return None
        return self.__class__(self._points[1:].copy())

    def __iter__(self) -> Iterator[np.ndarray]:
        """Iterate over the points in the path."""
        for point in self._points:
            yield point.copy()

    def __repr__(self) -> str:
        """String representation of the path."""
        return f"↶ Path ({len(self._points)} Points)"


if __name__ == "__main__":
    # Test vectors in various directions
    print(
        Path(
            [
                Vector(1, 0),  # Right
                Vector(1, 1),  # Up-Right
                Vector(0, 1),  # Up
                Vector(-1, 1),  # Up-Left
                Vector(-1, 0),  # Left
                Vector(-1, -1),  # Down-Left
                Vector(0, -1),  # Down
                Vector(1, -1),  # Down-Right
                Vector(0.5, 0.5),  # Up-Right (shorter)
                Vector(-3, 4),  # Up-Left (longer)
            ]
        )
    )

    print(Path())

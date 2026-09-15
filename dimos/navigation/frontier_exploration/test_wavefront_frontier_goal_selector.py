"""Parity and behaviour tests for the vectorized frontier detection.

The numpy/scipy implementation must find exactly the same frontiers as the
per-cell BFS it replaces. The old implementation is copied verbatim below as
the reference; parity is asserted on hand-built maps and on 25 randomized
maps (fixed seed), comparing centroid positions and cluster sizes before
ranking.
"""

from collections import deque
from dataclasses import dataclass
from enum import IntFlag

import numpy as np

from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.OccupancyGrid import CostValues, OccupancyGrid
from dimos.navigation.frontier_exploration.wavefront_frontier_goal_selector import (
    WavefrontConfig,
    WavefrontFrontierExplorer,
)

RESOLUTION = 0.05


# --- reference implementation (verbatim copy of the pre-vectorization BFS) ---


class PointClassification(IntFlag):
    """Point classification flags for frontier detection algorithm."""

    NoInformation = 0
    MapOpen = 1
    MapClosed = 2
    FrontierOpen = 4
    FrontierClosed = 8


@dataclass
class GridPoint:
    """Represents a point in the grid map with classification."""

    x: int
    y: int
    classification: int = PointClassification.NoInformation


class FrontierCache:
    """Cache for grid points to avoid duplicate point creation."""

    def __init__(self) -> None:
        self.points = {}  # type: ignore[var-annotated]

    def get_point(self, x: int, y: int) -> GridPoint:
        """Get or create a grid point at the given coordinates."""
        key = (x, y)
        if key not in self.points:
            self.points[key] = GridPoint(x, y)
        return self.points[key]  # type: ignore[no-any-return]

    def clear(self) -> None:
        """Clear the point cache."""
        self.points.clear()


def _noop(*args, **kwargs):
    return None


class ReferenceFrontierDetector:
    """The previous per-cell BFS, copied verbatim (logging stripped, ranking cut:
    it returns the raw centroids and sizes so parity is checked before ranking)."""

    def __init__(self, occupancy_threshold: int, min_frontier_perimeter: float) -> None:
        self._cache = FrontierCache()
        self.occupancy_threshold = occupancy_threshold
        self.min_frontier_perimeter = min_frontier_perimeter

    def _get_neighbors(self, point: GridPoint, costmap: OccupancyGrid) -> list[GridPoint]:
        """Get valid neighboring points for a given grid point."""
        neighbors = []

        # 8-connected neighbors
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue

                nx, ny = point.x + dx, point.y + dy

                # Check bounds
                if 0 <= nx < costmap.width and 0 <= ny < costmap.height:
                    neighbors.append(self._cache.get_point(nx, ny))

        return neighbors

    def _find_free_space(
        self, start_x: int, start_y: int, costmap: OccupancyGrid
    ) -> tuple[int, int]:
        """
        Find the nearest free space point using BFS from the starting position.
        """
        queue = deque([self._cache.get_point(start_x, start_y)])
        visited = set()

        while queue:
            point = queue.popleft()

            if (point.x, point.y) in visited:
                continue
            visited.add((point.x, point.y))

            # Check if this point is free space
            if costmap.grid[point.y, point.x] == CostValues.FREE:
                return (point.x, point.y)

            # Add neighbors to search
            for neighbor in self._get_neighbors(point, costmap):
                if (neighbor.x, neighbor.y) not in visited:
                    queue.append(neighbor)

        # If no free space found, return original position
        return (start_x, start_y)

    def _is_frontier_point(self, point: GridPoint, costmap: OccupancyGrid) -> bool:
        """
        Check if a point is a frontier point.
        A frontier point is an unknown cell adjacent to at least one free cell
        and not adjacent to any occupied cells.
        """
        # Point must be unknown
        cost = costmap.grid[point.y, point.x]
        if cost != CostValues.UNKNOWN:
            return False

        has_free = False

        for neighbor in self._get_neighbors(point, costmap):
            neighbor_cost = costmap.grid[neighbor.y, neighbor.x]

            # If adjacent to occupied space, not a frontier
            if neighbor_cost > self.occupancy_threshold:
                return False

            # Check if adjacent to free space
            if neighbor_cost == CostValues.FREE:
                has_free = True

        return has_free

    def _compute_centroid(self, frontier_points: list[Vector3]) -> Vector3:
        """Compute the centroid of a list of frontier points."""
        if not frontier_points:
            return Vector3(0.0, 0.0, 0.0)

        # Vectorized approach using numpy
        points_array = np.array([[point.x, point.y] for point in frontier_points])
        centroid = np.mean(points_array, axis=0)

        return Vector3(centroid[0], centroid[1], 0.0)

    def detect_frontiers(self, robot_pose: Vector3, costmap: OccupancyGrid) -> list[Vector3]:
        """
        Main frontier detection algorithm using wavefront exploration.

        Args:
            robot_pose: Current robot position in world coordinates
            costmap: Costmap for frontier detection

        Returns:
            List of frontier centroids in world coordinates
        """
        self._cache.clear()

        # Convert robot pose to grid coordinates
        grid_pos = costmap.world_to_grid(robot_pose)
        grid_x, grid_y = int(grid_pos.x), int(grid_pos.y)

        # Find nearest free space to start exploration
        free_x, free_y = self._find_free_space(grid_x, grid_y, costmap)
        start_point = self._cache.get_point(free_x, free_y)
        start_point.classification = PointClassification.MapOpen

        # Main exploration queue - explore ALL reachable free space
        map_queue = deque([start_point])
        frontiers = []
        frontier_sizes = []

        points_checked = 0
        frontier_candidates = 0

        while map_queue:
            current_point = map_queue.popleft()
            points_checked += 1

            # Skip if already processed
            if current_point.classification & PointClassification.MapClosed:
                continue

            # Mark as processed
            current_point.classification |= PointClassification.MapClosed

            # Check if this point starts a new frontier
            if self._is_frontier_point(current_point, costmap):
                frontier_candidates += 1
                current_point.classification |= PointClassification.FrontierOpen
                frontier_queue = deque([current_point])
                new_frontier = []

                # Explore this frontier region using BFS
                while frontier_queue:
                    frontier_point = frontier_queue.popleft()

                    # Skip if already processed
                    if frontier_point.classification & PointClassification.FrontierClosed:
                        continue

                    # If this is still a frontier point, add to current frontier
                    if self._is_frontier_point(frontier_point, costmap):
                        new_frontier.append(frontier_point)

                        # Add neighbors to frontier queue
                        for neighbor in self._get_neighbors(frontier_point, costmap):
                            if not (
                                neighbor.classification
                                & (
                                    PointClassification.FrontierOpen
                                    | PointClassification.FrontierClosed
                                )
                            ):
                                neighbor.classification |= PointClassification.FrontierOpen
                                frontier_queue.append(neighbor)

                    frontier_point.classification |= PointClassification.FrontierClosed

                # Check if we found a large enough frontier
                # Convert minimum perimeter to minimum number of cells based on resolution
                min_cells = int(self.min_frontier_perimeter / costmap.resolution)
                if len(new_frontier) >= min_cells:
                    world_points = []
                    for point in new_frontier:
                        world_pos = costmap.grid_to_world(
                            Vector3(float(point.x), float(point.y), 0.0)
                        )
                        world_points.append(world_pos)

                    # Compute centroid in world coordinates (already correctly scaled)
                    centroid = self._compute_centroid(world_points)
                    frontiers.append(centroid)  # Store centroid
                    frontier_sizes.append(len(new_frontier))  # Store frontier size

            # Add ALL neighbors to main exploration queue to explore entire free space
            for neighbor in self._get_neighbors(current_point, costmap):
                if not (
                    neighbor.classification
                    & (PointClassification.MapOpen | PointClassification.MapClosed)
                ):
                    # Check if neighbor is free space or unknown (explorable)
                    neighbor_cost = costmap.grid[neighbor.y, neighbor.x]

                    # Add free space and unknown space to exploration queue
                    if neighbor_cost == CostValues.FREE or neighbor_cost == CostValues.UNKNOWN:
                        neighbor.classification |= PointClassification.MapOpen
                        map_queue.append(neighbor)

        return frontiers, frontier_sizes


# --- helpers -----------------------------------------------------------------


def make_costmap(grid: np.ndarray) -> OccupancyGrid:
    return OccupancyGrid(grid=grid.astype(np.int8), resolution=RESOLUTION, frame_id="world")


def new_detection(
    grid: np.ndarray,
    robot_xy: tuple[float, float],
    occupancy_threshold: int = 99,
    min_perimeter: float = 0.3,
):
    """Run the vectorized detect_frontiers, capturing centroids and sizes
    before ranking (ranking is orthogonal and tested upstream)."""
    explorer = WavefrontFrontierExplorer.__new__(WavefrontFrontierExplorer)
    explorer.config = WavefrontConfig(
        occupancy_threshold=occupancy_threshold, min_frontier_perimeter=min_perimeter
    )
    captured = {}

    def capture(centroids, sizes, robot_pose, costmap):
        captured["centroids"], captured["sizes"] = centroids, sizes
        return centroids

    explorer._rank_frontiers = capture
    costmap = make_costmap(grid)
    result = explorer.detect_frontiers(Vector3(robot_xy[0], robot_xy[1], 0.0), costmap)
    return result, captured.get("centroids", []), captured.get("sizes", [])


def reference_detection(
    grid: np.ndarray,
    robot_xy: tuple[float, float],
    occupancy_threshold: int = 99,
    min_perimeter: float = 0.3,
):
    detector = ReferenceFrontierDetector(occupancy_threshold, min_perimeter)
    return detector.detect_frontiers(Vector3(robot_xy[0], robot_xy[1], 0.0), make_costmap(grid))


def as_sorted_set(centroids, sizes):
    return sorted((round(c.x, 6), round(c.y, 6), s) for c, s in zip(centroids, sizes, strict=True))


def assert_parity(grid: np.ndarray, robot_xy: tuple[float, float]) -> int:
    _, new_centroids, new_sizes = new_detection(grid, robot_xy)
    ref_centroids, ref_sizes = reference_detection(grid, robot_xy)
    assert as_sorted_set(new_centroids, new_sizes) == as_sorted_set(ref_centroids, ref_sizes)
    return len(new_centroids)


def room_grid(n: int = 120) -> np.ndarray:
    """A free room inside unknown space, with a wall along its north edge."""
    grid = np.full((n, n), CostValues.UNKNOWN, dtype=np.int8)
    grid[40:80, 40:80] = CostValues.FREE
    grid[39, 35:85] = 100
    return grid


# --- behaviour ---------------------------------------------------------------


def test_room_has_one_u_shaped_frontier_away_from_the_wall():
    grid = room_grid()
    _, centroids, sizes = new_detection(grid, (60 * RESOLUTION, 60 * RESOLUTION))
    # east, south and west edges touch at the corners: one 8-connected cluster;
    # the north edge is adjacent to the wall and must be excluded
    assert len(centroids) == 1
    assert sizes[0] >= 100
    assert centroids[0].y > 60 * RESOLUTION  # pulled south, away from the wall


def test_unreachable_region_is_not_a_frontier():
    grid = room_grid()
    # a second free pocket fully enclosed by obstacles: unreachable
    grid[100:110, 100:110] = CostValues.FREE
    grid[99, 99:111] = 100
    grid[111, 99:111] = 100
    grid[99:112, 99] = 100
    grid[99:112, 111] = 100
    count = assert_parity(grid, (60 * RESOLUTION, 60 * RESOLUTION))
    assert count == 1  # still only the room's own frontier


def test_robot_in_unknown_space_starts_from_nearest_free_cell():
    grid = room_grid()
    count = assert_parity(grid, (110 * RESOLUTION, 60 * RESOLUTION))
    assert count == 1


def test_no_free_space_returns_empty():
    grid = np.full((50, 50), CostValues.UNKNOWN, dtype=np.int8)
    result, _, _ = new_detection(grid, (1.0, 1.0))
    assert result == []


def test_min_perimeter_filters_small_clusters():
    grid = np.full((60, 60), CostValues.UNKNOWN, dtype=np.int8)
    grid[28:33, 28:33] = CostValues.FREE  # a 5x5 free pocket: 16-cell ring frontier
    _, _, sizes_small = new_detection(grid, (1.5, 1.5), min_perimeter=0.3)  # 6 cells min
    _, _, sizes_large = new_detection(grid, (1.5, 1.5), min_perimeter=1.5)  # 30 cells min
    assert sizes_small and all(s >= 6 for s in sizes_small)
    assert sizes_large == []


# --- parity with the reference BFS -------------------------------------------


def test_robot_between_two_components_keeps_chebyshev_semantics():
    """Regression for the Greptile finding on this PR: the robot stands in
    unknown space between two disconnected free regions. Region A is nearer in
    Chebyshev (8-connected BFS layers, the old semantics), region B is nearer
    in Euclidean. The start cell must fall in region A, as the BFS chose."""
    grid = np.full((40, 40), CostValues.UNKNOWN, dtype=np.int8)
    # wall splitting the map so the regions are disconnected
    grid[:, 20] = 100
    grid[10:14, 10:14] = CostValues.FREE  # region A: around (12, 12)
    grid[18:22, 24:28] = CostValues.FREE  # region B: around (20, 26)
    # robot at (19, 17): Chebyshev to A ~5 (via diagonal layers), Euclidean ~7.6;
    # Chebyshev to B 7, Euclidean 7 -> Chebyshev picks A, Euclidean would pick B
    robot = (17 * RESOLUTION, 19 * RESOLUTION)
    assert_parity(grid, robot)
    _, centroids, _ = new_detection(grid, robot)
    assert centroids, "region A must produce a frontier"
    assert all(c.x < 20 * RESOLUTION for c in centroids), (
        "all frontiers must belong to region A (left of the wall), the Chebyshev-nearest component"
    )


def test_parity_on_the_room():
    assert_parity(room_grid(), (60 * RESOLUTION, 60 * RESOLUTION))


def test_parity_on_randomized_maps():
    rng = np.random.default_rng(42)
    checked_with_frontiers = 0
    for _ in range(25):
        n = int(rng.integers(40, 90))
        grid = np.full((n, n), CostValues.UNKNOWN, dtype=np.int8)
        # random free blobs
        for _ in range(int(rng.integers(1, 4))):
            y, x = rng.integers(5, n - 15, size=2)
            h, w = rng.integers(6, 14, size=2)
            grid[y : y + h, x : x + w] = CostValues.FREE
        # random obstacle strokes
        for _ in range(int(rng.integers(0, 6))):
            y, x = rng.integers(0, n - 10, size=2)
            if rng.random() < 0.5:
                grid[y, x : x + int(rng.integers(4, 10))] = 100
            else:
                grid[y : y + int(rng.integers(4, 10)), x] = 100
        ys, xs = np.nonzero(grid == CostValues.FREE)
        if len(xs) == 0:
            continue
        if rng.random() < 0.5:
            robot = (float(xs[0]) * RESOLUTION, float(ys[0]) * RESOLUTION)
        else:  # robot standing in unknown/occupied space, possibly between components
            ry, rx = rng.integers(0, n, size=2)
            robot = (float(rx) * RESOLUTION, float(ry) * RESOLUTION)
        count = assert_parity(grid, robot)
        checked_with_frontiers += bool(count)
    assert checked_with_frontiers >= 10  # the fuzz actually exercised frontiers


def test_faster_than_reference_on_a_large_map():
    import time

    grid = np.full((300, 300), CostValues.UNKNOWN, dtype=np.int8)
    grid[50:250, 50:250] = CostValues.FREE
    for k in range(60, 240, 20):
        grid[k, 60:240:3] = 100
    robot = (150 * RESOLUTION, 150 * RESOLUTION)
    t0 = time.perf_counter()
    new_detection(grid, robot)
    t_new = time.perf_counter() - t0
    t0 = time.perf_counter()
    reference_detection(grid, robot)
    t_ref = time.perf_counter() - t0
    assert_parity(grid, robot)
    assert t_new < t_ref / 5

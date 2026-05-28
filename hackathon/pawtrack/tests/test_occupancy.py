"""Tests for the occupancy-grid obstacle filter (glass-reflection rejection)."""

from pawtrack.occupancy import is_near_obstacle


def _free_grid(width, height):
    """A width x height grid of free (cost 0) cells, indexed grid[y][x]."""
    return [[0] * width for _ in range(height)]


def test_no_grid_keeps_candidate():
    # No costmap yet -> never suppress greeting (fail open).
    assert is_near_obstacle(
        None, 0.1, (0.0, 0.0), (1.0, 1.0), clearance_m=0.3) is False


def test_empty_grid_keeps_candidate():
    assert is_near_obstacle(
        [], 0.1, (0.0, 0.0), (1.0, 1.0), clearance_m=0.3) is False


def test_free_cell_keeps_candidate():
    grid = _free_grid(10, 10)
    assert is_near_obstacle(
        grid, 0.1, (0.0, 0.0), (0.5, 0.5), clearance_m=0.3) is False


def test_unknown_cells_do_not_reject():
    # Unknown space (-1) is not a known wall, so a candidate there is kept.
    grid = [[-1] * 10 for _ in range(10)]
    assert is_near_obstacle(
        grid, 0.1, (0.0, 0.0), (0.5, 0.5), clearance_m=0.3) is False


def test_point_on_occupied_cell_is_rejected():
    grid = _free_grid(10, 10)
    grid[5][5] = 100  # lethal wall cell at grid[y=5][x=5]
    # world (0.55, 0.55) -> cell (5, 5)
    assert is_near_obstacle(
        grid, 0.1, (0.0, 0.0), (0.55, 0.55), clearance_m=0.3) is True


def test_occupied_cell_within_clearance_is_rejected():
    grid = _free_grid(20, 20)
    grid[10][10] = 100
    # ranging noise: the point lands ~0.2 m from the wall, within the 0.3 m
    # clearance, so it is still rejected.
    assert is_near_obstacle(
        grid, 0.1, (0.0, 0.0), (1.25, 1.05), clearance_m=0.3) is True


def test_occupied_cell_beyond_clearance_is_kept():
    grid = _free_grid(40, 40)
    grid[10][10] = 100
    # ~0.5 m from the wall, beyond the 0.3 m clearance -> a real subject.
    assert is_near_obstacle(
        grid, 0.1, (0.0, 0.0), (1.55, 1.05), clearance_m=0.3) is False


def test_point_off_grid_is_kept():
    grid = _free_grid(10, 10)
    grid[5][5] = 100
    assert is_near_obstacle(
        grid, 0.1, (0.0, 0.0), (50.0, 50.0), clearance_m=0.3) is False


def test_cost_threshold_respected():
    grid = _free_grid(10, 10)
    grid[5][5] = 40  # inflation cost, below the default wall threshold of 50
    assert is_near_obstacle(
        grid, 0.1, (0.0, 0.0), (0.55, 0.55), clearance_m=0.3) is False
    assert is_near_obstacle(
        grid, 0.1, (0.0, 0.0), (0.55, 0.55), clearance_m=0.3,
        cost_threshold=40) is True


def test_origin_offset_applied():
    grid = _free_grid(10, 10)
    grid[0][0] = 100
    # origin shifted to (-1, -1): world (-0.95, -0.95) maps to cell (0, 0).
    assert is_near_obstacle(
        grid, 0.1, (-1.0, -1.0), (-0.95, -0.95), clearance_m=0.1) is True


def test_zero_resolution_keeps_candidate():
    grid = _free_grid(10, 10)
    grid[5][5] = 100
    assert is_near_obstacle(
        grid, 0.0, (0.0, 0.0), (0.5, 0.5), clearance_m=0.3) is False

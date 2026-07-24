# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Focused tests for the planning collision snapshot lifecycle."""

from unittest.mock import MagicMock

import numpy as np
import pytest

from dimos.manipulation.planning.monitor.planning_collision_snapshot import (
    PlanningCollisionSnapshot,
)
from dimos.manipulation.planning.spec.enums import ObstacleType
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


def _cloud(
    points: list[list[float]], frame_id: str = "world", timestamp: float | None = None
) -> PointCloud2:
    return PointCloud2.from_numpy(
        np.asarray(points, dtype=np.float64),
        frame_id=frame_id,
        timestamp=timestamp,
    )


def test_frame_rejection_preserves_staged_and_committed_state() -> None:
    snapshot = PlanningCollisionSnapshot()
    monitor = MagicMock()
    snapshot.stage(_cloud([[1.0, 2.0, 3.0]]))
    snapshot.synchronize(monitor)
    committed = snapshot.committed()

    with pytest.raises(ValueError):
        snapshot.stage(_cloud([[4.0, 5.0, 6.0]], frame_id="camera"))

    committed_after = snapshot.committed()
    assert committed is not None and committed_after is not None
    assert committed_after.as_numpy()[0].tolist() == committed.as_numpy()[0].tolist()


def test_latest_snapshot_commits_without_freshness_rejection() -> None:
    snapshot = PlanningCollisionSnapshot()
    monitor = MagicMock()
    snapshot.stage(_cloud([[1.0, 0.0, 0.0]]))
    snapshot.stage(_cloud([[2.0, 0.0, 0.0]]))
    snapshot.synchronize(monitor)

    obstacle = monitor.add_obstacle.call_args.args[0]
    assert obstacle.obstacle_type == ObstacleType.OCTREE
    assert obstacle.octree_resolution == 0.05
    assert obstacle.points.tolist() == [[2.0, 0.0, 0.0]]


def test_staged_exposes_latest_valid_snapshot_before_commit() -> None:
    snapshot = PlanningCollisionSnapshot()
    snapshot.stage(_cloud([[0.0, 0.0, 0.0]], timestamp=1.0))
    snapshot.stage(_cloud([[1.0, 2.0, 3.0]], timestamp=2.0))

    staged = snapshot.staged()

    assert staged is not None
    assert staged.ts == 2.0
    np.testing.assert_allclose(staged.points_f32(), [[1.0, 2.0, 3.0]])
    assert snapshot.committed() is None


def test_nonempty_snapshots_update_one_stable_native_id() -> None:
    snapshot = PlanningCollisionSnapshot()
    monitor = MagicMock()
    monitor.add_obstacle.return_value = "native-id"
    monitor.update_obstacle.return_value = True
    snapshot.stage(_cloud([[1.0, 0.0, 0.0]]))
    snapshot.synchronize(monitor)
    snapshot.stage(_cloud([[2.0, 0.0, 0.0]]))
    committed = snapshot.synchronize(monitor)

    assert committed is not None
    assert committed.as_numpy()[0].tolist() == [[2.0, 0.0, 0.0]]
    monitor.add_obstacle.assert_called_once()
    monitor.update_obstacle.assert_called_once()
    native_id, replacement = monitor.update_obstacle.call_args.args
    assert native_id == "native-id"
    assert replacement.name == "planning-collision"
    assert replacement.points.tolist() == [[2.0, 0.0, 0.0]]
    monitor.remove_obstacle.assert_not_called()


def test_failed_update_preserves_committed_snapshot() -> None:
    snapshot = PlanningCollisionSnapshot()
    monitor = MagicMock()
    monitor.add_obstacle.return_value = "native-id"
    monitor.update_obstacle.side_effect = RuntimeError("backend replacement failed")

    snapshot.stage(_cloud([[1.0, 0.0, 0.0]]))
    snapshot.synchronize(monitor)
    snapshot.stage(_cloud([[2.0, 0.0, 0.0]]))
    with pytest.raises(RuntimeError, match="backend replacement failed"):
        snapshot.synchronize(monitor)

    committed = snapshot.committed()
    assert committed is not None
    assert committed.as_numpy()[0].tolist() == [[1.0, 0.0, 0.0]]
    assert snapshot._active_obstacle_id == "native-id"


def test_missing_active_obstacle_aborts_without_committing() -> None:
    snapshot = PlanningCollisionSnapshot()
    monitor = MagicMock()
    monitor.add_obstacle.return_value = "native-id"
    monitor.update_obstacle.return_value = False

    snapshot.stage(_cloud([[1.0, 0.0, 0.0]]))
    snapshot.synchronize(monitor)
    snapshot.stage(_cloud([[2.0, 0.0, 0.0]]))

    with pytest.raises(RuntimeError, match="is missing"):
        snapshot.synchronize(monitor)

    committed = snapshot.committed()
    assert committed is not None
    assert committed.as_numpy()[0].tolist() == [[1.0, 0.0, 0.0]]


def test_empty_snapshot_clears_active_generation() -> None:
    snapshot = PlanningCollisionSnapshot()
    monitor = MagicMock()
    monitor.add_obstacle.return_value = "active"
    monitor.remove_obstacle.return_value = True
    snapshot.stage(_cloud([[1.0, 0.0, 0.0]]))
    snapshot.synchronize(monitor)
    snapshot.stage(_cloud([]))
    committed = snapshot.synchronize(monitor)

    assert committed is not None
    assert len(committed) == 0
    monitor.remove_obstacle.assert_called_once_with("active")


def test_falsey_obstacle_id_is_a_failed_registration() -> None:
    snapshot = PlanningCollisionSnapshot()
    monitor = MagicMock()
    monitor.add_obstacle.return_value = ""
    snapshot.stage(_cloud([[1.0, 0.0, 0.0]]))

    with pytest.raises(RuntimeError, match="Failed to register"):
        snapshot.synchronize(monitor)

    assert snapshot.committed() is None

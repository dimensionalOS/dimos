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

from __future__ import annotations

import threading
import time
from types import SimpleNamespace

import open3d as o3d
import pytest
from pytest_mock import MockerFixture

from dimos.manipulation.planning.monitor.world_obstacle_monitor import WorldObstacleMonitor
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.perception.experimental.object import Object


def _object(object_id: str) -> Object:
    return Object(
        name=f"object-{object_id}",
        object_id=object_id,
        center=Vector3(0.4, 0.0, 0.2),
        size=Vector3(0.05, 0.05, 0.1),
        pose=PoseStamped(),
        pointcloud=PointCloud2(o3d.geometry.PointCloud()),
        bbox=(0.0, 0.0, 1.0, 1.0),
        track_id=0,
        class_id=0,
        confidence=1.0,
        ts=time.time(),
        image=Image(),
    )


def _monitor(mocker: MockerFixture) -> tuple[WorldObstacleMonitor, SimpleNamespace]:
    parent = SimpleNamespace(
        _lock=threading.RLock(),
        add_obstacle=mocker.Mock(
            side_effect=lambda obstacle: f"world-{obstacle.name}-{time.monotonic_ns()}"
        ),
        remove_obstacle=mocker.Mock(return_value=True),
    )
    monitor = WorldObstacleMonitor(parent)  # type: ignore[arg-type]
    monitor.start()
    return monitor, parent


def test_suppression_skips_target_but_refreshes_other_objects(
    mocker: MockerFixture,
) -> None:
    monitor, parent = _monitor(mocker)
    target = _object("target")
    other = _object("other")
    monitor.on_objects([target, other])
    monitor.refresh_obstacles()
    parent.add_obstacle.reset_mock()

    with monitor.suppress_object_obstacle("target") as suppression:
        refreshed = monitor.refresh_obstacles()

        assert suppression.removed is True
        assert [item["object_id"] for item in refreshed] == ["other"]
        assert set(monitor._object_obstacles) == {"other"}

    assert set(monitor._object_obstacles) == {"target", "other"}
    assert parent.remove_obstacle.call_count >= 1


def test_suppression_wins_race_with_in_progress_refresh(mocker: MockerFixture) -> None:
    monitor, _ = _monitor(mocker)
    monitor.on_objects([_object("target"), _object("other")])
    conversion_started = threading.Event()
    continue_conversion = threading.Event()
    original_conversion = monitor._object_to_obstacle

    def delayed_conversion(obj: Object):
        if obj.object_id == "target":
            conversion_started.set()
            assert continue_conversion.wait(timeout=1.0)
        return original_conversion(obj)

    mocker.patch.object(monitor, "_object_to_obstacle", side_effect=delayed_conversion)
    refreshed: list[list[dict[str, object]]] = []
    thread = threading.Thread(target=lambda: refreshed.append(monitor.refresh_obstacles()))
    thread.start()
    assert conversion_started.wait(timeout=1.0)

    with monitor.suppress_object_obstacle("target"):
        continue_conversion.set()
        thread.join(timeout=1.0)

        assert not thread.is_alive()
        assert [item["object_id"] for item in refreshed[0]] == ["other"]
        assert set(monitor._object_obstacles) == {"other"}

    assert set(monitor._object_obstacles) == {"target", "other"}


def test_nested_suppression_removes_and_restores_once(mocker: MockerFixture) -> None:
    monitor, parent = _monitor(mocker)
    monitor.on_objects([_object("target")])
    monitor.refresh_obstacles()
    parent.add_obstacle.reset_mock()
    parent.remove_obstacle.reset_mock()

    with monitor.suppress_object_obstacle("target"):
        with monitor.suppress_object_obstacle("target"):
            assert "target" not in monitor._object_obstacles
        assert "target" not in monitor._object_obstacles

    assert parent.remove_obstacle.call_count == 1
    assert parent.add_obstacle.call_count == 1
    assert "target" in monitor._object_obstacles


def test_suppression_restores_after_cancellation(mocker: MockerFixture) -> None:
    class Cancelled(BaseException):
        pass

    monitor, _ = _monitor(mocker)
    monitor.on_objects([_object("target")])
    monitor.refresh_obstacles()

    with pytest.raises(Cancelled):
        with monitor.suppress_object_obstacle("target"):
            raise Cancelled

    assert monitor._object_suppressions == {}
    assert "target" in monitor._object_obstacles


def test_suppression_reports_restore_failure_without_masking_body(
    mocker: MockerFixture,
) -> None:
    monitor, parent = _monitor(mocker)
    monitor.on_objects([_object("target")])
    monitor.refresh_obstacles()
    parent.add_obstacle.side_effect = None
    parent.add_obstacle.return_value = ""

    with monitor.suppress_object_obstacle("target") as suppression:
        body_completed = True

    assert body_completed is True
    assert suppression.cleanup_error == "failed to restore obstacle for object 'target'"
    assert "target" not in monitor._object_obstacles


def test_failed_suppression_removal_restores_internal_tracking(
    mocker: MockerFixture,
) -> None:
    monitor, parent = _monitor(mocker)
    monitor.on_objects([_object("target")])
    monitor.refresh_obstacles()
    parent.remove_obstacle.return_value = False

    with pytest.raises(RuntimeError, match="failed to suppress") as exc_info:
        with monitor.suppress_object_obstacle("target"):
            raise AssertionError("suppression body must not run")

    assert str(exc_info.value) == "failed to suppress obstacle for object 'target'"
    assert monitor._object_suppressions == {}
    assert "target" in monitor._object_obstacles

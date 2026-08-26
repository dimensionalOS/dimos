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

from collections.abc import Iterator

import pytest

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.teleop.webxr.body_tracking import BodyTrackingSnapshot
from dimos.teleop.webxr.body_tracking_monitor import (
    BodyTrackingMonitor,
    body_tracking_summary,
)


@pytest.fixture
def monitor() -> Iterator[BodyTrackingMonitor]:
    module = BodyTrackingMonitor()
    try:
        yield module
    finally:
        module.stop()


def test_body_tracking_summary_reports_best_effort_joint_positions() -> None:
    snapshot = BodyTrackingSnapshot(
        capture_time_s=10.0,
        frame_id="bounded-floor",
        joints={
            "hips": Pose(0.12345, 1.23456, -0.34567),
            "vendor-extra-joint": Pose(3.0, 2.0, 1.0),
        },
    )

    summary = body_tracking_summary(
        snapshot,
        snapshot_rate_hz=79.94,
        resolved_joint_ever_seen=True,
    )

    assert summary == {
        "snapshot_rate_hz": 79.9,
        "state": "tracking",
        "reference_space": "bounded-floor",
        "resolved_joint_count": 2,
        "resolved_joint_ever_seen": True,
        "joint_positions": {"hips": (0.123, 1.235, -0.346)},
    }


def test_body_tracking_monitor_logs_first_resolved_joint_once(
    monitor: BodyTrackingMonitor,
    mocker,
) -> None:
    mocker.patch(
        "dimos.teleop.webxr.body_tracking_monitor.time.monotonic",
        side_effect=[monitor._report_started_at + 1.0, monitor._report_started_at + 2.0],
    )
    info = mocker.patch("dimos.teleop.webxr.body_tracking_monitor.logger.info")
    snapshot = BodyTrackingSnapshot(1.0, "local-floor", {"hips": Pose()})

    monitor._on_body_tracking(snapshot)
    monitor._on_body_tracking(snapshot)

    info.assert_called_once_with(
        "WebXR body tracking acquired",
        reference_space="local-floor",
        resolved_joint_count=1,
    )


def test_body_tracking_monitor_warns_when_required_heartbeat_has_no_body(
    monitor: BodyTrackingMonitor,
    mocker,
) -> None:
    mocker.patch(
        "dimos.teleop.webxr.body_tracking_monitor.time.monotonic",
        return_value=monitor._report_started_at + 5.0,
    )
    warning = mocker.patch("dimos.teleop.webxr.body_tracking_monitor.logger.warning")

    monitor._on_body_tracking(BodyTrackingSnapshot(1.0, "local-floor", None))

    warning.assert_called_once_with(
        "WebXR body tracking has no resolved joints",
        snapshot_rate_hz=0.2,
        state="unavailable",
        reference_space="local-floor",
        resolved_joint_count=0,
        resolved_joint_ever_seen=False,
        joint_positions={},
    )


def test_body_tracking_monitor_subscribes_during_start(
    monitor: BodyTrackingMonitor,
    mocker,
) -> None:
    subscribe = mocker.patch.object(
        monitor.body_tracking,
        "subscribe",
        return_value=lambda: None,
    )

    monitor.start()

    subscribe.assert_called_once_with(monitor._on_body_tracking)

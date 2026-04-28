from __future__ import annotations

from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.robot.deeprobotics.m20.scripts.viewer_lcm_ws_bridge import (
    DEFAULT_TWIST_CHANNEL,
    point_to_click_message,
    twist_to_ws_message,
)


def test_point_to_click_message_preserves_viewer_point() -> None:
    point = PointStamped(x=1.25, y=-0.5, z=0.75, ts=12.345, frame_id="/world/floor")

    assert point_to_click_message(point) == {
        "type": "click",
        "x": 1.25,
        "y": -0.5,
        "z": 0.75,
        "entity_path": "/world/floor",
        "timestamp_ms": 12345,
    }


def test_default_twist_channel_matches_dimos_viewer_keyboard_output() -> None:
    assert DEFAULT_TWIST_CHANNEL == "/cmd_vel#geometry_msgs.Twist"


def test_twist_to_ws_message_preserves_teleop_twist(monkeypatch) -> None:  # type: ignore[no-untyped-def]
    monkeypatch.setattr(
        "dimos.robot.deeprobotics.m20.scripts.viewer_lcm_ws_bridge.time.time",
        lambda: 12.345,
    )
    twist = Twist(linear=[0.1, -0.2, 0.0], angular=[0.0, 0.0, 0.3])

    assert twist_to_ws_message(twist) == {
        "type": "twist",
        "linear_x": 0.1,
        "linear_y": -0.2,
        "linear_z": 0.0,
        "angular_x": 0.0,
        "angular_y": 0.0,
        "angular_z": 0.3,
        "timestamp_ms": 12345,
    }

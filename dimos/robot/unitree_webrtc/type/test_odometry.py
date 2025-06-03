from __future__ import annotations

import math
import os
import threading
from typing import Iterable, Optional
import reactivex.operators as ops

import pytest
from dotenv import load_dotenv

from dimos.robot.unitree_webrtc.type.odometry import Odometry, RawOdometryMessage
from dimos.robot.unitree_webrtc.unitree_go2 import UnitreeGo2
from dimos.utils.testing import SensorReplay, SensorStorage

_REPLAY_NAME = "raw_odometry_rotate_walk"
_EXPECTED_MESSAGE_COUNT = 179
_EXPECTED_TOTAL_DEGREES = -592.1696370412612
_EXPECTED_LAST_YAW_RAD = -1.0452624875249261


def _quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Return yaw (rotation about Z) in *radians* from a quaternion."""
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def test_dataset_size() -> None:
    """Ensure the replay contains the expected number of messages."""
    count = sum(1 for _ in SensorReplay(name=_REPLAY_NAME).iterate())
    assert count == _EXPECTED_MESSAGE_COUNT


def test_odometry_conversion_and_count() -> None:
    """Each replay entry converts to :class:`Odometry` and count is correct."""
    count = 0
    for raw in SensorReplay(name=_REPLAY_NAME).iterate():
        odom = Odometry.from_msg(raw)
        assert isinstance(raw, dict)
        assert isinstance(odom, Odometry)
        count += 1

    assert count == _EXPECTED_MESSAGE_COUNT


def test_last_yaw_value() -> None:
    """Verify yaw of the final message (regression guard)."""

    last_msg = SensorReplay(name=_REPLAY_NAME).stream(rate_hz=None).pipe(ops.last()).run()

    assert last_msg is not None, "Replay is empty"  # pragma: no cover

    o = last_msg["data"]["pose"]["orientation"]
    yaw = _quaternion_to_yaw(o["x"], o["y"], o["z"], o["w"])
    assert yaw == pytest.approx(_EXPECTED_LAST_YAW_RAD, abs=1e-10)


def test_total_rotation_from_quaternion() -> None:
    """Integrate yaw deltas computed directly from quaternions."""
    total_rad = 0.0
    prev_yaw: Optional[float] = None

    for msg in SensorReplay(name=_REPLAY_NAME).iterate():
        o = msg["data"]["pose"]["orientation"]
        yaw = _quaternion_to_yaw(o["x"], o["y"], o["z"], o["w"])

        if prev_yaw is not None:
            diff = yaw - prev_yaw
            diff = (diff + math.pi) % (2 * math.pi) - math.pi  # normalize
            total_rad += diff
        prev_yaw = yaw

    assert math.degrees(total_rad) == pytest.approx(_EXPECTED_TOTAL_DEGREES, abs=1e-6)


def test_total_rotation_from_odometry() -> None:
    """`Odometry.rot.z` integration should match quaternion path."""
    total_rad = 0.0
    prev_yaw: Optional[float] = None

    for odom in SensorReplay(name=_REPLAY_NAME, autocast=Odometry.from_msg).iterate():
        yaw = odom.rot.z
        if prev_yaw is not None:
            diff = yaw - prev_yaw
            diff = (diff + math.pi) % (2 * math.pi) - math.pi
            total_rad += diff
        prev_yaw = yaw

    assert math.degrees(total_rad) == pytest.approx(_EXPECTED_TOTAL_DEGREES, abs=1e-6)


# data collection tool
@pytest.mark.tool
def test_store_odometry_stream() -> None:
    load_dotenv()

    robot = UnitreeGo2(ip=os.getenv("ROBOT_IP"), mode="ai")
    robot.standup()

    storage = SensorStorage(_REPLAY_NAME)
    storage.save_stream(robot.raw_odom_stream())

    shutdown = threading.Event()

    try:
        while not shutdown.wait(0.1):
            pass
    except KeyboardInterrupt:
        shutdown.set()
    finally:
        robot.liedown()

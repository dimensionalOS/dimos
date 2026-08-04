# Copyright 2025-2026 Dimensional Inc.
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

"""D435i onboard IMU discovery + reading, fused into orientation via Madgwick AHRS."""

from __future__ import annotations

import threading
from typing import Any

import numpy as np

from dimos.perception.stereo_point_cloud.utils import R_OPT_TO_LINK
from dimos.perception.stereo_point_cloud.vio import MadgwickFilter
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

MILLISECONDS_PER_SECOND = 1000.0
GRAVITY_MS2 = 9.81


class RealSenseImuFeed:
    """Finds the D435i's onboard gyro/accel and fuses them into an orientation estimate."""

    def __init__(self, beta: float = 0.033, serial_number: str | None = None) -> None:
        self._madgwick = MadgwickFilter(beta=beta)
        self._serial_number = serial_number
        self._lock = threading.Lock()
        self._last_accel = np.array([0.0, 0.0, -GRAVITY_MS2], dtype=np.float32)
        self._imu_to_camera_link: np.ndarray = np.eye(3, dtype=np.float32)
        self._motion_sensor = None

    def start(self) -> bool:
        """Discover and start the motion sensor. Returns False if none is available."""
        try:
            import pyrealsense2 as rs
        except ImportError:
            return False
        context = rs.context()
        devices = context.query_devices()
        if not devices:
            return False
        device = devices[0]
        if self._serial_number is not None:
            matching = [d for d in devices if d.get_info(rs.camera_info.serial_number) == self._serial_number]
            if not matching:
                logger.warning(f"RealSenseImuFeed: no device with serial {self._serial_number}")
                return False
            device = matching[0]
        depth_profile = None
        for sensor in device.query_sensors():
            if sensor.is_motion_sensor():
                continue
            for profile in sensor.get_stream_profiles():
                if profile.stream_type() == rs.stream.depth:
                    depth_profile = profile
                    break
            if depth_profile is not None:
                break
        for sensor in device.query_sensors():
            if not sensor.is_motion_sensor():
                continue
            all_profiles   = sensor.get_stream_profiles()
            gyro_profiles  = [p for p in all_profiles if p.stream_type() == rs.stream.gyro]
            accel_profiles = [p for p in all_profiles if p.stream_type() == rs.stream.accel]
            if not gyro_profiles or not accel_profiles:
                break
            gyro_profile  = max(gyro_profiles,  key=lambda p: p.fps())
            accel_profile = max(accel_profiles, key=lambda p: p.fps())
            if depth_profile is not None:
                extrinsics                = accel_profile.get_extrinsics_to(depth_profile)
                imu_to_depth               = np.array(extrinsics.rotation, dtype=np.float32).reshape(3, 3)
                self._imu_to_camera_link   = R_OPT_TO_LINK @ imu_to_depth
            sensor.open([gyro_profile, accel_profile])
            sensor.start(self._on_motion)
            self._motion_sensor = sensor
            logger.info(f"RealSenseImuFeed: gyro@{gyro_profile.fps()}Hz accel@{accel_profile.fps()}Hz")
            return True
        return False

    def stop(self) -> None:
        if self._motion_sensor is None:
            return
        try:
            self._motion_sensor.stop()
            self._motion_sensor.close()
        except Exception:
            pass
        self._motion_sensor = None

    @property
    def R(self) -> np.ndarray:
        with self._lock:
            return self._madgwick.R

    def R_at(self, wall_clock_ts: float) -> np.ndarray:
        with self._lock:
            return self._madgwick.R_at(wall_clock_ts)

    def _on_motion(self, frame: Any) -> None:
        try:
            import pyrealsense2 as rs
            stream_type = frame.get_profile().stream_type()
            if stream_type == rs.stream.gyro:
                gyro_sample = frame.as_motion_frame().get_motion_data()
                gyro_camera_link = self._imu_to_camera_link @ np.array(
                    [gyro_sample.x, gyro_sample.y, gyro_sample.z], dtype=np.float32
                )
                timestamp_s = frame.get_timestamp() / MILLISECONDS_PER_SECOND
                with self._lock:
                    self._madgwick.update(gyro_camera_link, self._last_accel, timestamp_s)
            elif stream_type == rs.stream.accel:
                accel_sample = frame.as_motion_frame().get_motion_data()
                with self._lock:
                    self._last_accel = self._imu_to_camera_link @ np.array(
                        [accel_sample.x, accel_sample.y, accel_sample.z], dtype=np.float32
                    )
        except Exception:
            pass

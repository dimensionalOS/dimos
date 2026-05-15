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

from __future__ import annotations

import math
import threading
import time
from typing import Any, Protocol

from pyparrot.Anafi import Anafi
from pyparrot.DroneVision import DroneVision
from pyparrot.Model import Model
from reactivex import Observable
from reactivex.subject import Subject

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_TELEMETRY_HZ = 2.0
_VIDEO_HZ = 30.0


class AnafiConnectionProtocol(Protocol):
    def connect(self) -> bool: ...
    def disconnect(self) -> None: ...

    def odom_stream(self) -> Observable[PoseStamped]: ...
    def video_stream(self) -> Observable[Image]: ...
    def telemetry_stream(self) -> Observable[dict[str, Any]]: ...

    def move(self, twist: Twist, duration: float = 0.0) -> bool: ...
    def move_relative(self, dx: float, dy: float, dz: float, dradians: float = 0.0) -> bool: ...
    def takeoff(self, timeout: float = 5.0) -> bool: ...
    def land(self, timeout: float = 5.0) -> bool: ...
    def emergency_stop(self) -> bool: ...


class AnafiConnection:
    def __init__(
        self,
        ip_address: str,
        num_retries: int = 10,
        velocity_gain: float = 50.0,
        yaw_rate_gain: float = 50.0,
        video_buffer_size: int = 30,
    ) -> None:
        self.ip_address = ip_address
        self.num_retries = num_retries
        self.velocity_gain = velocity_gain
        self.yaw_rate_gain = yaw_rate_gain
        self.video_buffer_size = video_buffer_size
        self.connected = False

        self._odom_subject: Subject[PoseStamped] = Subject()
        self._video_subject: Subject[Image] = Subject()
        self._telemetry_subject: Subject[dict[str, Any]] = Subject()

        self._anafi: Anafi | None = None
        self._vision: DroneVision | None = None
        self._running = False
        self._video_thread: threading.Thread | None = None

    def connect(self) -> bool:
        logger.info(f"{self.__class__.__name__}: connecting to {self.ip_address}")
        self._anafi = Anafi(drone_type=Model.ANAFI, ip_address=self.ip_address)

        if not self._anafi.connect(num_retries=self.num_retries):
            logger.error(f"{self.__class__.__name__}: failed to connect to {self.ip_address}")
            return False

        self._anafi.smart_sleep(1)
        self._anafi.ask_for_state_update()
        self._anafi.smart_sleep(1)

        self._anafi.set_user_sensor_callback(self._on_sensor_update, args=())

        try:
            self._vision = DroneVision(
                self._anafi,
                model=Model.ANAFI,
                buffer_size=self.video_buffer_size,
                cleanup_old_images=True,
            )
            if self._vision.open_video():
                self._running = True
                self._video_thread = threading.Thread(target=self._video_loop, daemon=True)
                self._video_thread.start()
            else:
                logger.warning(f"{self.__class__.__name__}: video stream failed to start")
                self._vision = None
        except Exception as exc:
            logger.warning(f"{self.__class__.__name__}: vision disabled ({exc})")
            self._vision = None

        self.connected = True
        return True

    def disconnect(self) -> None:
        self._running = False
        if self._video_thread is not None and self._video_thread.is_alive():
            self._video_thread.join(timeout=2.0)
        if self._vision is not None:
            try:
                self._vision.close_video()
            except Exception as exc:
                logger.debug(f"{self.__class__.__name__}: close_video error: {exc}")
        if self._anafi is not None:
            try:
                self._anafi.disconnect()
            except Exception as exc:
                logger.debug(f"{self.__class__.__name__}: disconnect error: {exc}")
        self.connected = False
        self._odom_subject.on_completed()
        self._video_subject.on_completed()
        self._telemetry_subject.on_completed()

    def odom_stream(self) -> Observable[PoseStamped]:
        return self._odom_subject

    def video_stream(self) -> Observable[Image]:
        return self._video_subject

    def telemetry_stream(self) -> Observable[dict[str, Any]]:
        return self._telemetry_subject

    def move(self, twist: Twist, duration: float = 0.0) -> bool:
        if not self.connected or self._anafi is None:
            return False

        def clip(v):
            return max(-100.0, min(100.0, float(v)))

        try:
            self._anafi.fly_direct(
                roll=clip(-twist.linear.y * self.velocity_gain),
                pitch=clip(twist.linear.x * self.velocity_gain),
                yaw=clip(-twist.angular.z * self.yaw_rate_gain),
                vertical_movement=clip(twist.linear.z * self.velocity_gain),
                duration=duration if duration > 0 else None,
            )
        except Exception as exc:
            logger.warning(f"{self.__class__.__name__}: fly_direct failed: {exc}")
            return False
        return True

    def move_relative(self, dx: float, dy: float, dz: float, dradians: float = 0.0) -> bool:
        if not self.connected or self._anafi is None:
            return False
        try:
            self._anafi.move_relative(dx=dx, dy=dy, dz=dz, dradians=dradians)
        except Exception as exc:
            logger.warning(f"{self.__class__.__name__}: move_relative failed: {exc}")
            return False
        return True

    def takeoff(self, timeout: float = 5.0) -> bool:
        if not self.connected or self._anafi is None:
            return False
        return bool(self._anafi.safe_takeoff(timeout))

    def land(self, timeout: float = 5.0) -> bool:
        if not self.connected or self._anafi is None:
            return False
        return bool(self._anafi.safe_land(timeout))

    def emergency_stop(self) -> bool:
        if not self.connected or self._anafi is None:
            return False
        try:
            self._anafi.emergency()
        except Exception as exc:
            logger.warning(f"{self.__class__.__name__}: emergency stop failed: {exc}")
            return False
        return True

    def _on_sensor_update(self, *_: Any) -> None:
        if self._anafi is None:
            return
        sensors = self._anafi.sensors
        logger.info(f"{self.__class__.__name__}: sensor update: {sensors}")
        altitude = float(sensors.sensors_dict.get("AltitudeChanged_altitude", 0.0) or 0.0)
        roll = float(sensors.sensors_dict.get("AttitudeChanged_roll", 0.0) or 0.0)
        pitch = -float(sensors.sensors_dict.get("AttitudeChanged_pitch", 0.0) or 0.0)
        yaw = -float(sensors.sensors_dict.get("AttitudeChanged_yaw", 0.0) or 0.0)
        logger.info(
            f"{self.__class__.__name__}: altitude={altitude:.2f} m, "
            f"roll={math.degrees(roll):.1f} pitch={math.degrees(pitch):.1f} yaw={math.degrees(yaw):.1f} deg"
        )
        self._odom_subject.on_next(
            PoseStamped(
                position=Vector3(0.0, 0.0, altitude),
                orientation=Quaternion.from_euler(Vector3(roll, pitch, yaw)),
                frame_id="world",
                ts=time.time(),
            )
        )

        telemetry: dict[str, Any] = {
            "battery": sensors.sensors_dict.get("BatteryStateChanged_percent", None),
            "flying_state": sensors.sensors_dict.get("FlyingStateChanged_state", None),
            "altitude": altitude,
            "roll_deg": math.degrees(roll),
            "pitch_deg": math.degrees(pitch),
            "heading_deg": math.degrees(yaw),
        }
        gps = sensors.sensors_dict.get("GpsLocationChanged_latitude", None)
        if gps is not None:
            telemetry["gps"] = {
                "lat": gps,
                "lon": sensors.sensors_dict.get("GpsLocationChanged_longitude", None),
            }
        self._telemetry_subject.on_next(telemetry)

    def _video_loop(self) -> None:
        period = 1.0 / _VIDEO_HZ
        last: Any = None
        while self._running and self._vision is not None:
            frame = self._vision.get_latest_valid_picture()
            if frame is not None and frame is not last:
                self._video_subject.on_next(Image.from_numpy(frame, format=ImageFormat.BGR))
                last = frame
            time.sleep(period)


class FakeAnafiConnection:
    def __init__(self, **_: Any) -> None:
        self._odom_subject: Subject[PoseStamped] = Subject()
        self._video_subject: Subject[Image] = Subject()
        self._telemetry_subject: Subject[dict[str, Any]] = Subject()

    def connect(self) -> bool:
        return True

    def disconnect(self) -> None:
        pass

    def odom_stream(self) -> Observable[PoseStamped]:
        return self._odom_subject

    def video_stream(self) -> Observable[Image]:
        return self._video_subject

    def telemetry_stream(self) -> Observable[dict[str, Any]]:
        return self._telemetry_subject

    def move(self, twist: Twist, duration: float = 0.0) -> bool:
        return True

    def move_relative(self, dx: float, dy: float, dz: float, dradians: float = 0.0) -> bool:
        return True

    def takeoff(self, timeout: float = 5.0) -> bool:
        return True

    def land(self, timeout: float = 5.0) -> bool:
        return True

    def emergency_stop(self) -> bool:
        return True

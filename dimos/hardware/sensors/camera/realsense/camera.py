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

import atexit
from collections import deque
import threading
import time
from typing import TYPE_CHECKING

import numpy as np
from pydantic import Field
import reactivex as rx
from scipy.spatial.transform import Rotation

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.core.transport_factory import make_transport
from dimos.hardware.sensors.camera.spec import (
    OPTICAL_ROTATION,
    DepthCameraConfig,
    DepthCameraHardware,
)
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.ImuInfo import ImuInfo
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.spec import perception
from dimos.utils.logging_config import setup_logger
from dimos.utils.reactive import backpressure

logger = setup_logger()

MOTION_MODULE_NAME = "Motion Module"
IMU_CLOCK_WINDOW_SECONDS = 30


def ms_to_s(milliseconds: float) -> float:
    return milliseconds / 1000.0


if TYPE_CHECKING:
    import pyrealsense2 as rs  # type: ignore[import-not-found,import-untyped]

logger = setup_logger()


def default_base_transform() -> Transform:
    """Default identity transform for camera mounting."""
    return Transform(
        translation=Vector3(0.0, 0.0, 0.0),
        rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
    )


def default_imu_info() -> ImuInfo:
    """D455 IMU noise model, measured by the kalibr run in datasets/d455."""
    return ImuInfo(
        gyro_noise_density=2.0e-4,
        gyro_random_walk=1.0e-5,
        accel_noise_density=1.8e-3,
        accel_random_walk=1.0e-4,
    )


class RealSenseCameraConfig(ModuleConfig, DepthCameraConfig):
    width: int = 848
    height: int = 480
    fps: int = 15
    camera_name: str = "camera"
    base_frame_id: str = "base_link"
    base_transform: Transform | None = Field(default_factory=default_base_transform)
    align_depth_to_color: bool = True
    enable_depth: bool = True
    enable_color: bool = True
    # On, auto-exposure halves the color frame rate to lengthen exposure.
    color_auto_exposure_priority: bool = False
    enable_pointcloud: bool = False
    enable_infrared: bool = False
    # Dots make depth much better and the IR pair useless for feature tracking.
    emitter_enabled: bool = True
    enable_imu: bool = False
    # Gyro rate, and so the Imu output rate.
    imu_hz: int = 400
    # Noise model published on ``imu_info``; the driver stamps frame and rate itself.
    imu_info: ImuInfo | None = Field(default_factory=default_imu_info)
    pointcloud_fps: float = 5.0
    camera_info_fps: float = 1.0
    serial_number: str | None = None


class RealSenseCamera(DepthCameraHardware, Module, perception.DepthCamera):
    config: RealSenseCameraConfig
    color_image: Out[Image]
    depth_image: Out[Image]
    infrared_left: Out[Image]
    infrared_right: Out[Image]
    imu: Out[Imu]
    imu_info: Out[ImuInfo]
    pointcloud: Out[PointCloud2]
    camera_info: Out[CameraInfo]
    depth_camera_info: Out[CameraInfo]
    infrared_left_camera_info: Out[CameraInfo]
    infrared_right_camera_info: Out[CameraInfo]
    tf: Out[TFMessage]

    @property
    def _camera_link(self) -> str:
        return f"{self.config.camera_name}_link"

    @property
    def _color_frame(self) -> str:
        return f"{self.config.camera_name}_color_frame"

    @property
    def _color_optical_frame(self) -> str:
        return f"{self.config.camera_name}_color_optical_frame"

    @property
    def _depth_frame(self) -> str:
        return f"{self.config.camera_name}_depth_frame"

    @property
    def _depth_optical_frame(self) -> str:
        return f"{self.config.camera_name}_depth_optical_frame"

    @property
    def _infra1_frame(self) -> str:
        return f"{self.config.camera_name}_infra1_frame"

    @property
    def _infra1_optical_frame(self) -> str:
        return f"{self.config.camera_name}_infra1_optical_frame"

    @property
    def _infra2_frame(self) -> str:
        return f"{self.config.camera_name}_infra2_frame"

    @property
    def _infra2_optical_frame(self) -> str:
        return f"{self.config.camera_name}_infra2_optical_frame"

    @property
    def _imu_frame(self) -> str:
        return f"{self.config.camera_name}_accel_frame"

    @property
    def _imu_optical_frame(self) -> str:
        # accel and gyro are co-located on the motion module.
        return f"{self.config.camera_name}_accel_optical_frame"

    def __init__(self, *args, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(*args, **kwargs)
        self._pipeline: rs.pipeline | None = None
        self._profile: rs.pipeline_profile | None = None
        self._imu_pipeline: rs.pipeline | None = None
        self._accel_history: deque[tuple[float, tuple[float, float, float]]] = deque(maxlen=2)
        self._pending_gyro: deque[tuple[float, tuple[float, float, float]]] = deque(maxlen=16)
        self._imu_offsets: deque[tuple[int, float]] = deque()
        self._imu_offset: float = 0.0
        self._align: rs.align | None = None
        self._mount_edges: list[tuple[str, str, Vector3, Quaternion]] = []
        self._running = False
        self._thread: threading.Thread | None = None
        self._color_camera_info: CameraInfo | None = None
        self._depth_camera_info: CameraInfo | None = None
        self._infra1_camera_info: CameraInfo | None = None
        self._infra2_camera_info: CameraInfo | None = None
        self._depth_scale: float = 0.001
        self._last_frame_numbers: dict[str, int] = {}
        self._last_hardware_ts: float | None = None
        self._dropped_frames: dict[str, int] = {}
        self._repeated_frames: dict[str, int] = {}
        # Pointcloud generation state
        self._latest_color_img: Image | None = None
        self._latest_depth_img: Image | None = None
        self._pointcloud_lock = threading.Lock()

    @rpc
    def start(self) -> None:
        import pyrealsense2 as rs

        # The motion module refuses to open ("No device connected") once the
        # video pipeline holds the device, so the IMU pipeline must start first.
        if self.config.enable_imu:
            self._start_imu()

        self._pipeline = rs.pipeline()
        config = rs.config()

        if self.config.serial_number:
            config.enable_device(self.config.serial_number)

        if self.config.enable_color:
            config.enable_stream(
                rs.stream.color,
                self.config.width,
                self.config.height,
                rs.format.bgr8,
                self.config.fps,
            )

        if self.config.enable_depth:
            config.enable_stream(
                rs.stream.depth,
                self.config.width,
                self.config.height,
                rs.format.z16,
                self.config.fps,
            )

        if self.config.enable_infrared:
            # index 1 = left imager, index 2 = right
            for ir_index in (1, 2):
                config.enable_stream(
                    rs.stream.infrared,
                    ir_index,
                    self.config.width,
                    self.config.height,
                    rs.format.y8,
                    self.config.fps,
                )

        self._profile = self._pipeline.start(config)
        self._require_global_time()

        # The IR imagers are the depth sensor, and it owns the emitter option.
        if self.config.enable_depth or self.config.enable_infrared:
            depth_sensor = self._profile.get_device().first_depth_sensor()
            self._depth_scale = depth_sensor.get_depth_scale()
            if depth_sensor.supports(rs.option.emitter_enabled):
                depth_sensor.set_option(
                    rs.option.emitter_enabled, 1.0 if self.config.emitter_enabled else 0.0
                )

        if self.config.enable_color:
            color_sensor = self._profile.get_device().first_color_sensor()
            if color_sensor.supports(rs.option.auto_exposure_priority):
                color_sensor.set_option(
                    rs.option.auto_exposure_priority,
                    1.0 if self.config.color_auto_exposure_priority else 0.0,
                )

        if self.config.align_depth_to_color and self.config.enable_depth:
            if self.config.enable_color:
                self._align = rs.align(rs.stream.color)
            else:
                logger.info("align_depth_to_color ignored: color stream is disabled")

        self._build_camera_info()
        self._build_mount_edges()

        self._running = True
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

        if self.config.enable_pointcloud and self.config.enable_depth:
            interval_sec = 1.0 / self.config.pointcloud_fps
            self.register_disposable(
                backpressure(rx.interval(interval_sec)).subscribe(
                    on_next=lambda _: self._generate_pointcloud(),
                    on_error=lambda error: logger.error("RealSense pointcloud: %s", error),
                )
            )

        interval_sec = 1.0 / self.config.camera_info_fps
        self.register_disposable(
            rx.interval(interval_sec).subscribe(
                on_next=lambda _: self._publish_camera_info(),
                on_error=lambda error: logger.error("RealSense camera_info: %s", error),
            )
        )

    def _start_imu(self) -> None:
        """Stream the motion module on its own pipeline and callback."""
        import pyrealsense2 as rs

        offered = self._stream_rates(rs.stream.gyro)
        if self.config.imu_hz not in offered:
            raise ValueError(
                f"imu_hz={self.config.imu_hz} is not offered by this camera; it has {offered}"
            )
        accel_hz = max(self._stream_rates(rs.stream.accel))

        imu_config = rs.config()
        if self.config.serial_number:
            imu_config.enable_device(self.config.serial_number)
        imu_config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, accel_hz)
        imu_config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, self.config.imu_hz)

        self._imu_pipeline = rs.pipeline()
        imu_profile = self._imu_pipeline.start(imu_config, self._on_motion_frame)

        # librealsense's global-time regression polls the device over USB, and once the video
        # streams saturate the bus the late replies skew the least-squares fit into a rate
        # error that walks the IMU stamps seconds away from the host clock. Disabling it is
        # Intel's own standing advice, and the fix for the fit is unreleased:
        # https://github.com/IntelRealSense/librealsense/pull/15360
        # https://github.com/IntelRealSense/librealsense/issues/9131
        # The raw hardware clock is stable, so take it and re-anchor it in _to_host_time.
        #
        # The option only reaches the sensor object owned by the pipeline that opened it, so
        # this has to go through imu_profile rather than any other device handle.
        for sensor in imu_profile.get_device().query_sensors():
            if sensor.get_info(rs.camera_info.name) == MOTION_MODULE_NAME:
                sensor.set_option(rs.option.global_time_enabled, 0.0)

    def _stream_rates(self, stream_type: rs.stream) -> list[int]:
        """Every rate the device offers for a stream."""
        import pyrealsense2 as rs

        rates = {
            profile.fps()
            for device in rs.context().query_devices()
            if not self.config.serial_number
            or device.get_info(rs.camera_info.serial_number) == self.config.serial_number
            for sensor in device.query_sensors()
            for profile in sensor.get_stream_profiles()
            if profile.stream_type() == stream_type
        }
        return sorted(rates)

    def _require_global_time(self) -> None:
        """Put every sensor on the host clock rather than its own boot clock."""
        import pyrealsense2 as rs

        if self._profile is None:
            return
        for sensor in self._profile.get_device().query_sensors():
            if sensor.get_info(rs.camera_info.name) == MOTION_MODULE_NAME:
                continue  # _start_imu deliberately keeps it on the hardware clock
            if not sensor.supports(rs.option.global_time_enabled):
                logger.warning(
                    "RealSense %s has no global timestamps, so its stream is on the "
                    "device's own clock and cannot be fused with the others",
                    sensor.get_info(rs.camera_info.name),
                )
                continue
            sensor.set_option(rs.option.global_time_enabled, 1.0)

    def _on_motion_frame(self, frame: rs.frame) -> None:
        import pyrealsense2 as rs

        motion = frame.as_motion_frame()
        if not motion:
            return
        data = motion.get_motion_data()
        ts = self._to_host_time(ms_to_s(motion.get_timestamp()))
        stream = motion.get_profile().stream_type()
        if stream == rs.stream.accel:
            self._accel_history.append((ts, (data.x, data.y, data.z)))
        elif stream == rs.stream.gyro:
            self._pending_gyro.append((ts, (data.x, data.y, data.z)))
        self._publish_paired_imu()

    def _to_host_time(self, device_ts: float) -> float:
        """Map a motion-module hardware stamp onto the host clock.

        Delivery latency only ever pushes an arrival later, so the smallest offset seen
        recently is the closest estimate of the true one. Keeping one minimum per second
        lets the estimate follow the device's slow drift without rescanning every sample.
        """
        now = time.time()
        offset = now - device_ts
        current_second = int(now)
        if self._imu_offsets and self._imu_offsets[-1][0] == current_second:
            self._imu_offsets[-1] = (current_second, min(self._imu_offsets[-1][1], offset))
        else:
            self._imu_offsets.append((current_second, offset))
            while self._imu_offsets[0][0] < current_second - IMU_CLOCK_WINDOW_SECONDS:
                self._imu_offsets.popleft()
            self._imu_offset = min(seen for _, seen in self._imu_offsets)
        return device_ts + self._imu_offset

    def _publish_paired_imu(self) -> None:
        """Emit one Imu per gyro sample, with the accelerometer read at that instant.

        The two are sampled independently and never share a timestamp.
        """
        while self._pending_gyro and len(self._accel_history) == 2:
            (start_ts, start), (end_ts, end) = self._accel_history
            ts, angular = self._pending_gyro[0]
            if ts > end_ts:
                return  # no accelerometer sample past it yet
            self._pending_gyro.popleft()
            span = end_ts - start_ts
            ratio = 0.0 if span <= 0.0 else max(0.0, min(1.0, (ts - start_ts) / span))
            linear = tuple(a + (b - a) * ratio for a, b in zip(start, end, strict=True))
            self.imu.publish(
                Imu(
                    angular_velocity=Vector3(*angular),
                    linear_acceleration=Vector3(*linear),
                    frame_id=self._imu_optical_frame,
                    ts=ts,
                )
            )

    def _publish_camera_info(self) -> None:
        ts = self._last_hardware_ts
        if ts is None or not self._running:
            return
        for info, stream in (
            (self._color_camera_info, self.camera_info),
            (self._depth_camera_info, self.depth_camera_info),
            (self._infra1_camera_info, self.infrared_left_camera_info),
            (self._infra2_camera_info, self.infrared_right_camera_info),
        ):
            if info is not None:
                stream.publish(info.with_ts(ts))
        if self.config.enable_imu and self.config.imu_info is not None:
            imu_info = self.config.imu_info.with_ts(ts)
            imu_info.frame_id = self._imu_optical_frame
            imu_info.frequency = float(self.config.imu_hz)
            self.imu_info.publish(imu_info)

    def _build_camera_info(self) -> None:
        import pyrealsense2 as rs

        if self._profile is None:
            return

        # Color camera info
        color_intrinsics = None
        if self.config.enable_color:
            color_stream = self._profile.get_stream(rs.stream.color).as_video_stream_profile()
            color_intrinsics = color_stream.get_intrinsics()
            self._color_camera_info = self._intrinsics_to_camera_info(
                color_intrinsics, self._color_optical_frame
            )

        # Depth camera info
        if self.config.enable_depth:
            if self.config.align_depth_to_color and color_intrinsics is not None:
                # When aligned to color, depth uses color intrinsics and frame
                self._depth_camera_info = self._intrinsics_to_camera_info(
                    color_intrinsics, self._color_optical_frame
                )
            else:
                depth_stream = self._profile.get_stream(rs.stream.depth).as_video_stream_profile()
                depth_intrinsics = depth_stream.get_intrinsics()
                self._depth_camera_info = self._intrinsics_to_camera_info(
                    depth_intrinsics, self._depth_optical_frame
                )

        # Infrared stereo pair camera info
        if self.config.enable_infrared:
            infra1_stream = self._profile.get_stream(
                rs.stream.infrared, 1
            ).as_video_stream_profile()
            self._infra1_camera_info = self._intrinsics_to_camera_info(
                infra1_stream.get_intrinsics(), self._infra1_optical_frame
            )
            infra2_stream = self._profile.get_stream(
                rs.stream.infrared, 2
            ).as_video_stream_profile()
            self._infra2_camera_info = self._intrinsics_to_camera_info(
                infra2_stream.get_intrinsics(), self._infra2_optical_frame
            )
            # P[3] is -fx * baseline; left at 0 a stereo consumer sees infinite depth.
            # The pair is rectified, so the whole of their offset is along x.
            baseline = abs(float(infra2_stream.get_extrinsics_to(infra1_stream).translation[0]))
            projection = list(self._infra2_camera_info.P)
            projection[3] = -projection[0] * baseline
            self._infra2_camera_info.P = projection

    def _intrinsics_to_camera_info(self, intrinsics: rs.intrinsics, frame_id: str) -> CameraInfo:
        import pyrealsense2 as rs

        info = CameraInfo.from_intrinsics(
            intrinsics.fx,
            intrinsics.fy,
            intrinsics.ppx,
            intrinsics.ppy,
            intrinsics.width,
            intrinsics.height,
            frame_id,
        )
        info.D = list(intrinsics.coeffs) if intrinsics.coeffs else []
        info.distortion_model = {
            rs.distortion.none: "",
            rs.distortion.modified_brown_conrady: "plumb_bob",
            rs.distortion.inverse_brown_conrady: "plumb_bob",
            rs.distortion.ftheta: "equidistant",
            rs.distortion.brown_conrady: "plumb_bob",
            rs.distortion.kannala_brandt4: "equidistant",
        }.get(intrinsics.model, "")
        return info

    def _device_stream_profile(
        self, stream_type: rs.stream, index: int | None = None
    ) -> rs.stream_profile | None:
        """A profile for one of the device's streams, running or not."""
        if self._profile is None:
            return None
        for sensor in self._profile.get_device().query_sensors():
            for profile in sensor.get_stream_profiles():
                if profile.stream_type() == stream_type and (
                    index is None or profile.stream_index() == index
                ):
                    return profile
        return None

    def _build_mount_edges(self) -> None:
        """Place every imager below camera_link, which sits on the depth imager."""
        import pyrealsense2 as rs

        streams: list[tuple[str, rs.stream, int | None]] = [
            (self._depth_frame, rs.stream.depth, None)
        ]
        if self.config.enable_color:
            streams.append((self._color_frame, rs.stream.color, None))
        if self.config.enable_infrared:
            streams.append((self._infra1_frame, rs.stream.infrared, 1))
            streams.append((self._infra2_frame, rs.stream.infrared, 2))
        if self.config.enable_imu:
            streams.append((self._imu_frame, rs.stream.accel, None))

        origin = self._device_stream_profile(rs.stream.depth, None)
        if origin is None:
            logger.warning("RealSense has no depth stream; publishing no camera frames on tf")
            return

        self._mount_edges = []
        for frame_id, stream_type, index in streams:
            profile = self._device_stream_profile(stream_type, index)
            if profile is None:
                logger.warning("RealSense has no stream for %s; not published on tf", frame_id)
                continue
            translation, rotation = self._extrinsics_to_body(profile.get_extrinsics_to(origin))
            self._mount_edges.append((self._camera_link, frame_id, translation, rotation))
            self._mount_edges.append(
                (
                    frame_id,
                    f"{frame_id.removesuffix('_frame')}_optical_frame",
                    Vector3(0.0, 0.0, 0.0),
                    OPTICAL_ROTATION,
                )
            )

    @staticmethod
    def _extrinsics_to_body(extrinsics: rs.extrinsics) -> tuple[Vector3, Quaternion]:
        """Convert a librealsense extrinsic (optical axes) into body (REP-103) axes."""
        body_from_optical = np.eye(4)
        body_from_optical[:3, :3] = Rotation.from_quat(
            [OPTICAL_ROTATION.x, OPTICAL_ROTATION.y, OPTICAL_ROTATION.z, OPTICAL_ROTATION.w]
        ).as_matrix()

        optical = np.eye(4)
        optical[:3, :3] = np.array(extrinsics.rotation).reshape(3, 3)
        optical[:3, 3] = np.array(extrinsics.translation)

        body = body_from_optical @ optical @ body_from_optical.T
        rotation = Rotation.from_matrix(body[:3, :3]).as_quat()  # [x, y, z, w]
        return Vector3(*body[:3, 3]), Quaternion(*rotation)

    def _fresh(self, key: str, frame: rs.frame | None) -> float | None:
        """This frame's own timestamp, or None if the stream has not advanced.

        wait_for_frames() returns the latest frame of each stream, and under load they
        desync, so each stream is gated on its own frame number.
        """
        if frame is None:
            return None
        number = frame.get_frame_number()
        previous = self._last_frame_numbers.get(key)
        if number == previous:
            self._repeated_frames[key] = self._repeated_frames.get(key, 0) + 1
            return None
        if previous is not None and number > previous + 1:
            missed = number - previous - 1
            self._dropped_frames[key] = self._dropped_frames.get(key, 0) + missed
            logger.warning(
                "RealSense %s dropped %d frame(s) at %d (%d total)",
                key,
                missed,
                number,
                self._dropped_frames[key],
            )
        self._last_frame_numbers[key] = number
        stamp = ms_to_s(float(frame.get_timestamp()))
        self._last_hardware_ts = stamp
        return stamp

    def _capture_loop(self) -> None:
        import cv2

        while self._running and self._pipeline is not None:
            try:
                frames = self._pipeline.wait_for_frames(timeout_ms=1000)
            except AttributeError:
                # Pipeline cleared by stop() - exit loop
                break
            except RuntimeError:
                # Frame timeouts are transient, from warm-up or USB stalls. Keep waiting.
                if self._running:
                    logger.warning("RealSense: no frames within 1s - retrying")
                continue

            # Grab the infrared stereo pair from the raw frameset before align()
            # (align rebuilds the frameset around depth+color and drops IR).
            infra1_frame = frames.get_infrared_frame(1) if self.config.enable_infrared else None
            infra2_frame = frames.get_infrared_frame(2) if self.config.enable_infrared else None

            if self._align is not None:
                frames = self._align.process(frames)

            color_frame = frames.get_color_frame() if self.config.enable_color else None
            depth_frame = frames.get_depth_frame() if self.config.enable_depth else None

            color_ts = self._fresh("color", color_frame)
            depth_ts = self._fresh("depth", depth_frame)
            infra1_ts = self._fresh("infra1", infra1_frame)
            infra2_ts = self._fresh("infra2", infra2_frame)

            # Process color
            color_img = None
            if color_frame and color_ts is not None:
                color_data = np.asanyarray(color_frame.get_data())
                color_data = cv2.cvtColor(color_data, cv2.COLOR_BGR2RGB)
                color_img = Image(
                    data=color_data,
                    format=ImageFormat.RGB,
                    frame_id=self._color_optical_frame,
                    ts=color_ts,
                )
                self.color_image.publish(color_img)

            # Process depth
            depth_img = None
            if depth_frame and depth_ts is not None:
                depth_data = np.asanyarray(depth_frame.get_data())
                depth_frame_id = (
                    self._color_optical_frame
                    if self._align is not None
                    else self._depth_optical_frame
                )
                depth_img = Image(
                    data=depth_data,
                    format=ImageFormat.DEPTH16,
                    frame_id=depth_frame_id,
                    ts=depth_ts,
                )
                self.depth_image.publish(depth_img)

            if infra1_frame and infra1_ts is not None:
                self.infrared_left.publish(
                    Image(
                        data=np.asanyarray(infra1_frame.get_data()),
                        format=ImageFormat.GRAY,
                        frame_id=self._infra1_optical_frame,
                        ts=infra1_ts,
                    )
                )
            if infra2_frame and infra2_ts is not None:
                self.infrared_right.publish(
                    Image(
                        data=np.asanyarray(infra2_frame.get_data()),
                        format=ImageFormat.GRAY,
                        frame_id=self._infra2_optical_frame,
                        ts=infra2_ts,
                    )
                )

            # Store latest images for pointcloud generation
            if self.config.enable_pointcloud and color_img is not None and depth_img is not None:
                with self._pointcloud_lock:
                    self._latest_color_img = color_img
                    self._latest_depth_img = depth_img

            latest = [
                stamp for stamp in (color_ts, depth_ts, infra1_ts, infra2_ts) if stamp is not None
            ]
            if latest:
                self._publish_tf(max(latest))

    def _publish_tf(self, ts: float) -> None:
        transforms = []

        if self.config.base_transform is not None:
            transforms.append(
                Transform(
                    translation=self.config.base_transform.translation,
                    rotation=self.config.base_transform.rotation,
                    frame_id=self.config.base_frame_id,
                    child_frame_id=self._camera_link,
                    ts=ts,
                )
            )
        for parent, child, translation, rotation in self._mount_edges:
            transforms.append(
                Transform(
                    translation=translation,
                    rotation=rotation,
                    frame_id=parent,
                    child_frame_id=child,
                    ts=ts,
                )
            )
        self.tf.publish(TFMessage(*transforms))

    def _generate_pointcloud(self) -> None:
        """Generate and publish pointcloud from latest images (called by rx.interval)."""
        with self._pointcloud_lock:
            color_img = self._latest_color_img
            depth_img = self._latest_depth_img

        if color_img is None or depth_img is None or self._color_camera_info is None:
            return

        try:
            pcd = PointCloud2.from_rgbd(
                color_image=color_img,
                depth_image=depth_img,
                camera_info=self._color_camera_info,
                depth_scale=self._depth_scale,
            )
            pcd = pcd.voxel_downsample(0.005)
            self.pointcloud.publish(pcd)
        except Exception as error:
            logger.error("RealSense pointcloud generation failed: %s", error)

    @rpc
    def stop(self) -> None:
        self._running = False

        if self._imu_pipeline:
            try:
                self._imu_pipeline.stop()
            except Exception:
                pass  # Pipeline might already be stopped
            self._imu_pipeline = None

        # Stop pipeline first to unblock wait_for_frames()
        if self._pipeline:
            try:
                self._pipeline.stop()
            except Exception:
                pass  # Pipeline might already be stopped
            self._pipeline = None

        # Now join the thread (should exit quickly since pipeline is stopped)
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
            if self._thread.is_alive():
                # Force thread termination by clearing reference
                self._thread = None

        self._profile = None
        self._align = None
        if self._dropped_frames or self._repeated_frames:
            logger.info(
                "RealSense capture ended: dropped %s, repeats suppressed %s",
                dict(self._dropped_frames),
                dict(self._repeated_frames),
            )
        self._last_frame_numbers = {}
        self._latest_color_img = None
        self._latest_depth_img = None
        super().stop()

    @rpc
    def get_color_camera_info(self) -> CameraInfo | None:
        return self._color_camera_info

    @rpc
    def get_depth_camera_info(self) -> CameraInfo | None:
        return self._depth_camera_info

    @rpc
    def get_depth_scale(self) -> float:
        return self._depth_scale


def main() -> None:
    dimos = ModuleCoordinator()
    dimos.start()

    camera = dimos.deploy(RealSenseCamera, enable_pointcloud=True, pointcloud_fps=5.0)
    camera.color_image.transport = make_transport("/camera/color", Image)
    camera.depth_image.transport = make_transport("/camera/depth", Image)
    camera.pointcloud.transport = make_transport("/camera/pointcloud", PointCloud2)
    camera.camera_info.transport = make_transport("/camera/color_info", CameraInfo)
    camera.depth_camera_info.transport = make_transport("/camera/depth_info", CameraInfo)

    def cleanup() -> None:
        try:
            dimos.stop()
        except Exception:
            pass

    atexit.register(cleanup)
    dimos.start_all_modules()

    try:
        while True:
            time.sleep(0.1)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        atexit.unregister(cleanup)
        cleanup()


if __name__ == "__main__":
    main()

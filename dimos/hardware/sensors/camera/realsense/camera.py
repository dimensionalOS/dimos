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
import threading
import time
from typing import TYPE_CHECKING, Any

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
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.spec import perception
from dimos.utils.logging_config import setup_logger
from dimos.utils.reactive import backpressure

logger = setup_logger()

if TYPE_CHECKING:
    import pyrealsense2 as rs  # type: ignore[import-not-found,import-untyped]


def default_base_transform() -> Transform:
    """Default identity transform for camera mounting."""
    return Transform(
        translation=Vector3(0.0, 0.0, 0.0),
        rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
    )


def device_published_frames(camera_name: str = "camera") -> tuple[str, ...]:
    """The frames :meth:`RealSenseCamera._publish_tf` puts on tf from the device.

    A rig URDF that also describes this camera has to leave these edges alone:
    two publishers means two parents, and tf keeps whichever arrived last. The
    motion-module frames are absent on purpose -- accel and gyro come off a
    separate pipeline with no extrinsic to depth, so the URDF still owns those.
    """
    return tuple(
        f"{camera_name}_{name}"
        for name in (
            "depth_frame",
            "depth_optical_frame",
            "color_frame",
            "color_optical_frame",
            "infra1_frame",
            "infra1_optical_frame",
            "infra2_frame",
            "infra2_optical_frame",
        )
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
    # The colour imager is the heaviest stream the camera produces: bgr8 is 3
    # bytes/pixel against mono8's 1, so at 848x480/30 it is ~37 MB/s — more than
    # the whole IR pair. It used to be enabled, colour-converted and published
    # unconditionally, so VIO captures that never record colour still paid the USB
    # bandwidth, a full-frame cvtColor per frame, and the LCM traffic.
    enable_color: bool = True
    enable_pointcloud: bool = False
    # Publish the left/right infrared imagers as a stereo pair (infrared_left /
    # infrared_right). This is the global-shutter stereo pair used for VIO.
    enable_infrared: bool = False
    # The IR projector paints a dot pattern that greatly improves depth but
    # corrupts the infrared images for feature-based stereo/VIO. Turn it off for
    # clean stereo (relies on ambient/scene texture); keep it on for depth quality.
    emitter_enabled: bool = True
    # Publish the accel+gyro motion module as a combined Imu stream (for VIO). Runs
    # on its own pipeline/thread at the IMU's native rate, independent of the video.
    enable_imu: bool = False
    # Motion-module rates in Hz. One Imu is emitted per gyro sample, so imu_gyro_hz
    # sets the IMU output rate. D435i maxes at 400/400; falls back to sensor
    # defaults if the exact rate isn't offered.
    imu_accel_hz: int = 400
    imu_gyro_hz: int = 400
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
    def _imu_optical_frame(self) -> str:
        # accel and gyro are co-located on the D435i motion module.
        return f"{self.config.camera_name}_accel_optical_frame"

    # A slow start is normal; a minute of nothing is a real fault.
    MAX_CONSECUTIVE_TIMEOUTS = 60

    def __init__(self, *args, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(*args, **kwargs)
        self._pipeline: rs.pipeline | None = None
        self._profile: rs.pipeline_profile | None = None
        self._imu_pipeline: rs.pipeline | None = None
        # Latest accel sample (x, y, z), paired with each gyro sample to form an Imu.
        self._latest_accel: tuple[float, float, float] | None = None
        self._align: rs.align | None = None
        self._running = False
        self._thread: threading.Thread | None = None
        self._color_camera_info: CameraInfo | None = None
        self._depth_camera_info: CameraInfo | None = None
        self._infra1_camera_info: CameraInfo | None = None
        self._infra2_camera_info: CameraInfo | None = None
        self._depth_scale: float = 0.001
        self._color_to_depth_extrinsics: rs.extrinsics | None = None
        # frame_id -> that imager's extrinsic to depth, filled in at start.
        self._frame_extrinsics: dict[str, rs.extrinsics] = {}
        # Frame-continuity bookkeeping; see _capture_loop.
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
            # index 1 = left imager, index 2 = right imager (global shutter stereo pair)
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

        if self.config.enable_depth or self.config.enable_infrared:
            depth_sensor = self._profile.get_device().first_depth_sensor()
            self._depth_scale = depth_sensor.get_depth_scale()
            # The projector aids depth but paints dots over the IR stereo pair.
            if depth_sensor.supports(rs.option.emitter_enabled):
                depth_sensor.set_option(
                    rs.option.emitter_enabled, 1.0 if self.config.emitter_enabled else 0.0
                )

        # Aligning to colour needs the colour stream to actually be running.
        if self.config.align_depth_to_color and self.config.enable_depth:
            if self.config.enable_color:
                self._align = rs.align(rs.stream.color)
            else:
                logger.info("align_depth_to_color ignored: colour stream is disabled")

        self._build_camera_info()
        self._get_extrinsics()

        self._running = True
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

        if self.config.enable_imu:
            self._start_imu()

        if self.config.enable_pointcloud and self.config.enable_depth:
            interval_sec = 1.0 / self.config.pointcloud_fps
            self.register_disposable(
                backpressure(rx.interval(interval_sec)).subscribe(
                    on_next=lambda _: self._generate_pointcloud(),
                    on_error=lambda e: print(f"Pointcloud error: {e}"),
                )
            )

        interval_sec = 1.0 / self.config.camera_info_fps
        self.register_disposable(
            rx.interval(interval_sec).subscribe(
                on_next=lambda _: self._publish_camera_info(),
                on_error=lambda e: print(f"CameraInfo error: {e}"),
            )
        )

    def _start_imu(self) -> None:
        """Stream the accel+gyro motion module on its own pipeline and callback.

        The motion module runs at its native rate (~200-400 Hz), far faster than the
        video loop, so it gets a dedicated pipeline with an async frame callback. Each
        gyro sample is paired with the most recent accel sample to form one Imu.
        """
        import pyrealsense2 as rs

        self._imu_pipeline = rs.pipeline()
        imu_config = rs.config()
        if self.config.serial_number:
            imu_config.enable_device(self.config.serial_number)
        try:
            imu_config.enable_stream(
                rs.stream.accel, rs.format.motion_xyz32f, self.config.imu_accel_hz
            )
            imu_config.enable_stream(
                rs.stream.gyro, rs.format.motion_xyz32f, self.config.imu_gyro_hz
            )
            self._imu_pipeline.start(imu_config, self._on_motion_frame)
        except RuntimeError:
            # Requested rate not offered by this unit — fall back to sensor defaults.
            imu_config = rs.config()
            if self.config.serial_number:
                imu_config.enable_device(self.config.serial_number)
            imu_config.enable_stream(rs.stream.accel)
            imu_config.enable_stream(rs.stream.gyro)
            self._imu_pipeline.start(imu_config, self._on_motion_frame)

    def _on_motion_frame(self, frame: rs.frame) -> None:
        import pyrealsense2 as rs

        motion = frame.as_motion_frame()
        if not motion:
            return
        data = motion.get_motion_data()
        stream = motion.get_profile().stream_type()
        if stream == rs.stream.accel:
            self._latest_accel = (data.x, data.y, data.z)
        elif stream == rs.stream.gyro and self._latest_accel is not None:
            ax, ay, az = self._latest_accel
            # Use the RS hardware timestamp (global_time domain: host epoch but
            # hardware-precise), NOT time.time() at the callback — the latter adds
            # Python callback latency/jitter that desyncs cam<->IMU and makes VIO
            # (cuVSLAM etc.) diverge. Both IR and IMU use frame.get_timestamp() so
            # they share one clock. get_timestamp() is ms -> seconds.
            self.imu.publish(
                Imu(
                    angular_velocity=Vector3(data.x, data.y, data.z),
                    linear_acceleration=Vector3(ax, ay, az),
                    frame_id=self._imu_optical_frame,
                    ts=motion.get_timestamp() / 1000.0,
                )
            )

    def _publish_camera_info(self) -> None:
        # The hardware clock, not time.time(): a recording should carry one time
        # base, and camera_info that cannot be lined up with the frames it
        # describes is a trap for anything matching them by timestamp.
        ts = self._last_hardware_ts
        if ts is None:
            return
        if self._color_camera_info:
            self._color_camera_info.ts = ts
            self.camera_info.publish(self._color_camera_info)
        if self._depth_camera_info:
            self._depth_camera_info.ts = ts
            self.depth_camera_info.publish(self._depth_camera_info)
        if self._infra1_camera_info:
            self._infra1_camera_info.ts = ts
            self.infrared_left_camera_info.publish(self._infra1_camera_info)
        if self._infra2_camera_info:
            self._infra2_camera_info.ts = ts
            self.infrared_right_camera_info.publish(self._infra2_camera_info)

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

        # Infrared stereo pair camera info (raw, per-imager intrinsics).
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
            # The right eye's P[3] carries the stereo baseline as -fx * baseline.
            # Left at 0 (the default), every stereo consumer computes infinite
            # depth. Read the baseline off the device rather than hardcoding it:
            # it is per-model (D435 ~50mm, D455 ~95mm) and per-unit.
            extrinsics = infra2_stream.get_extrinsics_to(infra1_stream)
            self._ir_baseline = abs(float(extrinsics.translation[0]))
            projection = list(self._infra2_camera_info.P)
            projection[3] = -projection[0] * self._ir_baseline
            self._infra2_camera_info.P = projection
            logger.info(
                "RealSense IR baseline %.2f mm (fx %.2f) -> right P[3] = %.3f",
                self._ir_baseline * 1000.0,
                projection[0],
                projection[3],
            )

    def _intrinsics_to_camera_info(self, intrinsics: rs.intrinsics, frame_id: str) -> CameraInfo:
        import pyrealsense2 as rs

        fx, fy = intrinsics.fx, intrinsics.fy
        cx, cy = intrinsics.ppx, intrinsics.ppy

        K = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        P = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        D = list(intrinsics.coeffs) if intrinsics.coeffs else []

        distortion_model = {
            rs.distortion.none: "",
            rs.distortion.modified_brown_conrady: "plumb_bob",
            rs.distortion.inverse_brown_conrady: "plumb_bob",
            rs.distortion.ftheta: "equidistant",
            rs.distortion.brown_conrady: "plumb_bob",
            rs.distortion.kannala_brandt4: "equidistant",
        }.get(intrinsics.model, "")

        return CameraInfo(
            height=intrinsics.height,
            width=intrinsics.width,
            distortion_model=distortion_model,
            D=D,
            K=K,
            P=P,
            frame_id=frame_id,
        )

    def _get_extrinsics(self) -> None:
        """Where each imager sits relative to the depth origin, read off the device.

        Per-unit factory calibration, not a datasheet. This is the only source that
        gets a rig right when the nominal number is wrong for the model actually
        plugged in — a D455's 95mm IR baseline published as the D435's 50mm halves
        every depth derived from tf, and nothing downstream can tell.
        """
        import pyrealsense2 as rs

        if self._profile is None or not self.config.enable_depth:
            return
        depth_stream = self._profile.get_stream(rs.stream.depth)

        # The infrared streams need their index; the others are named by type alone.
        streams: list[tuple[str, tuple[Any, ...]]] = []
        if self.config.enable_color:
            streams.append((self._color_frame, (rs.stream.color,)))
        if self.config.enable_infrared:
            streams.append((self._infra1_frame, (rs.stream.infrared, 1)))
            streams.append((self._infra2_frame, (rs.stream.infrared, 2)))

        self._frame_extrinsics = {}
        for frame_id, stream in streams:
            source = self._profile.get_stream(*stream)
            self._frame_extrinsics[frame_id] = source.get_extrinsics_to(depth_stream)
            translation = self._frame_extrinsics[frame_id].translation
            logger.info(
                "RealSense %s at (%.1f, %.1f, %.1f) mm from depth (device calibration)",
                frame_id,
                translation[0] * 1000.0,
                translation[1] * 1000.0,
                translation[2] * 1000.0,
            )
        self._color_to_depth_extrinsics = self._frame_extrinsics.get(self._color_frame)

    def _extrinsics_to_transform(
        self,
        extrinsics: rs.extrinsics,
        frame_id: str,
        child_frame_id: str,
        ts: float,
    ) -> Transform:
        """Convert a librealsense extrinsic into a body-frame (REP-103) transform.

        librealsense reports extrinsics between *optical* frames — x right, y down,
        z forward — while camera_link and its children are body frames — x forward,
        y left, z up. Writing the optical translation straight into a body-frame joint
        puts a lateral imager offset on the forward axis instead. That does not fail;
        it silently misplaces the colour camera by its own baseline, which then shows
        up as unregistered depth and as reprojection error downstream.

        Changing basis means conjugating the whole rigid transform by the optical
        rotation, not just permuting the translation, so any rotation between the two
        imagers lands in the right frame too.
        """
        body_from_optical = np.eye(4)
        body_from_optical[:3, :3] = Rotation.from_quat(
            [
                OPTICAL_ROTATION.x,
                OPTICAL_ROTATION.y,
                OPTICAL_ROTATION.z,
                OPTICAL_ROTATION.w,
            ]
        ).as_matrix()

        optical = np.eye(4)
        optical[:3, :3] = np.array(extrinsics.rotation).reshape(3, 3)
        optical[:3, 3] = np.array(extrinsics.translation)

        body = body_from_optical @ optical @ np.linalg.inv(body_from_optical)
        quat = Rotation.from_matrix(body[:3, :3]).as_quat()  # [x, y, z, w]
        return Transform(
            translation=Vector3(*body[:3, 3]),
            rotation=Quaternion(quat[0], quat[1], quat[2], quat[3]),
            frame_id=frame_id,
            child_frame_id=child_frame_id,
            ts=ts,
        )

    def _fresh(self, key: str, frame: rs.frame | None) -> float | None:
        """This frame's own timestamp, or None if the stream has not advanced.

        wait_for_frames() hands back a frameset assembled from the latest frame of
        each stream, and under load those streams desync: colour can advance while
        depth and infrared stand still. Stamping every stream from
        ``frames.get_timestamp()`` then gives the stalled streams a fresh-looking
        time, and republishing them stores the same image twice -- byte-identical,
        with a repeated stamp, which is what broke consumers requiring strictly
        increasing time. Each stream is therefore gated and stamped on its own
        frame number and its own timestamp.
        """
        if frame is None:
            return None
        number = frame.get_frame_number()
        previous = self._last_frame_numbers.get(key)
        if number == previous:
            self._repeated_frames[key] = self._repeated_frames.get(key, 0) + 1
            return None
        if previous is not None and number > previous + 1:
            # The loss happens in the driver or on the wire, so it can only be
            # counted here -- but a capture that quietly shed frames should not
            # look complete afterwards.
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
        stamp = float(frame.get_timestamp()) / 1000.0
        self._last_hardware_ts = stamp
        return stamp

    def _capture_loop(self) -> None:
        import cv2

        consecutive_timeouts = 0
        while self._running and self._pipeline is not None:
            try:
                frames = self._pipeline.wait_for_frames(timeout_ms=1000)
            except AttributeError:
                # Pipeline went away underneath us.
                break
            except RuntimeError:
                # wait_for_frames() raises this on *timeout* as well as on a stopped
                # pipeline. Treating both as "stopped" meant a slow first frameset --
                # which happens on some startups -- silently killed capture for the whole
                # run, with the IMU still streaming from its own callback so the
                # recording looked alive. Retry, and only give up if frames really have
                # stopped coming.
                if not self._running:
                    break
                consecutive_timeouts += 1
                if consecutive_timeouts == 1 or consecutive_timeouts % 5 == 0:
                    logger.warning(
                        "RealSense: no frameset for %d s (waiting for the video stream)",
                        consecutive_timeouts,
                    )
                if consecutive_timeouts >= self.MAX_CONSECUTIVE_TIMEOUTS:
                    logger.error(
                        "RealSense: no frames for %d s, giving up on video capture",
                        consecutive_timeouts,
                    )
                    break
                continue
            consecutive_timeouts = 0

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
                # When aligned, depth is in color optical frame
                depth_frame_id = (
                    self._color_optical_frame
                    if self.config.align_depth_to_color
                    else self._depth_optical_frame
                )
                depth_img = Image(
                    data=depth_data,
                    format=ImageFormat.DEPTH16,
                    frame_id=depth_frame_id,
                    ts=depth_ts,
                )
                self.depth_image.publish(depth_img)

            # Process infrared stereo pair (single-channel, global shutter)
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

            # Publish TF against whichever stream advanced this iteration.
            latest = [t for t in (color_ts, depth_ts, infra1_ts, infra2_ts) if t is not None]
            if latest:
                self._publish_tf(max(latest))

    def _publish_tf(self, ts: float) -> None:
        """Publish everything at and below ``camera_link``, from the device.

        This module owns the camera's internals on tf, because it is the only thing
        that can read the per-unit factory calibration. Whatever describes the *mount*
        -- a URDF, usually -- owns ``camera_link`` upward, and must not also publish
        these edges: a frame with two parents makes tf pick whichever arrived last.
        """
        transforms = []

        # base_link -> camera_link, for a camera whose mount nothing else describes.
        # Leave base_transform unset when a rig URDF already places the camera.
        if self.config.base_transform is not None:
            base_to_camera = Transform(
                translation=self.config.base_transform.translation,
                rotation=self.config.base_transform.rotation,
                frame_id=self.config.base_frame_id,
                child_frame_id=self._camera_link,
                ts=ts,
            )
            transforms.append(base_to_camera)

        # camera_link -> camera_depth_frame (identity, depth is at camera_link origin)
        camera_link_to_depth = Transform(
            translation=Vector3(0.0, 0.0, 0.0),
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
            frame_id=self._camera_link,
            child_frame_id=self._depth_frame,
            ts=ts,
        )
        transforms.append(camera_link_to_depth)

        # camera_link -> every other imager, then each one's optical child. No
        # inversion: camera_link coincides with the depth frame, so the imager's
        # extrinsic to depth already IS its placement in camera_link, once
        # _extrinsics_to_transform has moved it into body axes.
        for frame_id, extrinsics in self._frame_extrinsics.items():
            transforms.append(
                self._extrinsics_to_transform(extrinsics, self._camera_link, frame_id, ts)
            )
        for frame_id in (self._depth_frame, *self._frame_extrinsics):
            transforms.append(
                Transform(
                    translation=Vector3(0.0, 0.0, 0.0),
                    rotation=OPTICAL_ROTATION,
                    frame_id=frame_id,
                    child_frame_id=f"{frame_id.removesuffix('_frame')}_optical_frame",
                    ts=ts,
                )
            )

        # With depth disabled there are no extrinsics at all, so colour falls back to
        # the camera_link origin rather than vanishing from tf.
        if self.config.enable_color and self._color_frame not in self._frame_extrinsics:
            transforms.append(
                Transform(
                    translation=Vector3(0.0, 0.0, 0.0),
                    rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
                    frame_id=self._camera_link,
                    child_frame_id=self._color_frame,
                    ts=ts,
                )
            )
            transforms.append(
                Transform(
                    translation=Vector3(0.0, 0.0, 0.0),
                    rotation=OPTICAL_ROTATION,
                    frame_id=self._color_frame,
                    child_frame_id=self._color_optical_frame,
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
        except Exception as e:
            print(f"Pointcloud generation error: {e}")

    @rpc
    def stop(self) -> None:
        self._running = False

        # Stop the motion module pipeline (its callback thread) first.
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
        self._color_to_depth_extrinsics = None
        self._frame_extrinsics = {}
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

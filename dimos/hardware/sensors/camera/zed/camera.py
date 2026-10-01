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

from dimos_generated.geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from dimos_generated.sensor_msgs.msg import CameraInfo, Image, PointCloud2
from dimos_generated.std_msgs.msg import Header
from dimos_generated.tf2_msgs.msg import TFMessage
from pydantic import Field
import pyzed.sl as sl
import reactivex as rx

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
from dimos.msgs.camera_info import camera_info_with_stamp
from dimos.msgs.geometry import compose_transforms, inverse_transform
from dimos.msgs.image import image_from_array
from dimos.msgs.pointcloud import pointcloud_from_rgbd, voxel_downsample_cloud
from dimos.msgs.time import time_from_seconds
from dimos.spec import perception
from dimos.utils.reactive import backpressure


def default_base_transform() -> Transform:
    """Default identity transform for camera mounting."""
    return Transform(
        translation=Vector3(),
        rotation=Quaternion(w=1.0),
    )


class ZEDCameraConfig(ModuleConfig, DepthCameraConfig):
    width: int = 1280
    height: int = 720
    fps: int = 15
    camera_name: str = "camera"
    base_frame_id: str = "base_link"
    base_transform: Transform | None = Field(default_factory=default_base_transform)
    align_depth_to_color: bool = True
    enable_depth: bool = True
    enable_pointcloud: bool = False
    pointcloud_fps: float = 5.0
    camera_info_fps: float = 1.0
    camera_id: int = 0
    serial_number: int | str | None = None
    resolution: str | None = None
    depth_mode: str | sl.DEPTH_MODE = "NEURAL"
    enable_fill_mode: bool = False
    enable_tracking: bool = True
    enable_imu_fusion: bool = True
    enable_pose_smoothing: bool = True
    enable_area_memory: bool = False
    set_floor_as_origin: bool = True
    world_frame: str = "world"


class ZEDCamera(DepthCameraHardware, Module, perception.DepthCamera):
    config: ZEDCameraConfig
    color_image: Out[Image]
    depth_image: Out[Image]
    pointcloud: Out[PointCloud2]
    camera_info: Out[CameraInfo]
    depth_camera_info: Out[CameraInfo]
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

    def __init__(self, *args, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(*args, **kwargs)
        self._zed: sl.Camera | None = None
        self._init_params: sl.InitParameters | None = None
        self._runtime_params: sl.RuntimeParameters | None = None
        self._running = False
        self._thread: threading.Thread | None = None
        self._color_camera_info: CameraInfo | None = None
        self._depth_camera_info: CameraInfo | None = None
        self._depth_scale: float = 1.0
        self._camera_link_to_color_extrinsics: sl.Transform
        self._latest_color_img: Image | None = None
        self._latest_depth_img: Image | None = None
        self._pointcloud_lock = threading.Lock()
        self._image_left: sl.Mat | None = None
        self._depth_map: sl.Mat | None = None
        self._pose: sl.Pose | None = None
        self._tracking_enabled = False
        self._stream_width = self.config.width
        self._stream_height = self.config.height
        self._sl_camera_info: sl.CameraInformation | None = None

    def _publish_camera_info(self) -> None:
        ts = time.time()
        if self._color_camera_info:
            self.camera_info.publish(
                camera_info_with_stamp(self._color_camera_info, time_from_seconds(ts))
            )
        if self._depth_camera_info:
            self.depth_camera_info.publish(
                camera_info_with_stamp(self._depth_camera_info, time_from_seconds(ts))
            )

    @rpc
    def start(self) -> None:
        self._zed = sl.Camera()
        self._init_params = sl.InitParameters()
        if self.config.resolution:
            self._init_params.camera_resolution = getattr(sl.RESOLUTION, self.config.resolution)
        else:
            self._init_params.camera_resolution = sl.RESOLUTION.HD720
        self._init_params.camera_fps = self.config.fps
        if isinstance(self.config.depth_mode, sl.DEPTH_MODE):
            self._init_params.depth_mode = self.config.depth_mode
        else:
            self._init_params.depth_mode = getattr(sl.DEPTH_MODE, self.config.depth_mode)
        self._init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
        self._init_params.coordinate_units = sl.UNIT.METER
        if self.config.serial_number is not None:
            self._init_params.set_from_serial_number(int(self.config.serial_number))
        else:
            self._init_params.set_from_camera_id(self.config.camera_id)

        err = self._zed.open(self._init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            self._zed = None
            raise RuntimeError(f"Failed to open ZED camera: {err}")

        self._runtime_params = sl.RuntimeParameters()
        self._runtime_params.enable_fill_mode = self.config.enable_fill_mode
        self._image_left = sl.Mat()
        self._depth_map = sl.Mat()
        self._pose = sl.Pose()

        self._sl_camera_info = self._zed.get_camera_information()
        if self._sl_camera_info is not None:
            self._stream_width = self._sl_camera_info.camera_configuration.resolution.width
            self._stream_height = self._sl_camera_info.camera_configuration.resolution.height

        self._build_camera_info()
        self._get_extrinsics()

        if self.config.enable_tracking:
            self._enable_tracking()

        interval_sec = 1.0 / self.config.camera_info_fps
        self.register_disposable(
            rx.interval(interval_sec).subscribe(
                on_next=lambda _: self._publish_camera_info(),
                on_error=lambda e: print(f"CameraInfo error: {e}"),
            )
        )

        self._running = True
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

        if self.config.enable_pointcloud and self.config.enable_depth:
            interval_sec = 1.0 / self.config.pointcloud_fps
            self.register_disposable(
                backpressure(rx.interval(interval_sec)).subscribe(
                    on_next=lambda _: self._generate_pointcloud(),
                    on_error=lambda e: print(f"Pointcloud error: {e}"),
                )
            )

    def _build_camera_info(self) -> None:
        if self._sl_camera_info is None:
            return
        calib = self._sl_camera_info.camera_configuration.calibration_parameters
        left_cam = calib.left_cam

        self._color_camera_info = self._intrinsics_to_camera_info(
            left_cam, self._color_optical_frame
        )

        if self.config.enable_depth:
            depth_frame = (
                self._color_optical_frame
                if self.config.align_depth_to_color
                else self._depth_optical_frame
            )
            self._depth_camera_info = self._intrinsics_to_camera_info(left_cam, depth_frame)

    def _intrinsics_to_camera_info(
        self, intrinsics: sl.CameraParameters, frame_id: str
    ) -> CameraInfo:
        fx, fy = intrinsics.fx, intrinsics.fy
        cx, cy = intrinsics.cx, intrinsics.cy

        K = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        P = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        D = list(intrinsics.disto)

        return CameraInfo(
            height=self._stream_height,
            width=self._stream_width,
            distortion_model="plumb_bob",
            d=D,
            k=K,
            r=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            p=P,
            header=Header(frame_id=frame_id),
        )

    def _get_extrinsics(self) -> None:
        if self._sl_camera_info is None:
            return
        sensors_config = self._sl_camera_info.sensors_configuration
        # camera_imu_transform gives the transform from IMU (body center) to left camera
        self._camera_link_to_color_extrinsics = sensors_config.camera_imu_transform

    def _extrinsics_to_transform(
        self,
        extrinsics: sl.Transform,
        frame_id: str,
        child_frame_id: str,
        ts: float,
    ) -> TransformStamped:
        translation = extrinsics.get_translation().get()
        quat = extrinsics.get_orientation().get()
        return TransformStamped(
            header=Header(frame_id=frame_id, stamp=time_from_seconds(ts)),
            child_frame_id=child_frame_id,
            transform=Transform(
                translation=Vector3(x=translation[0], y=translation[1], z=translation[2]),
                rotation=Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3]),
            ),
        )

    def _enable_tracking(self) -> None:
        if self._zed is None:
            return
        tracking_params = sl.PositionalTrackingParameters()
        tracking_params.enable_area_memory = self.config.enable_area_memory
        tracking_params.enable_pose_smoothing = self.config.enable_pose_smoothing
        tracking_params.enable_imu_fusion = self.config.enable_imu_fusion
        tracking_params.set_floor_as_origin = self.config.set_floor_as_origin
        err = self._zed.enable_positional_tracking(tracking_params)
        if err != sl.ERROR_CODE.SUCCESS:
            print(f"Failed to enable positional tracking: {err}")
            self._tracking_enabled = False
            return
        self._tracking_enabled = True

    def _capture_loop(self) -> None:
        import cv2

        while self._running and self._zed is not None:
            try:
                err = self._zed.grab(self._runtime_params)
            except Exception:
                break

            if err != sl.ERROR_CODE.SUCCESS:
                if not self._running:
                    break
                time.sleep(0.001)
                continue

            ts = time.time()

            color_img = None
            if self._image_left is not None:
                self._zed.retrieve_image(self._image_left, sl.VIEW.LEFT)
                color_data = self._image_left.get_data()
                if color_data.ndim == 3 and color_data.shape[2] == 4:
                    color_data = color_data[:, :, :3]
                color_data = cv2.cvtColor(color_data, cv2.COLOR_BGR2RGB)
                color_img = image_from_array(
                    color_data,
                    encoding="rgb8",
                    header=Header(frame_id=self._color_optical_frame, stamp=time_from_seconds(ts)),
                )
                self.color_image.publish(color_img)

            depth_img = None
            if self.config.enable_depth and self._depth_map is not None:
                self._zed.retrieve_measure(self._depth_map, sl.MEASURE.DEPTH)
                depth_data = self._depth_map.get_data()
                if depth_data.ndim == 3:
                    depth_data = depth_data[:, :, 0]
                depth_frame_id = (
                    self._color_optical_frame
                    if self.config.align_depth_to_color
                    else self._depth_optical_frame
                )
                depth_img = image_from_array(
                    depth_data,
                    encoding="32FC1",
                    header=Header(frame_id=depth_frame_id, stamp=time_from_seconds(ts)),
                )
                self.depth_image.publish(depth_img)

            if self.config.enable_pointcloud and color_img is not None and depth_img is not None:
                with self._pointcloud_lock:
                    self._latest_color_img = color_img
                    self._latest_depth_img = depth_img

            self._publish_tf(ts)

    def _tracking_transform(self, ts: float) -> TransformStamped | None:
        if not self._tracking_enabled or self._zed is None or self._pose is None:
            return None
        state = self._zed.get_position(self._pose, sl.REFERENCE_FRAME.WORLD)
        if state != sl.POSITIONAL_TRACKING_STATE.OK:
            return None

        world_to_camera = self._extrinsics_to_transform(
            self._pose, self.config.world_frame, self._camera_link, ts
        )
        if self.config.base_transform is None:
            return world_to_camera
        base_to_camera = TransformStamped(
            header=Header(frame_id=self.config.base_frame_id, stamp=time_from_seconds(ts)),
            child_frame_id=self._camera_link,
            transform=self.config.base_transform,
        )
        return compose_transforms(world_to_camera, inverse_transform(base_to_camera))

    def _publish_tf(self, ts: float) -> None:
        transforms: list[TransformStamped] = []
        stamp = time_from_seconds(ts)
        if self.config.base_transform is not None:
            transforms.append(
                TransformStamped(
                    header=Header(frame_id=self.config.base_frame_id, stamp=stamp),
                    child_frame_id=self._camera_link,
                    transform=self.config.base_transform,
                )
            )

        # SDK extrinsics are the IMU-to-left-camera coordinate transform;
        # retain the inverse used by the existing camera-link pose convention.
        for frame, optical_frame in (
            (self._depth_frame, self._depth_optical_frame),
            (self._color_frame, self._color_optical_frame),
        ):
            extrinsics = inverse_transform(
                self._extrinsics_to_transform(
                    self._camera_link_to_color_extrinsics, self._camera_link, frame, ts
                )
            )
            extrinsics.header.frame_id = self._camera_link
            extrinsics.child_frame_id = frame
            transforms.append(extrinsics)
            transforms.append(
                TransformStamped(
                    header=Header(frame_id=frame, stamp=stamp),
                    child_frame_id=optical_frame,
                    transform=Transform(rotation=OPTICAL_ROTATION),
                )
            )
        tracking_tf = self._tracking_transform(ts)
        if tracking_tf is not None:
            transforms.append(tracking_tf)
        self.tf.publish(TFMessage(transforms=transforms))

    def _generate_pointcloud(self) -> None:
        with self._pointcloud_lock:
            color_img = self._latest_color_img
            depth_img = self._latest_depth_img

        if color_img is None or depth_img is None or self._color_camera_info is None:
            return

        try:
            pcd = pointcloud_from_rgbd(
                color=color_img,
                depth=depth_img,
                calibration=self._color_camera_info,
                depth_scale=self._depth_scale,
            )
            pcd = voxel_downsample_cloud(pcd, 0.005)
            self.pointcloud.publish(pcd)
        except Exception as e:
            print(f"Pointcloud generation error: {e}")

    @rpc
    def stop(self) -> None:
        self._running = False

        if self._zed:
            if self._tracking_enabled:
                try:
                    self._zed.disable_positional_tracking()
                except Exception:
                    pass
            try:
                self._zed.close()
            except Exception:
                pass
            self._zed = None

        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
            if self._thread.is_alive():
                self._thread = None

        self._color_camera_info = None
        self._depth_camera_info = None
        self._latest_color_img = None
        self._latest_depth_img = None
        self._image_left = None
        self._depth_map = None
        self._pose = None
        self._sl_camera_info = None
        self._tracking_enabled = False
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

    camera = dimos.deploy(ZEDCamera, enable_pointcloud=True, pointcloud_fps=5.0)
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

ZEDModule = ZEDCamera

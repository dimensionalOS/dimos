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

"""Desk webcam stack that emits marker detections and mirrors them into TF."""

from __future__ import annotations

from pathlib import Path
import threading
import time

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.transport import LCMTransport
from dimos.hardware.sensors.camera.module import CameraModule
from dimos.hardware.sensors.camera.webcam import Webcam
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.vision_msgs.Detection2DArray import Detection2DArray
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.perception.fiducial.marker_module import MarkerModule
from dimos.perception.fiducial.marker_tf_module import MarkerTfModule

DESK_CAMERA_FRAME_ID = "camera_optical"
DESK_MARKER_NAMESPACE_PREFIX = "marker_tf"
DESK_MARKER_ARUCO_DICTIONARY = "DICT_APRILTAG_36h11"
DESK_MARKER_LENGTH_M = 0.05
DEFAULT_DESK_CAMERA_INFO_YAML = Path(__file__).resolve().parent / "fixtures" / "camera_info.yaml"


def create_desk_webcam(
    camera_info_yaml: str | Path = DEFAULT_DESK_CAMERA_INFO_YAML,
    camera_index: int = 0,
    fps: float = 15.0,
) -> Webcam:
    camera_info = create_desk_camera_info(camera_info_yaml)
    return Webcam(
        camera_index=camera_index,
        width=camera_info.width,
        height=camera_info.height,
        fps=fps,
        camera_info=camera_info,
    )


def create_desk_camera_info(
    camera_info_yaml: str | Path = DEFAULT_DESK_CAMERA_INFO_YAML,
) -> CameraInfo:
    camera_info = CameraInfo.from_yaml(str(camera_info_yaml))
    camera_info.frame_id = DESK_CAMERA_FRAME_ID
    return camera_info


class DeskStaticTfModuleConfig(ModuleConfig):
    world_frame: str = "world"
    base_frame: str = "base_link"
    camera_optical_frame: str = "camera_optical"
    camera_translation_m: tuple[float, float, float] = (
        0.25,
        0.0,
        0.15,
    )
    camera_rotation_rpy_rad: tuple[float, float, float] = (0.0, 0.0, 0.0)
    #: Republish fixed transforms so marker detection can resolve camera poses
    #: at image timestamps (single-shot stamps fall out of tolerance).
    static_tf_republish_hz: float = 10.0


class DeskStaticTfModule(Module):
    """Publish the fixed desk TF chain needed by marker pose estimation."""

    config: DeskStaticTfModuleConfig
    _last_publish_ts: float | None = None
    _republish_stop: threading.Event | None = None
    _republish_thread: threading.Thread | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self.publish_static_chain()
        hz = self.config.static_tf_republish_hz
        if hz > 0.0:
            self._republish_stop = threading.Event()
            period = 1.0 / hz

            def _republish_loop() -> None:
                assert self._republish_stop is not None
                while not self._republish_stop.wait(period):
                    self.publish_static_chain()

            self._republish_thread = threading.Thread(
                target=_republish_loop,
                name="desk_static_tf_republish",
                daemon=True,
            )
            self._republish_thread.start()

    @rpc
    def stop(self) -> None:
        if self._republish_stop is not None:
            self._republish_stop.set()
        if self._republish_thread is not None:
            self._republish_thread.join(timeout=2.0)
            self._republish_thread = None
        self._republish_stop = None
        super().stop()

    def publish_static_chain(self) -> None:
        ts = time.time()
        self._last_publish_ts = ts
        roll, pitch, yaw = self.config.camera_rotation_rpy_rad
        x, y, z = self.config.camera_translation_m

        self.tf.publish(
            Transform(
                translation=Vector3(0.0, 0.0, 0.0),
                rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
                frame_id=self.config.world_frame,
                child_frame_id=self.config.base_frame,
                ts=ts,
            ),
            Transform(
                # Default desk camera pose: about 25 cm forward and 15 cm above base_link.
                translation=Vector3(x, y, z),
                rotation=Quaternion.from_euler(Vector3(roll, pitch, yaw)),
                frame_id=self.config.base_frame,
                child_frame_id=self.config.camera_optical_frame,
                ts=ts,
            ),
        )


desk_marker_tf = autoconnect(
    DeskStaticTfModule.blueprint(),
    CameraModule.blueprint(
        hardware=create_desk_webcam,
        transform=None,
    ),
    MarkerModule.blueprint(
        marker_length_m=DESK_MARKER_LENGTH_M,
        aruco_dictionary=DESK_MARKER_ARUCO_DICTIONARY,
        camera_info=create_desk_camera_info(),
    ),
    MarkerTfModule.blueprint(
        marker_namespace_prefix=DESK_MARKER_NAMESPACE_PREFIX,
    ),
).transports(
    {
        ("detections_3d", Detection3DArray): LCMTransport(
            "/marker_detection/detections_3d",
            Detection3DArray,
        ),
        ("detections_2d", Detection2DArray): LCMTransport(
            "/marker_detection/detections_2d",
            Detection2DArray,
        ),
    }
)

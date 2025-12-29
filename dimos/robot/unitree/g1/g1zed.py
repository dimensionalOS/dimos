# Copyright 2025 Dimensional Inc.
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

from typing import TypedDict, cast

from dimos.constants import DEFAULT_CAPACITY_COLOR_IMAGE
from dimos.core import DimosCluster, LCMTransport, pSHMTransport
from dimos.hardware.sensors.camera import zed
from dimos.hardware.sensors.camera.module import CameraModule
from dimos.hardware.sensors.camera.webcam import Webcam
from dimos.msgs.geometry_msgs import (
    Quaternion,
    Transform,
    Vector3,
)
from dimos.msgs.sensor_msgs import CameraInfo
from dimos.navigation import rosnav
from dimos.navigation.rosnav import ROSNav
from dimos.robot import foxglove_bridge
from dimos.robot.unitree.connection import g1
from dimos.robot.unitree.connection.g1 import G1Connection
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class G1ZedDeployResult(TypedDict):
    nav: ROSNav
    connection: G1Connection
    camera: CameraModule
    camerainfo: CameraInfo


def deploy_g1_monozed(dimos: DimosCluster) -> CameraModule:
    camera = cast(
        "CameraModule",
        dimos.deploy(  # type: ignore[attr-defined]
            CameraModule,
            frequency=4.0,
            transform=Transform(
                translation=Vector3(0.05, 0.0, 0.0),
                rotation=Quaternion.from_euler(Vector3(0.0, 0.0, 0.0)),
                frame_id="sensor",
                child_frame_id="camera_link",
            ),
            hardware=lambda: Webcam(
                camera_index=0,
                frequency=5,
                stereo_slice="left",
                camera_info=zed.CameraInfo.SingleWebcam,
            ),
        ),
    )

    camera.color_image.transport = pSHMTransport(
        "/image", default_capacity=DEFAULT_CAPACITY_COLOR_IMAGE
    )
    camera.camera_info.transport = LCMTransport("/camera_info", CameraInfo)
    camera.start()
    return camera


def deploy(dimos: DimosCluster, ip: str):  # type: ignore[no-untyped-def]
    nav = rosnav.deploy(  # type: ignore[call-arg]
        dimos,
        sensor_to_base_link_transform=Transform(
            frame_id="sensor", child_frame_id="base_link", translation=Vector3(0.0, 0.0, 1.5)
        ),
    )
    connection = g1.deploy(dimos, ip, nav)
    zedcam = deploy_g1_monozed(dimos)

    foxglove_bridge.deploy(dimos)

    return {
        "nav": nav,
        "connection": connection,
        "camera": zedcam,
    }

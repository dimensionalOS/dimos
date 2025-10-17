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

import time

from dimos.constants import DEFAULT_CAPACITY_COLOR_IMAGE
from dimos.core import DimosCluster, LCMTransport, pSHMTransport, start, wait_exit
from dimos.hardware.camera import zed
from dimos.hardware.camera.module import CameraModule
from dimos.hardware.camera.webcam import Webcam
from dimos.msgs.geometry_msgs import (
    PoseStamped,
    Quaternion,
    Transform,
    Vector3,
)
from dimos.msgs.sensor_msgs import CameraInfo
from dimos.navigation import rosnav
from dimos.robot.unitree_webrtc.connection import g1
from dimos.robot.unitree_webrtc.modular.misc import deploy_foxglove
from dimos.utils.logging_config import setup_logger

logger = setup_logger(__name__)


def deploy_monozed(dimos) -> CameraModule:
    camera = dimos.deploy(
        CameraModule,
        frequency=4.0,
        transform=Transform(
            translation=Vector3(0.05, 0.0, 0.0),
            rotation=Quaternion.from_euler(Vector3(0.0, 0.2, 0.0)),
            frame_id="sensor",
            child_frame_id="camera_link",
        ),
        hardware=lambda: Webcam(
            camera_index=0,
            frequency=5,
            stereo_slice="left",
            camera_info=zed.CameraInfo.SingleWebcam,
        ),
    )
    camera.image.transport = pSHMTransport("/image", default_capacity=DEFAULT_CAPACITY_COLOR_IMAGE)
    camera.camera_info.transport = LCMTransport("/camera_info", CameraInfo)
    camera.start()
    return camera


def ivan_g1(dimos: DimosCluster, ip: str) -> None:
    nav = rosnav.deploy(dimos)
    connection = g1.deploy(dimos, ip, nav)
    zed = deploy_monozed(dimos)
    fg = deploy_foxglove(dimos)

    time.sleep(5)

    test_pose = PoseStamped(
        ts=time.time(),
        frame_id="map",
        position=Vector3(0.0, 0.0, 0.0),
        orientation=Quaternion(0.0, 0.0, 0.0, 0.0),
    )

    nav.navigate_to(test_pose)
    wait_exit()
    dimos.close_all()


if __name__ == "__main__":
    import argparse
    import os

    from dotenv import load_dotenv

    load_dotenv()

    parser = argparse.ArgumentParser(description="Unitree G1 Humanoid Robot Control")
    parser.add_argument("--ip", default=os.getenv("ROBOT_IP"), help="Robot IP address")

    args = parser.parse_args()
    ivan_g1(start(8), args.ip)

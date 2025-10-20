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

from dimos.core import DimosCluster, start, wait_exit
from dimos.perception.detection import module3D, moduleDB
from dimos.perception.detection.detectors.person.yolo import YoloPersonDetector
from dimos.robot.unitree_webrtc.modular import g1zed


def deploy(dimos: DimosCluster, ip: str) -> None:
    g1 = g1zed.deploy(dimos, ip)

    nav = g1.get("nav")
    camera = g1.get("camera")
    camerainfo = g1.get("camerainfo")

    person_detector = module3D.deploy(
        dimos,
        camerainfo,
        camera=camera,
        lidar=nav,
        detector=YoloPersonDetector,
    )

    detector3d = moduleDB.deploy(
        dimos,
        camerainfo,
        camera=camera,
        lidar=nav,
    )

    return {"person_detector": person_detector, "detector3d": detector3d} + g1


if __name__ == "__main__":
    import argparse
    import os

    from dotenv import load_dotenv

    load_dotenv()

    parser = argparse.ArgumentParser(description="Unitree G1 Humanoid Robot Control")
    parser.add_argument("--ip", default=os.getenv("ROBOT_IP"), help="Robot IP address")

    args = parser.parse_args()

    dimos = start(8)
    deploy(dimos, args.ip)
    wait_exit()
    dimos.close_all()

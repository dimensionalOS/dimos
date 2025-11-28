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

from dimos.core import DimosCluster
from dimos.perception.detection import module3D, moduleDB
from dimos.perception.detection.detectors.person.yolo import YoloPersonDetector
from dimos.robot.unitree.g1 import g1zed


def deploy(dimos: DimosCluster, ip: str):  # type: ignore[no-untyped-def]
    g1 = g1zed.deploy(dimos, ip)

    nav = g1.get("nav")
    camera = g1.get("camera")

    person_detector = module3D.deploy(
        dimos,
        camera=camera,
        lidar=nav,
        detector=YoloPersonDetector,
    )

    detector3d = moduleDB.deploy(
        dimos,
        camera=camera,
        lidar=nav,
        filter=lambda det: det.class_id != 0,
    )

    return {"person_detector": person_detector, "detector3d": detector3d, **g1}

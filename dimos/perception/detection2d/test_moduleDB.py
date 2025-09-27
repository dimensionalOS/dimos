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
from dimos.perception.detection2d import testing
from dimos.perception.detection2d.module2D import Detection2DModule
from dimos.perception.detection2d.module3D import Detection3DModule
from dimos.perception.detection2d.moduleDB import ObjectDBModule
from dimos.protocol.service import lcmservice as lcm
from dimos.robot.unitree_webrtc.modular.connection_module import ConnectionModule


def test_moduleDB():
    lcm.autoconf()

    module2d = Detection2DModule()
    module3d = Detection3DModule(camera_info=ConnectionModule._camera_info())
    moduleDB = ObjectDBModule(
        camera_info=ConnectionModule._camera_info(),
        goto=lambda obj_id: print(f"Going to {obj_id}"),
    )

    for i in range(5):
        seek_value = 10.0 + (i * 2)
        moment = testing.get_moment(seek=seek_value)
        imageDetections2d = module2d.process_image_frame(moment["image_frame"])

        # print(imageDetections2d)

        camera_transform = moment["tf"].get("camera_optical", moment.get("lidar_frame").frame_id)

        imageDetections3d = module3d.process_frame(
            imageDetections2d, moment["lidar_frame"], camera_transform
        )

        moduleDB.add_detections(imageDetections3d)
        print(moduleDB)

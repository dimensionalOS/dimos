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
from unittest.mock import patch

from reactivex import of

from dimos.hardware.camera import zed
from dimos.msgs.geometry_msgs import (
    Quaternion,
    Transform,
    Vector3,
)
from dimos.perception.detection2d import testing
from dimos.perception.detection2d.module2D import Detection2DModule
from dimos.perception.detection2d.module3D import Detection3DModule
from dimos.perception.detection2d.moduleDB import ObjectDBModule


def test_module2d_visual_query():
    moments = []
    for i in range(2):
        seek_value = 3.0 + (i * 2)
        moments.append(testing.get_moment(seek=seek_value, g1=True))

    test_image_stream = of(*[m["image_frame"] for m in moments])
    module2d = Detection2DModule()
    module3d = Detection3DModule(camera_info=zed.CameraInfo.SingleWebcam)
    objectdb = ObjectDBModule(goto=print, camera_info=zed.CameraInfo.SingleWebcam)
    cam_transform = Transform(
        translation=Vector3(0.05, 0.0, 0.0),
        rotation=Quaternion.from_euler(Vector3(0.0, 0.2, 0.0)),
        frame_id="sensor",
        child_frame_id="camera_link",
    )

    with patch.object(module2d, "sharp_image_stream", return_value=test_image_stream):
        moment = moments[0]
        testing.publish_moment(moment)
        detections = module2d.vlm_query("a guy, laptop, laptop")
        moment["detections2d"] = detections

        tf = moment["tf"]
        tf.receive_transform(cam_transform)
        camera_transform = tf.get("camera_optical", moment.get("lidar_frame").frame_id)
        detections3d = module3d.process_frame(detections, moment["lidar_frame"], camera_transform)
        moment["detections3d"] = detections3d

        objectdb.add_detections(detections3d)

        moment["objectdb"] = objectdb
        testing.publish_moment(moment)

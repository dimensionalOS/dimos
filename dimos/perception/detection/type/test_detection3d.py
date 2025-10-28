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


def test_guess_projection(get_moment_2d, publish_moment) -> None:
    moment = get_moment_2d()
    for key, value in moment.items():
        print(key, "====================================")
        print(value)

    moment.get("camera_info")
    detection2d = moment.get("detections2d")[0]
    tf = moment.get("tf")
    tf.get("camera_optical", "world", detection2d.ts, 5.0)

    # for stash
    # detection3d = Detection3D.from_2d(detection2d, 1.5, camera_info, transform)
    # print(detection3d)

    # foxglove bridge needs 2 messages per topic to pass to foxglove
    publish_moment(moment)
    time.sleep(0.1)
    publish_moment(moment)

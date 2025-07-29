#!/usr/bin/env python3


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

import pickle

from dimos.msgs.sensor_msgs import PointCloud2
from dimos.protocol.pubsub.lcmpubsub import LCM, Topic
from dimos.types.costmap import Costmap, pointcloud_to_costmap
from dimos.utils.testing import get_data


def test_costmap():
    file_path = get_data("lcm_msgs") / "sensor_msgs/PointCloud2.pickle"
    print("open", file_path)
    with open(file_path, "rb") as f:
        lcm_msg = pickle.loads(f.read())

    pointcloud = PointCloud2.lcm_decode(lcm_msg)
    print(pointcloud)

    costmap = pointcloud_to_costmap(pointcloud.pointcloud)
    print(costmap)

    lcm = LCM()
    lcm.start()
    lcm.publish(Topic("/global_map", PointCloud2), pointcloud)
    lcm.publish(Topic("/global_costmap", Costmap), costmap)

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

from dimos.multiprocess.actors2.o3dpickle import register_picklers
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.utils.testing import SensorReplay

register_picklers()


def test_enode_decode():
    lidardata = SensorReplay("office_lidar", autocast=LidarMessage.from_msg)
    lidarmsg = next(lidardata.iterate())

    binarypc = pickle.dumps(lidarmsg.pointcloud)

    # Test pickling and unpickling
    binary = pickle.dumps(lidarmsg)
    lidarmsg2 = pickle.loads(binary)

    # Verify the decoded message has the same properties
    assert isinstance(lidarmsg2, LidarMessage)
    assert len(lidarmsg2.pointcloud.points) == len(lidarmsg.pointcloud.points)

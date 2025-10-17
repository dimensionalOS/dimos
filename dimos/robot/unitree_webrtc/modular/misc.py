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

import logging

from dimos.core import DimosCluster
from dimos.robot.foxglove_bridge import FoxgloveBridge

logging.getLogger("FoxgloveServer").setLevel(logging.ERROR)


def deploy_foxglove(dimos: DimosCluster) -> FoxgloveBridge:
    foxglove_bridge = dimos.deploy(
        FoxgloveBridge,
        shm_channels=[
            "/image#sensor_msgs.Image",
            "/lidar#sensor_msgs.PointCloud2",
            "/map#sensor_msgs.PointCloud2",
        ],
    )
    foxglove_bridge.start()
    return foxglove_bridge

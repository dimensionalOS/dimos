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
from dimos.robot import foxglove_bridge
from dimos.robot.unitree.connection import go2
from dimos.utils.logging_config import setup_logger

logger = setup_logger(level=logging.INFO)


def deploy(dimos: DimosCluster, ip: str):  # type: ignore[no-untyped-def]
    connection = go2.deploy(dimos, ip)
    foxglove_bridge.deploy(dimos)

    # detector = moduleDB.deploy(
    #     dimos,
    #     camera=connection,
    #     lidar=connection,
    # )

    # agent = agents.deploy(dimos)
    # agent.register_skills(detector)
    return connection

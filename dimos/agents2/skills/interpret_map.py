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

from dimos.core.module import Module
from dimos.core.skill_module import SkillModule
from dimos.core.rpc_client import RpcCall
from dimos.protocol.skill.skill import rpc, skill
from dimos.core.stream import In, Out
from dimos.utils.logging_config import setup_logger
from dimos.msgs.nav_msgs import OccupancyGrid

logger = setup_logger("dimos.agents2.skills.interpret_map")

class InterpretMapSkill(SkillModule):
    _latest_local_costmap: OccupancyGrid | None = None
    
    local_costmap: In[OccupancyGrid] = None

    @rpc
    def start(self) -> None:
        super().start()
        self._disposables.add(self.local_costmap.subscribe(self._on_local_costmap))

    @rpc
    def stop(self) -> None:
        super().stop()

    def _on_local_costmap(self, costmap: OccupancyGrid) -> None:
        self._latest_local_costmap = costmap

    @skill()
    def get_map(self):
        """Provides current map in ASCII string.
        
            . represents free space
            # represents obstacles
            X represents robot position
            ? represents unknown space
        
        """
        if self._latest_local_costmap is None:
            logger.warning("No local costmap available.")
            return "No map available."

        return self._latest_local_costmap

interpret_map_skill = InterpretMapSkill.blueprint

__all__ = ["InterpretMapSkill", "interpret_map_skill"]

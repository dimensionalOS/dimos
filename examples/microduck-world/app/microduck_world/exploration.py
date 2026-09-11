# Copyright 2026 Dimensional Inc.
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

"""Keep stock frontier exploration under this duck's control-mode ownership."""

import json
import threading
from typing import Any

from dimos.agents.annotation import current_skill_context, skill
from dimos.agents.capabilities import CAP_MOVEMENT
from dimos.core.core import rpc
from dimos.core.stream import In
from dimos.navigation.frontier_exploration.wavefront_frontier_goal_selector import (
    WavefrontFrontierExplorer,
)
from reactivex.disposable import Disposable


class DuckExplorer(WavefrontFrontierExplorer):
    mode: In[str]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._agent_mode = threading.Event()

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.mode.subscribe(self._on_mode)))

    def _on_mode(self, raw: str) -> None:
        if json.loads(raw).get("mode") == "agent":
            self._agent_mode.set()
        else:
            self._agent_mode.clear()
            if self.is_exploration_active():
                self.stop_exploration()

    @skill(uses=[CAP_MOVEMENT], lifecycle="background")
    def begin_exploration(self) -> str:
        """Explore unknown frontiers in your own measured map. Requires Agent mode.

        Continues until end_exploration or exploration finishes.
        """
        if not self._agent_mode.is_set():
            return "Select Agent mode before asking me to explore."
        return super().begin_exploration(**{"_mcp_context": current_skill_context()})

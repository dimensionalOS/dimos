# Copyright 2025-2026 Dimensional Inc.
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

from dimos.agents.annotation import skill
from dimos.core.module import Module
from dimos.navigation.navigation_spec import NavigationInterfaceSpec


class NavigationStopSkill(Module):
    """Navigation cancellation, usable without semantic perception."""

    _navigation: NavigationInterfaceSpec

    @skill
    def stop_navigation(self) -> str:
        """Immediately cancel the navigation goal and stop moving."""
        self._navigation.cancel_goal()
        return "Stopped"

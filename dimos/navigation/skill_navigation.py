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

"""Shared terminal outcome for blocking navigation skills."""

import time

from dimos.agents.skill_result import SkillResult
from dimos.navigation.base import NavigationState
from dimos.navigation.navigation_spec import NavigationInterfaceSpec


def wait_for_navigation(navigation: NavigationInterfaceSpec, timeout: float = 100.0) -> SkillResult:
    """Wait through replanning; cancel the goal on timeout or observation failure."""
    deadline = time.monotonic() + timeout
    idle_since: float | None = None
    try:
        while time.monotonic() < deadline:
            if navigation.is_goal_reached():
                return SkillResult.ok("Navigation goal reached", status="succeeded")
            if navigation.get_state() == NavigationState.FOLLOWING_PATH:
                idle_since = None
            elif idle_since is None:
                idle_since = time.monotonic()
            elif time.monotonic() - idle_since > 2.0:
                navigation.cancel_goal()
                return SkillResult.fail("NAVIGATION_STOPPED", "Navigation was cancelled or failed")
            time.sleep(0.1)
        navigation.cancel_goal()
        return SkillResult.fail("EXECUTION_TIMEOUT", "Navigation timed out; goal cancelled")
    except BaseException:
        navigation.cancel_goal()
        raise

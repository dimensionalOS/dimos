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

from dimos.agents.annotation import skill
from dimos.agents.skill_result import SkillResult
from dimos.core.module import Module
from dimos.core.stream import In
from dimos.msgs.sensor_msgs.Image import Image


class ObserveSkill(Module):
    """Gives any robot agent the ability to observe the current camera image."""

    color_image: In[Image]

    _frame_timeout: float = 5.0

    @skill
    def observe(self) -> Image | SkillResult:
        """Returns the current video frame from the robot camera. Use this skill for any visual world queries.

        This skill provides the current camera view for perception tasks.
        """
        try:
            return self.color_image.get_next(timeout=self._frame_timeout)
        except Exception:
            return SkillResult.fail(
                "EXECUTION_TIMEOUT",
                f"No camera frame received within {self._frame_timeout} seconds; "
                "the camera may not be running.",
            )

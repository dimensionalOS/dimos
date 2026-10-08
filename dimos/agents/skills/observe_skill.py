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
from dimos.core.module import Module
from dimos.core.stream import In
from dimos.msgs.sensor_msgs.Image import Image


class ObserveSkill(Module):
    """Gives any robot agent the ability to observe the current camera image."""

    color_image: In[Image]

    _frame_timeout: float = 5.0

    @skill
    def observe(self) -> Image:
        """Returns the current video frame from the robot camera. Use this skill for any visual world queries.

        This skill provides the current camera view for perception tasks.
        Raises TimeoutError when no frame arrives within the frame timeout.
        """
        try:
            return self.color_image.get_next(timeout=self._frame_timeout)
        except Exception as exc:
            raise TimeoutError(
                f"No camera frame received within {self._frame_timeout} seconds; "
                "the camera may not be running."
            ) from exc


class ObserveWorkspaceSkill(Module):
    """Gives a robot agent the view of a fixed camera that sees the whole workspace."""

    overview_image: In[Image]

    _frame_timeout: float = 5.0

    @skill
    def observe_workspace(self) -> Image:
        """Returns the current frame from the fixed workspace camera, which sees the robot, the table and the objects on it from outside. Use it to check the arm and the objects, for example after a move or a grasp.

        Raises TimeoutError when no frame arrives within the frame timeout.
        """
        try:
            return self.overview_image.get_next(timeout=self._frame_timeout)
        except Exception as exc:
            raise TimeoutError(
                f"No workspace camera frame received within {self._frame_timeout} seconds; "
                "the workspace camera may not be enabled."
            ) from exc

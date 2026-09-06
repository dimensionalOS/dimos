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

"""Blueprint helper for privileged simulator perception."""

from __future__ import annotations

from collections.abc import Sequence
from typing import Any

from dimos.core.coordination.blueprints import Blueprint
from dimos.simulation.perception.sim_scene_registration import SimSceneRegistrationModule


def sim_scene_registration(
    *,
    target_frame: str = "world",
    aliases: dict[str, str] | None = None,
    robot_body_substrings: Sequence[str] = (),
    **kwargs: Any,
) -> Blueprint:
    """Ground-truth stand-in for ObjectSceneRegistrationModule.

    ``robot_body_substrings`` names the bodies that make up the robot, so the
    obstacle cloud does not contain the arm doing the reaching.
    """
    return SimSceneRegistrationModule.blueprint(
        target_frame=target_frame,
        aliases=aliases or {},
        scene_exclude_substrings=list(robot_body_substrings),
        **kwargs,
    )

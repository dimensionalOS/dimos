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

"""Basic Piper coordinator blueprints."""

from __future__ import annotations

from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.manipulators.common.blueprints import coordinator
from dimos.robot.manipulators.piper.config import (
    make_piper_model_config,
    piper_hardware,
)

_piper_hw = piper_hardware("arm")
_piper_model = make_piper_model_config()

coordinator_piper = autoconnect(
    coordinator(
        hardware=[_piper_hw],
        tasks=[joint_trajectory_task(_piper_hw.joints)],
    ),
)

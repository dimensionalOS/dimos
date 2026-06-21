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

"""Basic control coordinator blueprint.

Usage:
    dimos run coordinator-basic
"""

from __future__ import annotations

from dimos.control.coordinator import ControlCoordinator

coordinator_basic = ControlCoordinator.blueprint(
    tick_rate=100.0,
    publish_joint_state=True,
    joint_state_frame_id="coordinator",
)

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


"""Bodies the planner is checked against, not robots."""

from __future__ import annotations

from .base import Embodiment

SLIM = Embodiment(tag="slim", length=2.0, width=0.24, comfort=0.3)
# cannot crab, and has no legs to step over anything with
DIFFDRIVE = Embodiment(tag="diffdrive", strafe=50.0, reverse=3.0, steppable=0.0)

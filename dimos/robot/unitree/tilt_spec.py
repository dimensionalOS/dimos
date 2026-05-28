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

from typing import Protocol

from dimos.spec.utils import Spec


# Module-ref spec for tilting the robot body. Resolves (structurally) to
# UnitreeSkillContainer.tilt_body, letting another module (e.g. TakePictureSkill)
# aim the body-fixed camera without owning the WebRTC connection.
class TiltSpec(Spec, Protocol):
    def tilt_body(
        self, pitch_deg: float = 0.0, roll_deg: float = 0.0, yaw_deg: float = 0.0
    ) -> str: ...

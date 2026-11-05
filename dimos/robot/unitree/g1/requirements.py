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

"""Runtime requirements shared by the two G1 hardware blueprints."""

from dimos.core.global_config import global_config


def g1_hardware() -> str | None:
    if global_config.simulation:
        return (
            "G1 simulation requires the upcoming sim2 integration; legacy simulation was removed."
        )
    if global_config.replay:
        return "The G1 control blueprints do not support replay."
    return None

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

# Eager-import G1 connection variants so the connection registry is fully
# populated by the time `Blueprint.with_backend(...)` runs. The Mujoco
# variant defers its mujoco-engine import to instance __init__.
from dimos.robot.unitree.g1 import (  # noqa: F401
    connection,
    mujoco_sim,
)

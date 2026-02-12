#!/usr/bin/env python3
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

"""G1 sim stack with perception and memory."""

from dimos.core.blueprints import autoconnect
from dimos.robot.unitree.g1.blueprints.basic.unitree_g1_basic_sim import unitree_g1_basic_sim
from dimos.robot.unitree.g1.blueprints.perceptive._perception_and_memory import (
    _perception_and_memory,
)

unitree_g1_sim = autoconnect(
    unitree_g1_basic_sim,
    _perception_and_memory,
).global_config(n_dask_workers=8)

__all__ = ["unitree_g1_sim"]

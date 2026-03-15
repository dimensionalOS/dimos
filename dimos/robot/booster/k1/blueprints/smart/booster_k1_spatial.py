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

from dimos.core.blueprints import autoconnect
from dimos.perception.spatial_perception import spatial_memory
from dimos.robot.booster.k1.blueprints.smart.booster_k1 import booster_k1
from dimos.utils.monitoring import utilization

booster_k1_spatial = autoconnect(
    booster_k1,
    spatial_memory(),
    utilization(),
).global_config(n_dask_workers=8)

__all__ = ["booster_k1_spatial"]

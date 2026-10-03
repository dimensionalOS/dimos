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

"""Render-only removal of simulator toggle hints, without changing their geometry."""

from collections.abc import Iterable
from typing import Any

import numpy as np


def hide_toggle_markers(objects: Iterable[Any], toggle_state_type: type[Any]) -> int:
    """Hide official diagnostic geometry and verify its overlap extent is unchanged.

    The simulator's GeomPrim visibility setter owns the USD editing context.
    This never changes state.value, physics, collision flags, pose, scale or mass.
    It may hide an annotated control mesh; sensor readability needs a live check.
    """
    count = 0
    for obj in objects:
        state = getattr(obj, "states", {}).get(toggle_state_type)
        if state is None:
            continue
        marker = state.visual_marker
        if marker is None:
            raise RuntimeError("Toggle marker unavailable: policy RGB cannot be enabled")
        before = np.array(marker.extent.tolist(), dtype=float)
        marker.visible = False
        after = np.array(marker.extent.tolist(), dtype=float)
        if marker.visible or not np.array_equal(before, after):
            raise RuntimeError("Policy visual change modified toggle overlap geometry")
        count += 1
    return count

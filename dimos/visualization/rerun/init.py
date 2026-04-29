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

"""Shared Rerun initialization. Call ``rerun_init()`` instead of ``rr.init()``."""

from __future__ import annotations

import rerun as rr

from dimos.msgs.sensor_msgs.PointCloud2 import register_colormap_annotation


def rerun_init(app_id: str = "dimos", **kwargs: object) -> None:
    """Initialize Rerun with standard defaults."""
    rr.init(app_id, **kwargs)  # type: ignore[arg-type]
    register_colormap_annotation("turbo")

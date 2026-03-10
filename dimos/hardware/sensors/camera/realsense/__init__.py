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

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from dimos.hardware.sensors.camera.realsense.camera import (
        RealSenseCamera,
        RealSenseCameraConfig,
        realsense_camera,
    )

__all__ = ["RealSenseCamera", "RealSenseCameraConfig", "realsense_camera"]


def __getattr__(name: str) -> object:
    if name in __all__:
        from dimos.hardware.sensors.camera.realsense.camera import (
            RealSenseCamera,
            RealSenseCameraConfig,
            realsense_camera,
        )

        globals().update(
            RealSenseCamera=RealSenseCamera,
            RealSenseCameraConfig=RealSenseCameraConfig,
            realsense_camera=realsense_camera,
        )
        return globals()[name]
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")

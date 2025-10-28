# Copyright 2025 Dimensional Inc.
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

"""ZED camera hardware interfaces."""

from pathlib import Path

from dimos.msgs.sensor_msgs.CameraInfo import CalibrationProvider

# Check if ZED SDK is available
try:
    import pyzed.sl as sl

    HAS_ZED_SDK = True
except ImportError:
    HAS_ZED_SDK = False

# Only import ZED classes if SDK is available
if HAS_ZED_SDK:
    from dimos.hardware.camera.zed.camera import ZEDCamera, ZEDModule
else:
    # Provide stub classes when SDK is not available
    class ZEDCamera:
        def __init__(self, *args, **kwargs) -> None:
            raise ImportError(
                "ZED SDK not installed. Please install pyzed package to use ZED camera functionality."
            )

    class ZEDModule:
        def __init__(self, *args, **kwargs) -> None:
            raise ImportError(
                "ZED SDK not installed. Please install pyzed package to use ZED camera functionality."
            )


# Set up camera calibration provider (always available)
CALIBRATION_DIR = Path(__file__).parent
CameraInfo = CalibrationProvider(CALIBRATION_DIR)

__all__ = [
    "HAS_ZED_SDK",
    "CameraInfo",
    "ZEDCamera",
    "ZEDModule",
]

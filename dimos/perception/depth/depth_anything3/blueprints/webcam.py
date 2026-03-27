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

from dimos.core.blueprints import autoconnect
from dimos.hardware.sensors.camera.module import CameraModule
from dimos.perception.depth.depth_anything3.module import DA3Mode, DA3Model, DepthAnything3Module
from dimos.visualization.rerun.bridge import rerun_bridge

depth_anything3_webcam = autoconnect(
    CameraModule.blueprint(),
    DepthAnything3Module.blueprint(model=DA3Model.SMALL),
    rerun_bridge(),
)

depth_anything3_webcam_temporal = autoconnect(
    CameraModule.blueprint(),
    DepthAnything3Module.blueprint(model=DA3Model.SMALL, mode=DA3Mode.TEMPORAL, window_frames=5),
    rerun_bridge(),
)

__all__ = ["depth_anything3_webcam", "depth_anything3_webcam_temporal"]

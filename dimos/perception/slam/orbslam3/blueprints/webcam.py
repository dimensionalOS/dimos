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
from dimos.perception.slam.orbslam3.module import OrbSlam3
from dimos.visualization.rerun.bridge import _resolve_viewer_mode, rerun_bridge

orbslam3_webcam = autoconnect(
    rerun_bridge(viewer_mode=_resolve_viewer_mode()),
    CameraModule.blueprint(),
    OrbSlam3.blueprint(sensor_mode="MONOCULAR"),
).global_config(n_workers=3)

__all__ = ["orbslam3_webcam"]

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

"""Start the camera-nav + FlowBase keyboard teleop blueprint.

    python -m dimos.navigation.camera_nav.run_flowbase           # webcam (monocular)
    python -m dimos.navigation.camera_nav.run_flowbase --zed     # ZED Mini (stereo)
    python -m dimos.navigation.camera_nav.run_flowbase --trial   # laptop webcam, no robot

``--trial`` mode
    Opens the host webcam (index 0) and runs DepthAnythingV2 → Rerun without
    connecting to any robot.  Use this to verify the depth / point-cloud /
    visualisation pipeline on a dev machine before hardware deployment.

``--zed`` mode (ZED Mini connected, ZED SDK installed)
    Connects to the FlowBase at 172.6.2.20:11323.
    Uses ZED hardware stereo depth + VIO odometry.

Default (generic webcam mode)
    Connects to the FlowBase at 172.6.2.20:11323.
    Uses DepthAnythingV2 monocular depth + wheel odometry.

After stopping (--zed or default), run PGO correction:
  python -m dimos.navigation.camera_nav.correct_map traversal.db --rerun
"""

import argparse

from dimos.core.coordination.module_coordinator import ModuleCoordinator

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--zed", action="store_true", help="Use ZED Mini stereo camera (ZED SDK required)")
    group.add_argument("--trial", action="store_true", help="Laptop webcam only, no robot required")
    args = parser.parse_args()

    if args.trial:
        from dimos.navigation.camera_nav.blueprint_flowbase import camera_nav_static_trial
        ModuleCoordinator.build(camera_nav_static_trial).loop()
    elif args.zed:
        from dimos.navigation.camera_nav.blueprint_zed import camera_nav_zed_teleop
        ModuleCoordinator.build(camera_nav_zed_teleop).loop()
    else:
        from dimos.navigation.camera_nav.blueprint_flowbase import camera_nav_flowbase_teleop
        ModuleCoordinator.build(camera_nav_flowbase_teleop).loop()

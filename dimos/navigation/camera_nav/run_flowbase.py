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

    python -m dimos.navigation.camera_nav.run_flowbase                      # monocular webcam
    python -m dimos.navigation.camera_nav.run_flowbase --address 10.0.0.94  # custom IP
    python -m dimos.navigation.camera_nav.run_flowbase --zed                # ZED Mini stereo
    python -m dimos.navigation.camera_nav.run_flowbase --trial              # no robot, dev machine
"""

import argparse

from dimos.core.coordination.module_coordinator import ModuleCoordinator

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--zed", action="store_true", help="ZED Mini stereo camera (ZED SDK required)")
    group.add_argument("--zed-only", action="store_true", help="ZED Mini only, no robot — point cloud quality check")
    group.add_argument("--trial", action="store_true", help="Laptop webcam only, no robot")
    parser.add_argument("--address", default=None, help="FlowBase address override (default: 172.6.2.20:11323)")
    args = parser.parse_args()

    if args.trial:
        from dimos.navigation.camera_nav.blueprint_flowbase import camera_nav_static_trial
        ModuleCoordinator.build(camera_nav_static_trial).loop()

    elif args.zed_only:
        from dimos.navigation.camera_nav.blueprint_zed import camera_nav_zed_standalone
        ModuleCoordinator.build(camera_nav_zed_standalone).loop()

    elif args.zed:
        from dimos.navigation.camera_nav.blueprint_zed import _make_zed_teleop
        ModuleCoordinator.build(_make_zed_teleop(address=args.address)).loop()

    else:
        from dimos.navigation.camera_nav.blueprint_flowbase import (
            _make_flowbase_coordinator,
            _CAMERA_MOUNT,
            _cloud_points,
        )
        from dimos.core.coordination.blueprints import autoconnect
        from dimos.hardware.drive_trains.flowbase.odom_tf import FlowBaseOdomModule
        from dimos.hardware.sensors.camera.module import CameraModule
        from dimos.navigation.camera_nav.recorder import CameraNavRecorder
        from dimos.perception.depth.accumulator import DepthAccumulatorModule
        from dimos.perception.depth.monocular_depth_module import MonocularDepthModule
        from dimos.protocol.pubsub.impl.lcmpubsub import LCM
        from dimos.visualization.rerun.bridge import RerunBridgeModule

        blueprint = autoconnect(
            _make_flowbase_coordinator(address=args.address),
            FlowBaseOdomModule.blueprint(),
            CameraModule.blueprint(transform=_CAMERA_MOUNT),
            MonocularDepthModule.blueprint(),
            DepthAccumulatorModule.blueprint(),
            CameraNavRecorder.blueprint(db_path="traversal.db"),
            RerunBridgeModule.blueprint(
                pubsubs=[LCM()],
                rerun_open="web",
                visual_override={"world/global_map": _cloud_points, "world/frame_cloud": _cloud_points},
            ),
        )
        ModuleCoordinator.build(blueprint).loop()

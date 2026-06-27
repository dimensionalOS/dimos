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
    python -m dimos.navigation.camera_nav.run_flowbase --zed-only           # ZED only, no robot
    python -m dimos.navigation.camera_nav.run_flowbase --zed-nav            # ZED → ZEDNavBridge → VoxelGridMapper (fixes motion update)
    python -m dimos.navigation.camera_nav.run_flowbase --zed-voxel          # ZED → VoxelGridMapper via HardwareDepthModule
    python -m dimos.navigation.camera_nav.run_flowbase --zed-compare        # SDK cloud vs our pipeline
    python -m dimos.navigation.camera_nav.run_flowbase --trial              # webcam only, no robot
"""

import argparse

from dimos.core.coordination.module_coordinator import ModuleCoordinator

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--zed", action="store_true", help="ZED Mini stereo camera (ZED SDK required)")
    group.add_argument("--zed-only", action="store_true", help="ZED Mini only, no robot — point cloud quality check")
    group.add_argument("--zed-nav", action="store_true", help="ZED → ZEDNavBridge → VoxelGridMapper (fixes motion update, nav-ready odometry)")
    group.add_argument("--zed-voxel", action="store_true", help="ZED → VoxelGridMapper via HardwareDepthModule")
    group.add_argument("--zed-compare", action="store_true", help="ZED SDK native cloud vs our pipeline side-by-side")
    group.add_argument("--robot-only", action="store_true", help="FlowBase keyboard teleop only, no camera")
    group.add_argument("--trial", action="store_true", help="Laptop webcam only, no robot")
    parser.add_argument("--address", default=None, help="FlowBase address override (default: 172.6.2.20:11323)")
    parser.add_argument("--cpu", action="store_true", help="Force CPU inference (use when MPS/CUDA is unstable)")
    args = parser.parse_args()

    if args.robot_only:
        from dimos.navigation.camera_nav.blueprint_flowbase import _make_flowbase_coordinator
        ModuleCoordinator.build(_make_flowbase_coordinator(address=args.address)).loop()

    elif args.trial:
        from dimos.navigation.camera_nav.blueprint_flowbase import camera_nav_static_trial
        from dimos.perception.depth.monocular_depth_module import MonocularDepthModule
        if args.cpu:
            # Override device for trial mode when MPS/CUDA is unavailable or unstable.
            from dimos.core.coordination.blueprints import autoconnect
            from dimos.hardware.sensors.camera.module import CameraModule
            from dimos.navigation.camera_nav.viz import cloud_points, pinhole_setup
            from dimos.perception.depth.accumulator import DepthAccumulatorModule
            from dimos.protocol.pubsub.impl.lcmpubsub import LCM
            from dimos.visualization.rerun.bridge import RerunBridgeModule
            blueprint = autoconnect(
                CameraModule.blueprint(),
                MonocularDepthModule.blueprint(device="cpu"),
                DepthAccumulatorModule.blueprint(),
                RerunBridgeModule.blueprint(
                    pubsubs=[LCM()],
                    rerun_open="web",
                    visual_override={
                        "world/global_map": cloud_points,
                        "world/frame_cloud": cloud_points,
                        "world/camera_info": pinhole_setup,
                    },
                ),
            )
            ModuleCoordinator.build(blueprint).loop()
        else:
            ModuleCoordinator.build(camera_nav_static_trial).loop()

    elif args.zed_only:
        from dimos.navigation.camera_nav.blueprint_zed import camera_nav_zed_standalone
        ModuleCoordinator.build(camera_nav_zed_standalone).loop()

    elif args.zed_nav:
        from dimos.navigation.camera_nav.blueprint_zed import camera_nav_zed_nav
        ModuleCoordinator.build(camera_nav_zed_nav).loop()

    elif args.zed_voxel:
        from dimos.navigation.camera_nav.blueprint_zed import camera_nav_zed_voxel
        ModuleCoordinator.build(camera_nav_zed_voxel).loop()

    elif args.zed_compare:
        from dimos.navigation.camera_nav.blueprint_zed import camera_nav_zed_compare
        ModuleCoordinator.build(camera_nav_zed_compare).loop()

    elif args.zed:
        from dimos.navigation.camera_nav.blueprint_zed import _make_zed_teleop
        ModuleCoordinator.build(_make_zed_teleop(address=args.address)).loop()

    else:
        from dimos.navigation.camera_nav.blueprint_flowbase import camera_nav_flowbase_teleop
        ModuleCoordinator.build(camera_nav_flowbase_teleop).loop()

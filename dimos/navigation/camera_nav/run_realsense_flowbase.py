"""RealSense D435i depth map + FlowBase keyboard teleop.

FlowBase coordinator runs in a background thread; the depth pipeline
runs in the main thread and owns the Rerun session.

Usage:
    python -m dimos.navigation.camera_nav.run_realsense_flowbase
    python -m dimos.navigation.camera_nav.run_realsense_flowbase --address 10.0.0.87:11323
    python -m dimos.navigation.camera_nav.run_realsense_flowbase --robot-only
"""

import argparse
import threading


def _start_robot(address: str | None) -> None:
    from dimos.core.coordination.module_coordinator import ModuleCoordinator
    from dimos.navigation.camera_nav.blueprint_flowbase import _make_flowbase_coordinator
    try:
        ModuleCoordinator.build(_make_flowbase_coordinator(address=address)).loop()
    except Exception as e:
        print(f"[FlowBase] {e}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--address", default=None,
        help="FlowBase Portal RPC address (default: 172.6.2.20:11323). "
             "On office WiFi try 10.0.0.87:11323.",
    )
    parser.add_argument(
        "--robot-only", action="store_true",
        help="FlowBase keyboard teleop only, skip RealSense.",
    )
    args = parser.parse_args()

    if args.robot_only:
        _start_robot(args.address)
    else:
        t = threading.Thread(target=_start_robot, args=(args.address,), daemon=True)
        t.start()

        from dimos.navigation.camera_nav.realsense_depth_map import main
        main()

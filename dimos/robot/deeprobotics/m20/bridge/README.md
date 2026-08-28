# M20 robot-local ROS bridge

This native module is built and run on the M20 Pro GOS computer. It converts the
vendor's local ROS 2/DrDDS topics into typed DimOS streams without a Python
`rclpy` dependency or another application host.

The current C++ NativeModule SDK carries those streams over local LCM. The M20
blueprints therefore pin the complete onboard graph to LCM; this bridge does not
contain a private Zenoh implementation.

The bridge always subscribes to `/LIDAR/POINTS`, `/ODOM`, `/LOCATION_STATUS`,
and `/HES_STATUS`. It publishes lidar, pose, odometry, TF, and command-readiness
streams into the local DimOS graph.

`enable_command_output` defaults to `false`. When explicitly enabled, the bridge
owns a `/NAV_CMD` publisher but emits nonzero velocity only while:

- location status is fresh and exactly `1` (normal);
- hard-estop status is fresh and exactly `0` (not triggered);
- the `/NAV_CMD` publisher has a matched subscriber;
- the Python connection has explicitly armed and supplied a fresh bounded command.

The native watchdog uses a steady clock and sends zero after command timeout,
on health loss, and during shutdown. Starting the bridge never changes robot
mode, gait, planner service, charging state, or standing state.

Build on GOS after sourcing the vendor environment:

```bash
./build.sh
```

Deploy from a DimOS source checkout. The Python wheel does not include this C++
source tree or the in-repo native SDK, both of which the robot-local build uses.

The default setup path is `/opt/robot/scripts/setup_ros2.sh`; override it with
`M20_ROS_SETUP` if the inspected robot differs. The build intentionally fails
off-robot when Foxy and the installed `drdds` message package are unavailable.
It also requires CMake, a C++20 compiler, pkg-config, and the LCM development
package. For an offline build, clone the pinned `dimos-lcm` revision separately
and set `DIMOS_LCM_DIR` to that checkout before running `build.sh`.

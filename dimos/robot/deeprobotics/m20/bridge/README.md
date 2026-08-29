# M20 robot-local ROS bridge

This native module is built and run on the M20 Pro GOS computer. It converts the
vendor's local ROS 2/DrDDS topics into typed DimOS streams without a Python
`rclpy` dependency or another application host.

The current C++ NativeModule SDK carries those streams over local LCM. The M20
blueprints therefore pin the complete onboard graph to LCM; this bridge does not
contain a private Zenoh implementation.

The bridge always subscribes to `/LIDAR/POINTS`, `/IMU`, and `/HES_STATUS`. It
publishes the raw cloud, IMU, lidar-readiness, and command-readiness streams
into the local DimOS graph. `M20PointLio` consumes the cloud and IMU and is the
sole producer of pose, odometry, TF, map-ready lidar, and localization-readiness.

The inspected M20 publishes merged front/rear clouds at 10 Hz with reliable,
volatile DDS QoS. Each point uses the vendor's 26-byte layout (`x`, `y`, `z`,
`intensity`, `ring`, `timestamp`); the bridge preserves the fields and bytes.
Before forwarding, it rejects empty, malformed, big-endian, or XYZ-less clouds.
The native steady-clock watchdog marks the stream stale after 0.5 seconds
without a mapper-compatible cloud and logs every loss/recovery transition. The
vendor driver reports `lidar_link`, although its documented merge already
applies both sensor extrinsics into `base_link`, so the bridge normalizes that
known-mislabeled frame to `base_link`.

GOS starts `rsdriver.service` as root and creates its Fast DDS shared-memory
segment with mode `0644`. A normal `user` subscriber cannot attach, and the
vendor writer does not fall back to UDP for a same-host cloud reader. Install
the checked-in `deploy/rsdriver.service.d` drop-in and its permission helper.
The service stays root for real-time scheduling, but its active Fast DDS files
become group-writable by the existing `user` group so DimOS remains unprivileged.

`enable_command_output` defaults to `false`. When explicitly enabled, the bridge
owns a `/NAV_CMD` publisher but emits nonzero velocity only while:

- the local PointLIO estimate is fresh and advancing;
- `/MOTION_INFO` is fresh and confirms RL Control state `17`;
- any received hard-estop status is exactly `0` (not triggered);
- a valid merged lidar cloud has arrived within the lidar timeout;
- the `/NAV_CMD` publisher has a matched subscriber;
- the Python connection has explicitly armed and supplied a fresh bounded command.

The inspected firmware advertises `/HES_STATUS` with the documented DDS type
and QoS but emits no samples, including to the vendor `ros2 topic echo` tool.
The bridge therefore does not misuse it as a liveness heartbeat: an observed
trigger still vetoes commands, while the robot controller independently
enforces the physical hard stop below this API.

The native watchdog uses a steady clock and sends zero after command timeout,
on robot-control loss, and during shutdown. Lidar and PointLIO readiness remain
navigation diagnostics; they do not permanently disarm Go2-style manual motion.
Starting the bridge never changes robot mode, gait, planner service, charging
state, or standing state on startup. The normal explicit operator action is one
`M20Connection.standup()` RPC, which switches `basic_server` to navigation usage
mode, completes Stand → RL Control, resets and selects the navigation gait,
waits for command-path feedback, and arms.

The bridge diagnoses but does not remotely manage the vendor sensor pipeline.
For boot-persistent clouds, `multicast-relay.service` must be enabled on NOS and
`rsdriver.service` enabled on GOS; both must be active before DimOS starts.
Install the GOS service drop-in/helper from `deploy/`, and load
`deploy/dimos-m20.env` in the DimOS launcher. The explicit 16 MiB LCM receive
buffer is required for the observed 0.8-2.1 MB clouds; the LCM bus keeps its
default TTL 0, so this high-bandwidth stream remains local to GOS.

See [`deploy/README.md`](../deploy/README.md) for the complete packet path,
persistent installation steps, live checks, and remote Rerun attachment.

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

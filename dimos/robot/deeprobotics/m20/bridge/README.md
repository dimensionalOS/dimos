# M20 command/state ROS bridge

This native module runs on the M20 Pro GOS computer and links against the
installed ROS 2 Foxy and `drdds` packages. It carries only low-bandwidth robot
command and state traffic between ROS and typed DimOS streams.

Lidar and IMU do not pass through this bridge. `M20PointLio` subscribes directly
to `/LIDAR/POINTS` and `/IMU` in its own native process, validates the X20 cloud
contract, converts the IMU units, and publishes `lidar_ready`, localization,
odometry, TF, and the map-ready cloud. Keeping the sensor payload out of this
bridge avoids copying and serializing every 0.8-2.1 MB cloud through LCM.

The bridge subscribes to the vendor motion and hard-estop state topics. When
`enable_command_output` is explicitly enabled, it owns the `/NAV_CMD`,
`/MOTION_STATE`, and `/GAIT` publishers. Startup does not change the robot mode,
gait, charging state, or standing state.

The current C++ NativeModule SDK carries DimOS command/state streams over local
LCM. The M20 blueprints therefore pin the onboard graph to LCM; this bridge does
not contain a private Zenoh implementation.

Build on GOS after sourcing the vendor environment:

```bash
./build.sh
```

The default setup path is `/opt/robot/scripts/setup_ros2.sh`; override it with
`M20_ROS_SETUP` if necessary. The build intentionally fails off-robot when
Foxy and the installed `drdds` package are unavailable. For an offline build,
set `DIMOS_LCM_DIR` to the pinned `dimos-lcm` checkout.

See [`deploy/README.md`](../deploy/README.md) for the robot service setup,
command ownership, and launch procedure.

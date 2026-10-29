# sim2

sim2 runs a MuJoCo world as one DimOS module. The robot's `ConnectionModule`
stays in the blueprint in both hardware and simulation mode, with the same
public command and feedback streams. Its shared base selects either the real
device or an injected simulation device; the latter exchanges standard motor
and sensor messages with the world on private streams.

`dimos --simulation mujoco run <blueprint>` collects the robot description,
adds the world, and replaces supported sensor drivers with simulated workers.
The application keeps its controller, connection, mapping and planning modules.

## Layout

```text
dimos/sim2/
  spec.py                RobotConfig, RobotInstance, WorldConfig, SimulatedRobot
  blueprint.py           simulation(world, module): one world plus its sensor workers
  composition.py         simulate(): collect robots, inject devices, replace sensor drivers
  device.py              Simulation device: checked connection values to world streams
  module.py              SimulationModule: lifecycle, robot streams, scene and reset RPCs
  messages.py            StateFrame: the MuJoCo state snapshot sensor workers restore
  world/
    runtime.py           SimulationRuntime: compile, step, accept commands, build feedback
    scene.py             Compose scene and robot MJCFs; scene_path, scene_robot
    scene_types.py       The scene sidecar schema (entities, joints, regions, spawns)
    spec.py              SimulationSpec: describe() for sensor workers
  sensors/
    spec.py              Mounts and acquisition timing shared by all sensors
    composition.py       Supported sensor families
    device.py            Acquisition clock and delivery envelope
    reader.py            Worker-local model restored from snapshots
    module.py            Sensor worker lifecycle and publication
    camera/ lidar/ imu/  spec, device and module per sensor family
    lidar/models/mid360/ rolling Livox Mid360: firing pattern, return model
```

## How a blueprint becomes a simulation

Under `--simulation mujoco` the module coordinator calls `composition.simulate` on the
blueprint before anything is deployed. It looks for two declarations:

- A robot connection with `default_sim_scene: ClassVar[str]` and a static
  `simulated_robot(config, scene) -> SimulatedRobot`.
- A sensor driver with a static `simulated_sensor(config) -> Camera | Lidar | Imu`.

The robot connection is retained under its original name. The collector injects
a `simulation_device` factory into its configuration and adds one generic
`SimulationModule`. `ConnectionModule.connect()` selects that factory in MuJoCo
mode and the connection's `create_hardware_device()` in hardware mode. A missing
simulation device fails explicitly instead of opening the hardware.

The world uses `_sim2/<connection-instance>/<port>` streams for motor commands,
joint feedback and the control IMU. The device receives the world's feedback
and passes it to the connection's public output ports. Those ports keep the
blueprint's original remaps, so the controller continues using the same topics.

```text
ControlCoordinator -> G1WholeBodyConnection -> selected device
  hardware device  -> Unitree DDS
  simulation device -> private motor streams -> SimulationModule

SimulationModule -> private feedback -> simulation device
                 -> G1WholeBodyConnection -> public feedback -> ControlCoordinator
```

Sensor drivers are replaced with simulated workers under the same instance
names and public stream names. Mid360's simulated worker therefore still feeds
`lidar_raw` and `imu_raw` into Point-LIO. Its `frame_id` must name a site or
camera in the robot's MJCF; that site supplies the physical mounting pose.
Ground truth the hardware never had (`odom`, `tf`, a lidar's world-frame
`pointcloud`) gets plain or robot-qualified stream names. Automatic composition
currently supports one simulated robot per blueprint.
Its motor device currently requires `pd=True` and the `motor_command` /
`motor_states` port contract; other connection command interfaces need an
adapter before automatic simulation composition can support them.

### Adding a robot

1. Put the MJCF under the robot package and give it the named sites and cameras
   the sensors will mount on, using the frame names the hardware drivers use.
2. Write `simulation.py` next to the connection: a `RobotConfig` binding the
   coordinator's joint names to MJCF joints and actuators, a function placing a
   `RobotInstance` (start pose, hold gains, control IMU).
3. On the retained `ConnectionModule`, declare `default_sim_scene` and
   `simulated_robot(config, scene)` returning `SimulatedRobot(robot_id, instance)`.
   The connection implements `create_hardware_device()` for its real device;
   simulation selection belongs to the shared base.

No robot-specific world subclass is required. The connection owns its public
ports, command conversion, validation and deadman behavior in both modes.

`dimos/robot/unitree/g1/simulation.py` and `G1WholeBodyConnection` are the
reference.

### Adding a sensor

Give the hardware driver a static `simulated_sensor(config)` that returns the
device model with the driver's rates and `frame_id` as its mount. The driver's
output port names must be what the simulated module publishes: `lidar` and
`imu` for a lidar, `color_image`, `camera_info`, `depth_image`,
`depth_camera_info` for a camera. `Mid360.simulated_sensor` is the reference;
it attaches the device IMU at its factory offset from the laser through a
`Mount` on the lidar's own site.

## Robot IO

A robot in the world is a `RobotConfig` (MJCF, root body, joints bound by
name to MJCF joints and actuators, sensors mounted on named sites or cameras)
placed by a `RobotInstance` (pose, start pose, hold gains, control IMU,
command timeout). Both are plain data.

The collector supplies the world's `robot_topics` mapping, keyed by robot ID
and message kind (`motor_command`, `motor_states`, `imu`, `odom`, `tf`). The
generic world subscribes and publishes through these configured streams. The
simulation device converts checked named joint values back into
`MotorCommandArray` messages in the robot definition's joint order.

Each physics step publishes the robot's joint state, control IMU and ground
truth odometry with one timestamp, plus the `world` to `<frame_id>` TF edge.
A `MotorCommandArray` on the command port is applied as PD plus feedforward
torque until `command_timeout` passes without a newer one; then the robot holds
where it is, like a hardware deadman. Native position servos accept positions
only. Malformed commands are counted in `status()` and ignored. A port named
for a robot that is not in the world fails at start.

The G1 GR00T blueprint continues using its existing transport whole-body
adapter. `G1WholeBodyConnection.motor_states` publishes `g1/motor_states` and
its `motor_command` input receives `g1/motor_command` in both modes. The world
publishes its motor feedback only on the private stream, so it does not bypass
the connection or create a second public publisher.

## Sensors

Sensor workers run in their own processes. The world publishes a `StateFrame`
on its `snapshots` port every snapshot stride; each sensor module subscribes,
restores the state into its own copy of the compiled model (fetched through
`describe()`), and renders or raycasts there. Physics never waits on a sensor.
Cameras publish `color_image`, `camera_info` and, with depth, `depth_image`,
`depth_camera_info` and optionally `pointcloud`. Lidars publish what a real
driver does, `lidar` (sensor frame, per-point offsets and channels) and `imu`
(the device's own IMU), plus two truth extras a real device cannot give,
`pointcloud` in the world frame and `tf`; `Lidar(truth_outputs=False)` turns the
extras off when an estimator such as Point-LIO is under test. The rolling
`Mid360` model fires 200,000 points/s continuously across scan boundaries with
range noise and grazing-angle dropout. `simulation()` names each worker
`<robot>_<sensor>` and remaps its outputs to `<robot>/<sensor>/<port>`;
blueprints remap from there to the names their consumers use.

## Scenes

A scene is `scene.xml`, optionally with a `scene.json` sidecar
(`SceneDescription`) that names bodies, fixture joints, regions and spawn
poses. `scene_robot()` places a robot on a named spawn plus its own root
height. The world applies the sidecar's authored initial poses at load.
`describe_scene()` returns the sidecar; `reset()` restores the authored state
and starts a new episode; `set_paused()` stops physics while RPCs stay live.
Privileged scene state, scene editing and run evidence are not part of this
package.

## Running

```bash
dimos --simulation mujoco run unitree-g1-groot-wbc
dimos --simulation mujoco --scene-package workbench run unitree-g1-groot-wbc
uv run pytest dimos/sim2
uv run pytest -m self_hosted dimos/sim2   # needs a native OpenGL renderer
uv run mypy --follow-imports=silent dimos/sim2
```

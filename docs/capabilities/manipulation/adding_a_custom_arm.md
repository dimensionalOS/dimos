# How to Integrate a New Manipulator Arm

> Arm built on Damiao motors? Start with [Arms with Damiao motors](#arms-with-damiao-motors) below.

This guide walks through integrating a new robot arm with dimOS, from writing its connection module to creating blueprints for planning and control.

## Architecture Overview

Each arm sits behind a **connection module**: a `ConnectionModule` subclass
(`dimos/control/connection/connection_module.py`) that wraps the vendor SDK and
talks to the `ControlCoordinator` through named `JointState` streams:

```
┌──────────────────────────────────────────────────────────────┐
│              ManipulationModule (Planning)                   │
│  - Plans collision-free trajectories (RoboPlan or Drake)     │
│  - Sends trajectories to coordinator via RPC                 │
└───────────────────────┬──────────────────────────────────────┘
                        │ RPC: execute trajectory
┌───────────────────────▼──────────────────────────────────────┐
│              ControlCoordinator (100Hz control loop)         │
│  - Reads each robot's latest joint_state                     │
│  - Runs tasks (trajectory, velocity, gripper, ...)           │
│  - Arbitrates per-joint conflicts (priority-based)           │
│  - Sends each robot its complete command every tick          │
│  - Publishes aggregated joint state                          │
└───────────────────────┬──────────────────────────────────────┘
                        │ position_command / joint_state
┌───────────────────────▼──────────────────────────────────────┐
│              Your ConnectionModule                           │
│  - Wraps vendor SDK (TCP/IP, CAN, serial, etc.)              │
│  - Converts between vendor units and SI units                │
│  - Holds the arm when commands stop (deadman)                │
└──────────────────────────────────────────────────────────────┘
```

## Arms with Damiao motors

If dimOS talks to your arm's Damiao motors directly (DM4310, DM4340, DM8009, each on its own CAN ID, through a CAN
adapter or a Damiao USB-CAN bridge), don't write a vendor-SDK connection module (Step 1 below). Damiao arms are
connection modules: the motor protocol, the CAN transports, gripper calibration, gravity compensation and command
checks live in `dimos/robot/manipulators/damiao/` and the `can-motor-control` library (installed by the `control`
extra). Your arm declares its data and a small connection class in `dimos/robot/manipulators/<yourarm>/`. Copy the
closest example: `openyam/` or `makermods_metal/` (one arm, one bus) or `openarm/` (two arms, two buses).

1. **Files.** `config.py` (the URDF model, a `RobotModelConfig` factory that takes `robot="arm"` and passes the
    finger joints to `named_for_robot(..., fingers=...)`, and the robot data), `connection.py`,
    `blueprints/basic.py` (the coordinator, the trajectory and gripper `TaskConfig`s, the planner), `test_config.py`
    and `docs/capabilities/manipulation/<yourarm>_integration.md`; optionally `joints.py` (joint-name constants) and
    `coordinator.py` (an empty `DamiaoCoordinator` subclass, as in `openyam/coordinator.py`). There is no
    `_registry.py` or `__init__.py`.

2. **Robot data.** Declare one module-level `DamiaoRobot(buses=[...], model=..., gravity_model=..., base_link=...,
    joint_names=[...], gravity=True, home_joints=[...])` (`dimos/robot/manipulators/damiao/robot.py`). Each
    `DamiaoBus(arm=[...], kp=[...], kd=[...], gripper=..., name="arm")` is one CAN bus with one arm group and at
    most one gripper. Each arm motor is `DamiaoMotor(name, type, send_id, reply_id)` from the vendor's motor table;
    its name is the joint's name in `model` and `gravity_model`, and `reply_id` is the feedback ID. `joint_names`
    lists every arm motor in bus order, then each gripper's finger joint. Commands and states are matched by joint
    name, but `kp` and `kd` (per bus), `teach_kd`, `home_joints` and the CAN selectors are matched by position, so
    keep them in motor order. Nothing checks the motor table against the arm: a wrong or swapped CAN ID is not
    detected (only a reply ID used twice on one bus is rejected, and a gripper motor that doesn't answer on its IDs
    makes start fail). A motor that never answers reads 0.0, and one that stops answering keeps its last reading.
    Holds target the reading, so a joint with a wrong `reply_id` is driven toward 0 rad at full gain as soon as the
    arm starts. Check the IDs with the vendor's tool, and start the first time with the arm supported near its zero
    pose. Motor fault codes are not read either: a motor that trips its own protection goes limp without an error in
    dimOS.

3. **Motors.** `type` is `"DM4310"`, `"DM4340"` or `"DM8009"`. A wrong type scales velocity and torque incorrectly
    while positions still look right. Pass a `DamiaoProfile(code, p_max, v_max, t_max)` as
    `DamiaoMotor(..., profile=...)` (its `type` is then ignored; `code` is 0x8000 to 0xFFFF) only when the PMAX,
    VMAX and TMAX stored in the motor differ from the stock ones (MakerMods Metal is the example in this repo).
    dimOS has no per-joint sign or zero offset: each motor's zero pose and positive direction must match the URDF,
    or gravity compensation, the joint limits and planned motions are wrong. Do the vendor's motor ID and zero-pose
    procedure first, and confirm each joint's direction on the arm.

4. **Connection and transport.** In `connection.py`, subclass `DamiaoConnectionConfig` with a default `source`, one
    required `str` field per bus (`can_port`, or `left_can_port` and `right_can_port`) and a validator that rejects
    blank values and defaults `model` to your robot's model. Subclass `DamiaoConnection` with
    `config: <YourConfig>`, `robot = <YOUR_ROBOT>` and a `can_ports()` that returns the selectors in bus order (see
    `openyam/connection.py`). Don't implement Damiao framing or a serial protocol yourself. A selector that starts
    with `/dev/` uses the Damiao USB-CAN bridge on that serial port (`can_motor_control.DmSerialBus`); otherwise it
    is a SocketCAN interface such as `can0` on Linux, or a gs_usb adapter's serial number on macOS. There are no
    default selectors, and the connection doesn't configure the interface: on Linux bring it up first with
    `dimos hardware can setup can0` (1 Mbit/s by default); `dimos hardware can list` shows the selectable interfaces
    (gs_usb serial numbers on macOS). On a two-bus arm the buses can share motor IDs, so swapped selectors drive the
    wrong arm without any error. If a transport you need is missing, add it to the shared code, not to your arm.

5. **Gains.** `kp` and `kd` on each `DamiaoBus` hold one value per arm motor; grippers have no gains. dimOS checks
    the count when the robot data is built and, when it sends, that every value is finite; only the codec checks
    ranges, at send time. For every motor type the codec accepts `kp` from 0 to 500 and `kd` from 0 to 5, and it
    checks each command's position, velocity and torque, including the gravity term, against the motor type's limits
    (DM4310: 10 N·m, 30 rad/s; DM4340: 28 N·m, 10 rad/s; DM8009: 54 N·m, 45 rad/s; positions ±12.5 rad). With a
    value out of range the arm still starts and enables, then every command and hold raises `CodecError` at the
    first out-of-range row: that motor, the motors after it in bus order and the grippers stay enabled without
    commands (motors before it are still commanded), and it shows up only as a repeated warning and in
    `status().last_error`. Check gains copied from a vendor SDK against these ranges, and check the largest gravity
    torque over the joint range (for example with `PhysicalModel(...).gravity(q)`) against the motor's torque limit.

    Your arm's calibration is one file, `blueprints/calibrate.py`:
    `<yourarm>_calibrate = calibration_blueprint(<YourConnection>).remappings([])`, which hands the calibration module
    your connection's `robot`. `dimos run <yourarm>-calibrate` runs it and `dimos shell` drives the calibration module's
    `run`, `status`, `answer` and `abort`; the robot data's `name` (`<yourarm>`, `_` for `-`) names its arms'
    calibration files. `DamiaoConnection` serves `get_config()` and `set_config(patch, save)`,
    which the calibration drives; each arm id's entry is an `ArmConfigState` or `ArmConfigPatch` from
    `dimos/control/calibration/schema.py`.

6. **Gripper.** `DamiaoGripper(name, joint, open_at, motor, opening, current, calibration_travel)`. `joint` is the
    URDF finger joint (a second finger must be a mimic or fixed joint, see `RobotModel.with_mimic_joint`; if the
    URDF has no driven finger joint, add one as `dual_openyam/config.py` does), and the jaw is read and commanded in
    that joint's unit, with the `open_at` end (`"lower"` or `"upper"`) open. `opening` is the motor direction that
    opens the jaw (`"increasing_position"` or `"decreasing_position"`). `current` is a per-unit current in (0, 1],
    not amperes. `calibration_travel` (motor radians, default 4.0) must exceed the gripper's full stroke; a shorter
    one silently calibrates only part of it. A wrong `opening` or `open_at`, or a gripper task whose `open_at`
    differs from the robot data, inverts the jaw. These values depend on the physical gripper: take them from the
    vendor or measure them, and say which values are unverified. Activation calibrates the gripper by driving it to
    both of its stops, and the arm motors get no commands while it does.

7. **Model and gravity.** `model` is the URDF the connection drives and reports: its movable, non-mimic joints must
    be exactly `joint_names`, each with a finite position range. Don't make an arm joint continuous: the limit
    checks accept it, but if `gravity_model` has it as continuous too, `connect()` fails. `gravity_model` must list
    the arm joints in motor order (other joints are locked), or `connect()` fails even with `gravity=False`.
    `gravity` is on by default and needs real masses and inertias; set it to `False` only when you can't provide
    them (MakerMods Metal sets `gravity=False`). Readings are published as measured, even past a limit, while joints
    that no task drives are held at their last position clamped into the URDF limits, so check the URDF limits
    against the arm's real stops. Disabling the motors (a fault, a stop or a failed start) removes all torque, so an
    arm without brakes falls.

8. **Blueprints.** In `blueprints/basic.py`, compose the coordinator and the connection as the examples do:
    `autoconnect(<Coordinator>.blueprint(instance_name="ControlCoordinator", model=<model config with
    robot="arm">.model, tasks=[...], zero_stiffness_joints=[]), arm_connection(<YourConnection>))` remapped with
    `arm_remaps(<YourConnection>, <Coordinator>)`, a trajectory task over the `arm/<joint>` names, a
    `TaskConfig(type="gripper")` whose `open_at` matches the robot data, and a planner blueprint built with
    `planner(...)` and `gripper_tasks` (which maps each planning group to its gripper task). The CAN selector lives
    on the connection, so users pass `--can-port` (or `--left-can-port` and `--right-can-port`); `--simulation`
    swaps in `MockJointConnection`, and so does a `SIMULATION` environment variable or `.env` entry. Register
    blueprints by regenerating `dimos/robot/all_blueprints.py` with
    `pytest dimos/robot/test_all_blueprints_generation.py` (outside CI it rewrites the file, then fails until you
    stage it). The scanner registers only a plain top-level assignment to a public name (no leading `_`, no type
    annotation) whose value is an `autoconnect(...)` call or ends in `.blueprint`, `.remappings`, `.transports`,
    `.global_config`, `.requirements` or `.configurators`; a bare `x = yourarm_control()` is not registered, which
    is why the examples end with `.remappings([])`. Also add each new blueprint whose name doesn't start with
    `coordinator-` to `experimental/gateway/annotations.yaml`, or `experimental/gateway/tests/test_config.py` fails.
    Steps 4 (URDF and planning) and 5 (register blueprints) below apply; the other steps describe the vendor-SDK
    path.

9. **Tests and docs.** Add `test_config.py` modeled on `makermods_metal/test_config.py` (selectors, CLI options,
    tasks), and add your arm's wiring and gains to `ARMS` in `dimos/robot/manipulators/damiao/test_damiao_robot.py`
    and its connection to `ARMS` in `test_damiao_lifecycle.py`. These tests use a fake native robot that accepts any
    value, so they pass with gains or gravity torques the codec rejects; check those as described in step 5. Add
    `docs/capabilities/manipulation/<yourarm>_integration.md`, modeled on
    [OpenYAM integration](/docs/capabilities/manipulation/openyam_integration.md).

## Prerequisites

1. **Vendor SDK**: The Python SDK for your robot arm (e.g., `xarm-python-sdk`, `piper-sdk`)
2. **URDF/xacro**: A robot description file (only needed if you want motion planning)
3. **Connection info**: IP address, CAN port, serial device, etc.

## Step 1: Write the Connection Module

Each arm gets its own folder under `dimos/robot/manipulators/`. The A-750
(`dimos/robot/manipulators/a750/`) is a small, complete example to copy:

```
dimos/robot/manipulators/yourarm/
├── config.py            # URDF model, RobotModelConfig factory, ConnectionConfig helper
├── connection.py        # YourArmConnection(ConnectionModule)
├── test_connection.py   # The connection against a fake SDK
└── blueprints/
    └── basic.py         # Coordinator, connection and planner blueprints
```

`connection.py` declares a config and a module. The config says where the arm
is plugged in and which joints take which commands; the module implements
four methods. Values are in SI units (rad, m) under canonical
`<source>/<joint>` names, e.g. `arm/joint1`.

```python skip
class YourArmConfig(ConnectionConfig):
    """Where YourArm is plugged in.

    Attributes:
        address: The arm's IP address, e.g. "192.168.1.100".
    """

    address: str
    source: str = "arm"


class YourArmConnection(ConnectionModule):
    """YourArm driven over its SDK; joint targets in rad on ``position_command``."""

    config: YourArmConfig

    position_command: In[JointState]
    joint_state: Out[JointState]

    @rpc
    def start(self) -> None:
        super().start()  # Calls connect(), then starts the deadman.
        self.register_disposable(Disposable(self.position_command.subscribe(
            lambda msg: self.command(joint_state_to_values(msg, (Interface.POSITION,)))
        )))
        # Start a thread that publishes joint_state at the arm's own rate.

    def connect(self) -> None:
        """Open the SDK; raise if the arm does not answer."""

    def write(self, values: Values) -> None:
        """Send targets, e.g. {"arm/joint1/position": 0.3}. A joint left out keeps its target."""

    def halt(self) -> None:
        """Hold the arm where it is. Called when commands stop."""
```

See `dimos/robot/manipulators/a750/connection.py` for the full version,
including the read loop, and `a750/config.py` for the model and the
`ConnectionConfig` helper the coordinator shares.

## Step 2: Create Your Blueprints

The coordinator drives a robot through ports named `<robot>_<kind>`, declared
on a subclass. The blueprint remaps the connection's ports onto them. Create
`dimos/robot/manipulators/yourarm/blueprints/basic.py`:

```python skip
"""Blueprints for YourArm.

Usage:
    dimos run coordinator-yourarm --yourarmconnection.address 192.168.1.100
"""

from __future__ import annotations

from dimos.control.coordinator import ControlCoordinator
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.yourarm.config import yourarm_coordinator_connection
from dimos.robot.manipulators.yourarm.connection import YourArmConnection


class YourArmCoordinator(ControlCoordinator):
    arm_position_command: Out[JointState]
    arm_joint_state: In[JointState]


coordinator_yourarm = autoconnect(
    YourArmCoordinator.blueprint(
        instance_name="ControlCoordinator",  # RPC clients look for this name
        tick_rate=100.0,
        connections=[yourarm_coordinator_connection()],
        tasks=[joint_trajectory_task([f"arm/joint{i}" for i in range(1, 7)])],
    ),
    YourArmConnection.blueprint(),
).remappings([
    (YourArmConnection, "position_command", "arm_position_command"),
    (YourArmCoordinator, "arm_joint_state", "joint_state"),
])
```

If your arm has a gripper, give it a joint in the model (the A-750's is
`finger`), declare it in `joint_commands`, and add
`TaskConfig(name="arm_gripper", type="gripper", joint_names=["arm/finger"])`
(from `dimos.control.coordinator`) to `tasks`. With a planner (Step 3c), also
pass `gripper_tasks={"manipulator": "arm_gripper"}` to `planner(...)`.

### Blueprint field reference

| Field | Description |
|-------|-------------|
| `connections` | One `ConnectionConfig` per robot: its `source` name, model and per-joint command interfaces. The connection module gets the same declaration. |
| `task.name` | Name used by the ManipulationModule to invoke trajectory execution via RPC. |
| `task.type` | Task type, such as `"trajectory"`, `"velocity"`, `"cartesian_ik"` or `"gripper"`. |
| `task.priority` | Priority for per-joint arbitration. Higher number wins. |

## Step 3: Add URDF and Planning Integration (Optional)

If you want motion planning, you need a URDF and a planning blueprint. Add these
to your robot's own `blueprints/basic.py`.

### 3a. Add your URDF

Prefer an upstream `RobotDescriptionSource` beside your robot's config. Joining
paths from the source is lazy: importing the catalog does not clone or update
the repository, while the first concrete path access resolves it into the
robot asset cache. Use `LfsPath` only when the description is intentionally
vendored, locally modified, or has no suitable upstream source.

If the planning blueprint selects the RoboPlan TOPP-RA trajectory
parametrizer, dimOS pins RoboPlan `0.8.0`. Every movable joint in
each selected planning group must provide finite, positive velocity and
acceleration limits. An authored extended acceleration limit is accepted:

```xml
<joint name="joint1" type="revolute">
  <!-- parent, child, origin, and axis omitted -->
  <limit
    lower="-3.14"
    upper="3.14"
    effort="100"
    velocity="2.0"
    acceleration="4.0"
  />
</joint>
```

Standard URDF has no acceleration attribute. When the source omits it, apply
an explicit robot-level default before building `RobotModelConfig`:

```python skip
model = RobotModel.from_file(urdf_path).with_default_joint_acceleration_limit(2.0)
```

To weld another URDF tree onto an existing link, use `with_attached_model`.
The child is a regular `RobotModel`, so its own model transforms and package,
mesh, and texture resolution are applied before attachment.

```python skip
model = base_model.with_attached_model(
    arm_model,
    parent_link="mount_link",
    joint_name="left_mount",
    prefix="left_",
    xyz=(0.0, 0.3, 0.0),
    rpy=(0.0, 0.0, 0.0),
)
```

For a shared mounting frame with no geometry, create the root directly:

```python skip
base_model = RobotModel.from_root_link("mount_link", name="dual_arm")
```

This requires no URDF file. Its `source_path` is `None`, including after loading;
each attached model still resolves assets relative to its own source. Automatic
SRDF discovery is skipped for an in-memory root, but explicit SRDF paths work.

The named fixed joint connects the parent's specified link to the child's root.
The optional prefix namespaces the child's links, joints, materials, and mimic
references. Translations are in meters; `rpy` uses extrinsic xyz rotations in
radians. Construction returns a new immutable model without loading assets.
The host's source subtree filters and planar base compile first. Fixed frames
and attachments then apply in declaration order to the current XML tree. Host
joint fixes, position limits, joint renames, and acceleration defaults compile
last, in that order, and can address namespaced child joints. Child models use
their own complete load pipeline before attachment. Further attachments can
target links introduced by earlier attachments or fixed frames. No host-model
wrapper or replay of nested host snapshots is involved.

`load()` uses the existing in-memory model pipeline. It does not write a generated
URDF or introduce a separate cache. The model retains its host's `source_path` (if any);
backends consume the transformed XML from `load()`, as with other model views.
Each child resolves its assets separately, so package names can be reused by
different inputs. Missing parent links, duplicate names, and invalid child trees
fail explicitly. Controller/simulator extensions in child descriptions are
rejected rather than silently copying unrewritten deployment references.

The model is materialized and validated once. All planning backends then use
the same compiled position, velocity, and acceleration semantics. Missing,
zero, negative, or non-finite limits fail startup and identify the affected
joint.

```python skip
from dimos.robot.manipulators.common.blueprints import planner
from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.assets.model import RobotModel
from dimos.robot.assets.source import RobotDescriptionSource

_YOURARM_REPO = RobotDescriptionSource(
    url="https://github.com/example/yourarm_description",
    ref="main",
)
_YOURARM_URDF_PATH = _YOURARM_REPO / "urdf" / "yourarm.urdf.xacro"
_YOURARM_PACKAGE_PATHS = {"yourarm_description": _YOURARM_REPO / "."}


def _make_base_pose(x=0.0, y=0.0, z=0.0) -> PoseStamped:
    return PoseStamped(
        frame_id="map",
        position=Vector3(x=x, y=y, z=z),
        orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
    )
```

### 3b. Create a robot model config helper

```python skip
from dimos.robot.manipulators._modeling import named_for_robot


def _make_yourarm_config(y_offset: float = 0.0) -> RobotModelConfig:
    """Create YourArm robot config for planning.

    Args:
        y_offset: Y-axis offset for model placement.
    """
    # The joint names as they are in your URDF
    joint_names = [f"joint{i}" for i in range(1, 7)]

    config = RobotModelConfig(
        model=RobotModel.from_file(
            _YOURARM_URDF_PATH,
            package_paths=_YOURARM_PACKAGE_PATHS,
        ).with_default_joint_acceleration_limit(2.0),
        joint_names=joint_names,
        planning_groups=[
            PlanningGroupDefinition(
                name="manipulator",
                joint_names=tuple(joint_names),
                base_link="base_link",
                tip_link="link6",
            )
        ],
        base_pose=_make_base_pose(y=y_offset),  # world -> base_link placement
        base_link="base_link",                 # Robot-scoped placement/weld/strip link
        collision_exclusion_pairs=[],   # Pairs of links that can touch (e.g., gripper fingers)
        auto_convert_meshes=True,       # Convert DAE/STL meshes for Drake
    )
    # Match the arm/joint1... names the connection gives the coordinator.
    return named_for_robot(config, "arm")
```

### 3c. Create a planning blueprint

Add this to your `dimos/robot/manipulators/yourarm/blueprints/basic.py` alongside the coordinator blueprint:

```python skip
from dimos.core.coordination.blueprints import autoconnect

yourarm_planner = autoconnect(
    planner(
        model=_make_yourarm_config(),
        planning_timeout=10.0,
        visualization={"backend": "viser"},
        trajectory_parametrization={"backend": "simple_trapezoid"},
    ),
    coordinator_yourarm,
)
# The planner's `coordinator_joint_state` input auto-connects to the
# ControlCoordinator's output on the default `/coordinator_joint_state`
# topic, so no `.transports(...)` override is needed.
```

You may omit `trajectory_parametrization` when the world-based default is
appropriate: `world_backend="roboplan"` selects `roboplan_toppra`, while
`world_backend="drake"` selects `simple_trapezoid`.

To configure TOPP-RA tuning explicitly, select RoboPlan for the world and
parametrizer after adding the URDF limits described above:

```python skip
yourarm_planner = autoconnect(
    planner(
        model=_make_yourarm_config(),
        world_backend="roboplan",
        trajectory_parametrization={
            "backend": "roboplan_toppra",
            "velocity_scale": 0.8,
            "acceleration_scale": 0.8,
        },
        visualization={"backend": "viser"},
    ),
    coordinator_yourarm,
)
```

### Key config fields

| Field | Description |
|-------|-------------|
| `model` | Lazy `RobotModel` created from a `.urdf` or `.xacro` source |
| `joint_names` | Ordered canonical model joint set (must match the URDF and coordinator); not itself a planning group |
| `planning_groups` / `srdf_path` | Explicit planning groups or SRDF source; direct `RobotModelConfig(...)` helpers should pass explicit groups, while shared config helpers can discover groups from SRDF/fallback |
| `base_pose` / `base_link` | Optional robot placement: `base_pose` places `base_link` in the world for weld/strip behavior |
| `collision_exclusion_pairs` | List of `(link_a, link_b)` tuples for links that may legitimately touch (e.g., gripper fingers) |

Coordinator-facing joint states and trajectories use the model's canonical
joint names unchanged. Keep hardware-native name translation inside the
connection module.

Planning-group `base_link`/`tip_link` values define kinematic chains and pose
target frames. `base_link` is only the robot-scoped link placed by
`base_pose`; do not use it as a substitute for planning-group chain metadata.
See [Planning Groups](/docs/capabilities/manipulation/planning_groups.md).

### 3d. Add Cartesian and teleoperation IK

Cartesian, EEF-twist, and engagement-relative teleoperation tasks share the
Pink backend. Install its dependencies with the manipulation extra:

```bash skip
uv sync --extra manipulation --inexact
```

Use one control task for each robot model that Pink should solve as one system:

| Robot setup | Task layout |
| --- | --- |
| One arm | One task with one target frame |
| Two independent robot models | One task per arm |
| One bimanual robot model | One task with both arm joint sets and two target frames |

The task uses coordinator-facing joint names and URDF frame names. Give it the
same `RobotModelConfig` used by planning; hardware-native naming remains inside
the connection module.

```python skip
from dimos.control.coordinator import TaskConfig
from dimos.manipulation.planning.kinematics.config import PinkKinematicsConfig

robot_model = _make_yourarm_config()
teleop_task = TaskConfig(
    name="teleop_arm",
    type="teleop_ik",
    joint_names=list(robot_model.joint_names),
    params={
        "robot_model": robot_model,
        "bindings": [{"hand": "right", "target_frame": "link6"}],
        "pink": PinkKinematicsConfig(),
    },
)
```

For a bimanual model, pass both joint sets and bind each hand to its own frame:

```python skip
bimanual_task = TaskConfig(
    name="teleop_dual_arm",
    type="teleop_ik",
    joint_names=[*left_arm_joint_names, *right_arm_joint_names],
    params={
        "robot_model": dual_arm_model,
        "bindings": [
            {"hand": "left", "target_frame": "left_tool_frame"},
            {"hand": "right", "target_frame": "right_tool_frame"},
        ],
        "pink": PinkKinematicsConfig(),
    },
)
```

Do not pass planning groups into a control task. Planning groups select planning
requests; control IK needs only the controlled joints and target frames. A
single-hand task requires that hand's primary button. A bimanual task requires
both buttons, and releasing either clears both controller references.

Before connecting hardware, follow [Pink IK Configuration and Tuning](/docs/capabilities/manipulation/pink_ik_tuning.md)
to tune task weights, robot-specific objectives, velocity limits, feedback
tolerance, and command tracking.

## Step 4: Register Blueprints

The blueprint registry in `dimos/robot/all_blueprints.py` is **auto-generated** by scanning the codebase for blueprint declarations. After adding your blueprints:

1. Run the generation test to update the registry:
   ```bash
   pytest dimos/robot/test_all_blueprints_generation.py
   ```
2. Now you can run your arm via CLI:
   ```bash
   dimos run coordinator-yourarm
   dimos run yourarm-planner        # If you added a planning blueprint
   ```

## Step 5: Testing

Test the connection against a fake SDK, as
`dimos/robot/manipulators/a750/test_connection.py` does: start the module,
send a `position_command`, and check what reached the fake SDK and what
`joint_state` reports. Check that the arm holds where it is when commands stop.

The blueprint tests under `dimos/robot/` build every registered blueprint, so a
wiring mistake (a port without a declaration, a wrong remapping) fails there.

## Quick Reference Checklist

Files to create:

- [ ] `dimos/robot/manipulators/yourarm/config.py` (model, `RobotModelConfig`, `ConnectionConfig` helper)
- [ ] `dimos/robot/manipulators/yourarm/connection.py` (`ConnectionModule` subclass)
- [ ] `dimos/robot/manipulators/yourarm/test_connection.py` (against a fake SDK)
- [ ] `dimos/robot/manipulators/yourarm/blueprints/basic.py` (coordinator + planning blueprints)

Files to modify:

- [ ] `pyproject.toml`: Add vendor SDK to optional dependencies *(if applicable)*

Verification:

- [ ] `pytest dimos/robot/test_all_blueprints_generation.py` passes (regenerates `all_blueprints.py`)
- [ ] `dimos run coordinator-yourarm` starts successfully

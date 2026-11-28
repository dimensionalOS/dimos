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

import copy
from enum import Enum
from importlib import resources
import math
import sys
from threading import Event, Lock, Thread
import time
from typing import Any, Protocol

from pydantic import Field, model_validator
from reactivex import timer
from reactivex.disposable import Disposable
from reactivex.observable import Observable
from scipy.spatial.transform import Rotation

from dimos.agents.annotation import skill
from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.control.connection.connection_module import (
    ConnectionConfig as JointConnectionConfig,
    ConnectionModule,
)
from dimos.control.contract.convert import joint_state_to_values
from dimos.control.contract.keys import Interface
from dimos.core.core import rpc
from dimos.core.global_config import global_config
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.robot.assets.model import JointDescription, RobotModel
from dimos.robot.unitree.config import connection_type, require_robot_ip
from dimos.robot.unitree.connection import UnitreeWebRTCConnection
from dimos.robot.unitree.type.lowstate import LowStateMsg
from dimos.simulation.dimsim.config import DimSimConfig
from dimos.simulation.mujoco.config import MujocoConfig
from dimos.spec.perception import Camera, Pointcloud
from dimos.utils.generic import classproperty
from dimos.utils.logging_config import setup_logger
from dimos.utils.transform_utils import pose_to_values

if sys.version_info < (3, 13):
    from typing_extensions import Self, TypeVar
else:
    from typing import Self, TypeVar

logger = setup_logger()

_LIDAR_STARTUP_TIMEOUT_S = 10.0

_ROBOT_IP_GUIDANCE = (
    "Set ROBOT_IP or --robot-ip (or --go2connection.ip), "
    "or run without a robot using --replay or --simulation."
)


class Go2Mode(str, Enum):
    DEFAULT = "default"
    RAGE = "rage"


# Control-only native body resources, not generalized planar coordinates.
GO2_BASE_JOINTS: tuple[str, str, str] = ("go2/base_forward", "go2/base_lateral", "go2/base_turn")
GO2_MODEL = RobotModel.from_root_link("base_link", name="go2").with_added_joints(
    *(
        JointDescription(
            name=name,
            type="continuous" if i == 2 else "prismatic",
            parent_link="base_link",
            child_link=f"go2_body_control_{i}",
            axis=axis,
            velocity=velocity,
            acceleration=acceleration,
        )
        for i, (name, axis, velocity, acceleration) in enumerate(
            zip(
                GO2_BASE_JOINTS,
                ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
                (2.0, 2.0, 3.0),
                (5.0, 5.0, 8.0),
                strict=True,
            )
        )
    )
)


GO2_JOINT_COMMANDS: dict[str, tuple[Interface, ...]] = {
    name: (Interface.VELOCITY,) for name in GO2_BASE_JOINTS
}


def go2_joint_connection() -> JointConnectionConfig:
    """The Go2 as a coordinator sees it: robot name, model and velocity-only body joints.

    For a coordinator's ``connections``. ``ConnectionConfig()`` would need a robot IP.
    """
    return JointConnectionConfig(source="go2", model=GO2_MODEL, joint_commands=GO2_JOINT_COMMANDS)


class ConnectionConfig(JointConnectionConfig):
    source: str = "go2"
    model: RobotModel | None = GO2_MODEL
    joint_commands: dict[str, tuple[Interface, ...]] = GO2_JOINT_COMMANDS
    odom_timeout_s: float = Field(default=0.2, gt=0.0, allow_inf_nan=False)
    ip: str | None = Field(default_factory=lambda m: m["g"].robot_ip)
    mode: Go2Mode = Go2Mode.DEFAULT
    lidar: bool = True
    camera: bool = True
    velocity_api: bool = False
    # Seconds between resends of the command during a timed move.
    timed_move_resend_period_s: float = Field(default=0.01, gt=0.0, allow_inf_nan=False)
    # "mcf" for stair traversal, "normal" for basic, None to leave it as is
    motion_mode: str | None = None
    # Per-device AES-128 key (Go2 fw >=1.1.15).
    aes_128_key: str | None = Field(default=None, repr=False)
    obstacle_avoidance: bool = True
    replay_exit: bool = False
    replay_loop: bool = False
    mujoco: MujocoConfig = Field(default_factory=MujocoConfig)
    dimsim: DimSimConfig = Field(default_factory=DimSimConfig)
    # TF parent frame of the internal odometry (odom_frame_id -> base_link).
    # Rename (e.g. "go2_odom") when another odom source owns the tree root
    odom_frame_id: str = "world"
    # Turn off where another module owns the base_link edge. The odom port
    # keeps publishing either way.
    publish_tf: bool = True

    @model_validator(mode="after")
    def validate_connection(self) -> Self:
        self.validate_ip()
        mode = connection_type(replay=self.g.replay, simulation=self.g.simulation).lower()
        # make_connection gives these offline IP aliases precedence over simulation.
        if (
            mode == "dimsim"
            and self.ip not in ("fake", "mock", "replay", "mujoco")
            and self.g.transport != "lcm"
        ):
            raise ValueError(
                "DimSim publishes over LCM only; the transport is "
                f"{self.g.transport!r}. Use --transport lcm with --simulation dimsim."
            )
        return self

    def validate_ip(self) -> None:
        mode = connection_type(replay=self.g.replay, simulation=self.g.simulation).lower()
        if mode == "webrtc":
            require_robot_ip(self.ip, _ROBOT_IP_GUIDANCE)


class Go2ConnectionProtocol(Protocol):
    """Protocol defining the interface for Go2 robot connections."""

    def start(self) -> None: ...
    def stop(self) -> None: ...
    def lidar_stream(self) -> Observable[PointCloud2]: ...
    def odom_stream(self) -> Observable[PoseStamped]: ...
    def video_stream(self) -> Observable[Image]: ...
    def lowstate_stream(self) -> Observable[LowStateMsg]: ...
    def move(self, twist: Twist, duration: float = 0.0) -> bool: ...
    def stop_movement(self) -> None: ...
    def standup(self) -> bool: ...
    def liedown(self) -> bool: ...
    def balance_stand(self) -> bool: ...
    def sport_command(self, api_id: int) -> bool: ...
    def set_obstacle_avoidance(self, enabled: bool = True) -> bool: ...
    def set_rage_mode(self, enable: bool) -> bool: ...
    def set_light(self, level: int) -> bool: ...
    def switch_joystick(self, enable: bool = True) -> bool: ...
    def publish_request(self, topic: str, data: dict) -> dict: ...  # type: ignore[type-arg]


_FRONT_CAMERA_720_YAML = resources.files("dimos.robot.unitree.go2").joinpath(
    "front_camera_720.yaml"
)


def _camera_info_static() -> CameraInfo:
    with resources.as_file(_FRONT_CAMERA_720_YAML) as yaml_path:
        return CameraInfo.from_yaml(str(yaml_path))


def _prefixed(prefix: str | None, name: str) -> str:
    """Apply a TF namespace prefix (ModuleBase.frame_id_prefix) to a frame name."""
    if not prefix or not name:
        return name
    return f"{prefix}/{name}"


# Static camera mount chain: base_link -> camera_link -> camera_optical.
# TODO we need a standardized way to specify this for all cameras in dimos
BASE_TO_OPTICAL: Transform = Transform(
    translation=Vector3(0.3, 0.0, 0.0),
    rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
    frame_id="base_link",
    child_frame_id="camera_link",
) + Transform(
    translation=Vector3(0.0, 0.0, 0.0),
    rotation=Quaternion(-0.5, 0.5, -0.5, 0.5),
    frame_id="camera_link",
    child_frame_id="camera_optical",
)


def make_connection(
    ip: str | None,
    cfg: ConnectionConfig,
    aes_128_key: str | None = None,
    velocity_api: bool = False,
) -> Go2ConnectionProtocol:
    mode = connection_type(replay=cfg.g.replay, simulation=cfg.g.simulation).lower()

    if ip in _REPLAY_IPS or mode == "replay":
        raise ValueError("A Go2 replay is a Go2Replay module: GO2Connection.blueprint() picks it")
    elif ip == "mujoco" or mode in ("mujoco", "true"):
        from dimos.robot.unitree.mujoco_connection import MujocoConnection

        return MujocoConnection(cfg.mujoco, robot_model=cfg.g.robot_model)
    elif mode == "dimsim":
        from dimos.robot.unitree.dimsim_connection import DimSimConnection

        return DimSimConnection(cfg.dimsim, transport=cfg.g.transport)
    elif mode == "webrtc":
        ip = require_robot_ip(ip, _ROBOT_IP_GUIDANCE)
        return UnitreeWebRTCConnection(
            ip,
            aes_128_key=aes_128_key,
            velocity_api=velocity_api,
            timed_move_resend_period_s=cfg.timed_move_resend_period_s,
        )
    else:
        raise ValueError(f"Unknown simulator {cfg.g.simulation!r}. Choose from: mujoco, dimsim")


_REPLAY_IPS = ("fake", "mock", "replay")


_Config = TypeVar("_Config", bound=ConnectionConfig, default=ConnectionConfig)


class GO2Connection(ConnectionModule, Camera, Pointcloud):
    dedicated_worker = True

    config: ConnectionConfig
    cmd_vel: In[Twist]
    velocity_command: In[JointState]
    joint_state: Out[JointState]
    pointcloud: Out[PointCloud2]
    odom: Out[PoseStamped]
    odometry: Out[Odometry]
    lidar: Out[PointCloud2]
    color_image: Out[Image]
    camera_info: Out[CameraInfo]
    tf: Out[TFMessage]

    connection: Go2ConnectionProtocol
    camera_info_static: CameraInfo = _camera_info_static()
    _camera_info_thread: Thread | None = None
    _latest_lowstate: LowStateMsg | None = None

    @classmethod
    def rerun_views(cls):  # type: ignore[no-untyped-def]
        import rerun.blueprint as rrb

        """Return Rerun view blueprints for GO2 camera visualization."""
        return [
            rrb.Spatial2DView(
                name="Camera",
                origin="world/robot/camera/rgb",
            ),
        ]

    @classproperty
    def blueprint(cls) -> Any:  # noqa: N805
        """Under ``--replay`` (or a fake/mock/replay ip) the Go2 is a :class:`Go2Replay`."""
        from dimos.core.coordination.blueprints import Blueprint

        def create(**kwargs: Any) -> Blueprint:
            mode = connection_type(replay=global_config.replay, simulation=global_config.simulation)
            ip = kwargs.get("ip", global_config.robot_ip)
            if cls is GO2Connection and (mode.lower() == "replay" or ip in _REPLAY_IPS):  # type: ignore[comparison-overlap]
                from dimos.robot.unitree.go2.replay import Go2Replay

                return Blueprint.create(Go2Replay, **kwargs)
            return Blueprint.create(cls, **kwargs)  # type: ignore[arg-type]

        return create

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._odom_lock = Lock()
        self._previous_odom: PoseStamped | None = None
        self._camera_info_stop = Event()
        self.connection = make_connection(
            self.config.ip,
            self.config,
            aes_128_key=self.config.aes_128_key,
            velocity_api=self.config.velocity_api,
        )

        if hasattr(self.connection, "camera_info_static"):
            self.camera_info_static = self.connection.camera_info_static

        if self.frame_id_prefix and self.camera_info_static.frame_id:
            # Copy so the class-level default is not mutated.
            self.camera_info_static = copy.copy(self.camera_info_static)
            self.camera_info_static.frame_id = _prefixed(
                self.frame_id_prefix, self.camera_info_static.frame_id
            )

    @rpc
    def start(self) -> None:
        super().start()
        self._camera_info_stop.clear()
        if not hasattr(self, "connection"):
            return

        def onimage(image: Image) -> None:
            image.frame_id = _prefixed(self.frame_id_prefix, image.frame_id)
            self.color_image.publish(image)

        if self.config.lidar:
            lidar_received = Event()

            def on_lidar(message: PointCloud2) -> None:
                lidar_received.set()
                self.lidar.publish(message)

            def warn_missing_lidar(_: int) -> None:
                if not lidar_received.is_set():
                    logger.warning(
                        "No lidar message received. Mapping and navigation need lidar data. "
                        "Check the lidar stream.",
                        timeout_s=_LIDAR_STARTUP_TIMEOUT_S,
                    )

            self.register_disposable(self.connection.lidar_stream().subscribe(on_lidar))
            # Diagnose the physical robot. DimSim sends lidar directly over LCM,
            # and replay/simulation timing need not match hardware startup.
            if isinstance(self.connection, UnitreeWebRTCConnection):
                self.register_disposable(
                    timer(_LIDAR_STARTUP_TIMEOUT_S).subscribe(warn_missing_lidar)
                )
        self.register_disposable(self.connection.odom_stream().subscribe(self._publish_tf))
        self.register_disposable(self.connection.lowstate_stream().subscribe(self._on_lowstate))
        self.register_disposable(Disposable(self.cmd_vel.subscribe(self.move)))
        self.register_disposable(
            Disposable(
                self.velocity_command.subscribe(
                    lambda msg: self.command(joint_state_to_values(msg, (Interface.VELOCITY,)))
                )
            )
        )

        if self.config.camera:
            self.register_disposable(self.connection.video_stream().subscribe(onimage))
            self._camera_info_thread = Thread(
                target=self.publish_camera_info,
                daemon=True,
            )
            self._camera_info_thread.start()

        if self.config.motion_mode and isinstance(self.connection, UnitreeWebRTCConnection):
            self.connection.set_motion_mode(self.config.motion_mode)

        self.standup()
        if isinstance(self.connection, UnitreeWebRTCConnection):
            # The physical robot takes about 3 s to stand before it accepts BalanceStand.
            time.sleep(3)
        self.connection.balance_stand()

        if self.config.mode == Go2Mode.RAGE:
            self.connection.set_rage_mode(True)

        self.connection.set_obstacle_avoidance(self.config.obstacle_avoidance)

    def connect(self) -> None:
        with self._odom_lock:
            self._previous_odom = None
        self.connection.start()

    def write(self, values: dict[str, float]) -> None:
        vx, vy, wz = (values.get(f"{name}/velocity", 0.0) for name in GO2_BASE_JOINTS)
        if not all(math.isfinite(v) for v in (vx, vy, wz)):
            raise ValueError("body velocities must be finite")
        if not self.connection.move(Twist(Vector3(vx, vy, 0.0), Vector3(0.0, 0.0, wz))):
            raise RuntimeError("Go2 rejected movement")

    def halt(self) -> None:
        self.connection.stop_movement()

    @rpc
    def stop(self) -> None:
        # Best-effort steps: teardown must always reach the WebRTC disconnect.
        self._camera_info_stop.set()
        try:
            # Close the command gate and drain callbacks before lying down,
            # so an in-flight canonical command cannot restart motion in teardown.
            super().stop()
            try:
                self.liedown()
            except Exception:
                logger.warning("liedown on stop failed (link already down?) — continuing teardown")
            if self._camera_info_thread and self._camera_info_thread.is_alive():
                self._camera_info_thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        finally:
            if self.connection:
                try:
                    self.connection.stop()
                except Exception:
                    logger.warning("connection stop failed", exc_info=True)

    @classmethod
    def _odom_to_tf(cls, odom: PoseStamped, prefix: str = "") -> list[Transform]:
        # The odom parent frame (odom.frame_id) stays unprefixed so namespaced
        # robots still hang off one shared tree root.
        camera_link = Transform(
            translation=Vector3(0.3, 0.0, 0.0),
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
            frame_id=_prefixed(prefix, "base_link"),
            child_frame_id=_prefixed(prefix, "camera_link"),
            ts=odom.ts,
        )

        camera_optical = Transform(
            translation=Vector3(0.0, 0.0, 0.0),
            rotation=Quaternion(-0.5, 0.5, -0.5, 0.5),
            frame_id=_prefixed(prefix, "camera_link"),
            child_frame_id=_prefixed(prefix, "camera_optical"),
            ts=odom.ts,
        )

        return [
            Transform.from_pose(_prefixed(prefix, "base_link"), odom),
            camera_link,
            camera_optical,
        ]

    def _publish_tf(self, msg: PoseStamped) -> None:
        if self._publish_from_odom(msg) and self.odom.transport:
            self.odom.publish(msg)

    def _publish_from_odom(self, msg: PoseStamped) -> bool:
        """joint_state, odometry and tf from one odom pose. False for an invalid pose."""
        # The backend exposes measured poses, not native chassis rates. Estimate
        # average body rates over valid source-timestamp intervals, never commands.
        try:
            pose_to_values(msg, ("x", "y", "yaw"))
            if not math.isfinite(msg.ts) or msg.ts <= 0 or not math.isfinite(msg.position.z):
                raise ValueError("invalid odometry timestamp/position")
        except ValueError:
            with self._odom_lock:
                self._previous_odom = None
            return False
        with self._odom_lock:
            previous = self._previous_odom
            self._previous_odom = copy.deepcopy(msg)
        if (
            previous is not None
            and msg.frame_id == previous.frame_id
            and 0 < msg.ts - previous.ts <= self.config.odom_timeout_s
        ):
            dt = msg.ts - previous.ts
            # Express measured displacement and angular increment in current body frame.
            linear = msg.orientation.inverse().rotate_vector(
                (msg.position - previous.position) / dt
            )
            delta = previous.orientation.inverse() * msg.orientation
            angular = Vector3(*Rotation.from_matrix(delta.to_rotation_matrix()).as_rotvec()) / dt
            if all(math.isfinite(v) for v in (*linear, *angular)):
                twist = Twist(linear, angular)
                self.joint_state.publish(
                    JointState(
                        ts=msg.ts,
                        frame_id=_prefixed(self.config.frame_id_prefix, "base_link"),
                        name=list(GO2_BASE_JOINTS),
                        velocity=[linear.x, linear.y, angular.z],
                    )
                )
                self.odometry.publish(
                    Odometry(
                        ts=msg.ts,
                        frame_id=self.config.odom_frame_id,
                        child_frame_id=_prefixed(self.config.frame_id_prefix, "base_link"),
                        pose=Pose(msg.position, msg.orientation),
                        twist=twist,
                    )
                )
        msg.frame_id = self.config.odom_frame_id
        if self.config.publish_tf:
            transforms = self._odom_to_tf(msg, prefix=self.frame_id_prefix)
            self.tf.publish(TFMessage(*transforms))
        return True

    def publish_camera_info(self) -> None:
        while not self._camera_info_stop.is_set():
            self.camera_info.publish(self.camera_info_static.with_ts(time.time()))
            self._camera_info_stop.wait(1.0)

    @rpc
    def move(self, twist: Twist, duration: float = 0.0) -> bool:
        """Send movement command to robot."""
        return self.connection.move(twist, duration)

    @rpc
    def standup(self) -> bool:
        """Make the robot stand up."""
        return self.connection.standup()

    @rpc
    def liedown(self) -> bool:
        """Make the robot lie down."""
        return self.connection.liedown()

    @rpc
    def balance_stand(self) -> bool:
        """Enter BalanceStand: neutral state for switching locomotion modes"""
        return self.connection.balance_stand()

    @rpc
    def set_rage_mode(self, enable: bool) -> bool:
        """Toggle Rage Mode on/off (~2.5 m/s envelope when on).
        On the WebRTC backend this re-establishes the BalanceStand
        precondition before toggling; sim backends are no-ops.
        """
        result = self.connection.set_rage_mode(enable)
        logger.info("Rage Mode", enabled=enable)
        return result

    @rpc
    def sport_command(self, api_id: int) -> bool:
        """Send a parameterless SPORT_MOD command by api_id (Hello, Damp, ...)."""
        return self.connection.sport_command(api_id)

    @rpc
    def set_light(self, level: int) -> bool:
        """Head-LED brightness level 0-10 (0 = off)."""
        return self.connection.set_light(level)

    @rpc
    def set_obstacle_avoidance(self, enabled: bool = True) -> bool:
        """Toggle the onboard obstacle avoidance."""
        return self.connection.set_obstacle_avoidance(enabled)

    @rpc
    def switch_joystick(self, enable: bool = True) -> bool:
        """Firmware joystick listening on/off (WASD stick emulation needs it on)."""
        return self.connection.switch_joystick(enable)

    @rpc
    def stop_movement(self) -> None:
        """Zero the base immediately (webrtc deadman stop)."""
        self.connection.stop_movement()

    def _on_lowstate(self, msg: LowStateMsg) -> None:
        """Cache the latest low-level state push (battery, IMU, motors, etc.)."""
        self._latest_lowstate = msg

    @skill
    def get_battery_soc(self) -> int | None:
        """Returns the robot's battery state-of-charge as a percentage (0-100).

        Use this skill to answer battery / power / charge questions. Returns
        None if no low-level state has been received yet.
        """
        return self.battery_soc()

    @rpc
    def battery_soc(self) -> int | None:
        """Battery SOC 0-100 (or None until lowstate arrives). Plain RPC — no
        skill-log spam, for the hosted telemetry poll."""
        try:
            return int(self._latest_lowstate["data"]["bms_state"]["soc"])  # type: ignore[index]
        except (KeyError, TypeError, ValueError):
            return None

    @rpc
    def publish_request(self, topic: str, data: dict[str, Any]) -> dict[Any, Any]:
        """Publish a request to the WebRTC connection.
        Args:
            topic: The RTC topic to publish to
            data: The data dictionary to publish
        Returns:
            The result of the publish request
        """
        return self.connection.publish_request(topic, data)

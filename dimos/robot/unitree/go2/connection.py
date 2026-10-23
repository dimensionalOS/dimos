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
import json
import sys
from threading import Event, Lock, Thread
import time
from typing import Any, Protocol

from pydantic import Field, model_validator
from reactivex import defer, empty, operators as ops
from reactivex.abc import SchedulerBase
from reactivex.disposable import Disposable
from reactivex.observable import Observable

from dimos.agents.annotation import skill
from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.resource import CompositeResource
from dimos.core.stream import In, Out
from dimos.memory.replay import Replay, ReplayStream, resolve_db_path
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.BatteryState import BatteryState
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.String import String
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.protocol.service.spec import BaseConfig
from dimos.robot.unitree.config import connection_type
from dimos.robot.unitree.go2.constants import CMD_VEL_TIMEOUT
from dimos.robot.unitree.type.lowstate import LowStateMsg
from dimos.simulation.dimsim.config import DimSimConfig
from dimos.simulation.mujoco.config import MujocoConfig
from dimos.spec.perception import Camera, Pointcloud
from dimos.utils.decorators.decorators import cached_property, simple_mcache
from dimos.utils.logging_config import setup_logger

if sys.version_info < (3, 13):
    from typing_extensions import TypeVar
else:
    from typing import TypeVar

logger = setup_logger()


class Go2Mode(str, Enum):
    DEFAULT = "default"
    RAGE = "rage"


class WebRtcTiming(BaseConfig):
    """Every wait, timeout and buffer size of the Rust WebRTC module (seconds unless named otherwise)."""

    answer_timeout: float = 5.0
    cmd_vel_timeout: float = CMD_VEL_TIMEOUT
    move_period: float = 0.01
    # the physical robot takes about 3 s to stand before it accepts BalanceStand
    stand_up_settle: float = 3.0
    rage_balance_settle: float = 0.3
    rage_switch_settle: float = 2.0
    motion_mode_settle: float = 5.0
    reconnect_pause: float = 2.0
    signaling_timeout: float = 4.0
    ice_gather_timeout: float = 4.0
    validation_timeout: float = 8.0
    heartbeat_period: float = 2.0
    ice_keepalive_period: float = 2.0
    # silence of link_quiet_timeout marks the link disconnected (it can recover); link_lost_timeout ends it
    link_quiet_timeout: float = 5.0
    link_lost_timeout: float = 30.0
    keyframe_retry_period: float = 1.0
    clock_window: float = 30.0
    clock_reset: float = 1.0
    video_queue_packets: int = 2048
    capture_max_waiting: int = 16
    # how long RTP packets wait for a late earlier one (a NACK's retransmit, an RTT behind) before it counts as lost
    reorder_wait: float = 0.15
    max_frame_bytes: int = 4 << 20
    camera_info_period: float = 1.0
    stats_period: float = 10.0

    def connect_deadline(self) -> float:
        """The longest a connect can take: ICE gathering, signaling (con_notify, con_ing, or the legacy offer)
        and the data channel's validation."""
        return self.ice_gather_timeout + 3 * self.signaling_timeout + self.validation_timeout


class ConnectionConfig(NativeModuleConfig):
    # The real robot runs the Rust WebRTC module (webrtc/rust); replay and the simulators stay in Python.
    source_dir: str | None = "dimos/robot/unitree/go2/webrtc/rust"
    executable: str = "result/bin/go2_webrtc"
    build_command: str | None = "nix build -L path:."
    stdin_config: bool = True

    ip: str = Field(default_factory=lambda m: m["g"].robot_ip)
    mode: Go2Mode = Go2Mode.DEFAULT
    lidar: bool = True
    camera: bool = True
    velocity_api: bool = False
    # "mcf" for stair traversal, "normal" for basic, None to leave it as is
    motion_mode: str | None = None
    # Per-device AES-128 key (Go2 fw >=1.1.15).
    aes_128_key: str | None = Field(default=None, repr=False)
    obstacle_avoidance: bool = True
    replay_exit: bool = False
    mujoco: MujocoConfig = Field(default_factory=MujocoConfig)
    dimsim: DimSimConfig = Field(default_factory=DimSimConfig)
    # TF parent frame of the internal odometry (odom_frame_id -> base_link).
    # Rename (e.g. "go2_odom") when another odom source owns the tree root
    odom_frame_id: str = "world"
    # Turn off where another module owns the base_link edge. The odom port
    # keeps publishing either way.
    publish_tf: bool = True
    # Reconnect when the Go2 drops the WebRTC link (a Wi-Fi drop mustn't end a recording); off, the module exits.
    auto_reconnect: bool = True
    timing: WebRtcTiming = Field(default_factory=WebRtcTiming)

    @model_validator(mode="after")
    def _shutdown_outlasts_the_lie_down(self) -> "ConnectionConfig":
        # teardown waits for the verb running (one answer) and the lie-down (one more) before it lets go of the robot
        if "shutdown_timeout" not in self.model_fields_set:
            self.shutdown_timeout = 2 * self.timing.answer_timeout + DEFAULT_THREAD_JOIN_TIMEOUT
        return self

    def to_cli_args(self) -> list[str]:
        """Nothing: the Rust module reads its config from stdin, and the command line is logged (the AES key)."""
        return []

    def to_config_dict(self) -> dict[str, Any]:
        """Exactly the Rust module's Config: it rejects unknown fields and has no defaults."""
        camera_info = _camera_info_static()
        camera_info.frame_id = _prefixed(self.frame_id_prefix, camera_info.frame_id)
        return {
            "ip": self.ip,
            "aes_128_key": self.aes_128_key or "",
            "mode": Go2Mode(self.mode).value,
            "lidar": self.lidar,
            "camera": self.camera,
            "velocity_api": self.velocity_api,
            "motion_mode": self.motion_mode or "",
            "obstacle_avoidance": self.obstacle_avoidance,
            "odom_frame_id": self.odom_frame_id,
            "publish_tf": self.publish_tf,
            "frame_id_prefix": self.frame_id_prefix or "",
            "auto_reconnect": self.auto_reconnect,
            "camera_info": camera_info.to_dict(),
            "camera_offset": [CAMERA_MOUNT.x, CAMERA_MOUNT.y, CAMERA_MOUNT.z],
            "timing": self.timing.model_dump(),
        }


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
    """Apply a TF namespace prefix (ModuleConfig.frame_id_prefix) to a frame name."""
    if not prefix or not name:
        return name
    return f"{prefix}/{name}"


# Static camera mount chain: base_link -> camera_link -> camera_optical.
# TODO we need a standardized way to specify this for all cameras in dimos
CAMERA_MOUNT = Vector3(0.3, 0.0, 0.0)
BASE_TO_OPTICAL: Transform = Transform(
    translation=CAMERA_MOUNT,
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
) -> Go2ConnectionProtocol | None:
    """Replay's or a simulator's connection; None for the real robot, which GO2Connection's Rust module drives."""
    mode = connection_type(replay=cfg.g.replay, simulation=cfg.g.simulation).lower()

    if ip in ("fake", "mock", "replay") or mode == "replay":
        return ReplayConnection(dataset=cfg.g.replay_db, exit_on_complete=cfg.replay_exit)
    elif ip == "mujoco" or mode in ("mujoco", "true"):
        from dimos.robot.unitree.mujoco_connection import MujocoConnection

        return MujocoConnection(cfg.mujoco, robot_model=cfg.g.robot_model)
    elif mode == "dimsim":
        from dimos.robot.unitree.dimsim_connection import DimSimConnection

        return DimSimConnection(cfg.dimsim, transport=cfg.g.transport)
    elif mode == "webrtc":
        return None
    else:
        raise ValueError(f"Unknown simulator {cfg.g.simulation!r}. Choose from: mujoco, dimsim")


_T = TypeVar("_T")


class ReplayConnection(CompositeResource):
    def __init__(
        self,
        dataset: str = "go2_china_office",
        *,
        seek: float | None = None,
        duration: float | None = None,
        loop: bool = False,
        exit_on_complete: bool = False,
    ) -> None:
        self.dataset = dataset
        self._seek = seek
        self._duration = duration
        self._loop = loop
        self._exit_on_complete = exit_on_complete
        self._completion_lock = Lock()
        self._subscribed = 0
        self._completed = 0
        self._sealed = False
        self._fired = False

    @cached_property
    def replay(self) -> Replay:
        # One shared store + Replay so lidar/odom/video advance against the
        # same wall-clock anchor on subscribe.
        store = self.register_disposable(
            SqliteStore(path=str(resolve_db_path(self.dataset)), must_exist=True)
        )
        store.start()
        return store.replay(loop=self._loop, seek=self._seek, duration=self._duration)

    def start(self) -> None:
        pass

    def standup(self) -> bool:
        return True

    def liedown(self) -> bool:
        return True

    def balance_stand(self) -> bool:
        return True

    def sport_command(self, api_id: int) -> bool:
        return True

    def stop_movement(self) -> None:
        # No webrtc deadman timer to cancel; the cmd_vel timeout covers replay.
        pass

    def set_obstacle_avoidance(self, enabled: bool = True) -> bool:
        return True

    def set_rage_mode(self, enable: bool) -> bool:
        return True

    def set_light(self, level: int) -> bool:
        return True

    def switch_joystick(self, enable: bool = True) -> bool:
        return True

    def _stream_name(self, *names: str) -> str:
        """Return the first of ``names`` present in the dataset (stream naming
        changed over time: mid360 recordings use go2_lidar/go2_odom, older ones
        lidar/odom)."""
        available = self.replay.list_streams()
        for name in names:
            if name in available:
                return name
        raise KeyError(f"None of {names!r} in dataset {self.dataset!r}; available: {available}")

    def _tracked(self, source: Observable[_T]) -> Observable[_T]:
        """With exit_on_complete, count every subscription to a replay stream
        and request coordinator shutdown once the last one finishes.

        Completions only count once the module has sealed the subscription
        set: subscriptions register one at a time, so without the seal a
        stream that completes synchronously (an empty one, say) would match
        the counters before the next stream ever subscribed.
        """
        if not self._exit_on_complete:
            return source

        def factory(_scheduler: SchedulerBase | None) -> Observable[_T]:
            with self._completion_lock:
                self._subscribed += 1
            return source.pipe(ops.do_action(on_completed=self._stream_completed))

        return defer(factory)

    def seal_subscriptions(self) -> None:
        """Mark the subscription set complete; exit-on-complete may now fire.

        Called by the module once start() has wired every stream it is going
        to. A no-op without exit_on_complete.
        """
        with self._completion_lock:
            self._sealed = True
        self._maybe_request_shutdown()

    def _stream_completed(self) -> None:
        with self._completion_lock:
            self._completed += 1
        self._maybe_request_shutdown()

    def _maybe_request_shutdown(self) -> None:
        with self._completion_lock:
            done = (
                self._exit_on_complete
                and self._sealed
                and not self._fired
                and self._subscribed > 0
                and self._completed == self._subscribed
            )
            if done:
                self._fired = True
        if not done:
            return
        logger.info("Replay finished; requesting coordinator shutdown")
        # Off the replay emission thread: the RPC round trip must not block
        # the stream's own teardown.
        Thread(target=self._request_shutdown, daemon=True).start()

    def _request_shutdown(self) -> None:
        # Inline import: the connection must stay importable without the
        # coordination stack.
        from dimos.core.coordination.coordinator_rpc import CoordinatorRPC

        try:
            client = CoordinatorRPC.connect(timeout=10.0)
            try:
                client.call("shutdown")
            finally:
                client.stop()
        except Exception:
            logger.error("Failed to request coordinator shutdown", exc_info=True)

    @simple_mcache
    def lidar_stream(self) -> Observable[PointCloud2]:
        stream: ReplayStream[PointCloud2] = self.replay.stream(
            self._stream_name("go2_lidar", "lidar")
        )
        return self._tracked(stream.observable())

    @simple_mcache
    def odom_stream(self) -> Observable[PoseStamped]:
        stream: ReplayStream[PoseStamped] = self.replay.stream(
            self._stream_name("go2_odom", "odom")
        )
        return self._tracked(stream.observable())

    @simple_mcache
    def video_stream(self) -> Observable[Image]:
        return self._tracked(self.replay.streams.color_image.observable())

    @simple_mcache
    def lowstate_stream(self) -> Observable:  # type: ignore[type-arg]
        # Replay datasets carry no low-level state (battery/IMU) — emit nothing.
        return empty()

    def move(self, twist: Twist, duration: float = 0.0) -> bool:
        return True

    def publish_request(self, topic: str, data: dict):  # type: ignore[no-untyped-def, type-arg]
        """Fake publish request for testing."""
        return {"status": "ok", "message": "Fake publish"}


_Config = TypeVar("_Config", bound=ConnectionConfig, default=ConnectionConfig)


class GO2Connection(NativeModule, Camera, Pointcloud):
    """The Go2's streams and commands. On the real robot (webrtc) a Rust process (webrtc/rust) holds the WebRTC
    link and publishes the ports itself; the RPCs reach it as verbs on `command` (the Rust `parse_verb`).
    Replay and the simulators run in this process through make_connection."""

    dedicated_worker = True

    config: ConnectionConfig
    cmd_vel: In[Twist]
    pointcloud: Out[PointCloud2]
    odom: Out[PoseStamped]
    lidar: Out[PointCloud2]
    color_image: Out[Image]
    camera_info: Out[CameraInfo]
    tf: Out[TFMessage]
    # One verb per message to the Rust module, GO2DDS's vocabulary (the Rust `parse_verb`): "stand-up", "rage on",
    # "move 0.5 0 0 2". Fire-and-forget, as on GO2DDS; only the rpcs publish it.
    command: In[String]
    battery: Out[BatteryState]

    # None on the real robot: the Rust module holds the link
    connection: Go2ConnectionProtocol | None
    camera_info_static: CameraInfo = _camera_info_static()
    _camera_info_thread: Thread | None = None
    _latest_lowstate: LowStateMsg | None = None
    _latest_battery: BatteryState | None = None

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

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.connection = make_connection(self.config.ip, self.config)
        if self.connection is None:
            return

        if hasattr(self.connection, "camera_info_static"):
            self.camera_info_static = self.connection.camera_info_static

        if self.config.frame_id_prefix and self.camera_info_static.frame_id:
            # Copy so the class-level default is not mutated.
            self.camera_info_static = copy.copy(self.camera_info_static)
            self.camera_info_static.frame_id = _prefixed(
                self.config.frame_id_prefix, self.camera_info_static.frame_id
            )

    @rpc
    def build(self) -> None:
        if self.connection is None:
            super().build()
        else:
            Module.build(self)

    @rpc
    def start(self) -> None:
        if self.connection is None:
            # the Rust process publishes straight to the transports, so listen there
            linked = Event()
            for unsubscribe in (
                self.battery.transport.subscribe(self._on_battery, self.battery),
                self.odom.transport.subscribe(lambda _: linked.set(), self.odom),
            ):
                self.register_disposable(Disposable(unsubscribe))
            super().start()
            # a bad key or a busy robot fails the run here, as it does on the other backends
            deadline = self.config.timing.connect_deadline()
            if not linked.wait(deadline):
                raise RuntimeError(
                    f"the Go2 at {self.config.ip} didn't connect within {deadline} s (see the go2_webrtc log)"
                )
            return
        Module.start(self)
        self.connection.start()

        def onimage(image: Image) -> None:
            image.frame_id = _prefixed(self.config.frame_id_prefix, image.frame_id)
            self.color_image.publish(image)

        if self.config.lidar:
            self.register_disposable(self.connection.lidar_stream().subscribe(self.lidar.publish))
        self.register_disposable(self.connection.odom_stream().subscribe(self._publish_tf))
        self.register_disposable(self.connection.lowstate_stream().subscribe(self._on_lowstate))
        self.register_disposable(Disposable(self.cmd_vel.subscribe(self.move)))

        if self.config.camera:
            self.register_disposable(self.connection.video_stream().subscribe(onimage))
            self._camera_info_thread = Thread(
                target=self.publish_camera_info,
                daemon=True,
            )
            self._camera_info_thread.start()

        if isinstance(self.connection, ReplayConnection):
            # Every stream this module will subscribe is wired; a finished
            # replay may now shut the run down (--go2connection.replay-exit).
            self.connection.seal_subscriptions()

        self.standup()
        self.connection.balance_stand()

        if self.config.mode == Go2Mode.RAGE:
            self.connection.set_rage_mode(True)

        self.connection.set_obstacle_avoidance(self.config.obstacle_avoidance)

    @rpc
    def stop(self) -> None:
        # The Rust module lies the robot down in its own teardown.
        if self.connection is None:
            super().stop()
            return

        # Best-effort steps: teardown must always reach the connection's stop.
        try:
            self.liedown()
        except Exception:
            logger.warning("liedown on stop failed (link already down?) — continuing teardown")

        try:
            self.connection.stop()
        except Exception:
            logger.warning("connection stop failed", exc_info=True)

        if self._camera_info_thread and self._camera_info_thread.is_alive():
            self._camera_info_thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)

        Module.stop(self)

    def _send(self, verb: str) -> bool:
        """A verb for the Rust module, as Go2Base.send_command: it runs in order, and a failure is logged there."""
        # TODO: True only means "sent", not "ran" (hosted teleop acks on it); return the outcome once Rust modules have RPCs.
        self.command.transport.publish(String(verb))
        return True

    def _on_battery(self, msg: BatteryState) -> None:
        self._latest_battery = msg

    @classmethod
    def _odom_to_tf(cls, odom: PoseStamped, prefix: str = "") -> list[Transform]:
        # The odom parent frame (odom.frame_id) stays unprefixed so namespaced
        # robots still hang off one shared tree root.
        camera_link = Transform(
            translation=CAMERA_MOUNT,
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
        msg.frame_id = self.config.odom_frame_id
        if self.config.publish_tf:
            transforms = self._odom_to_tf(msg, prefix=self.config.frame_id_prefix or "")
            self.tf.publish(TFMessage(*transforms))
        if self.odom.transport:
            self.odom.publish(msg)

    def publish_camera_info(self) -> None:
        while True:
            self.camera_info.publish(self.camera_info_static.with_ts(time.time()))
            time.sleep(self.config.timing.camera_info_period)

    @rpc
    def move(self, twist: Twist, duration: float = 0.0) -> bool:
        """Send movement command to robot."""
        if self.connection is None:
            return self._send(
                f"move {twist.linear.x} {twist.linear.y} {twist.angular.z} {duration}"
            )
        return self.connection.move(twist, duration)

    @rpc
    def standup(self) -> bool:
        """Make the robot stand up."""
        if self.connection is None:
            return self._send("stand-up")
        return self.connection.standup()

    @rpc
    def liedown(self) -> bool:
        """Make the robot lie down."""
        if self.connection is None:
            return self._send("stand-down")
        return self.connection.liedown()

    @rpc
    def balance_stand(self) -> bool:
        """Enter BalanceStand: neutral state for switching locomotion modes"""
        if self.connection is None:
            return self._send("balance")
        return self.connection.balance_stand()

    @rpc
    def set_rage_mode(self, enable: bool) -> bool:
        """Toggle Rage Mode on/off (~2.5 m/s envelope when on).
        On the WebRTC backend this re-establishes the BalanceStand
        precondition before toggling; sim backends are no-ops.
        """
        if self.connection is None:
            result = self._send(f"rage {'on' if enable else 'off'}")
        else:
            result = self.connection.set_rage_mode(enable)
        logger.info("Rage Mode", enabled=enable)
        return result

    @rpc
    def sport_command(self, api_id: int) -> bool:
        """Send a parameterless SPORT_MOD command by api_id (Hello, Damp, ...)."""
        if self.connection is None:
            return self._send(str(api_id))
        return self.connection.sport_command(api_id)

    @rpc
    def set_light(self, level: int) -> bool:
        """Head-LED brightness level 0-10 (0 = off)."""
        if self.connection is None:
            return self._send(f"brightness {level}")
        return self.connection.set_light(level)

    @rpc
    def set_obstacle_avoidance(self, enabled: bool = True) -> bool:
        """Toggle the onboard obstacle avoidance."""
        if self.connection is None:
            return self._send(f"obstacle-avoidance {'on' if enabled else 'off'}")
        return self.connection.set_obstacle_avoidance(enabled)

    @rpc
    def switch_joystick(self, enable: bool = True) -> bool:
        """Firmware joystick listening on/off (WASD stick emulation needs it on)."""
        if self.connection is None:
            return self._send(f"joystick {'on' if enable else 'off'}")
        return self.connection.switch_joystick(enable)

    @rpc
    def stop_movement(self) -> None:
        """Zero the base immediately (webrtc deadman stop)."""
        if self.connection is None:
            self._send("stop")
            return
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
        if self.connection is None:
            battery = self._latest_battery
            return None if battery is None else round(battery.percentage * 100)
        lowstate = self._latest_lowstate
        try:
            return int(lowstate["data"]["bms_state"]["soc"])  # type: ignore[index]
        except (KeyError, TypeError, ValueError):
            return None

    @rpc
    def publish_request(self, topic: str, data: dict[str, Any]) -> dict[Any, Any] | None:
        """Publish a request to the WebRTC connection.
        Args:
            topic: The RTC topic to publish to
            data: The data dictionary to publish
        Returns:
            The result of the publish request (on the real robot it's sent without waiting: None)
        """
        if self.connection is None:
            parameter = data.get("parameter")
            verb = f"request {topic} {data['api_id']}"
            if parameter is not None:
                verb += f" {json.dumps(parameter)}"
            # TODO: return the robot's answer once Rust modules have RPCs.
            self._send(verb)
            return None
        return self.connection.publish_request(topic, data)

import threading
from threading import Thread
from typing import Any

from pydantic import Field
from reactivex.disposable import Disposable

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.drone.connection_module import DroneConnectionModule
from dimos.robot.unitree.mujoco_connection import MujocoConnection
from dimos.robot.unitree.type.odometry import Odometry as SimOdometry
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class DroneSimConfig(ModuleConfig):
    ip: str = Field(default_factory=lambda m: m["g"].robot_ip)


class DroneSimConnection(DroneConnectionModule):
    config: DroneSimConfig
    cmd_vel: In[Twist]
    lidar: Out[PointCloud2]
    odom: Out[PoseStamped]
    color_image: Out[Image]
    camera_info: Out[CameraInfo]
    connection: MujocoConnection | None = None
    _camera_info_thread: Thread | None = None

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._stop_event = threading.Event()

    @rpc
    def start(self) -> None:
        Module.start(self)

        from dimos.robot.unitree.mujoco_connection import MujocoConnection

        self.connection = MujocoConnection(self.config.g)
        assert self.connection is not None
        self.connection.start()

        self.register_disposable(Disposable(self.cmd_vel.subscribe(self.move)))
        self.register_disposable(self.connection.odom_stream().subscribe(self._publish_sim_odom))
        self.register_disposable(self.connection.lidar_stream().subscribe(self.lidar.publish))
        self.register_disposable(self.connection.video_stream().subscribe(self.color_image.publish))

        self._camera_info_thread = Thread(
            target=self._publish_camera_info_loop,
            daemon=True,
        )
        self._camera_info_thread.start()

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        assert self.connection is not None
        self.connection.stop()
        if self._camera_info_thread and self._camera_info_thread.is_alive():
            self._camera_info_thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        Module.stop(self)

    def _publish_camera_info_loop(self) -> None:
        assert self.connection is not None
        info = self.connection.camera_info_static
        while not self._stop_event.is_set():
            self.camera_info.publish(info)
            self._stop_event.wait(1.0)

    def _publish_sim_odom(self, msg: SimOdometry) -> None:
        self._publish_tf(
            PoseStamped(
                ts=msg.ts,
                frame_id=msg.frame_id,
                position=msg.position,
                orientation=msg.orientation,
            )
        )

    @rpc
    def move(self, twist: Twist, duration: float = 0.0) -> None:
        assert self.connection is not None
        self.connection.move(twist, duration)

    @rpc
    def publish_request(self, topic: str, data: dict[str, Any]) -> dict[Any, Any]:
        logger.info(f"Publishing request to topic: {topic} with data: {data}")
        assert self.connection is not None
        return self.connection.publish_request(topic, data)

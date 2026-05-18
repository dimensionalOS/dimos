#!/usr/bin/env python3

from typing import Any

from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.robot.unitree.mujoco_connection import MujocoConnection
from dimos.robot.unitree.type.odometry import Odometry as SimOdometry
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class DroneSimConnection(Module):
    """Standalone sim connection"""

    cmd_vel: In[Twist]
    odom: Out[PoseStamped]
    connection: MujocoConnection | None = None

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)

    @rpc
    def start(self) -> None:
        super().start()
        self.connection = MujocoConnection(self.config.g)
        self.connection.start()
        self.register_disposable(Disposable(self.cmd_vel.subscribe(self.move)))
        self.register_disposable(self.connection.odom_stream().subscribe(self._on_odom))

    @rpc
    def stop(self) -> None:
        if self.connection:
            self.connection.stop()
        super().stop()

    def _on_odom(self, msg: SimOdometry) -> None:
        pose = PoseStamped(
            ts=msg.ts,
            frame_id=msg.frame_id,
            position=msg.position,
            orientation=msg.orientation,
        )
        self.odom.publish(pose)
        self.tf.publish(Transform.from_pose("base_link", pose))

    @rpc
    def move(self, twist: Twist, duration: float = 0.0) -> None:
        if self.connection:
            self.connection.move(twist, duration)

    @rpc
    def publish_request(self, topic: str, data: dict[str, Any]) -> dict[Any, Any]:
        if self.connection:
            return self.connection.publish_request(topic, data)
        return {}


__all__ = ["DroneSimConnection"]

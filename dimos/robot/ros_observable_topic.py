import functools
import reactivex as rx
from reactivex import operators as ops
from typing import Type
from reactivex.disposable import Disposable
from reactivex.observable import Observable

from nav_msgs import msg
from dimos.utils.logging_config import setup_logger

from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

sensor_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    durability=QoSDurabilityPolicy.VOLATILE,
    depth=1,
)

logger = setup_logger("dimos.robot.ros_control.observable_topic")


class ROSObservableTopicAbility:
    @functools.lru_cache(maxsize=None)
    def topic(
        self, topic_name: str, msg_type: msg, qos=sensor_qos, latest_only=True
    ) -> Observable:
        logger.info(f"Subscribing to {topic_name} with {sensor_qos} QoS")

        def _on_subscribe(observer, scheduler):
            ros_sub = self._node.create_subscription(
                msg_type, topic_name, observer.on_next, qos
            )

            return Disposable(
                lambda: (
                    self._node.destroy_subscription(ros_sub),
                    observer.on_completed(),
                )
            )

        observable = rx.create(_on_subscribe)

        # this will store all messages in memory
        # and wait for them to be processed, make sure you know what you are doing :)
        if not latest_only:
            return observable

        # hot observable with replay
        return observable.pipe(ops.replay(buffer_size=1), ops.ref_count())

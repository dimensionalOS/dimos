import functools
import reactivex as rx
from reactivex import operators as ops
from reactivex.disposable import Disposable
from reactivex.scheduler import ThreadPoolScheduler
from rxpy_backpressure import BackPressure

from nav_msgs import msg
from dimos.utils.logging_config import setup_logger
from dimos.utils.threadpool import get_scheduler


from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

# TODO: should go to some shared file, this is copy pasted from ros_control.py
sensor_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    durability=QoSDurabilityPolicy.VOLATILE,
    depth=1,
)

logger = setup_logger("dimos.robot.ros_control.observable_topic")

# ROS thread ─► ReplaySubject─► observe_on(pool) ─► backpressure.latest ─► sub1 (fast)
#                          ├──► observe_on(pool) ─► backpressure.latest ─► sub2 (slow)
#                          └──► observe_on(pool) ─► backpressure.latest ─► sub3 (slower)
class ROSObservableTopicAbility:
    @functools.lru_cache(maxsize=None)
    def topic(
        self,
        topic_name: str,
        msg_type: msg,
        qos=sensor_qos,
        scheduler: ThreadPoolScheduler | None = None,
        drop_unprocessed: bool = True,
    ) -> rx.Observable:
        if scheduler is None:
            scheduler = get_scheduler()

        # upstream ROS callback
        def _on_subscribe(obs, _):
            ros_sub = self._node.create_subscription(
                msg_type, topic_name, obs.on_next, qos
            )
            return Disposable(lambda: self._node.destroy_subscription(ros_sub))

        upstream = rx.create(_on_subscribe)

        # hot, latest‑cached core
        core = upstream.pipe(
            ops.replay(buffer_size=1),
            ops.ref_count(),  # still synchronous!
        )

        # per-subscriber factory
        def per_sub():
            # hop off the ROS thread into the pool
            base = core.pipe(ops.observe_on(scheduler))

            # optional back-pressure handling
            if drop_unprocessed:
                def _subscribe(observer, sch=None):
                    return base.subscribe(BackPressure.LATEST(observer), scheduler=sch)
                return rx.create(_subscribe)

            return base

        # each `.subscribe()` call gets its own async backpressure chain
        return rx.defer(lambda *_: per_sub())

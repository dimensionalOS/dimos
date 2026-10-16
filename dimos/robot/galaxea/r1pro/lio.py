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

"""Hang the R1's ``base_link`` off Point-LIO's lidar pose instead of wheel odometry."""

from __future__ import annotations

import contextlib
from functools import cache
import os

import psutil
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.hardware.sensors.lidar.pointlio.module import PointLio
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.protocol.tf.static_tf_publisher import StaticTfPublisher, StaticTfPublisherConfig
from dimos.robot.assets.model import LoadedRobotModel
from dimos.robot.galaxea.r1pro.config import R1PRO_MODEL


class R1ProLioConfig(StaticTfPublisherConfig):
    base_frame: str = "base_link"
    # Point-LIO's moving sensor frame (its sensor_frame_id).
    lidar_frame: str = "lidar_pointlio_link"
    # The URDF's name for the same lidar.
    chassis_lidar_frame: str = "lidar_chassis_left_link"
    # On when something else (dimSLAM) owns odom -> base_link: the mount then hangs under base_link instead of over it.
    mount_under_base: bool = False


@cache
def lio_model(lidar_frame: str, chassis_lidar_frame: str) -> LoadedRobotModel:
    """The R1 URDF with Point-LIO's frame added at the chassis lidar, loaded on first use so importing a blueprint needs no robot assets."""
    return R1PRO_MODEL.with_fixed_frame(lidar_frame, chassis_lidar_frame).load()


def joint_transform(model: LoadedRobotModel, child_link: str) -> Transform:
    """The fixed joint that places ``child_link`` under its parent."""
    joint = next(j for j in model.joints if j.child_link == child_link)
    return Transform(
        translation=Vector3(*joint.origin_xyz),
        rotation=Quaternion.from_euler(Vector3(*joint.origin_rpy)),
        frame_id=joint.parent_link,
        child_frame_id=joint.child_link,
    )


def mount_transforms(config: R1ProLioConfig) -> list[Transform]:
    """Point-LIO's frame to base_link, then the URDF's chassis-lidar edge; the first is lidar -> base unless ``mount_under_base``."""
    model = lio_model(config.lidar_frame, config.chassis_lidar_frame)
    base_to_chassis_lidar = joint_transform(model, config.chassis_lidar_frame)
    assert base_to_chassis_lidar.frame_id == config.base_frame
    base_to_lidar = base_to_chassis_lidar + joint_transform(model, config.lidar_frame)
    return [base_to_lidar if config.mount_under_base else -base_to_lidar, base_to_chassis_lidar]


def base_transform(odometry: Odometry, lidar_to_base: Transform) -> Transform:
    """Point-LIO's lidar pose carried through the mount to base_link."""
    odom_to_lidar = Transform(
        translation=odometry.pose.position,
        rotation=odometry.pose.orientation,
        frame_id=odometry.frame_id,
        ts=odometry.ts,
    )
    return odom_to_lidar + lidar_to_base


def base_pose(odometry: Odometry, lidar_to_base: Transform) -> PoseStamped:
    odom_to_base = base_transform(odometry, lidar_to_base)
    return PoseStamped(
        ts=odometry.ts,
        frame_id=odometry.frame_id,
        position=odom_to_base.translation,
        orientation=odom_to_base.rotation,
    )


def base_odometry(odometry: Odometry, lidar_to_base: Transform, base_frame: str) -> Odometry:
    """The same pose as an Odometry (no twist: Point-LIO's is the lidar's, not base_link's)."""
    odom_to_base = base_transform(odometry, lidar_to_base)
    return Odometry(
        ts=odometry.ts,
        frame_id=odometry.frame_id,
        child_frame_id=base_frame,
        pose=Pose(odom_to_base.translation, odom_to_base.rotation),
    )


class R1ProLio(StaticTfPublisher):
    """Hangs base_link under Point-LIO's lidar frame (or the reverse), and publishes base_link's pose from Point-LIO's odometry."""

    config: R1ProLioConfig

    odometry: In[Odometry]
    pose: Out[PoseStamped]
    # The same pose as an Odometry, for a fusion module such as dimSLAM.
    base_odometry: Out[Odometry]

    def transforms(self) -> list[Transform]:
        return mount_transforms(self.config)

    @rpc
    def start(self) -> None:
        super().start()
        base_to_lidar = mount_transforms(self.config.model_copy(update={"mount_under_base": True}))[
            0
        ]
        self._lidar_to_base = -base_to_lidar
        self.register_disposable(Disposable(self.odometry.subscribe(self._on_odometry)))

    def _on_odometry(self, msg: Odometry) -> None:
        self.pose.publish(base_pose(msg, self._lidar_to_base))
        self.base_odometry.publish(base_odometry(msg, self._lidar_to_base, self.config.base_frame))


class PoseFromOdometry(Module):
    """An Odometry's pose as a PoseStamped, for the planners that read one."""

    odometry: In[Odometry]
    pose: Out[PoseStamped]

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.odometry.subscribe(self._on_odometry)))

    def _on_odometry(self, msg: Odometry) -> None:
        self.pose.publish(
            PoseStamped(
                ts=msg.ts,
                frame_id=msg.frame_id,
                position=msg.pose.position,
                orientation=msg.pose.orientation,
            )
        )


# Sharing the Orin's cores at load ~23, Point-LIO waited for a core ~60% of the time, ran 0.4-2 s behind and dropped scans: the robot stuttered in the viewer.
POINTLIO_RESERVED_CORES = 2


def set_affinity(pid: int, cores: set[int]) -> None:
    """Every thread of ``pid``: an affinity is per thread, and threads made later inherit their creator's."""
    with contextlib.suppress(FileNotFoundError):
        for tid in os.listdir(f"/proc/{pid}/task"):
            with contextlib.suppress(OSError):
                os.sched_setaffinity(int(tid), cores)


def run_pids() -> list[int]:
    """This dimos run: the coordinator (this worker's forkserver's parent) and everything under it."""
    coordinator = psutil.Process(psutil.Process().parent().ppid())
    return [coordinator.pid, *(p.pid for p in coordinator.children(recursive=True))]


class R1ProPointLio(PointLio):
    """Point-LIO on the last cores, with the rest of the run moved off them."""

    @rpc
    def start(self) -> None:
        super().start()
        if not hasattr(os, "sched_setaffinity") or self._process is None:
            return
        cpus = set(range(os.cpu_count() or 1))
        reserved = set(sorted(cpus)[-POINTLIO_RESERVED_CORES:])
        if reserved == cpus:
            return
        for pid in run_pids():
            set_affinity(pid, reserved if pid == self._process.pid else cpus - reserved)

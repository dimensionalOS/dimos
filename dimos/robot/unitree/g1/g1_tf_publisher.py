# Copyright 2026 Dimensional Inc.
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

"""Live G1 sensor-mount tf: mounts from g1.urdf, the base_link edge from the waist joints.

The Mid-360 ships mounted upside down, so the torso edge composes the URDF
mount with a 180 degree roll. The rt/lowstate subscriber is read-only.
"""

from __future__ import annotations

import asyncio
import math
import threading
import time
from typing import Any

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import Transform, TransformStamped, Vector3
from dimos_generated.std_msgs.msg import Header
from dimos_generated.tf2_msgs.msg import TFMessage
from pydantic import Field

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.msgs.geometry import compose_transforms, inverse_transform, quaternion_from_euler
from dimos.msgs.time import time_from_nanoseconds
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

MID360_PITCH = 0.04014257279586953
D435_PITCH = 0.8307767239493009

# g1.urdf fixed sensor mount origins on torso_link.
_TORSO_MID360_XYZ = (0.0002835, 0.00003, 0.41618)
_TORSO_D435_XYZ = (0.0576235, 0.01753, 0.42987)

# waist_roll_joint origin, the only nonzero offset in the pelvis -> torso chain.
_WAIST_ROLL_ORIGIN = (-0.0039635, 0.0, 0.044)


def _mount(
    parent: str,
    child: str,
    xyz: tuple[float, float, float] = (0.0, 0.0, 0.0),
    roll: float = 0.0,
    pitch: float = 0.0,
    yaw: float = 0.0,
) -> TransformStamped:
    return TransformStamped(
        header=Header(frame_id=parent, stamp=Time(sec=0, nanosec=0)),
        child_frame_id=child,
        transform=Transform(
            translation=Vector3(x=xyz[0], y=xyz[1], z=xyz[2]),
            rotation=quaternion_from_euler(roll, pitch, yaw),
        ),
    )


def torso_to_mid360() -> TransformStamped:
    """URDF mount pitch composed after the sensor's upside-down roll."""
    return _mount("torso_link", "mid360_link", _TORSO_MID360_XYZ, roll=math.pi, pitch=MID360_PITCH)


def torso_to_d435() -> TransformStamped:
    """torso_link -> d435_link, the URDF mount."""
    return _mount("torso_link", "d435_link", _TORSO_D435_XYZ, pitch=D435_PITCH)


# rt/lowstate motor indices, ordering from make_humanoid_joints("g1").
_WAIST_YAW_IDX = 12
_WAIST_ROLL_IDX = 13
_WAIST_PITCH_IDX = 14


def base_to_torso(waist_yaw: float, waist_roll: float, waist_pitch: float) -> TransformStamped:
    """base_link -> torso_link through the g1.urdf waist chain."""
    yaw = _mount("base_link", "waist_yaw_link", yaw=waist_yaw)
    roll = _mount("waist_yaw_link", "waist_roll_link", _WAIST_ROLL_ORIGIN, roll=waist_roll)
    pitch = _mount("waist_roll_link", "torso_link", pitch=waist_pitch)
    return compose_transforms(compose_transforms(yaw, roll), pitch)


def mount_transforms(
    waist_yaw: float = 0.0, waist_roll: float = 0.0, waist_pitch: float = 0.0
) -> list[TransformStamped]:
    """The mount tree as published: rooted at mid360_link."""
    return [
        inverse_transform(torso_to_mid360()),
        inverse_transform(base_to_torso(waist_yaw, waist_roll, waist_pitch)),
        torso_to_d435(),
    ]


class G1TfPublisherConfig(ModuleConfig):
    network_interface: str = "eth0"
    publish_hz: float = Field(default=20.0, gt=0.0)


class G1TfPublisher(Module):
    """Publishes the G1 sensor mount tree onto tf, waist edge live from lowstate."""

    config: G1TfPublisherConfig

    tf: Out[TFMessage]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._subscriber: Any = None
        self._waist = (0.0, 0.0, 0.0)
        self._waist_lock = threading.Lock()
        self._waist_live = False
        self._stop_event = threading.Event()
        self._reader_thread: threading.Thread | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self._stop_event.clear()
        self._subscriber = self._init_lowstate_subscriber()
        if self._subscriber is not None:
            self._reader_thread = threading.Thread(
                target=self._reader_loop, name="g1-tf-lowstate", daemon=True
            )
            self._reader_thread.start()
        self.spawn(self._publish_loop())
        logger.info(
            "G1TfPublisher publishing at %.1f Hz (waist %s)",
            self.config.publish_hz,
            "live" if self._subscriber is not None else "rest pose",
        )

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._reader_thread is not None and self._reader_thread.is_alive():
            self._reader_thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        self._reader_thread = None
        if self._subscriber is not None:
            try:
                self._subscriber.Close()
            except (OSError, RuntimeError) as e:
                logger.warning(f"ChannelSubscriber Close raised: {e}")
        self._subscriber = None
        super().stop()

    def _init_lowstate_subscriber(self) -> Any:
        # Lazy SDK imports - file must import cleanly outside the [unitree-dds] extra.
        try:
            from unitree_sdk2py.core.channel import (  # type: ignore[import-not-found]
                ChannelFactoryInitialize,
                ChannelSubscriber,
            )
            from unitree_sdk2py.idl.unitree_hg.msg.dds_ import (  # type: ignore[import-not-found]
                LowState_,
            )
        except ImportError:
            logger.warning("unitree_sdk2py unavailable - publishing rest-pose waist only")
            return None
        try:
            if self.config.network_interface:
                ChannelFactoryInitialize(0, self.config.network_interface)
            else:
                ChannelFactoryInitialize(0)
        except Exception as e:
            logger.warning(
                f"ChannelFactoryInitialize failed - publishing rest-pose waist only: {e}"
            )
            return None
        subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        subscriber.Init(None, 0)
        return subscriber

    def _reader_loop(self) -> None:
        period = 1.0 / self.config.publish_hz
        while not self._stop_event.is_set():
            sample = self._subscriber.Read(period)
            if sample is not None:
                waist = (
                    float(sample.motor_state[_WAIST_YAW_IDX].q),
                    float(sample.motor_state[_WAIST_ROLL_IDX].q),
                    float(sample.motor_state[_WAIST_PITCH_IDX].q),
                )
                with self._waist_lock:
                    self._waist = waist
                if not self._waist_live:
                    self._waist_live = True
                    logger.info("First LowState received - waist edge is live")
            self._stop_event.wait(period)

    async def _publish_loop(self) -> None:
        period = 1.0 / self.config.publish_hz
        while not self._stop_event.is_set():
            with self._waist_lock:
                waist_yaw, waist_roll, waist_pitch = self._waist
            transforms = mount_transforms(waist_yaw, waist_roll, waist_pitch)
            now = time.time_ns()
            for transform in transforms:
                transform.header.stamp = time_from_nanoseconds(now)
            self.tf.publish(TFMessage(transforms=transforms))
            await asyncio.sleep(period)

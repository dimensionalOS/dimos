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

from pydantic import Field

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

MID360_PITCH = 0.04014257279586953
D435_PITCH = 0.8307767239493009

# g1.urdf fixed sensor mount origins on torso_link.
_TORSO_MID360_XYZ = (0.0002835, 0.00003, 0.41618)
_TORSO_D435_XYZ = (0.0576235, 0.01753, 0.42987)

# waist_roll_joint origin, the only nonzero offset in the pelvis -> torso chain.
_WAIST_ROLL_ORIGIN = (-0.0039635, 0.0, 0.044)


def torso_to_mid360() -> Transform:
    """torso_link -> the Mid-360's own frame: URDF mount pitch plus the upside-down roll."""
    return Transform(
        translation=Vector3(*_TORSO_MID360_XYZ),
        rotation=Quaternion.from_euler(Vector3(0.0, MID360_PITCH, 0.0))
        * Quaternion.from_euler(Vector3(math.pi, 0.0, 0.0)),
        frame_id="torso_link",
        child_frame_id="mid360_link",
    )


def torso_to_d435() -> Transform:
    """torso_link -> d435_link, the URDF mount."""
    return Transform(
        translation=Vector3(*_TORSO_D435_XYZ),
        rotation=Quaternion.from_euler(Vector3(0.0, D435_PITCH, 0.0)),
        frame_id="torso_link",
        child_frame_id="d435_link",
    )


# rt/lowstate motor indices, ordering from make_humanoid_joints("g1").
_WAIST_YAW_IDX = 12
_WAIST_ROLL_IDX = 13
_WAIST_PITCH_IDX = 14


def base_to_torso(waist_yaw: float, waist_roll: float, waist_pitch: float) -> Transform:
    """base_link -> torso_link through the g1.urdf waist chain."""
    yaw = Transform(
        rotation=Quaternion.from_euler(Vector3(0.0, 0.0, waist_yaw)),
        frame_id="base_link",
        child_frame_id="waist_yaw_link",
    )
    roll = Transform(
        translation=Vector3(*_WAIST_ROLL_ORIGIN),
        rotation=Quaternion.from_euler(Vector3(waist_roll, 0.0, 0.0)),
        frame_id="waist_yaw_link",
        child_frame_id="waist_roll_link",
    )
    pitch = Transform(
        rotation=Quaternion.from_euler(Vector3(0.0, waist_pitch, 0.0)),
        frame_id="waist_roll_link",
        child_frame_id="torso_link",
    )
    return yaw + roll + pitch


def mount_transforms(
    waist_yaw: float = 0.0, waist_roll: float = 0.0, waist_pitch: float = 0.0
) -> list[Transform]:
    """The mount tree as published: rooted at mid360_link."""
    return [
        -torso_to_mid360(),
        -base_to_torso(waist_yaw, waist_roll, waist_pitch),
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
            now = time.time()
            for transform in transforms:
                transform.ts = now
            self.tf.publish(TFMessage(*transforms))
            await asyncio.sleep(period)

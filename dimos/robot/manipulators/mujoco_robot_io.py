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

"""Allowed camera/calibration/robot-relative TF into normal robot-side DimOS ports."""

from __future__ import annotations

import base64
from typing import Any

import zenoh

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.protocol.mujoco_eval import MAX_SENSOR_BYTES, Sensor, key, session_config


class MujocoRobotIOConfig(ModuleConfig):
    endpoint: str
    run: str
    episode: str


class MujocoRobotIO(Module):
    config: MujocoRobotIOConfig
    color_image: Out[Image]
    depth_image: Out[Image]
    camera_info: Out[CameraInfo]
    depth_camera_info: Out[CameraInfo]
    tf: Out[TFMessage]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._session: zenoh.Session | None = None
        self._subscriber: zenoh.Subscriber[None] | None = None
        self._sequence = -1
        self._info: CameraInfo | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self._session = zenoh.open(
            session_config(
                self.config.endpoint,
                trusted=False,
                run=self.config.run,
                episode=self.config.episode,
            )
        )
        self._subscriber = self._session.declare_subscriber(
            key(self.config.run, self.config.episode, "sensor"), self._receive
        )

    def _receive(self, sample: zenoh.Sample) -> None:
        raw = sample.payload.to_bytes()
        if len(raw) > MAX_SENSOR_BYTES:
            return
        try:
            packet = Sensor.model_validate_json(raw)
            if (packet.run, packet.episode) != (self.config.run, self.config.episode):
                return
            if packet.sequence <= self._sequence:
                return
            data = base64.b64decode(packet.data, validate=True)
            if packet.kind in ("color_image", "depth_image"):
                image = Image.lcm_decode(data)
                getattr(self, packet.kind).publish(image)
            elif packet.kind == "camera_info":
                self._info = CameraInfo.lcm_decode(data)
                self.camera_info.publish(self._info)
                self.depth_camera_info.publish(self._info)
            else:
                transforms = TFMessage.lcm_decode(data)
                if any(t.frame_id == "world" for t in transforms.transforms):
                    return
                self.tf.publish(transforms)
            self._sequence = packet.sequence
        except (ValueError, RuntimeError):
            return

    @rpc
    def get_color_camera_info(self) -> CameraInfo | None:
        return self._info

    @rpc
    def get_depth_camera_info(self) -> CameraInfo | None:
        return self._info

    @rpc
    def get_depth_scale(self) -> float:
        return 1.0

    @rpc
    def stop(self) -> None:
        if self._session is not None:
            self._session.close()
        self._session = None
        self._subscriber = None
        super().stop()

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

"""Publish the G1 pelvis -> head-camera transform from live joint state.

The D435 hangs off ``torso_link``, so the pelvis->camera transform passes
through waist yaw/roll/pitch — joints the GR00T policy moves to balance. A
fixed transform is only the rest-pose approximation, and at plant range a few
degrees of waist is centimetres of error in the latched pot pose.

Republished continuously rather than latched once: marker detection resolves
the camera pose at each image timestamp within a 0.5 s tolerance, so a
one-shot static transform falls out of tolerance and frames are silently
dropped.
"""

from __future__ import annotations

from pathlib import Path
import threading
import time
from typing import Any

import numpy as np
from pydantic import Field

from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_URDF_PATH = Path(__file__).resolve().parent / "g1.urdf"
# Coordinator joint names are `g1/waist_yaw`; the URDF calls them `waist_yaw_joint`.
_WAIST_JOINTS = ("waist_yaw", "waist_roll", "waist_pitch")


class G1HeadCameraTfConfig(ModuleConfig):
    base_frame: str = "pelvis"
    camera_frame: str = "d435_link"
    publish_hz: float = Field(10.0, gt=0.0)


class G1HeadCameraTf(Module):
    """Publish ``pelvis -> d435_link``, recomputed as the waist moves."""

    config: G1HeadCameraTfConfig

    coordinator_joint_state: In[JointState]
    tf: Out[TFMessage]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._urdf: Any = None
        self._waist = dict.fromkeys(_WAIST_JOINTS, 0.0)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        super().start()
        self.coordinator_joint_state.subscribe(self._on_joint_state)
        self._thread = threading.Thread(target=self._publish_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        super().stop()

    def _on_joint_state(self, msg: JointState) -> None:
        with self._lock:
            for name, position in zip(msg.name, msg.position, strict=False):
                short = name.split("/")[-1]
                if short in self._waist:
                    self._waist[short] = float(position)

    def _transform(self) -> Transform:
        import yourdfpy  # type: ignore[import-untyped]

        if self._urdf is None:
            self._urdf = yourdfpy.URDF.load(str(_URDF_PATH), load_meshes=False)

        with self._lock:
            waist = dict(self._waist)

        cfg = np.zeros(len(self._urdf.actuated_joint_names))
        for i, name in enumerate(self._urdf.actuated_joint_names):
            cfg[i] = waist.get(name.removesuffix("_joint"), 0.0)
        self._urdf.update_cfg(cfg)

        t = self._urdf.get_transform(self.config.camera_frame, self.config.base_frame)
        q = Quaternion.from_rotation_matrix(t[:3, :3])
        return Transform(
            translation=Vector3(*t[:3, 3]),
            rotation=q,
            frame_id=self.config.base_frame,
            child_frame_id=self.config.camera_frame,
            ts=time.time(),
        )

    def _publish_loop(self) -> None:
        period = 1.0 / self.config.publish_hz
        while not self._stop.wait(period):
            try:
                self.tf.publish(TFMessage(self._transform()))
            except Exception:
                logger.exception("G1HeadCameraTf: failed to publish camera transform")
                return

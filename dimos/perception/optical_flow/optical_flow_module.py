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

import logging
from typing import Any

import cv2
import numpy as np
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.std_msgs.Bool import Bool
from dimos.perception.optical_flow.backends.lucas_kanade import LucasKanadeBackend
from dimos.utils.logging_config import setup_logger

logger = setup_logger(level=logging.INFO)


class OpticalFlowConfig(ModuleConfig):
    tau_threshold: float = (
        2.0  # Time-to-Contact limit (in frames); alarm fires when τ drops below this
    )
    omega_max: float = 0.3  # rad/s; danger gated above this yaw rate


class OpticalFlowModule(Module):
    """
    Monocular obstacle avoidance via Lucas-Kanade optical flow on a uniform
    grid + per-cell time-to-contact via flow divergence.
    Alarm fires when a spatially-coherent blob of low-τ cells exceeds a
    minimum area (connected-components on the thresholded divergence map),
    not on a per-pixel statistic — real obstacles produce coherent blobs.

    Danger is suppressed when |ω| exceeds omega_max (rotation breaks the
    fronto-parallel divergence assumption). The angular_velocity stream is
    optional; production stacks would feed an IMU here.
    """

    config: OpticalFlowConfig

    color_image: In[Image]
    angular_velocity: In[Any]  # optional: yaw rate in rad/s from IMU

    danger_signal: Out[Bool]
    flow_data: Out[Any]  # (N, 5) float32: columns (x, y, tau, u, v)
    flow_visualization: Out[Image]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._backend = LucasKanadeBackend(tau_threshold=self.config.tau_threshold)
        self._last_omega: float = 0.0

    @rpc
    def start(self) -> None:
        super().start()
        unsub_img = self.color_image.subscribe(self._on_color_image)
        unsub_omega = self.angular_velocity.subscribe(self._on_angular_velocity)
        self.register_disposable(Disposable(unsub_img))
        self.register_disposable(Disposable(unsub_omega))
        logger.info("OpticalFlowModule started")

    @rpc
    def stop(self) -> None:
        super().stop()

    def _on_angular_velocity(self, msg: Any) -> None:
        try:
            self._last_omega = float(msg.data) if hasattr(msg, "data") else float(msg)
        except (TypeError, ValueError):
            pass

    def _on_color_image(self, msg: Image) -> None:
        frame = msg.data
        if not isinstance(frame, np.ndarray):
            frame = np.asarray(frame)

        if msg.format == ImageFormat.RGB:
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        result = self._backend.compute(frame)
        if result is None:
            return

        # Suppress danger during turns
        is_turning = abs(self._last_omega) > self.config.omega_max
        gated_danger = bool(result["danger"]) and not is_turning

        self.danger_signal.publish(Bool(data=gated_danger))
        self.flow_data.publish(result["flow_data"])

        viz = self._draw_visualization(frame, result, is_turning=is_turning)
        self.flow_visualization.publish(Image.from_numpy(viz, format=ImageFormat.BGR))

    def _draw_visualization(
        self,
        frame_bgr: np.ndarray,
        result: dict,
        is_turning: bool = False,
    ) -> np.ndarray:
        """Per-point flow arrows colored by tau band + status banner."""
        viz = frame_bgr.copy()
        thr = self.config.tau_threshold

        red = (0, 0, 255)
        yellow = (0, 220, 220)
        green = (0, 200, 0)

        for x, y, tau, u, v in result["flow_data"]:
            if tau < thr:
                color = red
            elif tau < 2.0 * thr:
                color = yellow
            else:
                # Includes NaN (non-expanding points): not a threat.
                color = green
            p0 = (int(x), int(y))
            p1 = (int(x + u), int(y + v))
            cv2.arrowedLine(viz, p0, p1, color, 1, tipLength=0.4)

        if is_turning:
            label = f"GATED (|ω|={abs(self._last_omega):.2f} rad/s)"
            cv2.putText(viz, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 255), 2)
        else:
            label = "DANGER" if result["danger"] else "CLEAR"
            color = red if result["danger"] else (0, 255, 0)
            cv2.putText(viz, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2)

        return viz

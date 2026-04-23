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
    tau_threshold: float = 3.0   # alarm when min_tau < tau_threshold
    grid_size:     int   = 5     # NxN tac grid
    omega_max:     float = 0.3   # rad/s; danger suppressed above this yaw rate


class OpticalFlowModule(Module):
    """
    Monocular obstacle avoidance via sparse optical flow + Time-to-Contact.

    Subscribes to a live RGB/BGR camera stream, detects FAST keypoints each
    frame, tracks them with Lucas-Kanade, and flags grid cells where the
    inverse of flow divergence (tau proxy) falls below tau_threshold.

    Note on units: the backend's tau is monotonically related to physical
    time-to-contact but is not literally seconds — it depends on frame rate
    and cell geometry. Treat tau_threshold as an empirically tuned urgency
    parameter, not a calibrated timer.

    The danger_signal is gated on angular rate: if the optional
    angular_velocity stream is connected, danger output is suppressed
    whenever |omega| > omega_max (rad/s) to prevent false alarms during turns.
    """

    config: OpticalFlowConfig

    color_image: In[Image]
    angular_velocity: In[Any]   # optional: yaw rate in rad/s from IMU or odom

    danger_signal: Out[Bool]
    tac_grid: Out[Any]
    flow_visualization: Out[Image]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)

        self._backend = LucasKanadeBackend(
            grid_size=self.config.grid_size,
            tau_threshold=self.config.tau_threshold,
        )
        self._last_omega: float = 0.0

    @rpc
    def start(self) -> None:
        super().start()
        unsub_img   = self.color_image.subscribe(self._on_color_image)
        unsub_omega = self.angular_velocity.subscribe(self._on_angular_velocity)
        self.register_disposable(Disposable(unsub_img))
        self.register_disposable(Disposable(unsub_omega))
        logger.info("OpticalFlowModule started")

    def _on_angular_velocity(self, msg: Any) -> None:
        """Update last known yaw rate. Accepts raw float or .data-wrapped type."""
        try:
            self._last_omega = float(msg.data) if hasattr(msg, "data") else float(msg)
        except (TypeError, ValueError):
            pass

    @rpc
    def stop(self) -> None:
        super().stop()

    def _on_color_image(self, msg: Image) -> None:
        frame = msg.data
        if not isinstance(frame, np.ndarray):
            frame = np.asarray(frame)

        # Backends expect BGR; convert if the incoming image is RGB
        if msg.format == ImageFormat.RGB:
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        result = self._backend.compute(frame)
        if result is None:
            return

        # Suppress danger during turns: rotation contaminates the divergence
        # estimate, producing false alarms. Requires angular_velocity stream.
        is_turning = abs(self._last_omega) > self.config.omega_max
        gated_danger = bool(result["danger"]) and not is_turning

        self.danger_signal.publish(Bool(data=gated_danger))
        self.tac_grid.publish(result["tac_grid"])

        viz = self._draw_visualization(frame, result, is_turning=is_turning)
        self.flow_visualization.publish(Image.from_numpy(viz, format=ImageFormat.BGR))

    def _draw_visualization(
        self, frame_bgr: np.ndarray, result: dict,  # type: ignore[type-arg]
        is_turning: bool = False,
    ) -> np.ndarray:
        """Draw flow vectors and tau-grid overlay onto a copy of the frame."""
        viz = frame_bgr.copy()
        h, w = viz.shape[:2]

        good_prev, good_curr = result["flow_pts"]
        for p, c in zip(good_prev, good_curr):
            p0 = (int(p[0]), int(p[1]))
            p1 = (int(c[0]), int(c[1]))
            cv2.arrowedLine(viz, p0, p1, (0, 255, 0), 1, tipLength=0.4)

        tac_grid = result["tac_grid"]
        cell_h = h / self.config.grid_size
        cell_w = w / self.config.grid_size
        for i in range(self.config.grid_size):
            for j in range(self.config.grid_size):
                tau = tac_grid[i, j]
                if np.isnan(tau):
                    continue
                danger = tau < self.config.tau_threshold
                color = (0, 0, 255) if danger else (0, 200, 200)
                x0 = int(j * cell_w)
                y0 = int(i * cell_h)
                x1 = int((j + 1) * cell_w)
                y1 = int((i + 1) * cell_h)
                cv2.rectangle(viz, (x0, y0), (x1, y1), color, 2)
                cv2.putText(
                    viz, f"{tau:.1f}s",
                    (x0 + 4, y0 + 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1,
                )

        if is_turning:
            label = f"GATED (|ω|={abs(self._last_omega):.2f} rad/s)"
            cv2.putText(viz, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 255), 2)
        else:
            label = "DANGER" if result["danger"] else "CLEAR"
            color = (0, 0, 255) if result["danger"] else (0, 255, 0)
            cv2.putText(viz, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2)

        cv2.putText(
            viz, f"min_tau={result['min_tau']:.2f}s",
            (10, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1,
        )

        return viz

# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""Native DimOS module for querying an OpenPI websocket policy server."""

from __future__ import annotations

import os
import threading
import time
from typing import Any

import numpy as np
from pydantic import Field
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.models.openpi.connection import OpenPIConnection
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.utils.logging_config import setup_logger

try:
    from openpi_client import image_tools  # type: ignore[import-untyped]
except ImportError:
    image_tools = None

logger = setup_logger()


class OpenPIClientConfig(ModuleConfig):
    """Connection and policy settings for :class:`OpenPIClient`."""

    host: str = "127.0.0.1"
    port: int = 8000
    prompt: str = "pick up the red cube"
    timeout_s: float = Field(default=30.0, gt=0)
    max_observation_age_s: float = Field(default=3.0, gt=0)


class OpenPIClient(Module):
    """Query an OpenPI websocket server from typed DimOS streams.

    Inputs use the YAM contract: top, left-wrist and right-wrist RGB images plus
    a 14-element absolute joint state in packed left-then-right order. The
    output is a :class:`JointTrajectory` containing the server's action chunk;
    gripper slots remain normalized (0 closed, 1 open).
    """

    top_image: In[Image]
    left_wrist_image: In[Image]
    right_wrist_image: In[Image]
    coordinator_joint_state: In[JointState]
    action_chunk: Out[JointTrajectory]
    arm_command: Out[JointState]
    left_gripper_command: Out[Float32]
    right_gripper_command: Out[Float32]

    config: OpenPIClientConfig

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._client: Any = None
        self._lock = threading.Lock()
        self._wake = threading.Event()
        self._stop = threading.Event()
        self._pause_requested = threading.Event()
        self._paused = threading.Event()
        self._worker: threading.Thread | None = None
        self._latest: dict[str, Any] = {}
        self._received_at: dict[str, float] = {}

    @rpc
    def start(self) -> None:
        super().start()
        if os.environ.get("DIMOS_OPENPI_START_PAUSED") == "1":
            self._pause_requested.set()
        self._client = self._make_client()
        self.register_disposable(
            Disposable(self.top_image.subscribe(lambda value: self._set("top", value)))
        )
        self.register_disposable(
            Disposable(self.left_wrist_image.subscribe(lambda value: self._set("left", value)))
        )
        self.register_disposable(
            Disposable(self.right_wrist_image.subscribe(lambda value: self._set("right", value)))
        )
        self.register_disposable(
            Disposable(
                self.coordinator_joint_state.subscribe(lambda value: self._set("state", value))
            )
        )
        self._worker = threading.Thread(target=self._run, name="openpi-client", daemon=True)
        self._worker.start()

    @rpc
    def stop(self) -> None:
        self._stop.set()
        self._pause_requested.clear()
        self._wake.set()
        if self._client is not None:
            self._client.close()
        if self._worker is not None:
            self._worker.join(timeout=5.0)
        self._client = None
        super().stop()

    @rpc
    def pause(self) -> bool:
        """Request a pause after the current action chunk completes."""
        if self._worker is None or not self._worker.is_alive() or self._stop.is_set():
            return False
        self._pause_requested.set()
        self._wake.set()
        return True

    @rpc
    def resume(self) -> bool:
        """Allow inference again after an operator hold has been released."""
        if self._worker is None or not self._worker.is_alive() or self._stop.is_set():
            return False
        self._pause_requested.clear()
        self._paused.clear()
        self._wake.set()
        return True

    @rpc
    def is_paused(self) -> bool:
        return self._paused.is_set()

    @rpc
    def set_prompt(self, prompt: str) -> str:
        """Set the natural-language instruction sent with future observations."""
        if not prompt.strip():
            raise ValueError("prompt must not be empty")
        self.config.prompt = prompt
        return prompt

    def _make_client(self) -> Any:
        return OpenPIConnection(self.config.host, self.config.port, self.config.timeout_s)

    def _set(self, key: str, value: Any) -> None:
        with self._lock:
            self._latest[key] = value
            self._received_at[key] = time.monotonic()
        self._wake.set()

    def _run(self) -> None:
        while not self._stop.is_set():
            if self._pause_requested.is_set():
                self._paused.set()
                self._wake.wait(0.1)
                self._wake.clear()
                continue
            self._paused.clear()
            self._wake.wait(0.1)
            self._wake.clear()
            if self._stop.is_set():
                break
            if self._pause_requested.is_set():
                continue
            with self._lock:
                values = dict(self._latest)
                received_at = dict(self._received_at)
            if set(values) != {"top", "left", "right", "state"}:
                continue
            if any(
                time.monotonic() - stamp > self.config.max_observation_age_s
                for stamp in received_at.values()
            ):
                continue
            try:
                logger.info("OpenPI querying policy", keys=sorted(values))
                trajectory = self._infer(values)
            except Exception:
                logger.exception("OpenPI inference failed; stopping policy execution")
                self._stop.set()
                break
            if any(
                time.monotonic() - stamp > self.config.max_observation_age_s
                for stamp in received_at.values()
            ):
                logger.warning("Discarding policy actions from stale observations")
                continue
            logger.info("OpenPI action chunk received", points=trajectory.num_points)
            try:
                self._publish_commands(trajectory)
                self.action_chunk.publish(trajectory)
            except Exception:
                logger.exception("OpenPI command publication failed")
                self._stop.set()
                break
            logger.info("OpenPI action chunk published", points=trajectory.num_points)
        self._paused.clear()

    def _infer(self, values: dict[str, Any]) -> JointTrajectory:
        names = [
            name
            for side in ("left", "right")
            for name in ([f"{side}_joint{i}" for i in range(1, 7)] + [f"{side}_arm/gripper"])
        ]
        source = values["state"]
        if len(source.name) != len(source.position) or len(set(source.name)) != len(source.name):
            raise ValueError("Joint state must have unique names matching its positions")
        by_name = dict(zip(source.name, source.position, strict=True))
        state = np.asarray([by_name[name] for name in names], dtype=np.float32)
        if not np.isfinite(state).all():
            raise ValueError("Joint state contains non-finite positions")
        if state.shape != (14,):
            raise ValueError(f"YAM joint state must contain 14 positions, got {state.shape}")

        def chw_rgb(image: Image) -> np.ndarray:
            rgb = image.to_rgb().data
            if rgb.ndim != 3 or rgb.shape[2] != 3:
                raise ValueError(f"OpenPI expects an RGB image, got shape {rgb.shape}")
            if image_tools is None:
                raise RuntimeError("Install openpi-client in the DimOS environment")
            resized = image_tools.resize_with_pad(np.asarray(rgb, dtype=np.uint8), 224, 224)
            return np.asarray(resized, dtype=np.uint8).transpose(2, 0, 1)

        observation = {
            "images": {
                "cam_high": chw_rgb(values["top"]),
                "cam_left_wrist": chw_rgb(values["left"]),
                "cam_right_wrist": chw_rgb(values["right"]),
            },
            "state": state,
            "prompt": self.config.prompt,
        }
        logger.info("OpenPI websocket infer start")
        result = self._client.infer(observation)
        logger.info("OpenPI websocket infer returned")
        actions = np.asarray(result["actions"], dtype=np.float64)
        if actions.shape != (16, 14) or not np.isfinite(actions).all():
            raise ValueError(f"OpenPI returned invalid action chunk shape {actions.shape}")
        points = [
            TrajectoryPoint(time_from_start=index / 30.0, positions=row.tolist())
            for index, row in enumerate(actions)
        ]
        return JointTrajectory(
            points=points,
            joint_names=names,
        )

    def _publish_commands(self, trajectory: JointTrajectory) -> None:
        """Stream the chunk at 30 Hz in the coordinator's command layout."""
        for point in trajectory.points:
            if self._stop.is_set():
                return
            values = point.positions
            if point is trajectory.points[0]:
                logger.info("OpenPI publishing first command")
            self.arm_command.publish(
                JointState(
                    name=[f"{side}_joint{i}" for side in ("left", "right") for i in range(1, 7)],
                    position=[*values[:6], *values[7:13]],
                )
            )
            self.left_gripper_command.publish(Float32(float(np.clip(values[6], 0.0, 1.0))))
            self.right_gripper_command.publish(Float32(float(np.clip(values[13], 0.0, 1.0))))
            if self._stop.wait(1.0 / 30.0):
                return


openpi_policy_client = OpenPIClient.blueprint

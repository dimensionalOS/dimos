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

from __future__ import annotations

import threading
from typing import Any

from pydantic import Field

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.memory.observationstore.memory import ListObservationStore
from dimos.memory.store.memory import MemoryStore
from dimos.memory.stream import Stream
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.perception.experimental.object_scene_registration_spec import (
    ObjectSceneRegistrationSpec,
)
from dimos.perception.memory.dandetect import DanDetector
from dimos.perception.memory.types import Localization, LocalizePolicy


class ObjectSceneRegistrationConfig(ModuleConfig):
    target_frame: str = "world"
    optical_frame: str = "camera_color_optical_frame"
    memory_window_seconds: float = Field(default=15.0, gt=0.0)
    max_image_observations: int = Field(default=90, ge=1)
    max_camera_info_observations: int = Field(default=10, ge=1)
    max_tf_observations: int = Field(default=5000, ge=1)
    candidate_floor: float = Field(default=0.25, ge=0.0, le=1.0)
    accept_score: float = Field(default=0.4, ge=0.0, le=1.0)
    refusal_margin: float = Field(default=0.15, ge=0.0, le=1.0)
    min_views: int = Field(default=2, ge=1)
    max_prompts_per_request: int = Field(default=12, ge=1)
    max_result_age_seconds: float = Field(default=5.0, gt=0.0)
    tf_tolerance_seconds: float = Field(default=0.12, ge=0.0)


class ObjectSceneRegistrationModule(Module, ObjectSceneRegistrationSpec):
    """Localize prompted objects from a bounded, request-time scene history."""

    color_image: In[Image]
    depth_image: In[Image]
    camera_info: In[CameraInfo]
    tf: In[TFMessage]

    config: ObjectSceneRegistrationConfig

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._store: MemoryStore | None = None
        self._detector: DanDetector | None = None
        self._index: Stream[Any, Any] | None = None
        self._state_lock = threading.RLock()
        self._inference_lock = threading.Lock()

    @staticmethod
    def _bounded_store(name: str, max_size: int) -> ListObservationStore[Any]:
        return ListObservationStore(name=name, max_size=max_size)

    def _create_streams(self, store: MemoryStore) -> None:
        store.stream("color_image", Image)
        store.stream("depth_image", Image)
        store.stream(
            "camera_info",
            CameraInfo,
            observation_store=self._bounded_store(
                "camera_info", self.config.max_camera_info_observations
            ),
        )
        store.stream(
            "tf",
            TFMessage,
            observation_store=self._bounded_store("tf", self.config.max_tf_observations),
        )

    def _record_color_image(self, message: Image) -> None:
        with self._state_lock:
            store = self._store
        if store is not None:
            store.streams.color_image.append(message, ts=message.ts)

    def _record_depth_image(self, message: Image) -> None:
        with self._state_lock:
            store = self._store
        if store is not None:
            store.streams.depth_image.append(message, ts=message.ts)

    def _record_camera_info(self, message: CameraInfo) -> None:
        with self._state_lock:
            store = self._store
        if store is not None:
            store.streams.camera_info.append(message, ts=message.ts)

    def _record_tf(self, message: TFMessage) -> None:
        with self._state_lock:
            store = self._store
        if store is None:
            return

        # Keep each edge at its own timestamp so static mount links survive
        # timestamped graph lookups beside newer dynamic transforms.
        for transform in message.transforms:
            store.streams.tf.append(TFMessage(transform), ts=transform.ts)

    @rpc
    def start(self) -> None:
        if self.config.target_frame != "world":
            raise ValueError("ObjectSceneRegistrationModule requires target_frame='world'")

        detector = DanDetector()
        detector.start()
        store = MemoryStore(max_size=self.config.max_image_observations)
        self._create_streams(store)
        with self._state_lock:
            self._store = store
            self._detector = detector

        self.color_image.subscribe(self._record_color_image)
        self.depth_image.subscribe(self._record_depth_image)
        self.camera_info.subscribe(self._record_camera_info)
        self.tf.subscribe(self._record_tf)
        index = detector.embed(
            store,
            live=True,
            optical_frame=self.config.optical_frame,
            world_frame=self.config.target_frame,
            tf_tolerance=self.config.tf_tolerance_seconds,
        )
        with self._state_lock:
            self._index = index
        super().start()

    @rpc
    def stop(self) -> None:
        with self._inference_lock:
            with self._state_lock:
                detector = self._detector
                store = self._store
                self._detector = None
                self._store = None
                self._index = None
            if detector is not None:
                detector.stop()
            if store is not None:
                store.stop()
        super().stop()

    @rpc
    def localize_objects(self, prompts: list[str]) -> list[Localization | None]:
        config = self.config
        if not prompts or len(prompts) > config.max_prompts_per_request:
            raise ValueError(f"Expected 1-{config.max_prompts_per_request} object prompts")
        if any(not prompt or prompt.strip() != prompt for prompt in prompts):
            raise ValueError("Object prompts must be non-empty and trimmed")

        with self._inference_lock:
            with self._state_lock:
                store = self._store
                detector = self._detector
                index = self._index
            if store is None or detector is None or index is None:
                raise RuntimeError("ObjectSceneRegistrationModule is not running")
            if not store.streams.camera_info.exists():
                raise RuntimeError("Camera calibration is not available")
            if not store.streams.color_image.exists():
                raise RuntimeError("No RGB observations are available")

            latest = store.streams.color_image.last()
            snapshot = index.time_range(
                latest.ts - config.memory_window_seconds,
                latest.ts,
            ).materialize()
            if not snapshot.exists():
                raise RuntimeError("No indexed RGB observations are available")

            results = detector.localize(
                store,
                prompts,
                index=snapshot,
                require_pose=True,
                world_frame=config.target_frame,
                optical_frame=config.optical_frame,
                tf_tolerance=config.tf_tolerance_seconds,
                policy=LocalizePolicy(
                    candidate_floor=config.candidate_floor,
                    accept_score=config.accept_score,
                    refusal_margin=config.refusal_margin,
                    min_views=config.min_views,
                ),
            )
            if not isinstance(results, list) or len(results) != len(prompts):
                raise RuntimeError("Detector returned an invalid batch result")

            validated: list[Localization | None] = []
            for result in results:
                if result is None or result.reason:
                    validated.append(None)
                    continue
                if result.frame_id != config.target_frame:
                    raise RuntimeError(
                        f"Localization frame mismatch: expected {config.target_frame!r}, "
                        f"got {result.frame_id!r}"
                    )
                if result.point_cloud is None:
                    raise RuntimeError("Localization is missing its point cloud")
                if result.point_cloud.frame_id != config.target_frame:
                    raise RuntimeError(
                        "Localization point cloud frame mismatch: expected "
                        f"{config.target_frame!r}, got {result.point_cloud.frame_id!r}"
                    )
                if latest.ts - result.last_seen_timestamp > config.max_result_age_seconds:
                    validated.append(None)
                    continue
                validated.append(result)
            return validated

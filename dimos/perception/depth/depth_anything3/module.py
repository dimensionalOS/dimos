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

from enum import Enum
from typing import TYPE_CHECKING, Any

import numpy as np
from reactivex import operators as ops
from reactivex.observable import Observable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat, sharpness_barrier
from dimos.spec import perception
from dimos.utils.decorators.decorators import simple_mcache
from dimos.utils.reactive import backpressure

if TYPE_CHECKING:
    from dimos.core.module_coordinator import ModuleCoordinator


class DA3Model(str, Enum):
    SMALL = "depth-anything/DA3-SMALL"
    BASE = "depth-anything/DA3-BASE"
    LARGE = "depth-anything/DA3-LARGE-1.1"
    METRIC = "depth-anything/DA3METRIC-LARGE"
    GIANT = "depth-anything/DA3NESTED-GIANT-LARGE-1.1"


class DA3Mode(str, Enum):
    SINGLE = "single"
    TEMPORAL = "temporal"
    MULTI = "multi"


class Config(ModuleConfig):
    model: DA3Model = DA3Model.LARGE
    mode: DA3Mode = DA3Mode.SINGLE
    device: str = "cuda"
    max_freq: float = 5.0
    process_res: int = 504

    # temporal mode
    window_frames: int = 5
    motion_threshold: float = 5.0  # min mean pixel diff to accept a new keyframe

    # multi-camera mode
    num_cameras: int = 1

    # inference options
    use_ray_pose: bool = False
    ref_view_strategy: str = "saddle_balanced"

    # outputs
    publish_confidence: bool = False


class DepthAnything3Module(Module[Config]):
    default_config = Config

    color_image: In[Image]

    depth_image: Out[Image]
    confidence: Out[Image]

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)

        from depth_anything_3.api import DepthAnything3

        self._model = DepthAnything3.from_pretrained(self.config.model.value)
        self._model = self._model.to(device=self.config.device)

        # Create extra camera inputs/outputs for multi mode
        if self.config.mode == DA3Mode.MULTI:
            for i in range(1, self.config.num_cameras):
                setattr(self, f"color_image_{i}", In(Image, f"color_image_{i}", self))
                setattr(self, f"depth_image_{i}", Out(Image, f"depth_image_{i}", self))

    def _predict(self, images: list[Image]) -> list[tuple[np.ndarray, np.ndarray]]:
        """Run DA3 inference. Returns list of (depth, confidence) arrays."""
        np_images = [img.data for img in images]
        prediction = self._model.inference(
            image=np_images,
            use_ray_pose=self.config.use_ray_pose,
            ref_view_strategy=self.config.ref_view_strategy,
            process_res=self.config.process_res,
        )
        return list(zip(prediction.depth, prediction.conf, strict=False))

    def _publish_result(self, image: Image, depth: np.ndarray, conf: np.ndarray) -> None:
        """Publish depth (and optionally confidence) for a single frame."""
        self.depth_image.publish(
            Image(data=depth, format=ImageFormat.DEPTH, frame_id=image.frame_id, ts=image.ts)
        )
        if self.config.publish_confidence:
            self.confidence.publish(
                Image(data=conf, format=ImageFormat.DEPTH, frame_id=image.frame_id, ts=image.ts)
            )

    # -- Stream builders --

    @simple_mcache
    def _sharp_stream(self) -> Observable[Image]:
        return backpressure(
            self.color_image.pure_observable().pipe(
                sharpness_barrier(self.config.max_freq),
            )
        )

    @simple_mcache
    def _single_stream(self) -> Observable[tuple[Image, np.ndarray, np.ndarray]]:
        def process(img: Image) -> tuple[Image, np.ndarray, np.ndarray]:
            results = self._predict([img])
            depth, conf = results[0]
            return (img, depth, conf)

        return backpressure(self._sharp_stream().pipe(ops.map(process)))

    @simple_mcache
    def _temporal_stream(self) -> Observable[tuple[Image, np.ndarray, np.ndarray]]:
        window_size = self.config.window_frames
        threshold = self.config.motion_threshold

        def accumulate_keyframes(keyframes: list[Image], img: Image) -> list[Image]:
            if not keyframes:
                return [img]
            # Compare against last keyframe — mean absolute pixel difference
            diff = float(
                np.mean(np.abs(img.data.astype(np.float32) - keyframes[-1].data.astype(np.float32)))
            )
            if diff >= threshold:
                keyframes = [*keyframes[-(window_size - 1) :], img]
            return keyframes

        def process_window(keyframes: list[Image]) -> tuple[Image, np.ndarray, np.ndarray] | None:
            if len(keyframes) < 2:
                return None
            results = self._predict(keyframes)
            depth, conf = results[-1]
            return (keyframes[-1], depth, conf)

        return backpressure(
            self._sharp_stream().pipe(
                ops.scan(accumulate_keyframes, []),
                ops.map(process_window),
                ops.filter(lambda r: r is not None),
            )
        )

    @rpc
    def start(self) -> None:
        if self.config.mode == DA3Mode.SINGLE:
            self._single_stream().subscribe(lambda r: self._publish_result(r[0], r[1], r[2]))

        elif self.config.mode == DA3Mode.TEMPORAL:
            self._temporal_stream().subscribe(lambda r: self._publish_result(r[0], r[1], r[2]))

        elif self.config.mode == DA3Mode.MULTI:
            from dimos.types.timestamped import align_timestamped

            camera_streams = [self.color_image.pure_observable()]
            for i in range(1, self.config.num_cameras):
                port: In[Image] = getattr(self, f"color_image_{i}")
                camera_streams.append(port.pure_observable())

            aligned = align_timestamped(
                camera_streams[0],
                *camera_streams[1:],
                match_tolerance=0.1,
                buffer_size=2.0,
            )

            def process_multi(frames: tuple[Image, ...]) -> None:
                images = list(frames)
                results = self._predict(images)
                # Publish depth for camera 0
                depth, conf = results[0]
                self._publish_result(images[0], depth, conf)
                # Publish depth for remaining cameras
                for i in range(1, len(results)):
                    d, c = results[i]
                    out: Out[Image] = getattr(self, f"depth_image_{i}")
                    out.publish(
                        Image(
                            data=d,
                            format=ImageFormat.DEPTH,
                            frame_id=images[i].frame_id,
                            ts=images[i].ts,
                        )
                    )
                    if self.config.publish_confidence:
                        conf_out: Out[Image] = getattr(self, f"confidence_{i}", None)  # type: ignore[assignment]
                        if conf_out:
                            conf_out.publish(
                                Image(
                                    data=c,
                                    format=ImageFormat.DEPTH,
                                    frame_id=images[i].frame_id,
                                    ts=images[i].ts,
                                )
                            )

            backpressure(aligned).subscribe(process_multi)

    @rpc
    def stop(self) -> None:
        return super().stop()  # type: ignore[no-any-return]


def deploy(
    dimos: ModuleCoordinator,
    camera: perception.Image,
    prefix: str = "/depth_anything3",
    **kwargs: Any,
) -> DepthAnything3Module:
    from dimos.core.transport import LCMTransport

    module = DepthAnything3Module(**kwargs)
    module.color_image.connect(camera.color_image)

    module.depth_image.transport = LCMTransport(f"{prefix}/depth", Image)
    module.confidence.transport = LCMTransport(f"{prefix}/confidence", Image)

    module.start()
    return module

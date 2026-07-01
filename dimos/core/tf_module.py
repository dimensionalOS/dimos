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
import threading
import time
from typing import Any

from pydantic import Field

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.msgs.geometry_msgs.Transform import Transform


class TfModuleConfig(ModuleConfig):
    static_transforms: dict[str, Transform] = Field(default_factory=dict)
    # TODO: in the future we should make self.tf.publish error if it tried to publish a transform that references a frame that is not mentioned in this dict (same with self.tf.get)
    static_publish_interval: float = 1.0


class TfModule(Module):
    """A Module that republishes its config's static (non-moving) transforms on an interval.

    Modules that need to publish fixed frames inherit from this instead of Module
    directly, so the base Module stays free of transform-publishing machinery that
    most modules don't need. `frame_mapping` resolution still lives on Module (every
    module needs it for blueprint-level frame namespacing); only the static-transform
    publishing is added here.
    """

    config: TfModuleConfig

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._static_publish_thread: threading.Thread | None = None
        # created lazily in start() so the module stays picklable for worker deployment
        self._static_publish_stop: threading.Event | None = None
        self.static_transforms = self._resolve_static_transforms()

    @rpc
    def start(self) -> None:
        # NOTE: there's basically always going to be some inital race around static transform frames and tf.get's
        # publishing the statics before main starts helps mitigate/reduce that
        self._start_static_publish()
        super().start()

    @rpc
    def stop(self) -> None:
        self._stop_static_publish()
        super().stop()

    def _resolve_static_transforms(self) -> dict[str, Transform]:
        frame_mapping_field = type(self.config).model_fields["frame_mapping"]
        existing_frames: dict[str, str] = frame_mapping_field.default_factory()  # type: ignore[misc,union-attr,call-arg]

        # step1 translate urdf_name=>common_name (see Module._setup_frame_mapping for what
        # "common name" vs "real frame id" means)
        reverse_mapping = {value: key for key, value in existing_frames.items()}
        static_transforms_common_names = {
            reverse_mapping.get(urdf_frame_id, urdf_frame_id): Transform(
                translation=transform.translation,
                rotation=transform.rotation,
                frame_id=reverse_mapping.get(transform.frame_id, transform.frame_id),
                child_frame_id=reverse_mapping.get(
                    transform.child_frame_id, transform.child_frame_id
                ),
            )
            for urdf_frame_id, transform in self.config.static_transforms.items()
        }
        # step2 map common_name=>real_frame_id using the module's resolved frame_mapping
        final_frame_mapping = self.frame_mapping
        return {
            final_frame_mapping.get(common_frame_id, common_frame_id): Transform(
                translation=transform.translation,
                rotation=transform.rotation,
                frame_id=final_frame_mapping.get(transform.frame_id, transform.frame_id),
                child_frame_id=final_frame_mapping.get(
                    transform.child_frame_id, transform.child_frame_id
                ),
            )
            for common_frame_id, transform in static_transforms_common_names.items()
        }

    def _start_static_publish(self) -> None:
        self._static_publish()
        if not self.static_transforms or self.config.static_publish_interval <= 0:
            return
        self._static_publish_stop = threading.Event()
        self._static_publish_thread = threading.Thread(
            target=self._static_publisher,
            daemon=True,
        )
        self._static_publish_thread.start()

    # TODO: later this should be replaced with latching streams
    def _static_publisher(self) -> None:
        stop = self._static_publish_stop
        assert stop is not None
        while not stop.wait(self.config.static_publish_interval):
            self._static_publish()

    def _static_publish(self) -> None:
        if not self.static_transforms:
            return
        now = time.time()
        self.tf.publish_static(
            *(
                Transform(
                    translation=transform.translation,
                    rotation=transform.rotation,
                    frame_id=transform.frame_id,
                    child_frame_id=transform.child_frame_id,
                    ts=now,
                )
                for transform in self.static_transforms.values()
            )
        )
        self._on_static_publish()

    def _on_static_publish(self) -> None:
        """
        This is a callback for modules to publish other data (ex: camera info) in the static loop
        This should be rarely used, but exists for the few cases where it is needed
        """

    def _stop_static_publish(self) -> None:
        if self._static_publish_stop is not None:
            self._static_publish_stop.set()
        if self._static_publish_thread and self._static_publish_thread.is_alive():
            self._static_publish_thread.join(timeout=self._loop_thread_timeout)
        self._static_publish_thread = None
        self._static_publish_stop = None

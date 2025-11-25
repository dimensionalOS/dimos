# Copyright 2025 Dimensional Inc.
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

from datetime import datetime
from typing import Union

from dimos.msgs.geometry_msgs import Transform
from dimos.protocol.service.lcmservice import LCMConfig, LCMService
from dimos.protocol.tf.tf import TFConfig, TFSpec


# this doesn't work due to tf_lcm_py package
class TFLCM(TFSpec, LCMService):
    """A service for managing and broadcasting transforms using LCM.
    This is not a separete module, You can include this in your module
    if you need to access transforms.

    Ideally we would have a generic pubsub for transforms so we are
    transport agnostic (TODO)

    For now we are not doing this because we want to use cpp buffer/lcm
    implementation. We also don't want to manually hook up tf stream
    for each module.
    """

    default_config = Union[TFConfig, LCMConfig]

    def __init__(self, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(**kwargs)

        import tf_lcm_py as tf  # type: ignore[import-not-found]

        self.l = tf.LCM()
        self.buffer = tf.Buffer(self.config.buffer_size)
        self.listener = tf.TransformListener(self.l, self.buffer)
        self.broadcaster = tf.TransformBroadcaster()
        self.static_broadcaster = tf.StaticTransformBroadcaster()

        # will call the underlying LCMService.start
        self.start()

    def send(self, *args: Transform) -> None:
        for t in args:
            self.broadcaster.send_transform(t.lcm_transform())

    def send_static(self, *args: Transform) -> None:
        for t in args:
            self.static_broadcaster.send_static_transform(t)

    def lookup(  # type: ignore[no-untyped-def]
        self,
        parent_frame: str,
        child_frame: str,
        time_point: float | None = None,
        time_tolerance: float | None = None,
    ):
        return self.buffer.lookup_transform(
            parent_frame,
            child_frame,
            datetime.now(),
            lcm_module=self.l,
        )

    def can_transform(
        self, parent_frame: str, child_frame: str, time_point: float | datetime | None = None
    ) -> bool:
        if not time_point:
            time_point = datetime.now()

        if isinstance(time_point, float):
            time_point = datetime.fromtimestamp(time_point)

        return self.buffer.can_transform(parent_frame, child_frame, time_point)  # type: ignore[no-any-return]

    def get_frames(self) -> set[str]:
        return set(self.buffer.get_all_frame_names())

    def start(self) -> None:
        super().start()
        ...

    def stop(self) -> None: ...

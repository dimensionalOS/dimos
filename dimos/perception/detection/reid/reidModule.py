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

from typing import Callable, Optional

from reactivex import operators as ops
from reactivex.observable import Observable

from dimos.core import In, Module, ModuleConfig, rpc
from dimos.msgs.sensor_msgs import Image
from dimos.msgs.vision_msgs import Detection2DArray
from dimos.perception.detection.reid.base import EmbeddingModel
from dimos.perception.detection.type import ImageDetections2D
from dimos.types.timestamped import align_timestamped
from dimos.utils.reactive import backpressure


class Config(ModuleConfig):
    embedding_model: Optional[Callable[..., "EmbeddingModel"]] = None


class ReidModule(Module):
    detections: In[Detection2DArray] = None  # type: ignore
    image: In[Image] = None  # type: ignore

    def detections_stream(self) -> Observable[ImageDetections2D]:
        return backpressure(
            align_timestamped(
                self.image.pure_observable(),
                self.detections.pure_observable(),
                match_tolerance=0.0,
                buffer_size=2.0,
            ).pipe(ops.map(lambda pair: ImageDetections2D.from_ros_detection2d_array(*pair)))  # type: ignore[misc]
        )

    @rpc
    def start(self):
        self.detections_stream().subscribe(print)

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

"""A synthetic image producer connected to a listener. No hardware required."""

import numpy as np
import reactivex as rx
from reactivex.disposable import Disposable

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat


def make_image() -> Image:
    return Image(data=np.zeros((120, 160, 3), dtype=np.uint8), format=ImageFormat.RGB)


def describe(image: Image) -> str:
    height, width = image.data.shape[:2]
    return f"image {width}x{height}"


class Producer(Module):
    color_image: Out[Image]

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(
            rx.interval(1.0).subscribe(lambda _: self.color_image.publish(make_image()))
        )


class Listener(Module):
    color_image: In[Image]

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(
            Disposable(self.color_image.subscribe(lambda image: print(describe(image), flush=True)))
        )


blueprint = autoconnect(Producer.blueprint(), Listener.blueprint())

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

from abc import ABC
from typing import Tuple, Callable
from dimos.types.path import Path
from dimos.types.vector import Vector

import reactivex as rx
from reactivex import operators as ops
from reactivex.observable import Observable
from reactivex.subject import Subject
from dimos.web.websocket_vis.types import Drawable


class Visualizable(ABC):
    """
    Base class for objects that can provide visualization data.
    """

    def vis_stream(self) -> Observable[Tuple[str, Drawable]]:
        if not hasattr(self, "_vis_subject"):
            self._vis_subject = Subject()
        return self._vis_subject

    def vis(self, name: str, drawable: Drawable) -> None:
        if not hasattr(self, "_vis_subject"):
            return
        self._vis_subject.on_next((name, drawable))


def vector_stream(
    name: str, pos: Callable[[], Vector], update_interval=0.1, precision=0.25, history=10
) -> Observable[Tuple[str, Drawable]]:
    return rx.interval(update_interval).pipe(
        ops.map(lambda _: pos()),
        ops.distinct_until_changed(
            comparer=lambda a, b: (a - b).length() < precision,
        ),
        ops.scan(
            lambda hist, cur: hist.ipush(cur).iclip_tail(history),
            seed=Path(),
        ),
        ops.flat_map(lambda path: rx.from_([(f"{name}_hst", path), (name, path.last())])),
    )

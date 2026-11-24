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

"""Scene scans answered from the simulator's true body poses instead of the
camera, in the shape of ``ManipulationLocalize.scan``."""

from __future__ import annotations

import threading
from typing import Any

from pydantic import BaseModel
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.tf2_msgs.TFMessage import TFMessage


class TruthObject(BaseModel):
    """One tracked object's box: ``size`` is its extents and ``centre`` where the
    box centre sits relative to the body origin, both in metres."""

    size: tuple[float, float, float]
    centre: tuple[float, float, float] = (0.0, 0.0, 0.0)


class TruthSceneModuleConfig(ModuleConfig):
    """``objects`` maps each tracked body name to its box."""

    objects: dict[str, TruthObject]


class TruthSceneModule(Module):
    """Answers ``scan`` with a world-frame box for every tracked body named in
    the query, from the latest ``body_poses`` message. Takes the place of
    ``ManipulationLocalize`` on a simulated stack."""

    config: TruthSceneModuleConfig

    body_poses: In[TFMessage]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = threading.Lock()
        self._latest: dict[str, Transform] = {}

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.body_poses.subscribe(self._on_body_poses)))

    def _on_body_poses(self, msg: TFMessage) -> None:
        with self._lock:
            for transform in msg.transforms:
                self._latest[transform.child_frame_id] = transform

    @rpc
    def scan(self, names: list[str], window_s: float = 10.0) -> list[dict[str, Any]]:
        """One world-frame box per tracked body that ``names`` names (a body matches
        a name that contains it as a word), from the latest body poses. ``window_s``
        (seconds) is ignored: the poses are always current."""
        with self._lock:
            latest = dict(self._latest)
        return [
            {
                "id": name,
                "label": name,
                "position": [
                    t.translation.x + self.config.objects[name].centre[0],
                    t.translation.y + self.config.objects[name].centre[1],
                    t.translation.z + self.config.objects[name].centre[2],
                ],
                "size": list(self.config.objects[name].size),
            }
            for name, t in latest.items()
            if name in self.config.objects and _named(name, names)
        ]


def _named(body: str, words: list[str]) -> bool:
    return any(body in word.lower().split() for word in words)

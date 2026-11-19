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

from typing import Any, Protocol

from dimos.spec.utils import Spec

# How far back a scan looks in the camera's memory, seconds: only frames after the last reset.
SCAN_WINDOW_S = 3.0


class ObjectSceneSpec(Spec, Protocol):
    """The scene a program reads: the objects found by name.

    ``scan`` returns one entry per object seen in the last ``window_s`` seconds,
    each with ``id``, ``label``, and ``position`` (box centre) and ``size`` in
    world metres.
    """

    def scan(self, names: list[str], window_s: float = 10.0) -> list[dict[str, Any]]: ...

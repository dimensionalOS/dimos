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

"""The sketch's deployment lines against the runtime rim surface (T8a, t8-rim.md §13.5).

Wiring assignments, subscribe, stats(), and transformer() typecheck for a
Stateless module without a single annotation on the module itself.

Static-typing fixture — never imported at runtime.
"""

from __future__ import annotations

from collections.abc import Callable, Iterator
from typing import Any

from dimos.pure import rim
from dimos.pure.rim import RimStats
from dimos.pure.typing import EngineSurface


class Cloud:
    ts: float
    points: bytes


class CostMapper(EngineSurface):
    class In:
        ts: float
        cloud: Cloud

    class Out:
        ts: float
        cost: Cloud

    def step(self, i: In) -> Out:
        raise NotImplementedError


def launch(m: CostMapper) -> RimStats:
    # the sketch's wiring lines: transports/sources bind structurally.
    m.i.cloud.transport = object()
    m.i.cloud.source = object()
    m.o.cost.transport = object()
    unsub = m.o.cost.subscribe(lambda row: None)
    unsub()
    # lifecycle rides EngineSurface-free module methods at runtime; the rim
    # entry points are typed on Any (runtime name validation, t4 §5.4 hole):
    rim.warmup_module(m)
    rim.start_module(m)
    rim.stop_module(m)
    # stats(m) returns RimStats (D10):
    return rim.stats(m)


# transformer(m) accepts a Stateless module and is an iterator transform (§10):
transform: Callable[[Iterator[Any]], Iterator[Any]] = rim.transformer(CostMapper())

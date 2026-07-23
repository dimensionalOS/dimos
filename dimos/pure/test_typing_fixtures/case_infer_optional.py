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

"""``-> Out | None`` step: TOut solves to Out — skips never reach consumers.

Static-typing fixture — never imported at runtime. Origin: opt2_variants.py
(Skipping). The NoneOnly reveal pins DOCUMENTED degenerate behavior: a
``-> None`` step leaves TOut unconstrained, mypy solves it to Never, and
over() yields Iterator[Never] — apt, since the module never emits.
"""

from __future__ import annotations

from dimos.pure.typing import EngineSurface


class Gate(EngineSurface):
    class In:
        ts: float

    class Out:
        ts: float
        quality: float

    def step(self, i: In) -> Out | None:
        raise NotImplementedError


reveal_type(next(Gate().over()))  # R: Gate.Out
reveal_type(next(Gate().over()).quality)  # R: builtins.float


class NoneOnly(EngineSurface):
    class In:
        ts: float

    def step(self, i: In) -> None:
        raise NotImplementedError


reveal_type(next(NoneOnly().over()))  # R: Never

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

"""Bare ``-> Out`` stateless step: over() yields Iterator[Tagger.Out].

Static-typing fixture — never imported at runtime; checked by
test_typing_static.py. Origin: design-session opt2_selfproto.py.
"""

from __future__ import annotations

from dimos.pure.typing import EngineSurface


class Image:
    brightness: float


class Tagger(EngineSurface):
    class In:
        ts: float
        image: Image

    class Out:
        ts: float
        located: str

    def step(self, i: In) -> Out:
        raise NotImplementedError


rows = Tagger().over()
reveal_type(rows)  # R: typing.Iterator[Tagger.Out]
row = next(rows)
reveal_type(row)  # R: Tagger.Out
reveal_type(row.located)  # R: builtins.str
reveal_type(row.ts)  # R: builtins.float

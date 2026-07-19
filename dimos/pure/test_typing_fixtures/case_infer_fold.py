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

"""A fold module matches no step protocol and lands on the Fold overload.

Static-typing fixture — never imported at runtime. Origin: sketch3 §7
(ScanBatcher).
"""

from __future__ import annotations

from collections.abc import Iterator

from dimos.pure.typing import EngineSurface


class Scan:
    pass


class Batcher(EngineSurface):
    class In:
        ts: float
        lidar: Scan

    class Out:
        ts: float
        batch: Scan

    def fold(self, rows: Iterator[In]) -> Iterator[Out]:
        raise NotImplementedError


reveal_type(next(Batcher().over()))  # R: Batcher.Out
reveal_type(next(Batcher().over()).batch)  # R: Scan

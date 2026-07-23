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

"""pm.State statics: dataclass_transform yields a frozen kw-only constructor and
a Self-typed replace(). Pins the typed surface against the shipped
dimos.pure.state.

Static-typing fixture — never imported at runtime.
"""

from __future__ import annotations

from dimos.pure.state import State


class Counter(State):
    n: int = 0
    label: str = "x"


class NeedsSeed(State):
    seed: int  # no default -> required kwarg


c = Counter(n=5)
reveal_type(c.n)  # R: builtins.int
reveal_type(c.replace(n=9))  # R: Counter
reveal_type(c.replace())  # R: Counter

Counter(bogus=1)  # E[call-arg]: Unexpected keyword argument "bogus"
Counter(n="x")  # E[arg-type]: incompatible type
Counter(5)  # E[misc]: Too many positional arguments
NeedsSeed()  # E[call-arg]: Missing named argument "seed"

# frozen at the static surface (frozen_default):
c.n = 3  # E[misc]: is read-only

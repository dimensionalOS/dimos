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

"""TIn contravariant (a step may accept wider); TOut covariant (may emit narrower).

Static-typing fixture — never imported at runtime.
"""

from __future__ import annotations

from dimos.pure.typing import EngineSurface, Stateless


class BaseIn:
    ts: float


class RichIn(BaseIn):
    extra: int


class BaseOut:
    ts: float


class RichOut(BaseOut):
    more: int


class WideAccepting(EngineSurface):
    """Accepts the base row: usable wherever richer rows are fed."""

    def step(self, i: BaseIn) -> BaseOut:
        raise NotImplementedError


class RichEmitting(EngineSurface):
    """Emits the derived row: usable where consumers expect the base."""

    def step(self, i: BaseIn) -> RichOut:
        raise NotImplementedError


class Demanding(EngineSurface):
    """Demands the derived row: NOT usable where only BaseIn is available."""

    def step(self, i: RichIn) -> BaseOut:
        raise NotImplementedError


def feed_rich(m: Stateless[RichIn, BaseOut]) -> None:
    pass


def feed_base(m: Stateless[BaseIn, BaseOut]) -> None:
    pass


def wants_rich_out(m: Stateless[BaseIn, RichOut]) -> None:
    pass


feed_rich(WideAccepting())
feed_base(RichEmitting())
feed_base(Demanding())  # E[arg-type]: incompatible type
wants_rich_out(WideAccepting())  # E[arg-type]: incompatible type

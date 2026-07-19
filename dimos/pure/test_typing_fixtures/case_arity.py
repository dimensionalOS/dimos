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

"""Mealy and Stateless are disjoint by arity — neither passes as the other.

Static-typing fixture — never imported at runtime.
"""

from __future__ import annotations

from dimos.pure.typing import EngineSurface, Mealy, Stateless


class A:
    ts: float


class B:
    ts: float


class S:
    pass


class Sync(EngineSurface):
    def step(self, i: A) -> B:
        raise NotImplementedError


class Stateful(EngineSurface):
    def step(self, state: S, i: A) -> tuple[S, B | None]:
        raise NotImplementedError


def wants_stateless(m: Stateless[A, B]) -> None:
    pass


def wants_mealy(m: Mealy[S, A, B]) -> None:
    pass


wants_stateless(Sync())
wants_mealy(Stateful())
wants_stateless(Stateful())  # E[arg-type]: incompatible type
wants_mealy(Sync())  # E[arg-type]: incompatible type

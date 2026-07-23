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

"""Async steps hit the AsyncStateless overload FIRST — the order canary.

If Stateless were listed before AsyncStateless in over() or the port
accessors, a coroutine-returning step would unify TOut with
Coroutine[Any, Any, Out] and every reveal below would break.

Static-typing fixture — never imported at runtime. Origin: opt2_variants.py.
"""

from __future__ import annotations

from collections.abc import Awaitable

from dimos.pure.typing import EngineSurface


class Captioner(EngineSurface):
    class In:
        ts: float

    class Out:
        ts: float
        caption: str

    async def step(self, i: In) -> Out:
        raise NotImplementedError


reveal_type(next(Captioner().over()))  # R: Captioner.Out
reveal_type(next(Captioner().over()).caption)  # R: builtins.str


class AsyncSkipping(EngineSurface):
    class In:
        ts: float

    class Out:
        ts: float

    async def step(self, i: In) -> Out | None:
        raise NotImplementedError


reveal_type(next(AsyncSkipping().over()))  # R: AsyncSkipping.Out


class AwaitableReturner(EngineSurface):
    """DELIBERATE static/runtime divergence, documented: a sync def returning
    an Awaitable structurally matches AsyncStateless under mypy (this reveal),
    but T3 rejects it at import with [step-returns-awaitable] — one-spelling
    doctrine: an async step is spelled ``async def``.
    """

    class In:
        ts: float

    class Out:
        ts: float

    def step(self, i: In) -> Awaitable[Out]:
        raise NotImplementedError


reveal_type(next(AwaitableReturner().over()))  # R: AwaitableReturner.Out

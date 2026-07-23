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

"""T7: @resource static surface — instance→T, class→Resource[T], async
factories unify on the payload, dispose=False preserves types, resources
are never config fields.  Static-typing fixture — never imported at runtime."""

from __future__ import annotations

from dimos.pure import pm
from dimos.pure.module import PureModule
from dimos.pure.resources import resource
from dimos.pure.rows import contract, tick


class Grid:
    def dispose(self) -> None: ...


class VLM:
    async def aclose(self) -> None: ...


class Mapper(PureModule):
    voxel: float = 0.05

    class In(pm.In):
        x: float = tick()

    class Out(pm.Out):
        y: float = contract(min_hz=1)

    @resource
    def grid(self) -> Grid:
        raise NotImplementedError

    @resource
    async def client(self) -> VLM:
        raise NotImplementedError

    @resource(dispose=False)
    def shared(self) -> Grid:
        raise NotImplementedError

    def step(self, i: In) -> Out:
        raise NotImplementedError


m = Mapper(voxel=0.1)
reveal_type(m.grid)  # R: Grid
reveal_type(m.client)  # R: VLM
reveal_type(m.shared)  # R: Grid
reveal_type(Mapper.grid)  # R: dimos.pure.resources.Resource[Grid]

Mapper(grid=Grid())  # E[call-arg]: Unexpected keyword argument

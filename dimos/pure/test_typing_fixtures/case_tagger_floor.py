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

"""THE API floor: the canonical four-declaration Tagger, zero added
annotations, full inference end to end.

Mirrors production wiring with stand-ins (dataclass_transform bases +
specifier-built bundles, sketch3 §1). If this fixture ever needs one more
annotation to pass, the design has regressed — fix the machinery, not this
file (index.md, cross-cutting rules).

Static-typing fixture — never imported at runtime.
"""

from __future__ import annotations

from typing import Any

from typing_extensions import dataclass_transform

from dimos.pure.typing import EngineSurface


def tick(*, expect_hz: float | None = None) -> Any:
    raise NotImplementedError


def interpolate() -> Any:
    raise NotImplementedError


def contract(*, min_hz: float | None = None) -> Any:
    raise NotImplementedError


@dataclass_transform(kw_only_default=True, field_specifiers=(tick, interpolate, contract))
class _RowRoot:
    pass


class InBase(_RowRoot):
    ts: float  # transformed layer → real kw-only field (see case_rows_required)


class OutBase(_RowRoot):
    ts: float


@dataclass_transform(kw_only_default=True)
class PureModule(EngineSurface):
    pass


class Image:
    brightness: float


class PoseStamped:
    x: float


# ── the floor: one class, four declarations, nothing said twice ─────────────
class Tagger(PureModule):
    class In(InBase):
        image: Image = tick(expect_hz=30)
        pose: PoseStamped = interpolate()

    class Out(OutBase):
        located: str = contract(min_hz=10)

    def step(self, i: In) -> Out:
        raise NotImplementedError


m = Tagger()

# tests hand-construct rows and call step — no engine:
row = Tagger.In(ts=0.0, image=Image(), pose=PoseStamped())
reveal_type(row.image)  # R: Image
out = m.step(row)
reveal_type(out)  # R: Tagger.Out

# the engine surface infers with zero added annotations:
reveal_type(next(m.over()))  # R: Tagger.Out
reveal_type(next(m.over()).located)  # R: builtins.str
reveal_type(m.i)  # R: dimos.pure.typing.InPorts[Tagger.In]
reveal_type(m.o)  # R: dimos.pure.typing.OutPorts[Tagger.Out]

# row construction stays fully checked (specifier without default → required):
Tagger.In(ts=0.0, image=Image())  # E[call-arg]: Missing named argument "pose"
Tagger.In(ts=0.0, image=Image(), pose=PoseStamped(), zz=1)  # E[call-arg]: Unexpected keyword

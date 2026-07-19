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

"""T1 seed (shared fixture): specifiers model required-vs-defaulted fields.

A specifier call WITHOUT ``default=`` leaves the field REQUIRED in the
synthesized constructor — the whole point of declaring field_specifiers
(index.md T1). Stand-in specifiers until rows.py lands; T1 adopts this case.

Verified consequence for T1: annotations on the @dataclass_transform-DECORATED
class are NOT fields (the decorated class is not itself transformed) — the
engine-stamped ``ts`` must live on a transformed layer BELOW the decorated
root, i.e. pm.In subclasses the decorated root and declares ``ts``.

Static-typing fixture — never imported at runtime.
"""

from __future__ import annotations

from typing import Any

from typing_extensions import dataclass_transform


def tick(*, expect_hz: float | None = None) -> Any:
    raise NotImplementedError


def latest(*, default: Any) -> Any:
    raise NotImplementedError


@dataclass_transform(kw_only_default=True, field_specifiers=(tick, latest))
class _RowRoot:
    pass


class InBase(_RowRoot):
    ts: float  # transformed layer → a real kw-only field, inherited by bundles


class Image:
    pass


class TagIn(InBase):
    image: Image = tick(expect_hz=30)
    pose: Any = latest(default=None)


ok = TagIn(ts=0.0, image=Image())
ok2 = TagIn(ts=0.0, image=Image(), pose=object())
reveal_type(ok.image)  # R: Image
reveal_type(ok.ts)  # R: builtins.float

TagIn(ts=0.0)  # E[call-arg]: Missing named argument "image"
TagIn(image=Image())  # E[call-arg]: Missing named argument "ts"
TagIn(ts=0.0, image=3)  # E[arg-type]: incompatible type
TagIn(ts=0.0, image=Image(), bogus=1)  # E[call-arg]: Unexpected keyword argument
TagIn(0.0, Image())  # E[misc]: Too many positional arguments

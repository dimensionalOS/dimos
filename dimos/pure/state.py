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

"""``pm.State``: frozen base for Mealy step state, with a public ``replace()``.

Data layer only — no engine, no streams; imports are stdlib + typing_extensions.
"""

from __future__ import annotations

import dataclasses
from typing import TYPE_CHECKING, cast

from typing_extensions import Self, dataclass_transform

if TYPE_CHECKING:
    from _typeshed import DataclassInstance

__all__ = ["State"]


@dataclass_transform(kw_only_default=True, frozen_default=True)
class State:
    """Frozen kw-only base for Mealy step state — a typed, public-``replace`` NamedTuple.

    ``replace`` (not ``update``): non-mutating, per ``str.replace`` /
    ``dataclasses.replace`` / PEP 661 ``copy.replace`` — ``update`` connotes
    dict mutation. On 3.13+ ``copy.replace(s, ...)`` works via ``__replace__``;
    ``s.replace(...)`` works everywhere.
    """

    def __init_subclass__(cls, **kwargs: object) -> None:
        """Apply frozen kw-only dataclass; a ``replace`` field would shadow the method."""
        super().__init_subclass__(**kwargs)
        if "replace" in cls.__dict__.get("__annotations__", {}):
            raise TypeError(
                f"{cls.__module__}.{cls.__qualname__}: field name 'replace' is reserved "
                "(it would shadow State.replace); rename the field."
            )
        dataclasses.dataclass(frozen=True, kw_only=True, eq=True, repr=True)(cls)

    def __replace__(self, **changes: object) -> Self:
        """New instance with ``changes`` applied; unknown field names raise TypeError."""
        return cast("Self", dataclasses.replace(cast("DataclassInstance", self), **changes))

    def replace(self, **changes: object) -> Self:
        """Non-mutating copy with ``changes`` applied (the ``copy.replace`` protocol)."""
        return self.__replace__(**changes)

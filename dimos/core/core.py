#!/usr/bin/env python3
# Copyright 2025 Dimensional Inc.
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

from __future__ import annotations

from typing import (
    TYPE_CHECKING,
    Any,
    TypeVar,
)

from dimos.core.o3dpickle import register_picklers

if TYPE_CHECKING:
    from collections.abc import Callable

# injects pickling system into o3d
register_picklers()
T = TypeVar("T")


def rpc(fn: Callable[..., Any]) -> Callable[..., Any]:
    fn.__rpc__ = True  # type: ignore[attr-defined]
    return fn

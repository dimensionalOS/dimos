# Copyright 2025-2026 Dimensional Inc.
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

from functools import lru_cache
import importlib
from typing import TYPE_CHECKING, Any, cast

from dimos.message_codegen.registry import message_types

if TYPE_CHECKING:
    from dimos.msgs.protocol import DimosMsg


def lcm_msg_type(msg_name: str) -> type[Any]:
    """The generated dimos_lcm class of a '<package>.<Type>' name. ImportError
    when the wheel has no such message."""
    package, name = msg_name.split(".")
    msg_type: type[Any] = getattr(importlib.import_module(f"dimos_lcm.{package}.{name}"), name)
    return msg_type


@lru_cache(maxsize=256)
def resolve_msg_type(type_name: str) -> type[DimosMsg] | None:
    """Resolve a qualified package/msg/Type through installed schema providers."""
    return cast("type[DimosMsg] | None", message_types().get(type_name))

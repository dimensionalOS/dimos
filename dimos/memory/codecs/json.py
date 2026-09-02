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

"""Portable JSON codec for derived records.

``PickleCodec`` is the default for plain Python payloads, but a pickle is only
readable by the exact Python classes that wrote it. Derived records that outlive
a release -- belief, audit trails, anything another tool or language has to read
-- need a wire format that survives refactors.

Two contracts are enforced when the codec is constructed, so a stream that would
be unreadable later fails at open time rather than at decode time:

1. **The payload type must reject unknown fields.** Silently dropping a field a
   newer writer added turns a schema mismatch into quiet data loss. Pydantic
   cannot be told this from the outside -- ``TypeAdapter`` refuses a ``config``
   override for models and dataclasses -- so the *type* declares it and this
   codec checks that it did.

2. **The payload type must carry a version field.** Portability without a
   version is only the appearance of portability: the bytes parse, but nothing
   can tell which shape they are.
"""

from __future__ import annotations

import dataclasses
from typing import TYPE_CHECKING, Any, Generic, TypeVar

from pydantic import BaseModel, TypeAdapter

if TYPE_CHECKING:
    from collections.abc import Iterable

T = TypeVar("T")

DEFAULT_VERSION_FIELD = "schema_version"


def _declared_extra(payload_type: type[Any]) -> str | None:
    """The ``extra`` policy the type declares, or None if it declares none."""
    config: Any
    if isinstance(payload_type, type) and issubclass(payload_type, BaseModel):
        config = payload_type.model_config
    else:
        config = getattr(payload_type, "__pydantic_config__", None)
    if not config:
        return None
    extra = config.get("extra")
    return str(extra) if extra is not None else None


def _field_names(payload_type: type[Any]) -> Iterable[str]:
    if isinstance(payload_type, type) and issubclass(payload_type, BaseModel):
        return payload_type.model_fields.keys()
    if dataclasses.is_dataclass(payload_type):
        return (f.name for f in dataclasses.fields(payload_type))
    return ()


class JsonCodec(Generic[T]):
    """Encode a Pydantic model or dataclass as JSON.

    Nested models, dataclasses, tuples and enums are handled by Pydantic's
    ``TypeAdapter``; this class only adds the two construction-time contracts.
    """

    def __init__(
        self,
        payload_type: type[T],
        *,
        version_field: str | None = DEFAULT_VERSION_FIELD,
    ) -> None:
        if _declared_extra(payload_type) != "forbid":
            raise TypeError(
                f"{payload_type.__name__} must declare extra='forbid' to be stored as JSON. "
                "Add model_config = ConfigDict(extra='forbid') to the model, or "
                "__pydantic_config__ = ConfigDict(extra='forbid') to the dataclass. "
                "Without it an unknown field is dropped instead of reported."
            )
        if version_field is not None and version_field not in set(_field_names(payload_type)):
            raise TypeError(
                f"{payload_type.__name__} must declare a {version_field!r} field to be stored "
                "as JSON, so a later reader can tell which shape the bytes are. "
                "Pass version_field=None to opt out."
            )
        self._payload_type = payload_type
        self._adapter: TypeAdapter[T] = TypeAdapter(payload_type)

    def encode(self, value: T) -> bytes:
        return self._adapter.dump_json(value)

    def decode(self, data: bytes) -> T:
        return self._adapter.validate_json(data)

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

"""Config machinery: collect flat class fields, synthesize the frozen pydantic model.

Spec: dimos/pure/tasks/t2-config.md. Flat annotated fields on a PureModule
subclass are the API; the synthesized per-subclass frozen pydantic model
(``extra="forbid"``, ``frozen=True``) is the substance, and its
``model_dump()`` is THE canonical config serialization (memo keys,
checkpoints, sweeps — module identity = class + config).
"""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any, Final

from pydantic import ConfigDict

from dimos.protocol.service.spec import BaseConfig

__all__ = [
    "RESERVED_CONFIG_FIELDS",
    "ConfigFieldError",
    "FrozenModuleError",
    "PureModuleConfig",
]

# Names owned by PureModule machinery (present and future tasks); a config
# field with one of these names raises ConfigFieldError at class definition.
RESERVED_CONFIG_FIELDS: Final[frozenset[str]] = frozenset(
    {
        "config",
        "step",
        "fold",
        "In",
        "Out",
        "State",
        "warmup",
        "start",
        "stop",
        "over",
        "checkpoint",
        "restore",
        "i",
        "o",
        "health",
    }
)


class ConfigFieldError(TypeError):
    """A class body declares an invalid config field (raised at class definition).

    NOTE(T3 seam): once dimos.pure.stepspec lands, the base flips to
    stepspec.PureModuleDefinitionError (itself a TypeError subclass) so all
    definition-time errors share one user-facing type — spec §10.1.
    """


class FrozenModuleError(AttributeError):
    """Attribute assignment/deletion on a PureModule instance (config is frozen)."""


class PureModuleConfig(BaseConfig):
    """Root of every synthesized per-module config model."""

    model_config = ConfigDict(
        arbitrary_types_allowed=True,
        extra="forbid",
        frozen=True,
        validate_default=True,
        protected_namespaces=(),
    )


def _collect_config_fields(cls: type) -> dict[str, tuple[Any, Any]]:
    """Collect ``{name: (resolved type, default | ...)}`` from one class's own body.

    Annotated, non-ClassVar names become fields; nested classes, descriptors,
    methods, and dunders never do. Raises ConfigFieldError per spec §3/§11.
    """
    raise NotImplementedError


def _synthesize_config_model(
    cls: type,
    fields: Mapping[str, tuple[Any, Any]],
    bases: tuple[type[PureModuleConfig], ...],
) -> type[PureModuleConfig]:
    """Build the per-subclass frozen model ``<ClassName>Config`` extending *bases*."""
    raise NotImplementedError

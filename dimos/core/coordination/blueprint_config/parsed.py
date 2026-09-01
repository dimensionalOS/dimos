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

"""The immutable result of parsing blueprint configuration."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field
from typing import Any, cast

from dimos.core.coordination.blueprint_config.errors import BlueprintConfigError
from dimos.core.coordination.blueprint_config.sources import validate_global_values
from dimos.core.coordination.blueprint_config.values import read_only_view, snapshot_mapping
from dimos.core.coordination.blueprints import (
    Blueprint,
    TransportSpec,
    config_key,
    transport_config_name,
)


@dataclass(frozen=True, slots=True)
class ParsedBlueprintConfig:
    """Validated, immutable configuration snapshot for one blueprint."""

    _module_config_values: Mapping[str, Any] = field(repr=False)
    _global_config_values: Mapping[str, Any] = field(repr=False)
    _global_explicit_values: Mapping[str, Any] = field(repr=False)
    _transport_config_values: Mapping[str, Any] = field(repr=False)
    _blueprint: Blueprint = field(repr=False, compare=False)

    @property
    def module_configs(self) -> Mapping[str, Any]:
        """Return a read-only view detached from the stored snapshot."""
        return cast("Mapping[str, Any]", read_only_view(self._module_config_values))

    @property
    def global_config(self) -> Mapping[str, Any]:
        """Return a read-only view detached from the stored snapshot."""
        return cast("Mapping[str, Any]", read_only_view(self._global_config_values))

    @property
    def transport_configs(self) -> Mapping[str, Any]:
        """Return a read-only view detached from the stored snapshot."""
        return cast("Mapping[str, Any]", read_only_view(self._transport_config_values))

    def assert_matches(self, blueprint: Blueprint) -> None:
        """Reject use with a blueprint other than the one that was parsed."""
        if blueprint is not self._blueprint:
            raise BlueprintConfigError(
                "Parsed blueprint configuration belongs to a different blueprint."
            )

    def global_config_values(self) -> dict[str, Any]:
        """Return an independent mutable copy of the resolved GlobalConfig values."""
        return snapshot_mapping(self._global_config_values)

    def explicit_global_config_values(self) -> dict[str, Any]:
        """Return only the validated GlobalConfig values a source explicitly set.

        Schema defaults are excluded, so a live coordinator can apply these
        without resetting unrelated fields.
        """
        return snapshot_mapping(self._global_explicit_values)

    def module_kwargs(self, instance_name: str) -> dict[str, Any]:
        """Return independent mutable kwargs for a module instance."""
        try:
            values = self._module_config_values[instance_name]
        except KeyError:
            escaped = config_key(instance_name)
            matching = [
                value
                for name, value in self._module_config_values.items()
                if config_key(name) == escaped
            ]
            if len(matching) != 1:
                raise KeyError(f"No parsed configuration for module {instance_name!r}") from None
            values = matching[0]
        if not isinstance(values, Mapping):
            raise TypeError(f"Module configuration for {instance_name!r} is not a mapping")
        return snapshot_mapping(values)

    def transport_overrides(self) -> dict[str, Any]:
        """Return independent mutable transport configuration overrides."""
        return snapshot_mapping(self._transport_config_values)

    def subset_for(
        self,
        blueprint: Blueprint,
        *,
        global_overrides: Mapping[str, Any] | None = None,
    ) -> ParsedBlueprintConfig:
        """Bind a configuration subset to a Blueprint fragment.

        Only module and transport configuration used by ``blueprint`` is retained.
        Global overrides are validated against ``GlobalConfig`` before the immutable
        snapshot is created.
        """
        module_values = {
            atom.name: self.module_kwargs(atom.name) for atom in blueprint.active_blueprints
        }
        transport_names = {
            transport_config_name(spec.config_cls)
            for spec in blueprint.transport_map.values()
            if isinstance(spec, TransportSpec) and spec.config_cls is not None
        }
        transport_values = {
            name: snapshot_mapping(values)
            for name, values in self._transport_config_values.items()
            if name in transport_names and isinstance(values, Mapping)
        }

        global_values = self.global_config_values()
        explicit_global_values = self.explicit_global_config_values()
        if global_overrides is not None:
            global_values.update(snapshot_mapping(global_overrides))
            global_values = validate_global_values(global_values)
            for name in global_overrides:
                explicit_global_values[name] = global_values[name]

        return ParsedBlueprintConfig(
            _module_config_values=snapshot_mapping(module_values),
            _global_config_values=snapshot_mapping(global_values),
            _global_explicit_values=snapshot_mapping(explicit_global_values),
            _transport_config_values=snapshot_mapping(transport_values),
            _blueprint=blueprint,
        )

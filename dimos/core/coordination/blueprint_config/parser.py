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

"""Blueprint-aware command-line configuration parsing.

This package deliberately has no dependency on Typer.  It owns the complete
configuration merge and validation flow so that CLI, tests, and programmatic
callers all resolve blueprint configuration in the same way.
"""

from __future__ import annotations

from collections import defaultdict
from collections.abc import Collection, Iterable, Mapping, Sequence
from pathlib import Path
from types import MappingProxyType
from typing import Any

from pydantic import BaseModel, ValidationError
from pydantic_core import PydanticUndefined

from dimos.core.coordination.blueprint_config.errors import (
    BlueprintConfigError,
    format_validation_error,
)
from dimos.core.coordination.blueprint_config.fields import (
    display_annotation,
    field_has_required_parent,
    field_is_required,
    leaf_fields,
    module_config_cls,
    nested_get,
    prepare_model_input,
    safe_field_default,
)
from dimos.core.coordination.blueprint_config.merging import (
    merge_cli,
    merge_environment,
    merge_root_source,
)
from dimos.core.coordination.blueprint_config.parsed import ParsedBlueprintConfig
from dimos.core.coordination.blueprint_config.schema import (
    ModuleSchema,
    OptionTarget,
    ParserSchema,
    TransportSchema,
    cli_path,
    normalize_option_name,
)
from dimos.core.coordination.blueprint_config.sources import (
    configuration_environment,
    global_environment_names,
    global_schema_defaults,
    merge_global_cli,
    merge_global_environment,
    read_config_file,
    reserved_global_short_names,
    validate_global_values,
)
from dimos.core.coordination.blueprint_config.values import (
    deep_merge,
    extract_shape,
    normalize_mapping_keys,
    plain,
    plain_mapping,
    snapshot_mapping,
)
from dimos.core.coordination.blueprints import (
    Blueprint,
    TransportSpec,
    config_key,
    transport_config_name,
)
from dimos.core.global_config import GlobalConfig


def split_run_arguments(tokens: Sequence[str]) -> tuple[tuple[str, ...], tuple[str, ...]]:
    """Split Typer's variadic run arguments into blueprint names and config tokens.

    Blueprint names must form the leading positional segment.  Once any
    dash-prefixed token is seen, all remaining tokens belong to option parsing.
    """
    split_at = next((i for i, token in enumerate(tokens) if token.startswith("-")), len(tokens))
    blueprint_names = tuple(tokens[:split_at])
    if not blueprint_names:
        raise BlueprintConfigError(
            "At least one blueprint name must precede configuration options. "
            "Usage: dimos run <blueprint> [--config-field value]."
        )
    return blueprint_names, tuple(tokens[split_at:])


class BlueprintConfigParser:
    """Resolve all configuration sources for a blueprint."""

    def __init__(self, blueprint: Blueprint) -> None:
        self.blueprint = blueprint
        self._schema: ParserSchema | None = None

    @classmethod
    def preparse_global_config(
        cls,
        cli_tokens: Iterable[str] = (),
        *,
        config_path: Path | str | None = None,
        environ: Mapping[str, str] | None = None,
        global_overrides: Mapping[str, Any] | None = None,
    ) -> dict[str, Any]:
        """Resolve GlobalConfig before importing a blueprint.

        Blueprint modules may choose their composition at import time from
        ``global_config``. This pre-pass consumes only recognized global
        sources and ignores blueprint-specific CLI tokens; the regular
        :meth:`parse` call still performs complete validation afterward.
        """
        values = global_schema_defaults()
        if config_path is not None:
            config_values = read_config_file(Path(config_path))
            raw_global = config_values.get("g")
            if raw_global is not None:
                if not isinstance(raw_global, Mapping):
                    raise BlueprintConfigError(
                        "config file section 'g' must be an object, not "
                        f"{type(raw_global).__name__}."
                    )
                deep_merge(values, normalize_mapping_keys(raw_global))

        merge_global_environment(
            values,
            configuration_environment(environ),
        )
        merge_global_cli(values, tuple(cli_tokens))
        if global_overrides is not None:
            deep_merge(values, normalize_mapping_keys(global_overrides))
        return validate_global_values(values)

    def parse(
        self,
        cli_tokens: Iterable[str] = (),
        *,
        config_path: Path | str | None = None,
        environ: Mapping[str, str] | None = None,
        global_overrides: Mapping[str, Any] | None = None,
        overrides: Mapping[str, Any] | None = None,
    ) -> ParsedBlueprintConfig:
        """Merge, validate, and freeze blueprint configuration.

        Precedence, from lowest to highest, is blueprint/schema defaults,
        blueprint-pinned values, JSON config, environment, programmatic
        ``overrides``, dynamic CLI flags, and explicit ``global_overrides``.
        """
        schema = self._get_schema()
        self._validate_structure(schema)

        module_values = {
            module.atom.name: {
                key: plain(value)
                for key, value in module.atom.kwargs.items()
                if key in module.config_cls.model_fields
            }
            for module in schema.modules
        }
        # Collect only values a source explicitly set; schema defaults are
        # overlaid just before validation. Equivalent to seeding the defaults
        # first because GlobalConfig is flat (no nested mappings for
        # deep_merge to recurse into).
        global_values: dict[str, Any] = {}
        deep_merge(global_values, plain_mapping(self.blueprint.global_config_overrides))
        transport_values: dict[str, Any] = {}

        if config_path is not None:
            config_values = read_config_file(Path(config_path))
            merge_root_source(
                module_values,
                global_values,
                transport_values,
                config_values,
                source="config file",
                schema=schema,
            )

        merge_environment(
            module_values,
            global_values,
            transport_values,
            configuration_environment(environ),
            schema,
        )

        if overrides is not None:
            merge_root_source(
                module_values,
                global_values,
                transport_values,
                overrides,
                source="programmatic overrides",
                schema=schema,
            )

        merge_cli(
            module_values,
            global_values,
            transport_values,
            tuple(cli_tokens),
            schema,
        )

        if global_overrides is not None:
            deep_merge(global_values, plain_mapping(global_overrides))

        parsed_modules = self._validate_modules(module_values, schema)
        resolved_global = global_schema_defaults()
        deep_merge(resolved_global, global_values)
        parsed_global = validate_global_values(resolved_global)
        # Pick the validated values of the explicitly-set fields, translating
        # alias keys (e.g. DIMOS_TRANSPORT) to field names first.
        field_names = global_environment_names()
        explicit_shape = {
            field_names.get(key.lower(), key): value for key, value in global_values.items()
        }
        explicit_global = extract_shape(parsed_global, explicit_shape)
        parsed_transports = self._validate_transports(transport_values, schema)

        return ParsedBlueprintConfig(
            _module_config_values=snapshot_mapping(parsed_modules),
            _global_config_values=snapshot_mapping(parsed_global),
            _global_explicit_values=snapshot_mapping(explicit_global),
            _transport_config_values=snapshot_mapping(parsed_transports),
            _blueprint=self.blueprint,
        )

    def format_help(self, reserved_options: Collection[str] = ()) -> str:
        """Render discoverable dynamic options without evaluating default factories."""
        schema = self._get_schema()
        lines = ["Blueprint configuration options:"]
        global_names = reserved_global_short_names()
        reserved = {normalize_option_name(option.removeprefix("--")) for option in reserved_options}

        for target in sorted(schema.targets, key=lambda item: item.qualified_name):
            aliases = schema.aliases[normalize_option_name(target.qualified_name)]
            relative_available = (
                target.section != "module"
                or len(target.path) != 1
                or target.path[0] not in global_names
            )
            relative_available = (
                relative_available and normalize_option_name(target.relative_name) not in reserved
            )
            relative_aliases = schema.aliases.get(normalize_option_name(target.relative_name), ())
            option_names = [f"--{target.qualified_name}"]
            if (
                relative_available
                and len(relative_aliases) == 1
                and relative_aliases[0].identity == target.identity
            ):
                option_names.insert(0, f"--{target.relative_name}")

            annotation = display_annotation(target.annotations)
            default = self._help_default(target, schema)
            parent_required = self._target_has_required_parent(target, schema)
            suffix = ""
            if default is not PydanticUndefined:
                suffix = f" (default: {default})"
                if parent_required:
                    suffix += " [parent required]"
            elif self._target_is_required(target, schema):
                suffix = " [required]"
            elif parent_required:
                suffix = " [parent required]"
            if len(aliases) > 1:
                option_names = [f"--{target.qualified_name}"]
            lines.append(f"  {', '.join(option_names)} <{annotation}>{suffix}")

        return "\n".join(lines)

    def _get_schema(self) -> ParserSchema:
        if self._schema is not None:
            return self._schema

        modules = tuple(
            ModuleSchema(atom=atom, config_cls=module_config_cls(atom))
            for atom in self.blueprint.active_blueprints
        )
        transports_by_name: dict[str, tuple[type[BaseModel], list[TransportSpec]]] = {}
        for spec in self.blueprint.transport_map.values():
            if not isinstance(spec, TransportSpec) or spec.config_cls is None:
                continue
            name = transport_config_name(spec.config_cls)
            existing = transports_by_name.get(name)
            if existing is None:
                transports_by_name[name] = (spec.config_cls, [spec])
            else:
                existing[1].append(spec)
        transports = tuple(
            TransportSchema(name=name, config_cls=config_cls, specs=tuple(specs))
            for name, (config_cls, specs) in transports_by_name.items()
        )

        target_annotations: dict[tuple[str, str, tuple[str, ...]], list[Any]] = defaultdict(list)
        for module in modules:
            for path, annotation in leaf_fields(
                module.config_cls,
                excluded={"g", "instance_name", "rpc_name"},
            ):
                target_annotations[("module", module.atom.name, path)].append(annotation)
        for path, annotation in leaf_fields(GlobalConfig):
            target_annotations[("global", "g", path)].append(annotation)
        for transport in transports:
            for path, annotation in leaf_fields(transport.config_cls):
                target_annotations[("transport", transport.name, path)].append(annotation)

        targets = tuple(
            OptionTarget(
                section=section,  # type: ignore[arg-type]
                root=root,
                path=path,
                annotations=tuple(annotations),
            )
            for (section, root, path), annotations in target_annotations.items()
        )
        aliases = self._build_aliases(targets)
        self._schema = ParserSchema(
            modules=modules,
            transports=transports,
            targets=targets,
            aliases=MappingProxyType(aliases),
        )
        return self._schema

    def _build_aliases(
        self, targets: tuple[OptionTarget, ...]
    ) -> dict[str, tuple[OptionTarget, ...]]:
        canonical: dict[str, dict[tuple[str, str, tuple[str, ...]], OptionTarget]] = defaultdict(
            dict
        )
        shorthand: dict[str, dict[tuple[str, str, tuple[str, ...]], OptionTarget]] = defaultdict(
            dict
        )
        global_names = reserved_global_short_names()

        for target in targets:
            canonical_names = {target.qualified_name}
            shorthand_names: set[str] = set()
            if target.section == "global":
                shorthand_names.add(target.relative_name)
            elif target.section == "transport":
                shorthand_names.add(target.relative_name)
            else:
                escaped = cli_path((config_key(target.root), *target.path))
                canonical_names.add(escaped)
                if len(target.path) != 1 or target.path[0] not in global_names:
                    shorthand_names.add(target.relative_name)
                canonical_names.add(cli_path(("modules", target.root, *target.path)))

            for name in canonical_names:
                canonical[normalize_option_name(name)][target.identity] = target
            for name in shorthand_names:
                shorthand[normalize_option_name(name)][target.identity] = target

        # A fully qualified address must always be usable. A relative alias
        # that happens to spell another target's qualified address is hidden
        # rather than making that stable address ambiguous.
        aliases = {name: dict(values) for name, values in canonical.items()}
        for name, values in shorthand.items():
            if name not in aliases:
                aliases[name] = dict(values)

        return {
            name: tuple(sorted(values.values(), key=lambda target: target.qualified_name))
            for name, values in aliases.items()
        }

    def _validate_structure(self, schema: ParserSchema) -> None:
        roots: dict[str, list[str]] = defaultdict(list)
        for module in schema.modules:
            roots[config_key(module.atom.name)].append(module.atom.name)
        conflicts = {key: names for key, names in roots.items() if len(names) > 1}
        reserved = {key: names for key, names in roots.items() if key in {"g", "transports"}}
        if conflicts or reserved:
            details = []
            for key, names in sorted({**conflicts, **reserved}.items()):
                details.append(f"{key!r}: {', '.join(sorted(names))}")
            raise BlueprintConfigError(
                "Blueprint configuration instance-key collision: " + "; ".join(details)
            )

        transport_types: dict[str, set[type[BaseModel]]] = defaultdict(set)
        for spec in self.blueprint.transport_map.values():
            if isinstance(spec, TransportSpec) and spec.config_cls is not None:
                transport_types[transport_config_name(spec.config_cls)].add(spec.config_cls)
        collisions = {name: types for name, types in transport_types.items() if len(types) > 1}
        if collisions:
            transport_details = ", ".join(
                f"{name}: {', '.join(sorted(cls.__name__ for cls in classes))}"
                for name, classes in sorted(collisions.items())
            )
            raise BlueprintConfigError(
                f"Transport configuration name collision: {transport_details}"
            )

    def _validate_modules(
        self, module_values: Mapping[str, Mapping[str, Any]], schema: ParserSchema
    ) -> dict[str, Any]:
        parsed: dict[str, Any] = {}
        for module in schema.modules:
            values = prepare_model_input(
                module.config_cls,
                module_values[module.atom.name],
            )
            try:
                model = module.config_cls.model_validate(values)
            except ValidationError as error:
                raise BlueprintConfigError(
                    format_validation_error(module.atom.name, error)
                ) from error
            dumped = model.model_dump(mode="python", exclude_unset=True)
            dumped.pop("g", None)
            dumped.pop("instance_name", None)
            dumped.pop("rpc_name", None)
            parsed[module.atom.name] = dumped
        return parsed

    def _validate_transports(
        self, values: Mapping[str, Any], schema: ParserSchema
    ) -> dict[str, Any]:
        known = {transport.name for transport in schema.transports}
        unknown = set(values) - known
        if unknown:
            names = ", ".join(sorted(unknown))
            raise BlueprintConfigError(f"Unknown transport configuration section(s): {names}.")

        parsed: dict[str, Any] = {}
        for transport in schema.transports:
            raw_overrides = values.get(transport.name, {})
            if not isinstance(raw_overrides, Mapping):
                raise BlueprintConfigError(
                    f"Transport configuration {transport.name!r} must be an object."
                )
            models: list[BaseModel] = []
            for spec in transport.specs:
                pinned = {
                    key: plain(value)
                    for key, value in spec.kwargs.items()
                    if key in transport.config_cls.model_fields
                }
                deep_merge(pinned, plain_mapping(raw_overrides))
                pinned = prepare_model_input(transport.config_cls, pinned)
                try:
                    models.append(transport.config_cls.model_validate(pinned))
                except ValidationError as error:
                    raise BlueprintConfigError(
                        format_validation_error(
                            f"transports.{transport.name}",
                            error,
                        )
                    ) from error
            if not models:
                continue
            full = models[0].model_dump(mode="python")
            if raw_overrides:
                parsed[transport.name] = extract_shape(full, raw_overrides)
        return parsed

    def _help_default(self, target: OptionTarget, schema: ParserSchema) -> Any:
        if target.section == "global":
            values: Any = self.blueprint.global_config_overrides
            configured = nested_get(values, target.path)
            if configured is not PydanticUndefined:
                return configured
            return safe_field_default(GlobalConfig, target.path)
        if target.section == "transport":
            transport = next(item for item in schema.transports if item.name == target.root)
            for spec in transport.specs:
                configured = nested_get(spec.kwargs, target.path)
                if configured is not PydanticUndefined:
                    return configured
            return safe_field_default(transport.config_cls, target.path)

        module = next(item for item in schema.modules if item.atom.name == target.root)
        configured = nested_get(module.atom.kwargs, target.path)
        if configured is not PydanticUndefined:
            return configured
        return safe_field_default(module.config_cls, target.path)

    def _target_is_required(self, target: OptionTarget, schema: ParserSchema) -> bool:
        return field_is_required(self._target_model(target, schema), target.path)

    def _target_has_required_parent(self, target: OptionTarget, schema: ParserSchema) -> bool:
        return field_has_required_parent(self._target_model(target, schema), target.path)

    @staticmethod
    def _target_model(target: OptionTarget, schema: ParserSchema) -> type[BaseModel]:
        if target.section == "global":
            return GlobalConfig
        if target.section == "transport":
            return next(item.config_cls for item in schema.transports if item.name == target.root)
        return next(item.config_cls for item in schema.modules if item.atom.name == target.root)

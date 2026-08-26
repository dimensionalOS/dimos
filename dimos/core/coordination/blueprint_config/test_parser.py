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

from collections.abc import Callable
from pathlib import Path
from typing import Annotated, Any, Literal

from pydantic import BaseModel, Field
import pytest

from dimos.core.coordination.blueprint_config.errors import BlueprintConfigError
from dimos.core.coordination.blueprint_config.parser import (
    BlueprintConfigParser,
    split_run_arguments,
)
from dimos.core.coordination.blueprints import TransportSpec, autoconnect
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Stream, Transport


class NestedConfig(BaseModel):
    enabled: bool = True
    mode: str = "auto"


class PrimaryConfig(ModuleConfig):
    map_file: str | None = None
    speed: float = 1.0
    nested: NestedConfig = Field(default_factory=NestedConfig)
    labels: list[str] = Field(default_factory=list)


class PrimaryModule(Module):
    config: PrimaryConfig


class SecondaryConfig(ModuleConfig):
    map_file: str | None = None


class SecondaryModule(Module):
    config: SecondaryConfig


class ProviderConfig(BaseModel):
    api_key: str
    retries: int = 1


class ConfigurableTransport(Transport[bytes]):
    _config_cls = ProviderConfig

    def broadcast(self, selfstream: Stream[bytes] | None, value: bytes) -> None:
        pass

    def subscribe(self, callback: Any, selfstream: Stream[bytes] | None = None) -> Any:
        return lambda: None


def test_parse_supports_value_forms_types_nested_defaults_and_last_wins() -> None:
    blueprint = PrimaryModule.blueprint(nested={"mode": "manual"})

    parsed = BlueprintConfigParser(blueprint).parse(
        [
            "--map-file=first",
            "--map-file",
            "recording_go2",
            "--speed",
            "-0.5",
            "--nested.enabled=false",
            "--labels",
            '["one", "two"]',
        ],
        environ={},
    )

    assert parsed.module_kwargs("primarymodule") == {
        "map_file": "recording_go2",
        "speed": -0.5,
        "nested": {"enabled": False, "mode": "manual"},
        "labels": ["one", "two"],
    }


def test_ambiguous_shorthand_requires_deterministic_qualified_option() -> None:
    blueprint = autoconnect(PrimaryModule.blueprint(), SecondaryModule.blueprint())
    parser = BlueprintConfigParser(blueprint)

    parser.parse(environ={})
    with pytest.raises(BlueprintConfigError) as error:
        parser.parse(["--map-file", "map"], environ={})

    assert str(error.value) == (
        "Option --map-file is ambiguous. Use one of: "
        "--primarymodule.map-file, --secondarymodule.map-file."
    )
    parsed = parser.parse(["--secondarymodule.map-file=map"], environ={})
    assert parsed.module_kwargs("secondarymodule") == {"map_file": "map"}


def test_qualified_option_wins_over_another_fields_relative_alias() -> None:
    class FooConfig(ModuleConfig):
        bar: str | None = None

    class Foo(Module):
        config: FooConfig

    class NestedFooConfig(BaseModel):
        bar: str | None = None

    class OtherConfig(ModuleConfig):
        foo: NestedFooConfig = Field(default_factory=NestedFooConfig)

    class Other(Module):
        config: OtherConfig

    blueprint = autoconnect(Foo.blueprint(), Other.blueprint())
    parser = BlueprintConfigParser(blueprint)

    parsed = parser.parse(["--foo.bar", "root"], environ={})

    assert parsed.module_kwargs("foo") == {"bar": "root"}
    assert parsed.module_kwargs("other") == {}
    assert "--other.foo.bar" in parser.format_help()
    assert "--foo.bar, --other.foo.bar" not in parser.format_help()


def test_global_short_name_is_reserved_when_module_field_collides() -> None:
    class CollisionConfig(ModuleConfig):
        robot_ip: str = "module-default"

    class CollisionModule(Module):
        config: CollisionConfig

    blueprint = CollisionModule.blueprint()
    parsed = BlueprintConfigParser(blueprint).parse(
        [
            "--robot-ip",
            "192.0.2.10",
            "--collisionmodule.robot-ip",
            "module-value",
            "--replay=false",
            "--no-obstacle-avoidance",
            "--record",
            "--record-format",
            "sqlite",
        ],
        environ={},
    )

    assert parsed.global_config_values()["robot_ip"] == "192.0.2.10"
    assert parsed.global_config_values()["replay"] is False
    assert parsed.global_config_values()["record"] is True
    assert parsed.global_config_values()["record_format"] == "sqlite"
    assert parsed.global_config_values()["obstacle_avoidance"] is False
    assert parsed.module_kwargs("collisionmodule") == {"robot_ip": "module-value"}


def test_negated_global_bool_is_reserved_from_module_shorthand() -> None:
    class CollisionConfig(ModuleConfig):
        no_replay: str | None = None

    class CollisionModule(Module):
        config: CollisionConfig

    blueprint = CollisionModule.blueprint().global_config(replay=True)
    parser = BlueprintConfigParser(blueprint)

    parsed = parser.parse(["--no-replay"], environ={})

    assert parsed.global_config_values()["replay"] is False
    assert parsed.module_kwargs("collisionmodule") == {}
    help_text = parser.format_help()
    assert "--collisionmodule.no-replay" in help_text
    assert "  --no-replay," not in help_text


def test_sources_deep_merge_in_documented_precedence(tmp_path: Path) -> None:
    blueprint = PrimaryModule.blueprint(
        speed=1.0,
        nested={"enabled": True, "mode": "blueprint"},
    ).global_config(viewer="none")
    config_path = tmp_path / "config.json"
    config_path.write_text(
        '{"primarymodule":{"speed":2,"nested":{"mode":"file"}},"g":{"robot_ip":"file"}}'
    )

    parsed = BlueprintConfigParser(blueprint).parse(
        ["--speed", "5", "--nested.enabled", "false"],
        config_path=config_path,
        environ={
            "PRIMARYMODULE__SPEED": "3",
            "PRIMARYMODULE__NESTED__MODE": "environment",
            "ROBOT_IP": "environment",
        },
        overrides={
            "primarymodule": {"speed": 4, "nested": {"mode": "override"}},
            "g": {"robot_ip": "override"},
        },
        global_overrides={"robot_ip": "explicit"},
    )

    assert parsed.module_kwargs("primarymodule") == {
        "speed": 5.0,
        "nested": {"enabled": False, "mode": "override"},
    }
    assert parsed.global_config_values()["robot_ip"] == "explicit"
    assert parsed.global_config_values()["viewer"] == "none"


def test_namespaced_instance_accepts_slash_qualified_option() -> None:
    blueprint = PrimaryModule.blueprint().namespace("robot0")

    parsed = BlueprintConfigParser(blueprint).parse(
        ["--robot0/primarymodule.map-file", "map"],
        environ={},
    )

    assert parsed.module_kwargs("robot0/primarymodule") == {
        "frame_id_prefix": "robot0",
        "map_file": "map",
    }


def test_disabled_module_does_not_make_shorthand_ambiguous() -> None:
    blueprint = autoconnect(
        PrimaryModule.blueprint(),
        SecondaryModule.blueprint(),
    ).disabled_modules(SecondaryModule)

    parsed = BlueprintConfigParser(blueprint).parse(["--map-file", "map"], environ={})

    assert set(parsed.module_configs) == {"primarymodule"}
    assert parsed.module_kwargs("primarymodule") == {"map_file": "map"}


def test_transport_options_support_relative_full_and_environment_forms() -> None:
    blueprint = PrimaryModule.blueprint().transports(
        {
            ("payload", bytes): TransportSpec(
                ConfigurableTransport,
                kwargs={"api_key": "blueprint-key"},
            )
        }
    )
    parser = BlueprintConfigParser(blueprint)

    from_environment = parser.parse(
        environ={"TRANSPORTS__PROVIDER__RETRIES": "2"},
    )
    from_cli = parser.parse(
        ["--transports.provider.retries=3", "--provider.api-key", "cli-key"],
        environ={},
    )

    assert from_environment.transport_overrides() == {"provider": {"retries": 2}}
    assert from_environment.transport_configs["provider"] == {"retries": 2}
    assert from_cli.transport_overrides() == {"provider": {"api_key": "cli-key", "retries": 3}}


def test_environment_ignores_unknown_transport_section() -> None:
    blueprint = PrimaryModule.blueprint()
    parser = BlueprintConfigParser(blueprint)

    parsed = parser.parse(
        environ={
            "TRANSPORTS__BROKER__BROKER_URL": "https://teleop.dimensionalos.com",
            "TRANSPORTS__BROKER__API_KEY": "",
        },
    )

    assert parsed.transport_configs == {}


def test_environment_values_coerce_null_and_json_like_cli() -> None:
    parsed = BlueprintConfigParser(PrimaryModule.blueprint(map_file="pinned")).parse(
        environ={
            "PRIMARYMODULE__MAP_FILE": "null",
            "PRIMARYMODULE__LABELS": '["one", "two"]',
        },
    )

    kwargs = parsed.module_kwargs("primarymodule")
    assert kwargs["map_file"] is None
    assert kwargs["labels"] == ["one", "two"]


def test_environment_str_field_keeps_bracketed_string() -> None:
    parsed = BlueprintConfigParser(PrimaryModule.blueprint()).parse(
        environ={"PRIMARYMODULE__NESTED__MODE": "[not json"},
    )

    assert parsed.module_kwargs("primarymodule")["nested"]["mode"] == "[not json"


def test_environment_invalid_json_names_the_variable() -> None:
    with pytest.raises(BlueprintConfigError, match="PRIMARYMODULE__LABELS"):
        BlueprintConfigParser(PrimaryModule.blueprint()).parse(
            environ={"PRIMARYMODULE__LABELS": '["one"'},
        )


def test_environment_json_object_merges_into_pinned_mapping() -> None:
    class MappingConfig(ModuleConfig):
        extras: dict[str, int] = Field(default_factory=dict)

    class MappingModule(Module):
        config: MappingConfig

    parsed = BlueprintConfigParser(MappingModule.blueprint(extras={"a": 1})).parse(
        environ={"MAPPINGMODULE__EXTRAS": '{"b": 2}'},
    )

    assert parsed.module_kwargs("mappingmodule")["extras"] == {"a": 1, "b": 2}


def test_explicit_global_config_values_exclude_schema_defaults() -> None:
    blueprint = PrimaryModule.blueprint().global_config(viewer="none")

    parsed = BlueprintConfigParser(blueprint).parse(["--n-workers", "3"], environ={})

    assert parsed.explicit_global_config_values() == {"viewer": "none", "n_workers": 3}
    assert parsed.global_config_values()["n_workers"] == 3


def test_required_module_field_can_be_supplied_by_blueprint_pin() -> None:
    class RequiredConfig(ModuleConfig):
        required_value: str

    class RequiredModule(Module):
        config: RequiredConfig

    parsed = BlueprintConfigParser(RequiredModule.blueprint(required_value="pinned")).parse(
        environ={}
    )

    assert parsed.module_kwargs("requiredmodule") == {"required_value": "pinned"}
    with pytest.raises(BlueprintConfigError, match="requiredmodule.required_value"):
        BlueprintConfigParser(RequiredModule.blueprint()).parse(environ={})


def test_constructor_only_blueprint_kwargs_are_not_treated_as_module_config() -> None:
    class ConstructorOnlyModule(Module):
        def __init__(self, resource_name: str, **kwargs: Any) -> None:
            self.resource_name = resource_name
            super().__init__(**kwargs)

    blueprint = ConstructorOnlyModule.blueprint(resource_name="camera")

    parsed = BlueprintConfigParser(blueprint).parse(environ={})

    assert parsed.module_kwargs("constructoronlymodule") == {}
    assert blueprint.blueprints[0].kwargs["resource_name"] == "camera"


def test_discriminated_union_exposes_one_logical_nested_path() -> None:
    class DisabledConfig(BaseModel):
        backend: Literal["disabled"] = "disabled"

    class EnabledConfig(BaseModel):
        backend: Literal["enabled"] = "enabled"
        level: int = 1

    BackendConfig = Annotated[
        DisabledConfig | EnabledConfig,
        Field(discriminator="backend"),
    ]

    class UnionConfig(ModuleConfig):
        backend_config: BackendConfig = Field(default_factory=DisabledConfig)

    class UnionModule(Module):
        config: UnionConfig

    parsed = BlueprintConfigParser(UnionModule.blueprint()).parse(
        [
            "--backend-config.backend",
            "enabled",
            "--backend-config.level",
            "4",
        ],
        environ={},
    )

    assert parsed.module_kwargs("unionmodule") == {
        "backend_config": {"backend": "enabled", "level": 4}
    }


def test_discriminated_union_leaf_override_preserves_default_backend() -> None:
    class FirstConfig(BaseModel):
        backend: Literal["first"] = "first"

    class SecondConfig(BaseModel):
        backend: Literal["second"] = "second"
        level: int = 1

    BackendConfig = Annotated[
        FirstConfig | SecondConfig,
        Field(discriminator="backend"),
    ]

    class UnionConfig(ModuleConfig):
        backend_config: BackendConfig = Field(default_factory=SecondConfig)

    class UnionModule(Module):
        config: UnionConfig

    parsed = BlueprintConfigParser(UnionModule.blueprint()).parse(
        ["--backend-config.level", "4"],
        environ={},
    )

    assert parsed.module_kwargs("unionmodule") == {
        "backend_config": {"backend": "second", "level": 4}
    }


@pytest.mark.parametrize(
    "tokens",
    [
        ("-o", "primarymodule.map_file=map"),
        ("--option", "primarymodule.map_file=map"),
        ("--option=primarymodule.map_file=map",),
    ],
)
def test_legacy_option_syntax_has_migration_error(tokens: tuple[str, ...]) -> None:
    with pytest.raises(BlueprintConfigError, match="legacy -o/--option syntax was removed"):
        BlueprintConfigParser(PrimaryModule.blueprint()).parse(tokens, environ={})


def test_format_help_does_not_call_default_factories_and_hides_reserved_shorthand() -> None:
    calls = 0

    def create_value() -> str:
        nonlocal calls
        calls += 1
        return "created"

    class HelpConfig(ModuleConfig):
        daemon: str = "module"
        generated: str = Field(default_factory=create_value)

    class HelpModule(Module):
        config: HelpConfig

    output = BlueprintConfigParser(HelpModule.blueprint()).format_help(
        reserved_options={"--daemon"}
    )

    assert calls == 0
    assert "--helpmodule.daemon <str>" in output
    assert "  --daemon," not in output
    assert "--generated, --helpmodule.generated <str>" in output
    assert "rpc-transport" not in output


class Anchor:
    def __str__(self) -> str:
        return "Anchor:\n  multi\n  line"


class ArbitraryConfig(ModuleConfig):
    scaling: Anchor = Field(default_factory=Anchor)
    hybrid: Anchor | str = "fallback"
    transform: Callable[[Any], str] | None = None
    handlers: dict[str, Callable[[Any], Any]] = Field(default_factory=dict)
    speed: float = 1.0


class ArbitraryModule(Module):
    config: ArbitraryConfig


def test_arbitrary_type_fields_are_hidden_and_rejected_as_unknown() -> None:
    parser = BlueprintConfigParser(ArbitraryModule.blueprint())

    help_text = parser.format_help()
    assert "scaling" not in help_text
    assert "transform" not in help_text
    assert "handlers" not in help_text
    assert "--hybrid, --arbitrarymodule.hybrid <" in help_text
    assert "Anchor | str> (default: fallback)" in help_text
    assert "--speed" in help_text

    with pytest.raises(
        BlueprintConfigError, match="Unknown blueprint configuration option --scaling"
    ):
        parser.parse(["--scaling", "x"], environ={})

    parsed = parser.parse(["--hybrid", "value"], environ={})
    assert parsed.module_kwargs("arbitrarymodule")["hybrid"] == "value"


def test_blueprint_pinned_arbitrary_value_survives_filtering() -> None:
    parsed = BlueprintConfigParser(ArbitraryModule.blueprint(scaling=Anchor())).parse(environ={})

    assert isinstance(parsed.module_kwargs("arbitrarymodule")["scaling"], Anchor)


def test_format_help_uses_nested_parent_default_instance() -> None:
    class NestedRequiredConfig(BaseModel):
        value: int

    class ParentDefaultConfig(ModuleConfig):
        nested: NestedRequiredConfig = NestedRequiredConfig(value=7)

    class ParentDefaultModule(Module):
        config: ParentDefaultConfig

    output = BlueprintConfigParser(ParentDefaultModule.blueprint()).format_help()
    value_line = next(line for line in output.splitlines() if "--nested.value" in line)

    assert "(default: 7)" in value_line
    assert "[required]" not in value_line


def test_format_help_marks_a_required_nested_parent_with_default_children() -> None:
    class NestedDefaultConfig(BaseModel):
        value: int = 7

    class RequiredParentConfig(ModuleConfig):
        nested: NestedDefaultConfig

    class RequiredParentModule(Module):
        config: RequiredParentConfig

    parser = BlueprintConfigParser(RequiredParentModule.blueprint())
    output = parser.format_help()
    value_line = next(line for line in output.splitlines() if "--nested.value" in line)

    assert "(default: 7) [parent required]" in value_line
    with pytest.raises(BlueprintConfigError, match="nested"):
        parser.parse(environ={})


def test_split_run_arguments_requires_leading_blueprint_names() -> None:
    assert split_run_arguments(("first-blueprint", "second-blueprint", "--map-file", "map")) == (
        ("first-blueprint", "second-blueprint"),
        ("--map-file", "map"),
    )
    with pytest.raises(BlueprintConfigError, match="must precede"):
        split_run_arguments(("--map-file", "map"))

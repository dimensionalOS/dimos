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
from dataclasses import dataclass, field
from typing import Any

from pydantic import BaseModel, Field
import pytest

from dimos.core.coordination.blueprint_config.errors import BlueprintConfigError
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import TransportSpec, autoconnect
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Stream, Transport


class NestedConfig(BaseModel):
    enabled: bool = True
    mode: str = "auto"


class PrimaryConfig(ModuleConfig):
    nested: NestedConfig = Field(default_factory=NestedConfig)
    labels: list[str] = Field(default_factory=list)


class PrimaryModule(Module):
    config: PrimaryConfig


class SecondaryConfig(ModuleConfig):
    count: int = 1


class SecondaryModule(Module):
    config: SecondaryConfig


@dataclass
class RecordingCallback:
    calls: list[int] = field(default_factory=list)

    def __call__(self, value: int) -> int:
        self.calls.append(value)
        return value + 1


class CallbackConfig(BaseModel):
    callbacks: dict[str, Callable[[int], int]]
    enabled: bool = True


class CallbackModuleConfig(ModuleConfig):
    nested: CallbackConfig


class CallbackModule(Module):
    config: CallbackModuleConfig


class CallbackTransport(Transport[int]):
    _config_cls = CallbackConfig

    def broadcast(self, selfstream: Stream[int] | None, value: int) -> None:
        pass

    def subscribe(self, callback: Any, selfstream: Stream[int] | None = None) -> Any:
        return lambda: None


def test_parsed_transport_overrides_preserve_callable_objects() -> None:
    blueprint = PrimaryModule.blueprint().transports(
        {("output", int): TransportSpec(CallbackTransport, {})}
    )
    parsed = BlueprintConfigParser(blueprint).parse(
        environ={},
        overrides={"transports": {"callback": {"callbacks": {"increment": RecordingCallback()}}}},
    )

    config = CallbackConfig(**parsed.transport_overrides()["callback"])
    assert config.callbacks["increment"](4) == 5


@pytest.mark.parametrize("model_input", [False, True])
def test_parsed_config_preserves_nested_callable_objects(model_input: bool) -> None:
    callback = RecordingCallback()
    nested = {"callbacks": {"increment": callback}}
    source = CallbackConfig(**nested) if model_input else nested
    parsed = BlueprintConfigParser(CallbackModule.blueprint(nested=source)).parse(environ={})

    kwargs = parsed.module_kwargs("callbackmodule")
    restored = CallbackModuleConfig(**kwargs)
    assert restored.nested.callbacks["increment"](4) == 5
    assert parsed.module_configs["callbackmodule"]["nested"]["callbacks"]["increment"](8) == 9
    assert callback.calls == []
    assert parsed.module_kwargs("callbackmodule")["nested"]["callbacks"]["increment"].calls == []


def test_dataclass_config_overrides_preserve_nested_callbacks() -> None:
    @dataclass
    class CallbackSettings:
        callbacks: dict[str, Callable[[int], int]]
        enabled: bool = True

    class DataclassConfig(ModuleConfig):
        nested: CallbackSettings

    class DataclassModule(Module):
        config: DataclassConfig

    callback = RecordingCallback()
    blueprint = DataclassModule.blueprint(
        nested=CallbackSettings(callbacks={"increment": callback})
    )
    parsed = BlueprintConfigParser(blueprint).parse(
        overrides={DataclassModule.name: {"nested": {"enabled": False}}},
        environ={},
    )

    config = DataclassConfig(**parsed.module_kwargs(DataclassModule.name))
    assert config.nested.enabled is False
    assert config.nested.callbacks["increment"](4) == 5
    assert callback.calls == []


def test_parsed_config_is_deeply_immutable_and_accessors_return_copies() -> None:
    parsed = BlueprintConfigParser(PrimaryModule.blueprint()).parse(
        ["--labels", '["one"]', "--nested.mode", "manual"],
        environ={},
    )

    with pytest.raises(TypeError):
        parsed.module_configs["primarymodule"] = {}  # type: ignore[index]
    with pytest.raises(TypeError):
        parsed.module_configs["primarymodule"]["nested"]["mode"] = "changed"
    assert parsed.module_configs["primarymodule"]["labels"] == ("one",)

    copy = parsed.module_kwargs("primarymodule")
    copy["nested"]["mode"] = "changed"
    copy["labels"].append("two")
    assert parsed.module_kwargs("primarymodule") == {
        "nested": {"mode": "manual"},
        "labels": ["one"],
    }


def test_parsed_config_isolated_from_mutable_opaque_values() -> None:
    class Box:
        def __init__(self) -> None:
            self.items: list[str] = []

    class OpaqueConfig(ModuleConfig):
        box: Box

    class OpaqueModule(Module):
        config: OpaqueConfig

    source = Box()
    parsed = BlueprintConfigParser(OpaqueModule.blueprint(box=source)).parse(environ={})

    source.items.append("source")
    parsed.module_configs["opaquemodule"]["box"].items.append("public-view")
    accessor_copy = parsed.module_kwargs("opaquemodule")
    accessor_copy["box"].items.append("accessor")

    assert parsed.module_kwargs("opaquemodule")["box"].items == []


def test_parsed_config_preserves_container_types_and_mapping_keys() -> None:
    class AnyConfig(ModuleConfig):
        value: Any

    class AnyModule(Module):
        config: AnyConfig

    source = {
        "tuple": ("one",),
        "frozenset": frozenset({"two"}),
        "mapping": {1: "integer key"},
    }
    parsed_configs = (
        BlueprintConfigParser(AnyModule.blueprint(value=source)).parse(environ={}),
        BlueprintConfigParser(AnyModule.blueprint()).parse(
            environ={},
            overrides={"anymodule": {"value": source}},
        ),
    )

    for parsed in parsed_configs:
        value = parsed.module_kwargs("anymodule")["value"]
        assert value["tuple"] == ("one",)
        assert isinstance(value["tuple"], tuple)
        assert value["frozenset"] == frozenset({"two"})
        assert isinstance(value["frozenset"], frozenset)
        assert value["mapping"] == {1: "integer key"}


def test_parsed_config_rejects_a_different_blueprint_identity() -> None:
    blueprint = PrimaryModule.blueprint()
    parsed = BlueprintConfigParser(blueprint).parse(environ={})

    parsed.assert_matches(blueprint)
    with pytest.raises(BlueprintConfigError, match="different blueprint"):
        parsed.assert_matches(PrimaryModule.blueprint())


def test_subset_binds_only_local_module_configuration() -> None:
    blueprint = autoconnect(PrimaryModule.blueprint(), SecondaryModule.blueprint())
    parsed = BlueprintConfigParser(blueprint).parse(
        environ={},
        overrides={
            "primarymodule": {"labels": ["local"]},
            "secondarymodule": {"count": 3},
        },
    )
    local_blueprint = PrimaryModule.blueprint()

    subset = parsed.subset_for(
        local_blueprint,
        global_overrides={"transport": "zenoh"},
    )

    subset.assert_matches(local_blueprint)
    assert set(subset.module_configs) == {"primarymodule"}
    assert subset.module_kwargs("primarymodule")["labels"] == ["local"]
    assert subset.global_config["transport"] == "zenoh"
    with pytest.raises(KeyError, match="secondarymodule"):
        subset.module_kwargs("secondarymodule")

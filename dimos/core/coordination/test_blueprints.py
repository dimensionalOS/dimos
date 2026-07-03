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


import pickle
from types import MappingProxyType
from typing import Protocol, get_type_hints

from pydantic import ValidationError
import pytest

from dimos.core._test_future_annotations_helper import (
    FutureData,
    FutureModuleIn,
    FutureModuleOut,
)
from dimos.core.coordination.blueprints import (
    Blueprint,
    BlueprintAtom,
    DisabledModuleProxy,
    ModuleRef,
    StreamRef,
    autoconnect,
    namespace,
)
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.core.transport import LCMTransport
from dimos.spec.utils import Spec


class Scratch:
    pass


class Petting:
    pass


class CatModule(Module):
    pet_cat: In[Petting]
    scratches: Out[Scratch]


class Data1:
    pass


class Data2:
    pass


class Data3:
    pass


class ModuleA(Module):
    data1: Out[Data1]
    data2: Out[Data2]

    @rpc
    def get_name(self) -> str:
        return "A, Module A"


class ModuleB(Module):
    data1: In[Data1]
    data2: In[Data2]
    data3: Out[Data3]

    module_a: ModuleA

    @rpc
    def what_is_as_name(self) -> str:
        return self.module_a.get_name()


def test_get_connection_set() -> None:
    assert BlueprintAtom.create(CatModule, kwargs={"k": "v"}) == BlueprintAtom(
        module=CatModule,
        streams=(
            StreamRef(name="pet_cat", type=Petting, direction="in"),
            StreamRef(name="scratches", type=Scratch, direction="out"),
        ),
        module_refs=(),
        kwargs={"k": "v"},
    )


def test_autoconnect() -> None:
    blueprint_set = autoconnect(ModuleA.blueprint(), ModuleB.blueprint())

    assert blueprint_set == Blueprint(
        blueprints=(
            BlueprintAtom(
                module=ModuleA,
                streams=(
                    StreamRef(name="data1", type=Data1, direction="out"),
                    StreamRef(name="data2", type=Data2, direction="out"),
                ),
                module_refs=(),
                kwargs={},
            ),
            BlueprintAtom(
                module=ModuleB,
                streams=(
                    StreamRef(name="data1", type=Data1, direction="in"),
                    StreamRef(name="data2", type=Data2, direction="in"),
                    StreamRef(name="data3", type=Data3, direction="out"),
                ),
                module_refs=(ModuleRef(name="module_a", spec=ModuleA),),
                kwargs={},
            ),
        )
    )


def test_config() -> None:
    blueprint = autoconnect(ModuleA.blueprint(), ModuleB.blueprint())
    config = blueprint.config()
    assert config.model_fields.keys() == {"modulea", "moduleb", "g"}
    assert config.model_fields["modulea"].annotation == get_type_hints(ModuleA)["config"] | None
    assert config.model_fields["moduleb"].annotation == get_type_hints(ModuleB)["config"] | None

    with pytest.raises(ValidationError, match="invalid_key"):
        config(module_a={"invalid_key": 5})


def test_transports() -> None:
    custom_transport = LCMTransport("/custom_topic", Data1)
    blueprint_set = autoconnect(ModuleA.blueprint(), ModuleB.blueprint()).transports(
        {("data1", Data1): custom_transport}
    )

    assert ("data1", Data1) in blueprint_set.transport_map
    assert blueprint_set.transport_map[("data1", Data1)] == custom_transport


def test_global_config() -> None:
    blueprint_set = autoconnect(ModuleA.blueprint(), ModuleB.blueprint()).global_config(
        option1=True, option2=42
    )

    assert "option1" in blueprint_set.global_config_overrides
    assert blueprint_set.global_config_overrides["option1"] is True
    assert "option2" in blueprint_set.global_config_overrides
    assert blueprint_set.global_config_overrides["option2"] == 42


def test_future_annotations_support() -> None:
    """Test that modules using `from __future__ import annotations` work correctly.

    PEP 563 (future annotations) stores annotations as strings instead of actual types.
    This test verifies that BlueprintAtom.create properly resolves string annotations
    to the actual In/Out types.
    """

    # Test that streams are properly extracted from modules with future annotations
    out_blueprint = BlueprintAtom.create(FutureModuleOut, kwargs={})
    assert len(out_blueprint.streams) == 1
    assert out_blueprint.streams[0] == StreamRef(name="data", type=FutureData, direction="out")

    in_blueprint = BlueprintAtom.create(FutureModuleIn, kwargs={})
    assert len(in_blueprint.streams) == 1
    assert in_blueprint.streams[0] == StreamRef(name="data", type=FutureData, direction="in")


def test_autoconnect_merges_disabled_modules() -> None:
    bp_a = Blueprint(
        blueprints=ModuleA.blueprint().blueprints,
        disabled_modules_tuple=(ModuleA,),
    )
    bp_b = Blueprint(
        blueprints=ModuleB.blueprint().blueprints,
        disabled_modules_tuple=(ModuleB,),
    )

    merged = autoconnect(bp_a, bp_b)
    assert merged.disabled_modules_tuple == (ModuleA, ModuleB)


class CalcSpec(Spec, Protocol):
    @rpc
    def compute(self, a: int, b: int) -> int: ...


class ModuleWithOptionalRef(Module):
    data1: In[Data1]
    calc: CalcSpec | None = None


def test_optional_module_ref_detected() -> None:
    atom = BlueprintAtom.create(ModuleWithOptionalRef, kwargs={})
    assert len(atom.module_refs) == 1
    ref = atom.module_refs[0]
    assert ref.name == "calc"
    assert ref.optional is True


def test_autoconnect_eliminates_duplicates_keeps_newer() -> None:
    bp1 = Blueprint.create(ModuleA, key1="old")
    bp2 = Blueprint.create(ModuleA, key1="new")

    merged = autoconnect(bp1, bp2)

    module_a_atoms = [a for a in merged.blueprints if a.module is ModuleA]
    assert len(module_a_atoms) == 1
    assert module_a_atoms[0].kwargs == {"key1": "new"}


def test_disabled_module_proxy_pickle_roundtrip() -> None:
    proxy = DisabledModuleProxy("SomeSpec")
    restored = pickle.loads(pickle.dumps(proxy))

    assert repr(restored) == "<DisabledModuleProxy spec=SomeSpec>"
    assert restored.any_method(1, 2, 3) is None


def test_blueprint_pickle_roundtrip() -> None:
    blueprint = (
        autoconnect(ModuleA.blueprint(), ModuleB.blueprint())
        .global_config(option1=True, option2=42)
        .remappings([(ModuleA, "module_a", ModuleB)])
    )

    restored = pickle.loads(pickle.dumps(blueprint))

    assert restored == blueprint
    for name in ("transport_map", "global_config_overrides", "remapping_map"):
        assert isinstance(getattr(restored, name), MappingProxyType)
    assert dict(restored.global_config_overrides) == {"option1": True, "option2": 42}
    assert restored.remapping_map[(ModuleA.name, "module_a")] is ModuleB
    with pytest.raises(TypeError):
        restored.global_config_overrides["x"] = 1


def test_active_blueprints_filters_disabled() -> None:
    blueprint = autoconnect(ModuleA.blueprint(), ModuleB.blueprint()).disabled_modules(ModuleA)

    active_modules = {bp.module for bp in blueprint.active_blueprints}
    assert ModuleA not in active_modules
    assert ModuleB in active_modules


def test_namespace_prefixes_names_streams_and_frames() -> None:
    blueprint = namespace("robot0", ModuleA.blueprint(), ModuleB.blueprint())

    atom_a = next(a for a in blueprint.blueprints if a.module is ModuleA)
    assert atom_a.name == "robot0/modulea"
    assert atom_a.kwargs["instance_name"] == "robot0/modulea"
    assert atom_a.kwargs["frame_id_prefix"] == "robot0"
    # Every stream is remapped under the prefix.
    assert blueprint.remapping_map[("robot0/modulea", "data1")] == "robot0/data1"
    assert blueprint.remapping_map[("robot0/moduleb", "data3")] == "robot0/data3"


def test_namespace_expose_keeps_names_global() -> None:
    blueprint = namespace("robot0", ModuleA.blueprint(), expose={"data1"})

    assert ("robot0/modulea", "data1") not in blueprint.remapping_map
    assert blueprint.remapping_map[("robot0/modulea", "data2")] == "robot0/data2"


def test_namespace_expose_typo_raises() -> None:
    with pytest.raises(ValueError, match="data_typo"):
        namespace("robot0", ModuleA.blueprint(), expose={"data_typo"})


def test_namespace_invalid_prefix_raises() -> None:
    with pytest.raises(ValueError, match="Invalid namespace prefix"):
        namespace("a/b", ModuleA.blueprint())


def test_namespace_nesting_composes() -> None:
    blueprint = namespace("fleet", namespace("robot0", ModuleA.blueprint()))

    atom = blueprint.blueprints[0]
    assert atom.name == "fleet/robot0/modulea"
    assert atom.kwargs["frame_id_prefix"] == "fleet/robot0"
    assert blueprint.remapping_map[("fleet/robot0/modulea", "data1")] == "fleet/robot0/data1"


def test_namespace_keeps_user_frame_id_prefix() -> None:
    blueprint = namespace("robot0", ModuleA.blueprint(frame_id_prefix="custom"))

    assert blueprint.blueprints[0].kwargs["frame_id_prefix"] == "custom"


def test_namespace_allows_duplicate_module_classes() -> None:
    blueprint = autoconnect(
        namespace("robot0", ModuleA.blueprint(key1="a")),
        namespace("robot1", ModuleA.blueprint(key1="b")),
    )

    atoms = [a for a in blueprint.blueprints if a.module is ModuleA]
    assert {a.name for a in atoms} == {"robot0/modulea", "robot1/modulea"}
    # Later-wins dedup still applies per instance name.
    merged = autoconnect(blueprint, namespace("robot1", ModuleA.blueprint(key1="c")))
    robot1 = next(a for a in merged.blueprints if a.name == "robot1/modulea")
    assert robot1.kwargs["key1"] == "c"


def test_namespace_config_keys_escaped() -> None:
    blueprint = autoconnect(
        namespace("robot0", ModuleA.blueprint()),
        namespace("robot1", ModuleA.blueprint()),
    )
    config = blueprint.config()
    assert {"robot0_modulea", "robot1_modulea", "g"} <= set(config.model_fields.keys())


def test_explicit_instance_name_sets_blueprint_identity() -> None:
    blueprint = ModuleA.blueprint(instance_name="custom/modulea")

    atom = blueprint.blueprints[0]
    assert atom.name == "custom/modulea"
    assert atom.instance_name == "custom/modulea"
    assert set(blueprint.config().model_fields) == {"custom_modulea", "g"}


def test_namespace_prefixes_pinned_transports() -> None:
    blueprint = namespace(
        "robot0",
        autoconnect(ModuleA.blueprint(), ModuleB.blueprint()).transports(
            {("data1", Data1): LCMTransport("/custom_topic", Data1)}
        ),
    )

    transport = blueprint.transport_map[("robot0/data1", Data1)]
    assert transport.topic.pattern == "/robot0/custom_topic"
    assert ("data1", Data1) not in blueprint.transport_map


def test_namespace_remappings_by_instance_name() -> None:
    # A remapping added after namespacing can target an instance by name.
    blueprint = autoconnect(
        namespace("robot0", ModuleA.blueprint()),
        namespace("robot1", ModuleA.blueprint()),
    ).remappings([("robot0/modulea", "data1", "special_data")])

    assert blueprint.remapping_map[("robot0/modulea", "data1")] == "special_data"
    # Targeting the class is ambiguous with two instances.
    with pytest.raises(ValueError, match="multiple instances"):
        autoconnect(
            namespace("robot0", ModuleA.blueprint()),
            namespace("robot1", ModuleA.blueprint()),
        ).remappings([(ModuleA, "data1", "other")])

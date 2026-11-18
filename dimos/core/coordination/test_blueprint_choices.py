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

from typing import Any

import pytest
from pytest_mock import MockerFixture

from dimos.core.coordination.blueprints import (
    Blueprint,
    ChoiceAtom,
    ChoiceResolutionError,
    autoconnect,
)
from dimos.core.module import Module
from dimos.core.stream import In, Out, Transport, TransportSpec
from dimos.core.transport import LCMTransport
from dimos.protocol.encode import cbor
from dimos.protocol.service.system_configurator.base import SystemConfigurator


class VisModule(Module):
    points: In[int]


class RaytracerModule(Module):
    points: Out[int]


class VoxelMapper(Module):
    points: Out[int]


def _clearing() -> ChoiceAtom:
    return ChoiceAtom(
        name="PointClearing",
        default="raycasting",
        choices={
            "raycasting": RaytracerModule.blueprint(),
            "column_carving": VoxelMapper.blueprint(),
        },
    )


@pytest.fixture
def clearing() -> ChoiceAtom:
    return _clearing()


def test_blueprint_without_choices_is_already_solid() -> None:
    composed = autoconnect(VisModule.blueprint())

    assert composed == VisModule.blueprint()
    assert composed.solidify() is composed
    assert autoconnect().solidify() == Blueprint(blueprints=())


@pytest.mark.parametrize(
    ("choices", "selected"),
    [({}, RaytracerModule), ({"PointClearing": "column_carving"}, VoxelMapper)],
)
def test_selects_only_the_requested_fragment(
    clearing: ChoiceAtom, choices: dict[str, str], selected: type[Module]
) -> None:
    mapping = autoconnect(VisModule.blueprint(), clearing)

    resolved = mapping.solidify(choices=choices)

    assert resolved == autoconnect(VisModule.blueprint(), selected.blueprint()).solidify()
    # Resolving an override must not replace the default for later calls.
    assert (
        mapping.solidify()
        == autoconnect(VisModule.blueprint(), RaytracerModule.blueprint()).solidify()
    )


def test_nested_composition_resolves_multiple_choices_at_the_root(clearing: ChoiceAtom) -> None:
    inner = autoconnect(clearing)
    mapping = autoconnect(
        inner,
        ChoiceAtom(
            name="Viewer",
            choices={"on": VisModule.blueprint(), "off": autoconnect()},
        ),
    )

    resolved = mapping.solidify(choices={"PointClearing": "column_carving", "Viewer": "off"})

    assert resolved == VoxelMapper.blueprint()


@pytest.mark.parametrize(
    ("choices", "expected"),
    [
        ({}, VisModule.blueprint()),
        ({"Mapping": "on", "Required": "voxel"}, VoxelMapper.blueprint()),
    ],
)
def test_only_selected_alternatives_require_nested_choices(
    choices: dict[str, str], expected: Blueprint
) -> None:
    mapping = autoconnect(
        ChoiceAtom(
            name="Mapping",
            default="off",
            choices={
                "off": VisModule.blueprint(),
                "on": autoconnect(
                    ChoiceAtom(name="Required", choices={"voxel": VoxelMapper.blueprint()})
                ),
            },
        )
    )

    assert mapping.solidify(choices=choices) == expected


def test_choice_without_a_default_picks_its_first_option() -> None:
    choice = ChoiceAtom(
        name="Clearing",
        choices={"voxel": VoxelMapper.blueprint(), "ray": RaytracerModule.blueprint()},
    )

    assert choice.default == "voxel"
    assert autoconnect(choice).solidify() == VoxelMapper.blueprint()


@pytest.mark.parametrize(
    ("choices", "message"),
    [
        ({"PointClearing": "typo"}, "Invalid option.*typo.*PointClearing.*raycasting"),
        ({"Misspelled": "raycasting"}, "Unknown or inactive choices: Misspelled"),
    ],
)
def test_rejects_invalid_overrides(
    clearing: ChoiceAtom, choices: dict[str, str], message: str
) -> None:
    with pytest.raises(ChoiceResolutionError, match=message):
        autoconnect(clearing).solidify(choices=choices)


def test_rejects_overrides_for_inactive_choices(clearing: ChoiceAtom) -> None:
    mapping = autoconnect(
        ChoiceAtom(
            name="Mapping",
            default="off",
            choices={"off": autoconnect(), "on": autoconnect(clearing)},
        )
    )

    with pytest.raises(ChoiceResolutionError, match="inactive choices: PointClearing"):
        mapping.solidify(choices={"PointClearing": "column_carving"})


def test_rejects_distinct_active_choices_with_the_same_name(clearing: ChoiceAtom) -> None:
    other = ChoiceAtom(name="PointClearing", choices={"none": autoconnect()}, default="none")

    with pytest.raises(ChoiceResolutionError, match="Duplicate choice name: 'PointClearing'"):
        autoconnect(clearing, autoconnect(other)).solidify()
    with pytest.raises(ChoiceResolutionError, match="Duplicate choice name: 'PointClearing'"):
        autoconnect(clearing, _clearing()).solidify()


def test_shared_choice_stays_one_choice_through_operations(clearing: ChoiceAtom) -> None:
    mapping = autoconnect(
        autoconnect(clearing).namespace("left"), autoconnect(clearing).global_config(n_workers=2)
    )

    resolved = mapping.solidify(choices={"PointClearing": "column_carving"})

    assert [atom.name for atom in resolved.blueprints] == ["left/voxelmapper", "voxelmapper"]


def test_shared_choice_can_be_reused_in_compositions(clearing: ChoiceAtom) -> None:
    common = autoconnect(clearing)
    mapping = autoconnect(autoconnect(common, VisModule.blueprint()), common)

    assert (
        mapping.solidify(choices={"PointClearing": "column_carving"})
        == autoconnect(VisModule.blueprint(), VoxelMapper.blueprint()).solidify()
    )


def test_option_keeps_its_own_remappings_and_placement() -> None:
    selected = (
        RaytracerModule.blueprint(tick_rate=2)
        .hosted(host="selected")
        .remappings([(RaytracerModule, "points", "cleared")])
    )
    unselected = VoxelMapper.blueprint().hosted(host="unused")
    mapping = autoconnect(
        VisModule.blueprint(),
        ChoiceAtom(name="Clearing", choices={"ray": selected, "voxel": unselected}),
    )

    assert mapping.solidify() == autoconnect(VisModule.blueprint(), selected)
    assert mapping.solidify(choices={"Clearing": "voxel"}) == autoconnect(
        VisModule.blueprint(), unselected
    )


@pytest.mark.parametrize(
    "parts",
    [
        [ChoiceAtom(name="C", choices={"a": VisModule.blueprint(), "b": VisModule.blueprint()})],
        [VisModule.blueprint(), ChoiceAtom(name="C", choices={"a": VisModule.blueprint()})],
    ],
)
def test_rejects_a_module_under_two_conditions(parts: list[Blueprint | ChoiceAtom]) -> None:
    with pytest.raises(ChoiceResolutionError, match="'vismodule'.*own instance_name"):
        autoconnect(*parts)


def test_rejects_blueprint_wide_settings_inside_an_option() -> None:
    option = VoxelMapper.blueprint().global_config(n_workers=8)

    with pytest.raises(ChoiceResolutionError, match="'voxel'.*outside the choice"):
        autoconnect(ChoiceAtom(name="C", choices={"voxel": option}))


@pytest.mark.parametrize(
    ("choices", "selected"),
    [({}, RaytracerModule), ({"PointClearing": "column_carving"}, VoxelMapper)],
)
def test_fluent_operations_run_after_selecting_modules(
    clearing: ChoiceAtom, choices: dict[str, str], selected: type[Module], mocker: MockerFixture
) -> None:
    requirement = mocker.Mock(return_value=None)
    configurator = mocker.create_autospec(SystemConfigurator, instance=True)
    mapping = (
        autoconnect(VisModule.blueprint(), clearing)
        .disabled_modules(VisModule)
        .transports({("points", int): LCMTransport.spec("/points", int)})
        .remappings([(selected, "points", "cleared")])
        .namespace("robot")
        .hosted(host="edge")
        .global_config(n_workers=2)
        .requirements(requirement)
        .configurators(configurator)
    )

    resolved = mapping.solidify(choices=choices)

    assert [atom.module for atom in resolved.active_blueprints] == [selected]
    assert resolved.remapping_map[(f"robot/{selected.name}", "points")] == "robot/cleared"
    assert set(resolved.transport_map) == {("robot/points", int)}
    assert resolved.hosted_placements[0].module_names == (
        "robot/vismodule",
        f"robot/{selected.name}",
    )
    assert resolved.hosted_placements[0].host == "edge"
    assert resolved.global_config_overrides == {"n_workers": 2}
    assert resolved.requirement_checks == (requirement,)
    assert resolved.configurator_checks == (configurator,)
    requirement.assert_not_called()
    configurator.check.assert_not_called()


def test_operations_keep_their_fragment_scope_and_call_order(clearing: ChoiceAtom) -> None:
    inner = autoconnect(clearing).global_config(n_workers=2)
    hosted = inner.hosted(host="edge").namespace("mapper")
    mapping = autoconnect(VisModule.blueprint(), hosted).global_config(n_workers=3)
    override = mapping.global_config(n_workers=4)

    resolved = override.solidify(choices={"PointClearing": "column_carving"})

    assert [atom.name for atom in resolved.blueprints] == ["vismodule", "mapper/voxelmapper"]
    assert resolved.hosted_placements[0].module_names == ("mapper/voxelmapper",)
    assert resolved.global_config_overrides == {"n_workers": 4}
    assert mapping.solidify().global_config_overrides == {"n_workers": 3}
    assert inner.solidify() == RaytracerModule.blueprint().global_config(n_workers=2)


def test_choices_can_be_configured_before_selection() -> None:
    mapping = (
        autoconnect(ChoiceAtom(name="Only", choices={"voxel": VoxelMapper.blueprint()}))
        .global_config(n_workers=4)
        .hosted(host="edge")
    )

    assert mapping.solidify() == (
        VoxelMapper.blueprint().global_config(n_workers=4).hosted(host="edge")
    )


def test_operations_are_validated_where_they_are_called() -> None:
    with pytest.raises(ValueError, match="hosted.*at least one module"):
        autoconnect(ChoiceAtom(name="Clearing", choices={"off": autoconnect()})).hosted()
    with pytest.raises(ValueError, match="expose names.*typo"):
        autoconnect(VisModule.blueprint(), _clearing()).namespace("robot", expose=["typo"])
    with pytest.raises(ValueError, match="RaytracerModule has multiple instances"):
        autoconnect(
            RaytracerModule.blueprint(),
            ChoiceAtom(
                name="Second",
                choices={"on": RaytracerModule.blueprint().namespace("other")},
                default="on",
            ),
        ).remappings([(RaytracerModule, "points", "cleared")])


def test_expose_may_name_a_stream_that_only_an_option_has() -> None:
    mapping = autoconnect(_clearing()).namespace("robot", expose=["points"])

    assert mapping.solidify().remapping_map == {}


def test_arguments_are_snapshotted_and_reusable(clearing: ChoiceAtom) -> None:
    transports: dict[tuple[str, type], TransportSpec | Transport[Any]] = {
        ("points", int): LCMTransport.spec("/points", int)
    }
    remappings = [(VisModule, "points", "cleared")]
    tags = {"gpu": True}
    minimum = {"gpu_mem": 4.0}
    mapping = (
        autoconnect(VisModule.blueprint(), clearing)
        .transports(transports)
        .remappings(remappings)
        .namespace("robot", expose=iter(["cleared"]))
        .hosted(tags=tags, minimum=minimum)
    )
    first = mapping.solidify()
    transports.clear()
    remappings.clear()
    tags.clear()
    minimum.clear()

    assert mapping.solidify() == first
    selected = mapping.solidify(choices={"PointClearing": "column_carving"})
    assert selected.remapping_map[("robot/vismodule", "points")] == "cleared"
    assert selected.hosted_placements[0].tags == {"gpu": True}
    assert selected.hosted_placements[0].minimum == {"gpu_mem": 4.0}


@pytest.mark.parametrize(
    ("name", "choices", "default", "message"),
    [
        ("", {"a": autoconnect()}, None, "Invalid choice name ''"),
        ("Invalid=Name", {"a": autoconnect()}, None, "Invalid choice name 'Invalid=Name'"),
        ("robot/clearing", {"a": autoconnect()}, None, "letters, digits and underscores"),
        ("Empty", {}, None, "nonempty option names"),
        ("Empty", {"": autoconnect()}, None, "nonempty option names"),
        ("Default", {"a": autoconnect()}, "b", "Invalid default.*b.*Default.*a"),
    ],
)
def test_rejects_invalid_choice_definitions(
    name: str, choices: dict[str, Blueprint], default: str | None, message: str
) -> None:
    with pytest.raises(ChoiceResolutionError, match=message):
        ChoiceAtom(name=name, choices=choices, default=default)


def test_unsolidified_blueprint_round_trips_through_cbor(clearing: ChoiceAtom) -> None:
    mapping = autoconnect(VisModule.blueprint(), clearing).namespace("robot")

    decoded = cbor.decode(cbor.encode(mapping))

    assert decoded == mapping
    assert decoded.solidify(choices={"PointClearing": "column_carving"}) == mapping.solidify(
        choices={"PointClearing": "column_carving"}
    )

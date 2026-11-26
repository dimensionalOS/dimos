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

import pytest
from pytest_mock import MockerFixture

from dimos.core.coordination.blueprints import (
    Blueprint,
    ChoiceAtom,
    ChoiceResolutionError,
    MetaBlueprint,
    autoconnect,
)
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.core.transport import LCMTransport
from dimos.protocol.service.system_configurator.base import SystemConfigurator


class VisModule(Module):
    points: In[int]


class RaytracerModule(Module):
    points: Out[int]


class VoxelMapper(Module):
    points: Out[int]


@pytest.fixture
def clearing() -> ChoiceAtom:
    return ChoiceAtom(
        name="PointClearing",
        default="raycasting",
        choices={
            "raycasting": RaytracerModule.blueprint(),
            "column_carving": VoxelMapper.blueprint(),
        },
    )


def test_autoconnect_without_choices_requires_solidification() -> None:
    composed = autoconnect(VisModule.blueprint())

    assert isinstance(composed, MetaBlueprint)
    assert composed.solidify() == VisModule.blueprint()
    assert autoconnect().solidify() == Blueprint(blueprints=())


@pytest.mark.parametrize(
    ("choices", "selected"),
    [({}, RaytracerModule), ({"PointClearing": "column_carving"}, VoxelMapper)],
)
def test_selects_only_the_requested_fragment(clearing, choices, selected) -> None:
    mapping = autoconnect(VisModule.blueprint(), clearing)

    resolved = mapping.solidify(choices=choices)

    assert resolved == autoconnect(VisModule.blueprint(), selected.blueprint()).solidify()
    # Resolving an override must not replace the default for later calls.
    assert (
        mapping.solidify()
        == autoconnect(VisModule.blueprint(), RaytracerModule.blueprint()).solidify()
    )


def test_nested_composition_resolves_multiple_choices_at_the_root(clearing) -> None:
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
def test_only_selected_alternatives_require_nested_choices(choices, expected) -> None:
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


def test_required_choice_errors_only_when_solidified() -> None:
    mapping = autoconnect(ChoiceAtom(name="Required", choices={"voxel": VoxelMapper.blueprint()}))

    with pytest.raises(ChoiceResolutionError, match="Required.*required.*voxel"):
        mapping.solidify()

    assert mapping.solidify(choices={"Required": "voxel"}) == VoxelMapper.blueprint()


@pytest.mark.parametrize(
    ("choices", "message"),
    [
        ({"PointClearing": "typo"}, "Invalid option.*typo.*PointClearing.*raycasting"),
        ({"Misspelled": "raycasting"}, "Unknown or inactive choices: Misspelled"),
    ],
)
def test_rejects_invalid_overrides(clearing, choices, message) -> None:
    with pytest.raises(ChoiceResolutionError, match=message):
        autoconnect(clearing).solidify(choices=choices)


def test_rejects_overrides_for_inactive_choices(clearing) -> None:
    mapping = autoconnect(
        ChoiceAtom(
            name="Mapping",
            default="off",
            choices={"off": autoconnect(), "on": autoconnect(clearing)},
        )
    )

    with pytest.raises(ChoiceResolutionError, match="inactive choices: PointClearing"):
        mapping.solidify(choices={"PointClearing": "column_carving"})


def test_rejects_distinct_active_choices_with_the_same_name(clearing) -> None:
    other = ChoiceAtom(name="PointClearing", choices={"none": autoconnect()}, default="none")

    with pytest.raises(ChoiceResolutionError, match="Duplicate choice name: 'PointClearing'"):
        autoconnect(clearing, autoconnect(other)).solidify()


def test_shared_choice_can_be_reused_in_compositions(clearing) -> None:
    common = autoconnect(clearing)
    mapping = autoconnect(autoconnect(common, VisModule.blueprint()), common)

    assert (
        mapping.solidify(choices={"PointClearing": "column_carving"})
        == autoconnect(VisModule.blueprint(), VoxelMapper.blueprint()).solidify()
    )


def test_selected_branch_preserves_concrete_blueprint_behavior() -> None:
    base = autoconnect(
        VisModule.blueprint(), RaytracerModule.blueprint(tick_rate=1).hosted(host="old")
    ).solidify()
    selected = (
        RaytracerModule.blueprint(tick_rate=2)
        .hosted(host="selected")
        .global_config(n_workers=2)
        .transports({("points", int): LCMTransport.spec("/selected", int)})
        .remappings([(RaytracerModule, "points", "cleared")])
    )
    unselected = VoxelMapper.blueprint().global_config(n_workers=99).hosted(host="unused")
    mapping = autoconnect(
        base,
        ChoiceAtom(name="Clearing", choices={"ray": selected, "voxel": unselected}, default="ray"),
    )

    assert mapping.solidify() == autoconnect(base, selected).solidify()


@pytest.mark.parametrize(
    ("choices", "selected"),
    [({}, RaytracerModule), ({"PointClearing": "column_carving"}, VoxelMapper)],
)
def test_fluent_operations_run_after_selecting_modules(clearing, choices, selected, mocker) -> None:
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


def test_operations_keep_their_fragment_scope_and_call_order(clearing) -> None:
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


def test_required_choices_can_be_configured_before_selection() -> None:
    mapping = (
        autoconnect(ChoiceAtom(name="Required", choices={"voxel": VoxelMapper.blueprint()}))
        .global_config(n_workers=4)
        .hosted(host="edge")
    )

    with pytest.raises(ChoiceResolutionError, match="Required.*required"):
        mapping.solidify()

    assert mapping.solidify(choices={"Required": "voxel"}) == (
        VoxelMapper.blueprint().global_config(n_workers=4).hosted(host="edge")
    )


def test_unselected_operations_are_not_validated() -> None:
    mapping = autoconnect(
        ChoiceAtom(
            name="Clearing",
            choices={"on": autoconnect(VoxelMapper.blueprint()), "off": autoconnect().hosted()},
            default="on",
        )
    )

    assert mapping.solidify() == VoxelMapper.blueprint()
    with pytest.raises(ValueError, match="hosted.*at least one module"):
        mapping.solidify(choices={"Clearing": "off"})


def test_deferred_arguments_are_snapshotted_and_reusable(clearing) -> None:
    transports = {("points", int): LCMTransport.spec("/points", int)}
    remappings = [(VisModule, "points", "cleared")]
    tags = {"gpu": True}
    minimum = {"gpu_mem": 4.0}
    mapping = (
        autoconnect(VisModule.blueprint(), clearing)
        .transports(transports)
        .remappings(remappings)
        .namespace("robot", shared=iter(["cleared"]))
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
        ("", {"a": autoconnect()}, None, "names must be nonempty"),
        ("Invalid=Name", {"a": autoconnect()}, None, "cannot contain '='"),
        ("Empty", {}, None, "nonempty option names"),
        ("Empty", {"": autoconnect()}, None, "nonempty option names"),
        ("Default", {"a": autoconnect()}, "b", "Invalid default.*b.*Default.*a"),
    ],
)
def test_rejects_invalid_choice_definitions(name, choices, default, message) -> None:
    with pytest.raises(ChoiceResolutionError, match=message):
        ChoiceAtom(name=name, choices=choices, default=default)


def test_build_passes_only_a_solid_blueprint_to_the_coordinator(
    clearing, mocker: MockerFixture
) -> None:
    build = mocker.patch.object(ModuleCoordinator, "build")

    coordinator = autoconnect(clearing).build(choices={"PointClearing": "column_carving"})

    build.assert_called_once_with(VoxelMapper.blueprint())
    assert coordinator is build.return_value


def test_build_rejects_choices_before_starting_the_coordinator(
    clearing, mocker: MockerFixture
) -> None:
    build = mocker.patch.object(ModuleCoordinator, "build")

    with pytest.raises(ChoiceResolutionError, match="Invalid option"):
        autoconnect(clearing).build(choices={"PointClearing": "typo"})

    build.assert_not_called()

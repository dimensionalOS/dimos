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
    ChoiceAtom,
    ChoiceResolutionError,
    autoconnect,
)
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.module import Module
from dimos.core.stream import In, Out


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


def test_plain_blueprints_are_untouched() -> None:
    blueprint = autoconnect(VisModule.blueprint(), RaytracerModule.blueprint())

    assert blueprint.pending == ()
    assert blueprint.solidify() is blueprint
    assert [atom.module for atom in blueprint.active_blueprints] == [VisModule, RaytracerModule]
    with pytest.raises(ChoiceResolutionError, match="Unknown or inactive choices: Viewer"):
        blueprint.solidify(choices={"Viewer": "on"})


@pytest.mark.parametrize(
    ("choices", "selected"),
    [({}, RaytracerModule), ({"PointClearing": "column_carving"}, VoxelMapper)],
)
def test_selects_only_the_requested_fragment(clearing, choices, selected) -> None:
    mapping = autoconnect(VisModule.blueprint(), clearing)

    assert mapping.solidify(choices=choices) == autoconnect(
        VisModule.blueprint(), selected.blueprint()
    )
    with pytest.raises(ChoiceResolutionError, match="unresolved choices"):
        list(mapping.active_blueprints)


def test_later_parts_still_override_a_fragment_with_choices(clearing) -> None:
    mapping = autoconnect(clearing, RaytracerModule.blueprint(tick_rate=2))

    resolved = mapping.solidify()

    assert [atom.kwargs for atom in resolved.blueprints] == [{"tick_rate": 2}]


def test_nested_choices_resolve_at_the_root(clearing) -> None:
    mapping = autoconnect(
        autoconnect(clearing),
        ChoiceAtom(name="Viewer", choices={"on": VisModule.blueprint(), "off": autoconnect()}),
    )

    resolved = mapping.solidify(choices={"PointClearing": "column_carving", "Viewer": "off"})

    assert resolved == VoxelMapper.blueprint()


@pytest.mark.parametrize(
    ("choices", "expected"),
    [({}, VisModule), ({"Mapping": "on", "Required": "voxel"}, VoxelMapper)],
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

    assert mapping.solidify(choices=choices) == expected.blueprint()


def test_choice_without_default_must_be_selected() -> None:
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
def test_rejects_invalid_selections(clearing, choices, message) -> None:
    with pytest.raises(ChoiceResolutionError, match=message):
        autoconnect(clearing).solidify(choices=choices)


def test_rejects_distinct_choices_with_the_same_name(clearing) -> None:
    other = ChoiceAtom(name="PointClearing", choices={"none": autoconnect()}, default="none")

    with pytest.raises(ChoiceResolutionError, match="Duplicate choice name: 'PointClearing'"):
        autoconnect(clearing, autoconnect(other))


def test_operations_keep_their_fragment_scope_and_call_order(clearing) -> None:
    inner = autoconnect(clearing).global_config(n_workers=2)
    mapper = (
        inner.hosted(host="edge")
        .remappings([(VoxelMapper, "points", "cleared")])
        .namespace("mapper")
    )
    mapping = autoconnect(VisModule.blueprint(), mapper).global_config(n_workers=3)

    resolved = mapping.global_config(n_workers=4).solidify(
        choices={"PointClearing": "column_carving"}
    )

    assert [atom.name for atom in resolved.blueprints] == ["vismodule", "mapper/voxelmapper"]
    assert resolved.hosted_placements[0].module_names == ("mapper/voxelmapper",)
    assert resolved.remapping_map[("mapper/voxelmapper", "points")] == "mapper/cleared"
    assert resolved.global_config_overrides == {"n_workers": 4}
    assert mapping.solidify().global_config_overrides == {"n_workers": 3}
    assert inner.solidify() == RaytracerModule.blueprint().global_config(n_workers=2)


def test_bad_default_branch_fails_at_construction(clearing) -> None:
    with pytest.raises(ValueError, match="host must be a non-empty string"):
        autoconnect(clearing).hosted(host="")


def test_coordinator_builds_the_default_selection(clearing, mocker: MockerFixture) -> None:
    mocker.patch.object(ModuleCoordinator, "start")
    deploy = mocker.patch("dimos.core.coordination.module_coordinator._deploy_all_modules")
    mocker.patch.object(ModuleCoordinator, "_connect_streams")
    mocker.patch("dimos.core.coordination.module_coordinator._connect_module_refs")
    mocker.patch.object(ModuleCoordinator, "build_all_modules")
    mocker.patch.object(ModuleCoordinator, "start_all_modules", return_value={})
    mocker.patch("dimos.core.coordination.module_coordinator._log_blueprint_graph")

    ModuleCoordinator.build(
        autoconnect(VisModule.blueprint(), clearing).global_config(viewer="none")
    )

    built = deploy.call_args.args[0]
    assert [atom.module for atom in built.active_blueprints] == [VisModule, RaytracerModule]

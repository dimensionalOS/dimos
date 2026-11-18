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

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.coordination.meta_blueprints import (
    ChoiceAtom,
    ChoiceResolutionError,
    MetaBlueprint,
    autoconnect_with_choices,
)
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.core.transport import LCMTransport


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


def test_composition_without_choices_preserves_autoconnect_behavior() -> None:
    concrete = autoconnect(VisModule.blueprint(), RaytracerModule.blueprint())
    meta = autoconnect_with_choices(VisModule.blueprint(), RaytracerModule.blueprint())

    assert isinstance(concrete, Blueprint)
    assert isinstance(meta, MetaBlueprint)
    assert meta.solidify() == concrete
    assert autoconnect_with_choices().solidify() == autoconnect()


@pytest.mark.parametrize(
    ("choices", "selected"),
    [({}, RaytracerModule), ({"PointClearing": "column_carving"}, VoxelMapper)],
)
def test_selects_only_the_requested_fragment(clearing, choices, selected) -> None:
    mapping = autoconnect_with_choices(VisModule.blueprint(), clearing)

    resolved = mapping.solidify(choices=choices)

    assert resolved == autoconnect(VisModule.blueprint(), selected.blueprint())
    assert mapping.solidify() == autoconnect(VisModule.blueprint(), RaytracerModule.blueprint())


def test_nested_composition_resolves_multiple_choices_at_the_root(clearing) -> None:
    mapping = autoconnect_with_choices(
        autoconnect_with_choices(clearing),
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
    mapping = autoconnect_with_choices(
        ChoiceAtom(
            name="Mapping",
            default="off",
            choices={
                "off": VisModule.blueprint(),
                "on": autoconnect_with_choices(
                    ChoiceAtom(name="Required", choices={"voxel": VoxelMapper.blueprint()})
                ),
            },
        )
    )

    assert mapping.solidify(choices=choices) == expected


def test_required_choice_errors_only_when_solidified() -> None:
    mapping = autoconnect_with_choices(
        ChoiceAtom(name="Required", choices={"voxel": VoxelMapper.blueprint()})
    )

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
        autoconnect_with_choices(clearing).solidify(choices=choices)


def test_rejects_overrides_for_inactive_choices(clearing) -> None:
    mapping = autoconnect_with_choices(
        ChoiceAtom(
            name="Mapping",
            default="off",
            choices={"off": autoconnect(), "on": autoconnect_with_choices(clearing)},
        )
    )

    with pytest.raises(ChoiceResolutionError, match="inactive choices: PointClearing"):
        mapping.solidify(choices={"PointClearing": "column_carving"})


def test_rejects_distinct_active_choices_with_the_same_name(clearing) -> None:
    other = ChoiceAtom(name="PointClearing", choices={"none": autoconnect()}, default="none")

    with pytest.raises(ChoiceResolutionError, match="Duplicate choice name: 'PointClearing'"):
        autoconnect_with_choices(clearing, autoconnect_with_choices(other)).solidify()


def test_shared_choice_can_be_reused_in_compositions(clearing) -> None:
    common = autoconnect_with_choices(clearing)
    mapping = autoconnect_with_choices(
        autoconnect_with_choices(common, VisModule.blueprint()), common
    )

    assert mapping.solidify(choices={"PointClearing": "column_carving"}) == autoconnect(
        VisModule.blueprint(), VoxelMapper.blueprint()
    )


def test_selected_branch_preserves_concrete_blueprint_behavior() -> None:
    base = autoconnect(
        VisModule.blueprint(), RaytracerModule.blueprint(tick_rate=1).hosted(host="old")
    )
    selected = (
        RaytracerModule.blueprint(tick_rate=2)
        .hosted(host="selected")
        .global_config(n_workers=2)
        .transports({("points", int): LCMTransport.spec("/selected", int)})
        .remappings([(RaytracerModule, "points", "cleared")])
    )
    unselected = VoxelMapper.blueprint().global_config(n_workers=99).hosted(host="unused")
    mapping = autoconnect_with_choices(
        base,
        ChoiceAtom(name="Clearing", choices={"ray": selected, "voxel": unselected}, default="ray"),
    )

    assert mapping.solidify() == autoconnect(base, selected)


def test_choice_options_are_snapshotted() -> None:
    options = {"ray": RaytracerModule.blueprint()}
    mapping = autoconnect_with_choices(ChoiceAtom(name="Clearing", choices=options, default="ray"))
    options["ray"] = VoxelMapper.blueprint()

    assert mapping.solidify() == RaytracerModule.blueprint()


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

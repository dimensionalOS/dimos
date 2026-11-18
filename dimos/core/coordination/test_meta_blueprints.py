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
from dimos.core.coordination.module_coordinator import ModuleCoordinator
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
            default="on",
            choices={"on": VisModule.blueprint(), "off": autoconnect()},
        ),
    )

    resolved = mapping.solidify(choices={"PointClearing": "column_carving", "Viewer": "off"})

    assert resolved == VoxelMapper.blueprint()


@pytest.fixture
def nested(clearing) -> MetaBlueprint:
    return autoconnect_with_choices(
        ChoiceAtom(
            name="Mapping",
            default="off",
            choices={"off": VisModule.blueprint(), "on": autoconnect_with_choices(clearing)},
        )
    )


@pytest.mark.parametrize(
    ("choices", "expected"),
    [
        ({}, VisModule.blueprint()),
        ({"Mapping": "on"}, RaytracerModule.blueprint()),
        ({"Mapping": "on", "PointClearing": "column_carving"}, VoxelMapper.blueprint()),
    ],
)
def test_choices_inside_an_alternative_resolve_when_it_is_selected(
    nested, choices, expected
) -> None:
    assert nested.solidify(choices=choices) == expected


def test_active_choices_lists_what_solidify_resolves(nested, clearing) -> None:
    assert [choice.name for choice in nested.active_choices()] == ["Mapping"]
    assert nested.active_choices(choices={"Mapping": "on"})[1:] == (clearing,)


@pytest.mark.parametrize(
    ("choices", "message"),
    [
        ({"PointClearing": "typo"}, "Invalid option.*typo.*PointClearing.*raycasting"),
        (
            {"PointClearng": "raycasting"},
            r"Unknown choice 'PointClearng'\. Available choices: PointClearing\. "
            r"Did you mean: PointClearing\?",
        ),
        ({"pointclearing": "raycasting"}, "Unknown choice.*Did you mean: PointClearing"),
        ({"Viewer": "on"}, r"Unknown choice 'Viewer'\. Available choices: PointClearing$"),
    ],
)
def test_rejects_invalid_overrides(clearing, choices, message) -> None:
    with pytest.raises(ChoiceResolutionError, match=message):
        autoconnect_with_choices(clearing).solidify(choices=choices)


def test_rejects_overrides_when_there_are_no_choices() -> None:
    with pytest.raises(ChoiceResolutionError, match="'Viewer'. This blueprint has no choices"):
        autoconnect_with_choices(VisModule.blueprint()).solidify(choices={"Viewer": "on"})


def test_rejects_overrides_for_inactive_choices(nested) -> None:
    message = r"Choice 'PointClearing' is inactive because Mapping=off\. It needs Mapping=on"

    with pytest.raises(ChoiceResolutionError, match=message):
        nested.solidify(choices={"PointClearing": "column_carving"})

    with pytest.raises(ChoiceResolutionError, match="Available choices: Mapping, PointClearing"):
        nested.solidify(choices={"Typo": "on"})


def test_inactive_choice_error_names_the_outermost_unselected_parent(clearing) -> None:
    mapping = autoconnect_with_choices(
        ChoiceAtom(
            name="Stack",
            default="full",
            choices={
                "minimal": autoconnect(),
                "full": autoconnect_with_choices(
                    ChoiceAtom(
                        name="Mapping",
                        default="off",
                        choices={"off": autoconnect(), "on": autoconnect_with_choices(clearing)},
                    )
                ),
            },
        )
    )

    with pytest.raises(ChoiceResolutionError, match="inactive because Mapping=off"):
        mapping.solidify(choices={"PointClearing": "column_carving"})

    with pytest.raises(ChoiceResolutionError, match="inactive because Stack=minimal"):
        mapping.solidify(choices={"Stack": "minimal", "PointClearing": "column_carving"})


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


def test_equal_choices_built_by_a_factory_can_be_reused_in_compositions() -> None:
    def viewer_choice() -> ChoiceAtom:
        return ChoiceAtom(
            name="Viewer",
            default="on",
            choices={"on": VisModule.blueprint(), "off": autoconnect()},
        )

    mapping = autoconnect_with_choices(
        autoconnect_with_choices(viewer_choice(), RaytracerModule.blueprint()),
        autoconnect_with_choices(viewer_choice()),
    )

    assert mapping.active_choices() == (viewer_choice(),)
    assert mapping.solidify(choices={"Viewer": "off"}) == RaytracerModule.blueprint()


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
        ("", {"a": autoconnect()}, "a", "names must be nonempty"),
        ("Invalid=Name", {"a": autoconnect()}, "a", "cannot contain '='"),
        ("Empty", {}, "a", "at least one option"),
        ("Empty", {"": autoconnect()}, "", "nonempty strings, got ''"),
        ("NotStr", {1: autoconnect()}, 1, "nonempty strings, got 1"),
        ("Default", {"a": autoconnect()}, "b", "Invalid default.*b.*Default.*a"),
    ],
)
def test_rejects_invalid_choice_definitions(name, choices, default, message) -> None:
    with pytest.raises(ValueError, match=message) as error:
        ChoiceAtom(name=name, choices=choices, default=default)

    # A definition bug is not a selection error, so the CLI does not report it as one.
    assert not isinstance(error.value, ChoiceResolutionError)


def test_choice_requires_a_default() -> None:
    with pytest.raises(TypeError, match="default"):
        ChoiceAtom(name="Viewer", choices={"on": VisModule.blueprint()})  # type: ignore[call-arg]


@pytest.mark.parametrize(
    ("alternative", "message"),
    [
        (VisModule, "Option 'a' of choice 'Viewer' must be a Blueprint or MetaBlueprint, got the"),
        (None, "got NoneType"),
        (ChoiceAtom(name="Nested", choices={"a": autoconnect()}, default="a"), "got ChoiceAtom"),
    ],
)
def test_rejects_choice_options_that_are_not_blueprints(alternative, message) -> None:
    with pytest.raises(TypeError, match=message):
        ChoiceAtom(name="Viewer", choices={"a": alternative}, default="a")


def test_rejects_parts_that_are_not_blueprints_or_choices(clearing) -> None:
    with pytest.raises(TypeError, match="got the class VisModule"):
        autoconnect_with_choices(clearing, VisModule)  # type: ignore[arg-type]


@pytest.mark.parametrize(
    ("argument", "got"),
    [
        (autoconnect_with_choices(), "MetaBlueprint"),
        (ChoiceAtom(name="Viewer", choices={"a": autoconnect()}, default="a"), "ChoiceAtom"),
        (VisModule, "the class VisModule"),
    ],
)
def test_autoconnect_rejects_arguments_that_are_not_blueprints(argument, got) -> None:
    with pytest.raises(TypeError, match=f"got {got}.*autoconnect_with_choices"):
        autoconnect(VisModule.blueprint(), argument)


def test_coordinator_build_asks_for_a_solidified_blueprint(clearing) -> None:
    with pytest.raises(TypeError, match=r"got MetaBlueprint.*\.solidify\(\)"):
        ModuleCoordinator.build(autoconnect_with_choices(clearing))  # type: ignore[arg-type]

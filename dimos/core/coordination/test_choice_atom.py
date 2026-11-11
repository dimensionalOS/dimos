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

from typing import Protocol

import pytest

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import Blueprint, ChoiceAtom, autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.spec.utils import Spec


class Cloud:
    pass


class OtherCloud:
    pass


class ClearerSpec(Spec, Protocol):
    @rpc
    def method(self) -> str: ...


class Lidar(Module):
    cloud: Out[Cloud]


class Raycaster(Module):
    cloud: In[Cloud]

    @rpc
    def method(self) -> str:
        return "raycasting"


class ColumnCarver(Module):
    cloud: In[Cloud]

    @rpc
    def method(self) -> str:
        return "column_carving"


class WrongTypeCarver(Module):
    cloud: In[OtherCloud]

    @rpc
    def method(self) -> str:
        return "wrong"


class NotAClearer(Module):
    cloud: In[Cloud]


class Vis(Module):
    cloud: In[Cloud]
    clearer: ClearerSpec

    @rpc
    def clearing_method(self) -> str:
        return self.clearer.method()


def point_clearing(default: str | None = "raycasting", **extra: Blueprint) -> ChoiceAtom:
    return ChoiceAtom(
        name="PointClearing",
        default=default,
        choices=dict(
            raycasting=Raycaster.blueprint(),
            column_carving=ColumnCarver.blueprint(),
            **extra,
        ),
    )


def modules(blueprint: Blueprint) -> list[type]:
    return [atom.module for atom in blueprint.active_blueprints]


def test_plain_blueprint_is_solidified_and_solidify_is_a_no_op() -> None:
    blueprint = autoconnect(Lidar.blueprint(), Vis.blueprint())
    assert blueprint.solidified
    assert blueprint.solidify() is blueprint


def test_choice_makes_a_blueprint_unsolidified() -> None:
    blueprint = autoconnect(Lidar.blueprint(), Vis.blueprint(), point_clearing())
    assert not blueprint.solidified
    assert blueprint.all_choice_names() == {"PointClearing"}
    # Composing again keeps the choice.
    assert not autoconnect(blueprint, Lidar.blueprint()).solidified


def test_solidify_uses_the_default() -> None:
    solid = autoconnect(Lidar.blueprint(), Vis.blueprint(), point_clearing()).solidify()
    assert solid.solidified
    assert set(modules(solid)) == {Lidar, Vis, Raycaster}


def test_solidify_uses_choose_then_explicit_picks() -> None:
    blueprint = autoconnect(Lidar.blueprint(), Vis.blueprint(), point_clearing())
    chosen = blueprint.choose(PointClearing="column_carving")
    assert set(modules(chosen.solidify())) == {Lidar, Vis, ColumnCarver}
    assert set(modules(chosen.solidify(PointClearing="raycasting"))) == {Lidar, Vis, Raycaster}
    assert set(modules(blueprint.solidify({"PointClearing": "column_carving"}))) == {
        Lidar,
        Vis,
        ColumnCarver,
    }


def test_solidify_is_cached_so_parsed_config_matches() -> None:
    blueprint = autoconnect(Lidar.blueprint(), Vis.blueprint(), point_clearing())
    assert blueprint.solidify() is blueprint.solidify()
    assert BlueprintConfigParser(blueprint).blueprint is blueprint.solidify()


def test_no_default_and_no_pick_fails() -> None:
    blueprint = autoconnect(Lidar.blueprint(), Vis.blueprint(), point_clearing(default=None))
    with pytest.raises(ValueError, match="has no default"):
        blueprint.solidify()


def test_unknown_choice_or_option_fails() -> None:
    blueprint = autoconnect(Lidar.blueprint(), Vis.blueprint(), point_clearing())
    with pytest.raises(ValueError, match="Unknown choice"):
        blueprint.solidify(PointClaring="raycasting")
    with pytest.raises(ValueError, match="no option 'voxels'"):
        blueprint.solidify(PointClearing="voxels")


def test_bad_default_is_rejected_up_front() -> None:
    with pytest.raises(ValueError, match="default 'nope'"):
        point_clearing(default="nope")


def test_nested_choices() -> None:
    inner = ChoiceAtom(
        name="Carver",
        default="column",
        choices=dict(column=ColumnCarver.blueprint(), ray=Raycaster.blueprint()),
    )
    outer = ChoiceAtom(
        name="PointClearing",
        default="raycasting",
        choices=dict(raycasting=Raycaster.blueprint(), carving=autoconnect(inner)),
    )
    blueprint = autoconnect(Lidar.blueprint(), Vis.blueprint(), outer)
    assert blueprint.all_choice_names() == {"PointClearing", "Carver"}
    assert list(blueprint.choice_combinations()) == [
        {"PointClearing": "raycasting"},
        {"PointClearing": "carving", "Carver": "column"},
        {"PointClearing": "carving", "Carver": "ray"},
    ]
    assert set(modules(blueprint.solidify(PointClearing="carving"))) == {Lidar, Vis, ColumnCarver}
    assert set(modules(blueprint.solidify(PointClearing="carving", Carver="ray"))) == {
        Lidar,
        Vis,
        Raycaster,
    }


def test_stream_types_are_checked_in_every_branch() -> None:
    blueprint = autoconnect(
        Lidar.blueprint(), Vis.blueprint(), point_clearing(wrong=WrongTypeCarver.blueprint())
    )
    # The default branch is fine; the unpicked one has a mistyped stream.
    with pytest.raises(ValueError, match=r"PointClearing=wrong(.|\n)*conflicting types"):
        blueprint.solidify()


def test_rpc_refs_are_checked_in_every_branch() -> None:
    blueprint = autoconnect(
        Lidar.blueprint(), Vis.blueprint(), point_clearing(missing=NotAClearer.blueprint())
    )
    with pytest.raises(ValueError, match=r"PointClearing=missing(.|\n)*No module met that spec"):
        blueprint.solidify()


def test_outer_settings_win_over_a_branch() -> None:
    branch = (
        Raycaster.blueprint().global_config(n_workers=1).remappings([(Raycaster, "cloud", "a")])
    )
    choice = ChoiceAtom(name="PointClearing", default="ray", choices=dict(ray=branch))
    blueprint = (
        autoconnect(Lidar.blueprint(), choice)
        .global_config(n_workers=3)
        .remappings([(Raycaster, "cloud", "cloud")])
    )
    solid = blueprint.solidify()
    assert solid.global_config_overrides["n_workers"] == 3
    assert solid.remapping_map[Raycaster.name, "cloud"] == "cloud"


def test_namespace_prefixes_choices_and_their_branches() -> None:
    blueprint = autoconnect(Lidar.blueprint(), Vis.blueprint(), point_clearing())
    spaced = blueprint.namespace("robot0", expose={"cloud"}).choose(
        {"robot0/PointClearing": "column_carving"}
    )
    assert spaced.all_choice_names() == {"robot0/PointClearing"}
    assert {atom.name for atom in spaced.solidify().active_blueprints} == {
        f"robot0/{Lidar.name}",
        f"robot0/{Vis.name}",
        f"robot0/{ColumnCarver.name}",
    }


def test_hosted_refuses_unpicked_choices() -> None:
    with pytest.raises(ValueError, match="unpicked choices"):
        autoconnect(Lidar.blueprint(), point_clearing()).hosted()


@pytest.mark.parametrize(("pick", "expected"), [(None, "raycasting"), ("column_carving", None)])
def test_build_solidifies(pick: str | None, expected: str | None) -> None:
    blueprint = autoconnect(Lidar.blueprint(), Vis.blueprint(), point_clearing())
    if pick is not None:
        blueprint = blueprint.choose(PointClearing=pick)
    parsed = BlueprintConfigParser(blueprint).parse(environ={}, overrides={"g": {"viewer": "none"}})
    coordinator = ModuleCoordinator.build(blueprint, parsed)
    try:
        assert coordinator.get_instance(Vis).clearing_method() == (expected or pick)
    finally:
        coordinator.stop()

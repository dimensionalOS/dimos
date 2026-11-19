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

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import BlueprintAtom, MetaBlueprint, autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import Out
from dimos.core.tf_module import TfModule, TfModuleConfig
from dimos.msgs.tf2_msgs.TFMessage import TFMessage


class OdomConfig(TfModuleConfig):
    tf_frames_local: dict[str, str] = {"body": "base_link"}
    tf_frames_global: dict[str, str] = {"world": "world"}
    tf_edges: list[tuple[str, str]] = [("world", "body")]


class Odom(TfModule):
    config: OdomConfig

    @rpc
    def frames(self) -> tuple[dict[str, str], dict[str, str]]:
        return self.config.tf_frames_local, self.config.tf_frames_global


def test_namespaced_local_frames_are_prefixed_and_global_frames_are_not() -> None:
    blueprint = autoconnect(
        Odom.blueprint().namespace("robot0"),
        Odom.blueprint(tf_frames_local={"body": "torso"}).namespace("inner").namespace("robot1"),
    ).solidify()
    parsed = BlueprintConfigParser(blueprint).parse(environ={}, overrides={"g": {"viewer": "none"}})
    coordinator = ModuleCoordinator.build(blueprint, parsed)
    try:
        assert coordinator.get_instance("robot0/odom").frames() == (
            {"body": "robot0/base_link"},
            {"world": "world"},
        )
        assert coordinator.get_instance("robot1/inner/odom").frames() == (
            {"body": "robot1/inner/torso"},
            {"world": "world"},
        )
    finally:
        coordinator.stop()


def test_unnamespaced_local_frames_are_untouched() -> None:
    blueprint = Odom.blueprint()
    parsed = BlueprintConfigParser(blueprint).parse(environ={}, overrides={"g": {"viewer": "none"}})
    coordinator = ModuleCoordinator.build(blueprint, parsed)
    try:
        assert coordinator.get_instance(Odom).frames()[0] == {"body": "base_link"}
    finally:
        coordinator.stop()


class BackupOdom(Odom):
    """Publishes the same edge as Odom; autoconnect merges two copies of one class."""


class LidarConfig(TfModuleConfig):
    tf_frames_local: dict[str, str] = {"body": "base_link", "lidar": "lidar"}
    tf_edges: list[tuple[str, str]] = [("body", "lidar")]


class Lidar(TfModule):
    config: LidarConfig


class LegacyTfPublisher(Module):
    tf: Out[TFMessage]


def _verify(blueprint: MetaBlueprint) -> None:
    TfModule.blueprint_checks(list(blueprint.solidify().active_blueprints))


def test_one_tree_passes() -> None:
    _verify(autoconnect(Odom.blueprint(), Lidar.blueprint()))


def test_two_parents_is_refused() -> None:
    with pytest.raises(
        ValueError, match=r"'base_link' has several parents: odom_frame \(from lidar\), world"
    ):
        _verify(
            autoconnect(
                Odom.blueprint(),
                Lidar.blueprint(
                    tf_frames_local={"body": "base_link", "lidar": "odom_frame"},
                    tf_edges=[("lidar", "body")],
                ),
            )
        )


def test_same_edge_from_two_modules_warns(mocker: MockerFixture) -> None:
    logger = mocker.patch("dimos.core.tf_module.logger")
    _verify(autoconnect(Odom.blueprint(), BackupOdom.blueprint()))
    (message,), _ = logger.warning.call_args
    assert "'world' -> 'base_link' is declared by several modules (odom, backupodom)" in message


def test_cycle_is_refused() -> None:
    with pytest.raises(ValueError, match="form a cycle"):
        _verify(
            autoconnect(
                Lidar.blueprint(),
                Odom.blueprint(
                    tf_frames_local={"body": "base_link"},
                    tf_frames_global={"world": "lidar"},
                ),
            )
        )


def test_skip_tf_checks_leaves_a_module_out_of_the_tree_check() -> None:
    second_parent = Lidar.blueprint(
        tf_frames_local={"body": "base_link", "lidar": "odom_frame"},
        tf_edges=[("lidar", "body")],
        skip_tf_checks=True,
    )
    _verify(autoconnect(Odom.blueprint(), second_parent))


def test_separate_trees_are_refused_unless_a_legacy_module_may_join_them() -> None:
    lidar_alone = Lidar.blueprint(tf_frames_local={"body": "mount", "lidar": "lidar"})
    with pytest.raises(ValueError, match="2 separate trees"):
        _verify(autoconnect(Odom.blueprint(), lidar_alone))
    _verify(autoconnect(Odom.blueprint(), lidar_alone, LegacyTfPublisher.blueprint()))


def test_build_checks_the_namespaced_tree() -> None:
    blueprint = autoconnect(
        Odom.blueprint().namespace("robot0"),
        Odom.blueprint().namespace("robot1"),
        Odom.blueprint(),
        BackupOdom.blueprint(tf_frames_global={"world": "map"}),
    ).solidify()
    with pytest.raises(ValueError, match="'base_link' has several parents"):
        ModuleCoordinator.build(
            blueprint,
            BlueprintConfigParser(blueprint).parse(environ={}, overrides={"g": {"viewer": "none"}}),
        )


calls: list[int] = []


class CountedCheck(Module):
    @staticmethod
    def blueprint_checks(atoms: list[BlueprintAtom]) -> None:
        calls.append(len(atoms))


class CountedCheckChild(CountedCheck):
    pass


def test_a_shared_blueprint_check_runs_once() -> None:
    calls.clear()
    blueprint = autoconnect(CountedCheck.blueprint(), CountedCheckChild.blueprint()).solidify()
    coordinator = ModuleCoordinator.build(
        blueprint,
        BlueprintConfigParser(blueprint).parse(environ={}, overrides={"g": {"viewer": "none"}}),
    )
    coordinator.stop()
    assert calls == [2]


class OtherCheck(Module):
    @staticmethod
    def blueprint_checks(atoms: list[BlueprintAtom]) -> None:
        calls.append(-len(atoms))


class BothChecks(OtherCheck, CountedCheck):
    pass


def test_every_base_keeps_its_blueprint_check() -> None:
    calls.clear()
    blueprint = BothChecks.blueprint()
    coordinator = ModuleCoordinator.build(
        blueprint,
        BlueprintConfigParser(blueprint).parse(environ={}, overrides={"g": {"viewer": "none"}}),
    )
    coordinator.stop()
    assert sorted(calls) == [-1, 1]

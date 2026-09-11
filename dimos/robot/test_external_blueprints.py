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

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Any

import pytest

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.module import Module
from dimos.robot import external_blueprints as external
from dimos.robot.get_all_blueprints import get_by_name


class ExternalTestModule(Module):
    pass


@dataclass(frozen=True)
class FakeEntryPoint:
    name: str
    value: str
    target: Any = None
    error: Exception | None = None
    group: str = external.ENTRY_POINT_GROUP
    dist: Any = None

    def load(self) -> Any:
        if self.error is not None:
            raise self.error
        return self.target


@dataclass(frozen=True)
class FakeDistribution:
    name: str
    entry_points: tuple[FakeEntryPoint, ...]
    metadata_name: str | None = None

    @property
    def metadata(self) -> dict[str, str]:
        if self.metadata_name is None:
            return {"Name": self.name}
        if self.metadata_name == "":
            return {}
        return {"Name": self.metadata_name}


def patch_distributions(monkeypatch: pytest.MonkeyPatch, *distributions: FakeDistribution) -> None:
    entry_points = [
        replace(entry_point, dist=distribution)
        for distribution in distributions
        for entry_point in distribution.entry_points
    ]

    monkeypatch.setattr(
        external.importlib_metadata,
        "entry_points",
        lambda *, group=None: [
            entry_point
            for entry_point in entry_points
            if group is None or entry_point.group == group
        ],
    )


@pytest.mark.parametrize(
    ("distribution_name", "expected"),
    [
        ("My_Robot.Stack", "my-robot-stack"),
        ("my---robot___stack", "my-robot-stack"),
        ("my.robot_stack", "my-robot-stack"),
        ("my-robot-stack", "my-robot-stack"),
    ],
)
def test_canonicalize_distribution_namespace(distribution_name: str, expected: str) -> None:
    assert external.canonicalize_distribution_namespace(distribution_name) == expected


@pytest.mark.parametrize("name", ["go2", "keyboard-teleop", "g1-sim2"])
def test_valid_external_local_blueprint_names(name: str) -> None:
    assert external.is_valid_external_local_blueprint_name(name)


@pytest.mark.parametrize("name", ["", "Go2", "go2_sim", "go2.sim", "go2/real", "go2--sim"])
def test_invalid_external_local_blueprint_names(name: str) -> None:
    assert not external.is_valid_external_local_blueprint_name(name)


def test_list_external_blueprint_names_without_loading_targets(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    entry_point = FakeEntryPoint(
        name="demo",
        value="external_stack.demo:demo_blueprint",
        error=AssertionError("list must not load targets"),
    )
    patch_distributions(monkeypatch, FakeDistribution("My_Test.Stack", (entry_point,)))

    assert external.list_external_blueprint_names() == ["my-test-stack.demo"]


def test_resolve_external_blueprint_object(monkeypatch: pytest.MonkeyPatch) -> None:
    blueprint = ExternalTestModule.blueprint()
    patch_distributions(
        monkeypatch,
        FakeDistribution(
            "My-Test-Stack",
            (FakeEntryPoint("demo", "external_stack.demo:demo_blueprint", blueprint),),
        ),
    )

    assert external.resolve_external_blueprint_by_name("my-test-stack.demo") is blueprint


def test_resolve_external_module_class(monkeypatch: pytest.MonkeyPatch) -> None:
    patch_distributions(
        monkeypatch,
        FakeDistribution(
            "My-Test-Stack",
            (
                FakeEntryPoint(
                    "module-demo", "external_stack.demo:ExternalTestModule", ExternalTestModule
                ),
            ),
        ),
    )

    blueprint = external.resolve_external_blueprint_by_name("my-test-stack.module-demo")

    assert isinstance(blueprint, Blueprint)
    assert blueprint.blueprints[0].module is ExternalTestModule


@pytest.mark.parametrize("target", [lambda: ExternalTestModule.blueprint(), 42, object()])
def test_rejects_unsupported_external_targets(monkeypatch: pytest.MonkeyPatch, target: Any) -> None:
    patch_distributions(
        monkeypatch,
        FakeDistribution(
            "My-Test-Stack", (FakeEntryPoint("demo", "external_stack.demo:target", target),)
        ),
    )

    with pytest.raises(external.ExternalBlueprintError):
        external.resolve_external_blueprint_by_name("my-test-stack.demo")


def test_unknown_external_namespace(monkeypatch: pytest.MonkeyPatch) -> None:
    patch_distributions(monkeypatch, FakeDistribution("My-Test-Stack", ()))

    with pytest.raises(external.ExternalBlueprintError):
        external.resolve_external_blueprint_by_name("missing-stack.demo")


def test_unknown_external_namespace_lists_available_namespaces(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    patch_distributions(
        monkeypatch,
        FakeDistribution(
            "My-Test-Stack",
            (FakeEntryPoint("demo", "external_stack.demo:ExternalTestModule", ExternalTestModule),),
        ),
    )

    with pytest.raises(external.ExternalBlueprintError) as exc_info:
        external.resolve_external_blueprint_by_name("missing-stack.demo")

    assert "Available external namespaces: my-test-stack" in str(exc_info.value)


def test_resolve_external_name_requires_namespace_separator(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    patch_distributions(monkeypatch)

    with pytest.raises(external.ExternalBlueprintError):
        external.resolve_external_blueprint_by_name("my-test-stack")


def test_namespace_exists_but_local_name_missing(monkeypatch: pytest.MonkeyPatch) -> None:
    patch_distributions(
        monkeypatch,
        FakeDistribution(
            "My-Test-Stack",
            (
                FakeEntryPoint(
                    "demo", "external_stack.demo:demo_blueprint", ExternalTestModule.blueprint()
                ),
            ),
        ),
    )

    with pytest.raises(external.ExternalBlueprintError):
        external.resolve_external_blueprint_by_name("my-test-stack.arm")


def test_local_name_missing_message_omits_empty_available_names() -> None:
    error = external._external_blueprint_local_name_not_found_error("my-test-stack", "arm", [])

    assert "Available local blueprints" not in str(error)


def test_entry_point_load_failure(monkeypatch: pytest.MonkeyPatch) -> None:
    patch_distributions(
        monkeypatch,
        FakeDistribution(
            "My-Test-Stack",
            (
                FakeEntryPoint(
                    "demo", "external_stack.demo:demo_blueprint", error=ImportError("boom")
                ),
            ),
        ),
    )

    with pytest.raises(external.ExternalBlueprintError, match="ImportError: boom"):
        external.resolve_external_blueprint_by_name("my-test-stack.demo")


def test_invalid_external_metadata_name(monkeypatch: pytest.MonkeyPatch) -> None:
    patch_distributions(
        monkeypatch,
        FakeDistribution("My-Test-Stack", (FakeEntryPoint("Go2", "external_stack.demo:go2"),)),
    )

    assert external.list_external_blueprint_names() == []
    with pytest.raises(external.ExternalBlueprintError):
        external.resolve_external_blueprint_by_name("my-test-stack.demo")


def test_invalid_requested_external_local_name(monkeypatch: pytest.MonkeyPatch) -> None:
    patch_distributions(
        monkeypatch,
        FakeDistribution(
            "My-Test-Stack",
            (FakeEntryPoint("demo", "external_stack.demo:ExternalTestModule", ExternalTestModule),),
        ),
    )

    with pytest.raises(external.ExternalBlueprintError) as exc_info:
        external.resolve_external_blueprint_by_name("my-test-stack.Go2")

    message = str(exc_info.value)
    assert "Invalid external blueprint local name 'Go2'" in message
    assert "entry point name" not in message
    assert "distribution" not in message


def test_invalid_external_metadata_does_not_block_unrelated_valid_package(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    patch_distributions(
        monkeypatch,
        FakeDistribution("Broken-Stack", (FakeEntryPoint("BadName", "broken_stack.demo:demo"),)),
        FakeDistribution(
            "My-Test-Stack",
            (FakeEntryPoint("demo", "external_stack.demo:ExternalTestModule", ExternalTestModule),),
        ),
    )

    assert external.list_external_blueprint_names() == ["my-test-stack.demo"]
    blueprint = external.resolve_external_blueprint_by_name("my-test-stack.demo")

    assert blueprint.blueprints[0].module is ExternalTestModule


def test_entry_points_without_distribution_are_ignored(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(
        external.importlib_metadata,
        "entry_points",
        lambda *, group=None: [
            FakeEntryPoint("demo", "external_stack.demo:ExternalTestModule", ExternalTestModule)
        ],
    )

    assert external.list_external_blueprint_names() == []


def test_entry_points_without_distribution_name_are_ignored(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    patch_distributions(
        monkeypatch,
        FakeDistribution(
            "My-Test-Stack",
            (FakeEntryPoint("demo", "external_stack.demo:ExternalTestModule", ExternalTestModule),),
            metadata_name="",
        ),
    )

    assert external.list_external_blueprint_names() == []


def test_all_invalid_colliding_distribution_does_not_block_valid_package(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    patch_distributions(
        monkeypatch,
        FakeDistribution("my_robot.stack", (FakeEntryPoint("Go2", "invalid_stack.demo:go2"),)),
        FakeDistribution(
            "My-Test-Stack",
            (FakeEntryPoint("demo", "external_stack.demo:ExternalTestModule", ExternalTestModule),),
        ),
    )

    assert external.list_external_blueprint_names() == ["my-test-stack.demo"]
    blueprint = external.resolve_external_blueprint_by_name("my-test-stack.demo")

    assert blueprint.blueprints[0].module is ExternalTestModule


def test_colliding_external_namespace_uses_matching_valid_entry(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    patch_distributions(
        monkeypatch,
        FakeDistribution(
            "My-Test-Stack", (FakeEntryPoint("demo", "a:b", ExternalTestModule.blueprint()),)
        ),
        FakeDistribution(
            "my_test.stack", (FakeEntryPoint("other", "c:d", ExternalTestModule.blueprint()),)
        ),
    )

    blueprint = external.resolve_external_blueprint_by_name("my-test-stack.demo")

    assert isinstance(blueprint, Blueprint)


def test_bare_names_never_search_external_entry_points(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(
        external.importlib_metadata,
        "entry_points",
        lambda: (_ for _ in ()).throw(AssertionError("bare lookup searched external metadata")),
    )

    with pytest.raises(ValueError, match="Unknown blueprint or module"):
        get_by_name("missing-bare-blueprint")


def test_get_by_name_resolves_external_names(monkeypatch: pytest.MonkeyPatch) -> None:
    patch_distributions(
        monkeypatch,
        FakeDistribution(
            "My-Test-Stack",
            (FakeEntryPoint("demo", "external_stack.demo:ExternalTestModule", ExternalTestModule),),
        ),
    )

    blueprint = get_by_name("my-test-stack.demo")

    assert blueprint.blueprints[0].module is ExternalTestModule


def test_mixed_builtin_and_external_names_resolve_before_composition(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    patch_distributions(
        monkeypatch,
        FakeDistribution(
            "My-Test-Stack",
            (FakeEntryPoint("demo", "external_stack.demo:ExternalTestModule", ExternalTestModule),),
        ),
    )

    mixed_blueprint = autoconnect(
        get_by_name("demo-mcp-stress-test"), get_by_name("my-test-stack.demo")
    )

    assert any(atom.module is ExternalTestModule for atom in mixed_blueprint.blueprints)

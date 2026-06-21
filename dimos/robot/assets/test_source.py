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

import os
from pathlib import Path

import pytest

from dimos.robot.assets.git_cache import GitAssetCache, GitAssetCheckout
from dimos.robot.assets.source import RobotDescriptionPath, RobotDescriptionSource


class RecordingGitAssetCache(GitAssetCache):
    def __init__(self, checkout_path: Path) -> None:
        self.checkout_path = checkout_path
        self.resolve_calls: list[tuple[str, str]] = []

    def resolve(self, repo_url: str, ref: str) -> GitAssetCheckout:
        self.resolve_calls.append((repo_url, ref))
        return GitAssetCheckout(path=self.checkout_path, repo_url=repo_url, ref=ref)


@pytest.fixture()
def robot_source(tmp_path: Path) -> tuple[RobotDescriptionSource, RecordingGitAssetCache]:
    checkout = tmp_path / "checkout" / "testbot_description"
    (checkout / "robots" / "testbot").mkdir(parents=True)
    (checkout / "robots" / "testbot" / "model.urdf").write_text("<robot name='testbot'/>")
    (checkout / "packages" / "testbot_description").mkdir(parents=True)
    (checkout / "packages" / "testbot_description" / "package.xml").write_text("<package/>")

    git_cache = RecordingGitAssetCache(checkout)
    source = RobotDescriptionSource(
        url="https://example.invalid/testbot_description.git",
        ref="main",
        git_cache=git_cache,
    )
    return source, git_cache


def test_source_path_joining_defers_checkout(
    robot_source: tuple[RobotDescriptionSource, RecordingGitAssetCache],
) -> None:
    source, git_cache = robot_source

    model_path = source / "robots" / "testbot" / "model.urdf"
    package_path = source / "packages" / "testbot_description"

    assert isinstance(model_path, Path)
    assert isinstance(model_path, RobotDescriptionPath)
    assert git_cache.resolve_calls == []

    assert str(model_path).endswith("robots/testbot/model.urdf")
    assert git_cache.resolve_calls == [("https://example.invalid/testbot_description.git", "main")]

    assert os.fspath(package_path).endswith("packages/testbot_description")
    assert git_cache.resolve_calls == [("https://example.invalid/testbot_description.git", "main")]

    assert model_path.exists()
    assert (package_path / "package.xml").exists()


def test_source_parent_can_express_package_root_without_resolving(
    robot_source: tuple[RobotDescriptionSource, RecordingGitAssetCache],
) -> None:
    source, git_cache = robot_source

    package_root = source.parent

    assert isinstance(package_root, RobotDescriptionPath)
    assert git_cache.resolve_calls == []
    assert package_root.resolve().name == "checkout"
    assert git_cache.resolve_calls == [("https://example.invalid/testbot_description.git", "main")]


def test_lazy_path_metadata_uses_relative_path_without_checkout(
    robot_source: tuple[RobotDescriptionSource, RecordingGitAssetCache],
) -> None:
    source, git_cache = robot_source

    model_path = source / "robots" / "testbot" / "model.urdf"

    assert model_path.name == "model.urdf"
    assert model_path.stem == "model"
    assert model_path.suffix == ".urdf"
    assert model_path.parent.name == "testbot"
    assert git_cache.resolve_calls == []


def test_custom_source_handle_needs_no_registration(tmp_path: Path) -> None:
    checkout = tmp_path / "custom_robot_description"
    (checkout / "urdf").mkdir(parents=True)
    (checkout / "urdf" / "custom.urdf").write_text("<robot name='custom'/>")
    git_cache = RecordingGitAssetCache(checkout)

    custom_source = RobotDescriptionSource(
        url="https://example.invalid/custom_robot_description.git",
        ref="feature/custom",
        git_cache=git_cache,
    )

    model_path = custom_source / "urdf" / "custom.urdf"

    assert model_path.read_text() == "<robot name='custom'/>"
    assert git_cache.resolve_calls == [
        ("https://example.invalid/custom_robot_description.git", "feature/custom")
    ]

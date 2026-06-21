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

"""Lazy path handles for Git-backed robot description sources."""

from __future__ import annotations

from pathlib import Path

from dimos.robot.assets.git_cache import GitAssetCache


class RobotDescriptionSource:
    """Git-backed robot description source root.

    Joining paths is lazy. The Git checkout is only resolved when the resulting
    :class:`RobotDescriptionPath` is observed as a filesystem path.
    """

    def __init__(
        self,
        url: str,
        ref: str,
        git_cache: GitAssetCache | None = None,
    ) -> None:
        self.url = url
        self.ref = ref
        self._git_cache = git_cache or GitAssetCache()
        self._checkout_path_cache: Path | None = None

    def checkout_path(self) -> Path:
        """Return the local checkout root, cloning/updating if needed."""
        if self._checkout_path_cache is None:
            self._checkout_path_cache = self._git_cache.resolve(self.url, self.ref).path
        return self._checkout_path_cache

    def path(self) -> RobotDescriptionPath:
        """Return a lazy path for the checkout root."""
        return RobotDescriptionPath(self, Path("."))

    @property
    def parent(self) -> RobotDescriptionPath:
        """Return the checkout root's parent as a lazy path."""
        return RobotDescriptionPath(self, Path(".."))

    def __truediv__(self, other: object) -> RobotDescriptionPath:
        return self.path() / other

    def __repr__(self) -> str:
        return f"RobotDescriptionSource(url={self.url!r}, ref={self.ref!r})"

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, RobotDescriptionSource):
            return NotImplemented
        return self.url == other.url and self.ref == other.ref

    def __hash__(self) -> int:
        return hash((self.url, self.ref))


class RobotDescriptionPath(type(Path())):  # type: ignore[misc]
    """Lazy Path subclass rooted at a :class:`RobotDescriptionSource`."""

    def __new__(
        cls,
        source: RobotDescriptionSource,
        relative_path: Path | str,
    ) -> RobotDescriptionPath:
        instance: RobotDescriptionPath = super().__new__(cls, ".")
        object.__setattr__(instance, "_robot_description_source", source)
        object.__setattr__(instance, "_robot_description_relative_path", Path(relative_path))
        object.__setattr__(instance, "_robot_description_resolved_cache", None)
        return instance

    def __init__(
        self,
        source: RobotDescriptionSource,
        relative_path: Path | str,
    ) -> None:
        del source, relative_path

    def _resolve(self) -> Path:
        cache: Path | None = object.__getattribute__(self, "_robot_description_resolved_cache")
        if cache is None:
            source: RobotDescriptionSource = object.__getattribute__(
                self, "_robot_description_source"
            )
            relative_path: Path = object.__getattribute__(self, "_robot_description_relative_path")
            cache = source.checkout_path() / relative_path
            object.__setattr__(self, "_robot_description_resolved_cache", cache)
        return cache

    def __getattribute__(self, name: str) -> object:
        try:
            object.__getattribute__(self, "_robot_description_source")
        except AttributeError:
            return object.__getattribute__(self, name)

        if name.startswith("_robot_description_") or name in {"_resolve"}:
            return object.__getattribute__(self, name)

        if name == "parent":
            source: RobotDescriptionSource = object.__getattribute__(
                self, "_robot_description_source"
            )
            relative_path: Path = object.__getattribute__(self, "_robot_description_relative_path")
            return RobotDescriptionPath(source, relative_path.parent)

        if name in {"name", "stem", "suffix", "parts"}:
            relative_path = object.__getattribute__(self, "_robot_description_relative_path")
            return getattr(relative_path, name)

        return getattr(object.__getattribute__(self, "_resolve")(), name)

    def __str__(self) -> str:
        return str(self._resolve())

    def __fspath__(self) -> str:
        return str(self._resolve())

    def __truediv__(self, other: object) -> RobotDescriptionPath:
        source: RobotDescriptionSource = object.__getattribute__(self, "_robot_description_source")
        relative_path: Path = object.__getattribute__(self, "_robot_description_relative_path")
        return RobotDescriptionPath(source, relative_path / str(other))

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, RobotDescriptionPath):
            return self._resolve() == other
        source: RobotDescriptionSource = object.__getattribute__(self, "_robot_description_source")
        other_source: RobotDescriptionSource = object.__getattribute__(
            other, "_robot_description_source"
        )
        relative_path: Path = object.__getattribute__(self, "_robot_description_relative_path")
        other_relative_path: Path = object.__getattribute__(
            other, "_robot_description_relative_path"
        )
        return source == other_source and relative_path == other_relative_path

    def __hash__(self) -> int:
        return hash(
            (
                object.__getattribute__(self, "_robot_description_source"),
                object.__getattribute__(self, "_robot_description_relative_path"),
            )
        )

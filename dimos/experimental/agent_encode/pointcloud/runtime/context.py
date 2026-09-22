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

"""What every handler gets to work with: the cloud, its finite points, and
where image files go."""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Iterator
from contextlib import contextmanager
from dataclasses import dataclass, field, replace
import hashlib
from itertools import count
import os
from pathlib import Path
import tempfile
from typing import TYPE_CHECKING, Generic, Protocol, TypeVar, cast, runtime_checkable

import numpy as np

from dimos.experimental.agent_encode.pointcloud.render import raster as render
from dimos.experimental.agent_encode.pointcloud.runtime.recipe import canonical, describe

if TYPE_CHECKING:
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

T = TypeVar("T")
T_co = TypeVar("T_co", covariant=True)


@dataclass(frozen=True)
class EncodeContext:
    cloud: PointCloud2
    points: np.ndarray
    """Finite (N, 3) float32."""
    out_dir: Path
    stem: str
    """File name prefix for anything a handler writes."""
    cache: dict[tuple[str, int], object] = field(default_factory=dict, compare=False, repr=False)
    """Each request's result, by its description and the points it saw."""
    stems: Iterator[int] = field(default_factory=count, compare=False, repr=False)
    """Numbers each evaluated request's file names."""
    parent: EncodeContext | None = field(default=None, compare=False, repr=False)

    @property
    def root(self) -> EncodeContext:
        return self if self.parent is None else self.parent.root

    def evaluate(self, node: Node[T]) -> T:
        """Evaluate a request once per description for this point selection."""
        if not isinstance(node, Node):
            raise TypeError(
                f"{type(node).__name__} is not a request; shapes and grids go inside one, "
                "as in Overlap(Box(...))"
            )
        key = (canonical(describe(node)), id(self.points))
        if key not in self.cache:
            child = replace(self, stem=f"{self.stem}_{next(self.stems):04d}", parent=self.root)
            self.cache[key] = node.run(child)
        return cast("T", self.cache[key])

    @contextmanager
    def artifact(self, suffix: str) -> Iterator[tuple[Path, Path]]:
        """Stage an artifact beside its deterministic final path, then replace atomically."""
        final = self.out_dir / f"{self.stem}_{suffix}"
        final.parent.mkdir(parents=True, exist_ok=True)
        descriptor, temporary = tempfile.mkstemp(
            prefix=f".{final.stem}-", suffix=final.suffix, dir=final.parent
        )
        os.close(descriptor)
        staging = Path(temporary)
        try:
            yield staging, final
            os.replace(staging, final)
        finally:
            staging.unlink(missing_ok=True)

    def select(self, source: Selection | None) -> EncodeContext:
        """Use a field's points, preserving coordinates and the shared evaluation cache."""
        if source is None:
            return self
        if not isinstance(source, Selection):
            raise TypeError(
                f"source must be a selection such as Select(...), not {type(source).__name__}"
            )
        cloud, points = self.evaluate(_Selected(source))
        return replace(self, cloud=cloud, points=points, parent=self.root)

    @property
    def spacing_m(self) -> float:
        """The typical gap between neighbouring returns: the median of each
        return's distance to its nearest neighbour. Renders draw no finer
        than this, so their cells and splats close at the cloud's own
        resolution."""
        return self.evaluate(_Spacing())

    @property
    def fingerprint(self) -> str:
        """SHA-256 of the whole cloud's finite points, frame and timestamp."""
        return self.root.evaluate(_Fingerprint())


@runtime_checkable
class Node(Protocol[T_co]):
    """Anything ``EncodeContext.evaluate`` computes once per call."""

    def run(self, ctx: EncodeContext) -> T_co: ...


@dataclass(frozen=True)
class Result:
    """What a request measured, as the agent reads it."""

    def summary(self) -> Result:
        """The part a named output reports; results holding raw arrays override it."""
        return self


R_co = TypeVar("R_co", bound=Result, covariant=True)


class Request(ABC, Generic[R_co]):
    """What an agent can name in ``agent_encode()``: a query, a render, an output or a field."""

    @abstractmethod
    def run(self, ctx: EncodeContext) -> R_co: ...


class Selection(ABC):
    """A lazy set of returns, passed as another node's ``source``."""

    @abstractmethod
    def run(self, ctx: EncodeContext) -> np.ndarray:
        """The selected (N, 3) returns."""


@dataclass(frozen=True)
class _Selected:
    source: Selection

    def run(self, ctx: EncodeContext) -> tuple[PointCloud2, np.ndarray]:
        points = ctx.evaluate(self.source)
        cloud = type(ctx.cloud).from_numpy(
            points, frame_id=ctx.cloud.frame_id, timestamp=ctx.cloud.ts
        )
        return cloud, points


@dataclass(frozen=True)
class _Spacing:
    def run(self, ctx: EncodeContext) -> float:
        return render.cloud_spacing(ctx.points)


@dataclass(frozen=True)
class _Fingerprint:
    def run(self, ctx: EncodeContext) -> str:
        digest = hashlib.sha256(np.ascontiguousarray(ctx.points, dtype="<f4").tobytes())
        digest.update(
            canonical({"frame": ctx.cloud.frame_id, "ts": describe(ctx.cloud.ts)}).encode()
        )
        return digest.hexdigest()

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

from collections.abc import Iterator
from contextlib import contextmanager
from dataclasses import dataclass, field
from functools import cached_property
import os
from pathlib import Path
import tempfile
from typing import TYPE_CHECKING, Any, Protocol

import numpy as np

from dimos.experimental.agent_encode.pointcloud.render import raster as render

if TYPE_CHECKING:
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


@dataclass(frozen=True)
class EncodeContext:
    cloud: PointCloud2
    points: np.ndarray
    """Finite (N, 3) float32."""
    out_dir: Path
    stem: str
    """File name prefix for anything a handler writes."""
    cache: dict[tuple[Any, ...], Any] = field(default_factory=dict, compare=False, repr=False)
    parent: EncodeContext | None = field(default=None, compare=False, repr=False)

    @property
    def root(self) -> EncodeContext:
        return self if self.parent is None else self.parent.root

    def evaluate(self, node: Any) -> Any:
        """Evaluate a shared request once for this point selection."""
        key = (id(node), id(self.points))
        if key not in self.cache:
            self.cache[("node", id(node))] = node
            stem_key = ("stem_index", id(node), id(self.points))
            if stem_key not in self.cache:
                index = int(self.cache.get(("stem_count",), 0))
                self.cache[("stem_count",)] = index + 1
                self.cache[stem_key] = index
            child = EncodeContext(
                self.cloud,
                self.points,
                self.out_dir,
                f"{self.stem}_{self.cache[stem_key]:04d}",
                self.cache,
                self.root,
            )
            self.cache[key] = node.run(child)
        return self.cache[key]

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

    def select(self, source: Any) -> EncodeContext:
        """Use a field's points, preserving coordinates and the shared evaluation cache."""
        if source is None:
            return self
        points = self.evaluate(source)
        if not isinstance(points, np.ndarray) or points.ndim != 2 or points.shape[1] != 3:
            raise TypeError("source must evaluate to an (N, 3) point array")
        key = ("cloud", id(points))
        if key not in self.cache:
            self.cache[key] = type(self.cloud).from_numpy(
                points, frame_id=self.cloud.frame_id, timestamp=self.cloud.ts
            )
        return EncodeContext(
            self.cache[key], points, self.out_dir, self.stem, self.cache, self.root
        )

    @cached_property
    def spacing_m(self) -> float:
        """The typical gap between neighbouring returns: the median of each
        return's distance to its nearest neighbour. Renders draw no finer
        than this, so their cells and splats close at the cloud's own
        resolution."""
        return render.cloud_spacing(self.points)


class Handler(Protocol):
    """One request inside ``agent_encode(*handlers)``: a render or a query,
    fully parameterised by the caller."""

    def run(self, ctx: EncodeContext) -> dict[str, Any]: ...

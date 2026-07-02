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

from collections.abc import Iterator
from typing import TYPE_CHECKING

from dimos.protocol.tf.db_tf import DbTf
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.memory2.store.base import Store
    from dimos.memory2.stream import Stream
    from dimos.memory2.type.observation import Observation
    from dimos.msgs.geometry_msgs.Transform import Transform
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

logger = setup_logger()

WORLD_FRAME = "world"


class _Unset:
    pass


_UNSET = _Unset()


class WorldRegistrar:
    """Bring frame-tagged observations into ``world`` using a recording's tf stream.

    Messages already in ``world`` (or with no frame) pass through untouched.
    Otherwise the ``world <- frame_id`` transform is looked up in the recording's
    ``tf`` stream (loaded lazily on first non-world frame). A lookup that fails —
    no tf stream, or no chain to that frame — is warned about, counted in
    :attr:`skipped`, and the message is dropped.
    """

    def __init__(self, store: Store) -> None:
        self._store = store
        self._db_tf: DbTf | None | _Unset = _UNSET
        self.skipped = 0

    def _tf(self) -> DbTf | None:
        if isinstance(self._db_tf, _Unset):
            self._db_tf = DbTf.from_store(self._store)
        return self._db_tf

    def world_transform(self, frame_id: str, ts: float) -> tuple[bool, Transform | None]:
        """Resolve how to place ``frame_id`` data into world at ``ts``.

        Returns ``(keep, transform)``: ``keep=False`` means skip the message;
        ``transform=None`` means it is already world (render/accumulate directly).
        """
        if not frame_id or frame_id == WORLD_FRAME:
            return True, None
        db_tf = self._tf()
        transform = db_tf.get(WORLD_FRAME, frame_id, time_point=ts) if db_tf is not None else None
        if transform is None:
            self.skipped += 1
            logger.warning(f"tf: no '{WORLD_FRAME} <- {frame_id}' at ts={ts:.3f} — skipping frame")
            return False, None
        return True, transform

    def register_cloud(self, cloud: PointCloud2, ts: float) -> PointCloud2 | None:
        """World-registered copy of ``cloud`` (itself if already world), or ``None`` to skip."""
        keep, transform = self.world_transform(cloud.frame_id, ts)
        if not keep:
            return None
        return cloud if transform is None else cloud.transform(transform)

    def register_clouds(self, stream: Stream[PointCloud2]) -> Stream[PointCloud2]:
        """World-register a PointCloud2 stream, dropping frames that can't be placed."""

        def _register(
            upstream: Iterator[Observation[PointCloud2]],
        ) -> Iterator[Observation[PointCloud2]]:
            for observation in upstream:
                cloud = self.register_cloud(observation.data, observation.ts)
                if cloud is not None:
                    yield observation.derive(data=cloud)

        return stream.transform(_register)

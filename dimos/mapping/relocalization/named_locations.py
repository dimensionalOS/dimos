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

"""Named locations on disk and in tf; the module side is in ``module.py``."""

from __future__ import annotations

from pathlib import Path

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.visualization_msgs.InteractiveMarkerUpdate import (
    InteractiveMarkerUpdate,
    pose_of,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def load_locations(path: str, stream: str) -> dict[str, PoseStamped]:
    """The locations in the last snapshot on *stream* of a .db or .mcap."""
    from dimos.mapping.relocalization.module import find_stream
    from dimos.memory.cli.dataset import open_store

    if not Path(path).exists():
        logger.warning(f"named locations file {path} does not exist yet, starting with none")
        return {}
    with open_store(path, decode_metadata=True) as store:
        name = find_stream(store.list_streams(), stream)
        if name is None:
            logger.warning(f"{path} has no {stream!r} stream, starting with no named locations")
            return {}
        last: InteractiveMarkerUpdate = store.stream(name).last().data
    return {named.name: pose_of(named) for named in last.poses}

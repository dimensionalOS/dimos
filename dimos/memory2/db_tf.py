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

"""Compatibility stub for the removed ``dimos.memory2.db_tf`` module.

Some older recordings register a ``tf_graph`` stream whose payload type is
``dimos.memory2.db_tf.TfGraph``. That module no longer exists, so any tool that
resolves every stream's payload type up front (e.g. ``dimos map summary`` /
``dimos map replay`` via ``_stream_payload_types``) fails to import the whole
recording with ``ModuleNotFoundError: No module named 'dimos.memory2.db_tf'``.

The ``tf_graph`` stream is irrelevant for mapping — those tools only iterate the
PointCloud2 and odometry streams — so this is a do-nothing placeholder that just
lets the payload-type lookup succeed. It is not a functional TF graph; iterating
or decoding a ``tf_graph`` stream through it is not supported.
"""

from __future__ import annotations

from typing import Any


class TfGraph:
    """Inert stand-in for the old ``db_tf.TfGraph`` payload type."""

    def __init__(self, *_args: Any, **_kwargs: Any) -> None:
        pass

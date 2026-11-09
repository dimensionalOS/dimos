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
``dimos.memory2.db_tf.TfGraph``. That module is gone, so any tool resolving every stream's
payload type up front (``dimos map summary``, ``dimos map replay``) fails to import the whole
recording with ``ModuleNotFoundError``. The ``tf_graph`` stream is irrelevant for mapping, so
this is a do-nothing placeholder. Iterating or decoding a ``tf_graph`` stream through it is not
supported.
"""

from __future__ import annotations

from typing import Any


class TfGraph:
    """Inert stand-in for the old ``db_tf.TfGraph`` payload type."""

    def __init__(self, *_args: Any, **_kwargs: Any) -> None:
        pass

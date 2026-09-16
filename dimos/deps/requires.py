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

"""Runtime requirements, declared next to the code that has them.

A file says what its code needs when it runs, not only when it is imported,
with a literal ``Requires(...)`` assigned at module scope::

    from dimos.deps.requires import Requires

    REQUIRES = Requires(extras=("perception",))

or in a class body (``requires = Requires(...)``). The dependency catalog
reads the literal with ``ast`` without importing the file, so the call must
use constant keyword arguments only: tuples or lists of strings, and a dict
of strings for ``selectors``. A file's requirements are the union of its
declarations, and every third-party import in the file, eager or lazy, must
be covered by core or by a declared extra; the catalog generation test names
the declaration to add.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field


@dataclass(frozen=True)
class Requires:
    extras: tuple[str, ...] = ()
    """pyproject extras the file's code needs; whole extras, not packages."""
    backends: tuple[str, ...] = ()
    """Abstract backends the hardware profile resolves, such as ``onnxruntime``."""
    native: tuple[str, ...] = ()
    """In-tree native artifacts built separately, such as ``dimos-memory-recorder``."""
    system: tuple[str, ...] = ()
    """Host-provided import names, such as ``rclpy``."""
    tools: tuple[str, ...] = ()
    """Executables, such as ``deno`` or ``ffmpeg``."""
    defers: tuple[str, ...] = ()
    """Top-level names this file imports lazily on behalf of callers that declare them."""
    subprocesses: tuple[str, ...] = ()
    """Dotted modules this file runs as subprocesses; their requirements count too."""
    selectors: Mapping[str, str] = field(default_factory=dict)
    """Configuration field -> registry family whose entry it selects.

    ``"hardware": "adapter"`` on a Module class keys on that module's own
    field; ``"g.unitree_connection_type": "connection"`` keys on global config.
    """

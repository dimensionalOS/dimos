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

"""Source-tree traversal shared by repository checks."""

from collections.abc import Iterator
from pathlib import Path

VIRTUAL_ENVIRONMENT_DIRECTORIES = frozenset({".venv", "venv"})


def python_source_files(root: Path) -> Iterator[Path]:
    """Yield Python source files while excluding generated virtual environments."""
    for path in root.rglob("*.py"):
        if not VIRTUAL_ENVIRONMENT_DIRECTORIES.intersection(path.parts):
            yield path

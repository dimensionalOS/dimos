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

from pathlib import Path
import shutil
import sys


def find_executable(name: str) -> str | None:
    """Find a tool beside this interpreter first, then on PATH.

    Keep the interpreter path unresolved so a venv's tools win over another
    installation on PATH, even when the venv has not been activated.
    """
    if sys.executable:
        sibling = shutil.which(name, path=str(Path(sys.executable).parent))
        if sibling is not None:
            return sibling
    return shutil.which(name)

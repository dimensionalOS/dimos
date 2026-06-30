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
import subprocess
import sys


def test_runtime_protocol_import_does_not_import_dimos_or_backends() -> None:
    repo = Path(__file__).resolve().parents[3]
    protocol_src = repo / "packages" / "dimos-runtime-protocol" / "src"
    code = f"""
import sys
sys.path.insert(0, r'{protocol_src}')
import dimos_runtime_protocol
for name in ('dimos', 'robosuite', 'libero', 'omnigibson'):
    if name in sys.modules:
        raise SystemExit(f'{{name}} was imported')
print(dimos_runtime_protocol.PROTOCOL_VERSION)
"""

    result = subprocess.run(
        [sys.executable, "-c", code],
        check=True,
        capture_output=True,
        text=True,
    )

    assert result.stdout.strip()

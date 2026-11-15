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

"""Unit tests for the pure DataPrep helpers in `core.py`.

No I/O: a tiny in-memory fake stands in for `SqliteStore`, exposing only the
surface the helpers touch (`stream(name)` → iterable of `.ts`/`.data` records,
with `.time_range(t0, t1)`). Keeps these fast and dependency-free.
"""

from pathlib import Path
import subprocess
import sys
import textwrap


def test_sqlite_and_hdf5_do_not_require_mcap(tmp_path: Path) -> None:
    # A fresh interpreter also catches accidental transitive MCAP imports.
    script = textwrap.dedent("""
        from pathlib import Path
        import sys

        sys.modules["mcap"] = None
        from dimos.imitation.dataprep.build import _open_recording
        from dimos.imitation.dataprep.core import get_writer
        from dimos.memory.store.sqlite import SqliteStore

        path = Path(sys.argv[1]) / "recording.db"
        store = SqliteStore(path=str(path))
        store.start()
        store.stop()
        reader = _open_recording(path)
        try:
            assert reader.list_streams() == []
        finally:
            reader.stop()
        assert callable(get_writer("hdf5"))

        try:
            _open_recording(path.with_suffix(".mcap"))
        except ModuleNotFoundError as error:
            assert error.name == "mcap.reader", error.name
        else:
            raise AssertionError("MCAP-specific import must fail without MCAP")
    """)
    result = subprocess.run(
        [sys.executable, "-c", script, str(tmp_path)],
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert result.returncode == 0, result.stdout + result.stderr

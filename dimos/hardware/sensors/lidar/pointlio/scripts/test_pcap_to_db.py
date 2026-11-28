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

from contextlib import closing
from pathlib import Path
import sqlite3

from dimos.hardware.sensors.lidar.pointlio.scripts.pcap_to_db import _odom_stats


def test_polling_odometry_reads_the_exact_file(tmp_path: Path) -> None:
    path = tmp_path / "drive #1.db"
    with closing(sqlite3.connect(path)) as conn:
        conn.execute("CREATE TABLE odom(ts REAL)")
        conn.execute("INSERT INTO odom VALUES(3.25)")
        conn.commit()
    original = path.read_bytes()

    assert _odom_stats(path, "odom") == (1, 3.25, 3.25)
    assert list(tmp_path.iterdir()) == [path]
    assert path.read_bytes() == original

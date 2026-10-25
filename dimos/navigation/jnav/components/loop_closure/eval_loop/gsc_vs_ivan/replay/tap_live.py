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

"""Record a running blueprint's nav topics into one db (dimos run --record is off under --replay).
usage: tap_live.py <out.db> <seconds>"""

from pathlib import Path
import sys
import time

from dimos.core.transport_factory import make_transport
from dimos.memory.store.sqlite import SqliteStore
from dimos.memory.tap import TransportRecorder
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path as NavPath
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

out, seconds = Path(sys.argv[1]), float(sys.argv[2])
out.unlink(missing_ok=True)
store = SqliteStore(path=str(out))
store.start()
recorder = TransportRecorder(store, "*", queue_size=100000)
topics = {
    "path": NavPath,
    "global_map": PointCloud2,
    "corrected_odometry": Odometry,
    "loop_closure_records": PointCloud2,
    "closure_edges": PointCloud2,
    "goal": PointStamped,
    "odom": PoseStamped,
}
unsubs = [recorder.tap(name, kind, make_transport(name, kind)) for name, kind in topics.items()]
deadline = time.time() + seconds
while time.time() < deadline:
    time.sleep(1)
for u in unsubs:
    if u:
        u()
recorder.close()
store.stop()
print("wrote", out, "dropped", recorder.dropped, flush=True)

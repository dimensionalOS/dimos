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

"""Message-input Point-LIO (rust) fed by a replayed raw recording — the whole
sensor->SLAM chain with no network, no SDK, no sudo (issue #2821)::

    uv run python -m dimos.hardware.sensors.lidar.livox.scripts.pcap_to_raw_db \\
        --pcap data/mid360_shake_stairs/mid360_shake_stairs.pcap
    uv run dimos run pointlio-rust-replay
"""

from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.sensors.lidar.livox.replay import RawSensorReplay
from dimos.hardware.sensors.lidar.pointlio.rust_module import PointLioRust
from dimos.visualization.vis_module import vis_module

pointlio_rust_replay = autoconnect(
    RawSensorReplay.blueprint(),
    PointLioRust.blueprint(),
    vis_module("rerun"),
).global_config(n_workers=2, robot_model="pointlio_rust_replay")

# Copyright 2025-2026 Dimensional Inc.
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

"""Spot: drive it from the Rerun web UI while recording every data stream.

The default `spot` blueprint (click/teleop driving + full sensor streaming +
Rerun) with `SpotRecorder` added, so every one of `SpotHighLevel`'s streams
(the five fisheye + five depth cameras and odometry, plus the live tf tree) is
written to a memory2 SQLite db as you drive. `autoconnect` wires the recorder's
In ports to `SpotHighLevel`'s outputs by name.

Usage:
    dimos run spot-record \
        -o spothighlevel.username=admin -o spothighlevel.password=<password>
    # choose where the recording lands:
    dimos run spot-record ... -o spotrecorder.db_path=/path/to/spot.db
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.bosdyn.spot.blueprints.spot import spot
from dimos.robot.bosdyn.spot.recorder import SpotRecorder

spot_record = autoconnect(spot, SpotRecorder.blueprint())
